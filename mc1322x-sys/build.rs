extern crate bindgen;

use std::env;
use std::io::Write;
use std::path::PathBuf;
use std::process::Command;

fn main() {
    let root = PathBuf::from("./libmc1322x")
        .canonicalize()
        .expect("cannot canonicalize path");
    let lib = root.join("lib");
    let include = lib.join("include");
    let tests = root.join("tests");
    let header = include.join("mc1322x.h");
    let header_str = header.to_str().unwrap();
    let gpio_util = include.join("gpio-util.h");
    let gpio_util_str = gpio_util.to_str().unwrap();
    let src = root.join("src");
    let srclib = src.join("src.a");

    println!("cargo:include={}", include.display());

    println!("cargo:rerun-if-changed={}", header_str);
    println!("cargo:rerun-if-changed={}", gpio_util_str);

    let output = Command::new("sh")
        .current_dir(&tests)
        .arg("-c")
        .arg("make")
        .output()
        .expect("failed to execute make");
    println!(
        "mab:make_done stdout: {:}",
        std::str::from_utf8(&output.stdout).unwrap()
    );
    println!(
        "mab:make_done stderr: {:}",
        std::str::from_utf8(&output.stderr).unwrap()
    );

    println!("cargo:rerun-if-changed={}", lib.display());
    println!("cargo:rustc-link-search=native={}", lib.display());

    println!("cargo:rustc-link-lib=mc1322x");

    println!("cargo:rerun-if-changed={}", srclib.display());
    println!("cargo:rustc-link-search=native={}", src.display());
    println!("cargo:rustc-link-lib=static:+verbatim=src.a");

    let multilib_flags = [
        "-march=armv4t",
        "-mtune=arm7tdmi-s",
        "-mlong-calls",
        "-msoft-float",
        "-mthumb",
    ];
    let libc_path = arm_none_eabi_gcc_file(&multilib_flags, "libc.a");
    println!(
        "cargo:rustc-link-search=native={}",
        libc_path.parent().unwrap().display()
    );
    println!("cargo:rustc-link-lib=c");

    let bindings = bindgen::Builder::default()
        .header(header_str)
        .header(gpio_util_str)
        .clang_arg(format!("-I{}", include.display()))
        .use_core()
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        .generate()
        .expect("Unable to generate bindings");

    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());

    let mut bindgen_output = Vec::<u8>::new();
    bindings
        .write(Box::new(&mut bindgen_output))
        .expect("String writing never fails");
    let bindgen_output = std::str::from_utf8(&bindgen_output)
        .expect("Rust source code is UTF-8")
        .to_string();

    let new_output = [
        "ASM",
        "UART1",
        "UART2",
        "CRM",
        "AUTO_ADC",
        "ADC",
        "GPIO_08",
        "GPIO_09",
        "GPIO_10",
        "GPIO_11",
        "XTAL32_EXISTS",
        "TIMER_WU_EN",
        "RTC_WU_EN",
        "EXT_WU_EN",
        "EXT_WU_EDGE",
        "EXT_WU_POL",
        "TIMER_WU_IEN",
        "RTC_WU_IEN",
        "EXT_WU_IEN",
        "RTC_WU_EVT",
        "EXT_WU_EVT",
        "ROSC_EN",
        "ROSC_FTUNE",
        "ROSC_CTUNE",
        "XTAL32_EN",
        "XTAL32_GAIN",
    ]
    .iter()
    .fold(bindgen_output, |a, s| {
        let lower = s.to_lowercase();
        a.replace(
            format!("{}: u32", s).as_str(),
            format!("{}: u32", lower).as_str(),
        )
        .replace(
            format!("::core::mem::transmute({})", s).as_str(),
            format!("::core::mem::transmute({})", lower).as_str(),
        )
        .replace(
            format!("{} as u64", s).as_str(),
            format!("{} as u64", lower).as_str(),
        )
    });

    std::fs::File::create(out_path.join("bindings.rs"))
        .expect("Failed to create bindings.rs")
        .write_all(new_output.as_bytes())
        .expect("Failed to write to bindings.rs");
}

/// Ask `arm-none-eabi-gcc` where `file` (e.g. `libc.a`) lives for the given
/// set of target flags, so the multilib variant always matches whatever
/// toolchain is actually installed.
fn arm_none_eabi_gcc_file(flags: &[&str], file: &str) -> PathBuf {
    let output = Command::new("arm-none-eabi-gcc")
        .args(flags)
        .arg(format!("-print-file-name={file}"))
        .output()
        .expect("failed to run arm-none-eabi-gcc");
    let path = std::str::from_utf8(&output.stdout)
        .expect("arm-none-eabi-gcc output is UTF-8")
        .trim();
    PathBuf::from(path)
        .canonicalize()
        .unwrap_or_else(|_| panic!("arm-none-eabi-gcc could not locate {file}"))
}
