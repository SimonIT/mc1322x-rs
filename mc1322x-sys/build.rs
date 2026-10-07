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
    // `src-romvars.a` instead of `src.a`: ROM routines (`nvm_*`, radio) call through the ROM
    // patch table vectors at RAM offsets 0x20/0x60/0xa0/0xe0, which this variant's `start.S`
    // (`USE_ROM_VARS`) reserves and fills with `bx lr` stubs. libmc1322x's tests/Makefile uses
    // it for the same targets (`TARGETS_WITH_ROM_VARS`).
    let srclib = src.join("src-romvars.a");

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

    // The plain `make` above doesn't reliably build `src-romvars.a`, so request it explicitly.
    // It must run from `tests/`: `src/` has no Makefile, and `tests/Makefile` (with
    // `MC1322X := ..`) defines the `$(MC1322X)/src/src-romvars.a` rule under this path.
    let romvars_output = Command::new("make")
        .current_dir(&tests)
        .arg("../src/src-romvars.a")
        .output()
        .expect("failed to execute make for src-romvars.a");
    assert!(
        romvars_output.status.success(),
        "make src-romvars.a failed:\nstdout: {}\nstderr: {}",
        std::str::from_utf8(&romvars_output.stdout).unwrap(),
        std::str::from_utf8(&romvars_output.stderr).unwrap()
    );

    println!("cargo:rerun-if-changed={}", lib.display());
    println!("cargo:rustc-link-search=native={}", lib.display());

    println!("cargo:rustc-link-lib=mc1322x");

    println!("cargo:rerun-if-changed={}", srclib.display());
    println!("cargo:rustc-link-search=native={}", src.display());
    println!("cargo:rustc-link-lib=static:+verbatim=src-romvars.a");

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

    // Extract the libgcc objects `.cargo/config.toml` force-links (32-bit division, Thumb-1
    // `switch` jump tables); see the comments there.
    extract_libgcc_objects(&multilib_flags);

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

/// Extract the libgcc objects that `.cargo/config.toml` force-links from the installed
/// toolchain's `libgcc.a` into `<workspace root>/.libgcc-thumbv4t/`. A fixed, gitignored path
/// rather than `$OUT_DIR`, so the static rustflags can name the files.
fn extract_libgcc_objects(multilib_flags: &[&str]) {
    let libgcc_path = arm_none_eabi_gcc_file(multilib_flags, "libgcc.a");

    let workspace_root = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("mc1322x-sys is a direct child of the workspace root")
        .to_path_buf();
    let out_dir = workspace_root.join(".libgcc-thumbv4t");
    std::fs::create_dir_all(&out_dir).expect("failed to create .libgcc-thumbv4t");

    let members = [
        "_udivsi3.o",
        "_divsi3.o",
        "_dvmd_tls.o",
        "_thumb1_case_uqi.o",
        "_thumb1_case_sqi.o",
    ];
    let output = Command::new("arm-none-eabi-ar")
        .arg("x")
        .arg(&libgcc_path)
        .args(members)
        .current_dir(&out_dir)
        .output()
        .expect("failed to run arm-none-eabi-ar");
    assert!(
        output.status.success(),
        "arm-none-eabi-ar x {} failed:\nstdout: {}\nstderr: {}",
        libgcc_path.display(),
        std::str::from_utf8(&output.stdout).unwrap(),
        std::str::from_utf8(&output.stderr).unwrap()
    );
}

/// Ask `arm-none-eabi-gcc` where `file` (e.g. `libc.a`) lives for the multilib selected by
/// `flags`.
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
