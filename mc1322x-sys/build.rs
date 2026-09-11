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
    // Linked against instead of the plain `src.a`: any program calling a ROM routine that
    // itself calls through the boot ROM's "ROM Patch Table Vector" mechanism (fixed RAM
    // offsets 0x20/0x60/0xa0/0xe0 from the load address - see `start.S`'s `USE_ROM_VARS`
    // block) needs those slots reserved and populated with `bx lr` stubs, or the ROM call
    // jumps into whatever unrelated code happens to occupy that RAM instead. `libmc1322x`'s
    // own tests/Makefile builds every target touching `nvm_*`/radio ROM calls against this
    // variant (`TARGETS_WITH_ROM_VARS`) for exactly this reason.
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

    // The plain `make` above builds `src.a` as a side effect of the default (non-ROM-vars)
    // TARGETS, but doesn't reliably reach `src-romvars.a` (a dependency of the
    // TARGETS_WITH_ROM_VARS .bin files, several boards deep in `Makefile.include`'s
    // per-board recursion) - ask for it directly so it exists regardless.
    //
    // This must run from `tests/` (not `src/`, which has no `Makefile` - only the
    // variable-less `Makefile.src` fragment it `-include`s): `tests/Makefile` sets `MC1322X :=
    // ..` and defines the real `$(MC1322X)/src/src-romvars.a` rule, so the target is requested
    // by that same relative path. A stale `src/src-romvars.a` normally masked this working by
    // accident (cargo's build-script caching meant this command wasn't actually re-run for a
    // long time) - if this ever regresses, deleting `src/{start,start-romvars}.o` and
    // `src/src{,-romvars}.a` forces a clean rebuild that exercises this path for real.
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

    // `libmc1322x/lib/pwm.c`'s `switch` statements on `int timer_num` compile to Thumb-1
    // jump-table lookups needing `__gnu_thumb1_case_uqi`, a libgcc helper - nothing else in
    // this workspace pulls pwm.c's object into the link, so this only surfaced once
    // `mc1322x-hal::pwm` was first actually exercised (`examples/pwm-selftest`).
    let libgcc_path = arm_none_eabi_gcc_file(&multilib_flags, "libgcc.a");
    println!(
        "cargo:rustc-link-search=native={}",
        libgcc_path.parent().unwrap().display()
    );
    println!("cargo:rustc-link-lib=gcc");

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
