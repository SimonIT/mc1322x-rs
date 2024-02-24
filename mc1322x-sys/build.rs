extern crate bindgen;

use std::env;
use std::path::PathBuf;
use std::process::Command;

fn main() {
    let root = PathBuf::from("./libmc1322x")
        .canonicalize()
        .expect("cannot canonicalize path");
    let lib = root.join("lib");
    let include = lib.join("include");
    let src = root.join("src");
    let tests = root.join("tests");
    let header = include.join("mc1322x.h");
    let header_str = header.to_str().unwrap();

    println!(
        "cargo:include={}",
        include.display()
    );

    println!(
        "cargo:rerun-if-changed={}",
        header_str
    );

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

    println!(
        "cargo:rustc-link-search=native={}",
        lib.display()
    );

    println!("cargo:rustc-link-lib=mc1322x");

    println!("cargo:warning={}", header_str);
    let bindings = bindgen::Builder::default()
        .header(header_str)
        .clang_arg(format!("-I{}", &include.display()))
        .use_core()
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        .generate()
        .expect("Unable to generate bindings");

    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");
}

