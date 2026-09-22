fn main() {
    println!("cargo::rustc-link-arg=-Tembedded-test.x");
    println!("cargo::rustc-check-cfg=cfg(rust_analyzer)");
}
