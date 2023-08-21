fn main() {
    let outdir = std::path::PathBuf::from(std::env::var("OUT_DIR").unwrap());

    bindgen::Builder::default()
        .header("acquire-core-libs/src/acquire-device-kit/device/kit/driver.h")
        .header("acquire-core-libs/src/acquire-device-kit/device/kit/camera.h")
        .clang_args([
            "-Iacquire-core-libs/src/acquire-device-kit",
            "-Iacquire-core-libs/src/acquire-device-properties",
        ])
        .generate()
        .expect("Unable to generate bindings")
        .write_to_file(outdir.join("devkit-capi.rs"))
        .expect("Failed to write bindings");
}
