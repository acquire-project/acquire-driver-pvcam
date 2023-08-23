fn prepare_acquire_core_libs() {
    let outdir = std::path::PathBuf::from(std::env::var("OUT_DIR").unwrap());

    let headers = {
        let adk = "acquire-core-libs/src/acquire-device-kit/device/kit";
        let acl = "acquire-core-libs/src/acquire-core-logger/";
        [
            format!("{adk}/driver.h"),
            format!("{adk}/camera.h"),
            format!("{acl}/logger.h"),
        ]
    };

    bindgen::Builder::default()
        .header(&headers[0])
        .header(&headers[1])
        .clang_args([
            "-Iacquire-core-libs/src/acquire-core-logger",
            "-Iacquire-core-libs/src/acquire-device-kit",
            "-Iacquire-core-libs/src/acquire-device-properties",
        ])
        .parse_callbacks(Box::new(bindgen::CargoCallbacks))
        .generate()
        .expect("Unable to generate bindings. Did you update submodules?")
        .write_to_file(outdir.join("acquire-capi.rs"))
        .expect("Failed to write bindings");
}

fn prepare_pvcam() {
    struct ImportedLib {
        include_path: String,
        library_path: String,
        library_name: String,
    }
    #[cfg(target_os = "windows")]
    fn find_pvcam() -> Option<ImportedLib> {
        Some(ImportedLib {
            include_path: "C:\\Program Files\\Photometrics\\PVCamSDK\\Inc".into(),
            library_path: "C:\\Program Files\\Photometrics\\PVCamSDK\\Lib\\amd64".into(),
            library_name: "pvcam64".into(),
        })
    }
    #[cfg(target_os = "macos")]
    fn find_pvcam() -> Option<ImportedLib> {
        None
    }

    let outdir = std::path::PathBuf::from(std::env::var("OUT_DIR").unwrap());
    let lib = find_pvcam().expect("Could not locate PVCAM");

    let headers = {
        [
            format!("{}\\master.h", lib.include_path),
            format!("{}\\pvcam.h", lib.include_path),
        ]
    };

    println!("cargo:rustc-link-search={}", lib.library_path);
    println!("cargo:rustc-link-lib={}", lib.library_name);

    bindgen::Builder::default()
        .header(&headers[0])
        .header(&headers[1])
        .clang_arg(format!("-I{}", lib.include_path))
        .parse_callbacks(Box::new(bindgen::CargoCallbacks))
        .generate()
        .expect("Unable to generate bindings")
        .write_to_file(outdir.join("pvcam-capi.rs"))
        .expect("Failed to write bindings");
}

fn main() {
    prepare_acquire_core_libs();
    // prepare_pvcam();
}
