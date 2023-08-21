fn prepare_acquire_core_libs() {
    let outdir = std::path::PathBuf::from(std::env::var("OUT_DIR").unwrap());

    let headers = {
        let root = "acquire-core-libs/src/acquire-device-kit/device/kit";
        [format!("{root}/driver.h"), format!("{root}/camera.h")]
    };

    bindgen::Builder::default()
        .header(&headers[0])
        .header(&headers[1])
        .clang_args([
            "-Iacquire-core-libs/src/acquire-device-kit",
            "-Iacquire-core-libs/src/acquire-device-properties",
        ])
        .parse_callbacks(Box::new(bindgen::CargoCallbacks))
        .generate()
        .expect("Unable to generate bindings")
        .write_to_file(outdir.join("acquire-capi.rs"))
        .expect("Failed to write bindings");
}

fn prepare_pvcam() {
    struct ImportedLib {
        include_path: String,
        library_path: String,
        library_name: String,
    }
    fn find_pvcam() -> ImportedLib {
        #[cfg(target_os = "windows")]
        ImportedLib {
            include_path: "C:\\Program Files\\Photometrics\\PVCamSDK\\Inc".into(),
            library_path: "C:\\Program Files\\Photometrics\\PVCamSDK\\Lib\\amd64".into(),
            library_name: "pvcam64".into(),
        }
    }

    let outdir = std::path::PathBuf::from(std::env::var("OUT_DIR").unwrap());
    let lib = find_pvcam();

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
    prepare_pvcam();
}
