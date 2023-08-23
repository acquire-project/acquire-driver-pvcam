mod capi;

use log::debug;
use std::ffi::{c_char, c_int, CStr};

type ReporterCallback = Option<
    unsafe extern "C" fn(
        is_error: c_int,
        file: *const c_char,
        line: c_int,
        function: *const c_char,
        msg: *const c_char,
    ),
>;

extern "C" fn reporter(
    is_error: c_int,
    file: *const c_char,
    line: c_int,
    function: *const c_char,
    msg: *const c_char,
) {
    fn make_string(ptr: *const c_char) -> String {
        if ptr.is_null() {
            String::new()
        } else {
            unsafe { CStr::from_ptr(ptr) }.to_string_lossy().to_string()
        }
    }

    let file = make_string(file);
    let function = make_string(function);
    let msg = make_string(msg);

    if `
}

#[test]
fn init_and_shutdown() {
    println!("I'm in {}", std::env::current_exe().unwrap().display());

    unsafe {
        let lib = libloading::Library::new(
            std::env::current_exe()
                .unwrap()
                .join("../acquire_driver_pvcam.dll"),
        )
        .expect("Could not find acquire-driver-pvcam");

        let entry: libloading::Symbol<extern "C" fn(ReporterCallback) -> *mut capi::Driver> = lib
            .get(b"acquire_driver_init_v0")
            .expect("Expected to find entry point: acquire_driver_init_v0");
        let driver = entry(Some(reporter));
    }

    todo!()
}
