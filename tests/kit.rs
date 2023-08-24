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
    let sev = if is_error > 0 { "ERROR" } else { "INFO" };
    let line = line as i32;

    println!("{sev} {file}:{line} {function} - {msg}");
}

#[test]
fn init_and_shutdown() {
    let dylib_path = test_cdylib::build_current_project();
    println!("I'm in {}", dylib_path.display());

    unsafe {
        let lib =
            libloading::Library::new(dylib_path).expect("Could not find acquire-driver-pvcam");

        let entry: libloading::Symbol<extern "C" fn(ReporterCallback) -> *mut crate::aq::Driver> =
            lib.get(b"acquire_driver_init_v0")
                .expect("Expected to find entry point: acquire_driver_init_v0");
        let driver = entry(Some(reporter));
        assert_eq!(
            unsafe { (*driver).shutdown.unwrap()(driver) },
            crate::aq::DeviceStatusCode_Device_Ok
        );
    }
}

pub(crate) mod aq {
    #![allow(non_upper_case_globals)]
    #![allow(non_camel_case_types)]
    #![allow(non_snake_case)]
    #![allow(unused)]

    include!(concat!(env!("OUT_DIR"), "/acquire-capi.rs"));
}
