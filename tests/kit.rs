use more_asserts::assert_gt;
use std::ffi::{c_char, c_int, CStr};
use std::ptr::NonNull;

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

struct TestPvcamDriver {
    driver: NonNull<aq::Driver>,
}

/// Generates boiler plate for calling a c-method as a member.
/// Example: `call(self.driver,shutdown,args)`
macro_rules! call {
    ($ptr:expr,$function:ident) => {
        unsafe {
            $ptr.as_ref().$function.expect(concat!(
                "Expected non-NULL method for '",
                stringify!($function),
                "'"
            ))($ptr.as_ptr())
        }
    };
    ($ptr:expr,$function:ident, $($args:tt),+) => {
        unsafe {
            $ptr.as_ref().$function.expect(concat!(
                "Expected non-NULL method for '",
                stringify!($function),
                "'"
            ))($ptr.as_ptr(), $($args:tt),+)
        }
    };
}

impl TestPvcamDriver {
    fn new() -> Self {
        let dylib_path = test_cdylib::build_current_project();
        println!("Loading lib path: \"{}\"", dylib_path.display());

        let lib = unsafe { libloading::Library::new(dylib_path) }
            .expect("Could not find acquire-driver-pvcam");

        let entry: libloading::Symbol<extern "C" fn(ReporterCallback) -> *mut crate::aq::Driver> =
            unsafe { lib.get(b"acquire_driver_init_v0") }
                .expect("Expected to find entry point: acquire_driver_init_v0");

        Self {
            driver: NonNull::new(entry(Some(reporter)))
                .expect("acquire_driver_init_v0() returned NULL"),
        }
    }

    fn device_count(&self) -> u32 {
        call!(self.driver, device_count)
    }
}

impl Drop for TestPvcamDriver {
    fn drop(&mut self) {
        assert_eq!(
            call!(self.driver, shutdown),
            crate::aq::DeviceStatusCode_Device_Ok
        );
    }
}

#[test]
fn init_and_shutdown() {
    let _driver = TestPvcamDriver::new();
}

#[test]
fn device_count_is_greater_than_zero() {
    let driver = TestPvcamDriver::new();
    assert_gt!(driver.device_count(), 0)
}

pub(crate) mod aq {
    #![allow(non_upper_case_globals)]
    #![allow(non_camel_case_types)]
    #![allow(non_snake_case)]
    #![allow(unused)]

    include!(concat!(env!("OUT_DIR"), "/acquire-capi.rs"));
}
