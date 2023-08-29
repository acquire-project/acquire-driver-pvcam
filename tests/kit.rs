use std::ffi::{c_char, c_int, CStr};
use std::ptr::NonNull;

use log::{debug, error, info};
use more_asserts::assert_gt;

use aq::Driver;

pub(crate) mod aq {
    #![allow(non_upper_case_globals)]
    #![allow(non_camel_case_types)]
    #![allow(non_snake_case)]
    #![allow(unused)]

    include!(concat!(env!("OUT_DIR"), "/acquire-capi.rs"));
}

fn setup() {
    dotenv::dotenv().ok();
    pretty_env_logger::init_timed();
}

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
    // let sev = if is_error > 0 { "ERROR" } else { "INFO" };
    let line = line as i32;

    if is_error > 0 {
        error!("{file}:{line} {function} - {msg}");
    } else {
        info!("{file}:{line} {function} - {msg}");
    }
    // println!("{sev} {file}:{line} {function} - {msg}");
}

struct TestPvcamDriver {
    driver: NonNull<Driver>,
}

/// Generates boiler plate for calling a c-method as a member.
/// Example: `call(self.driver,shutdown,args)`
macro_rules! call {
    ($ptr:expr,$function:ident) => {
        unsafe {
            let drv=$ptr.as_ptr();
            (*drv).$function.expect(concat!(
                "Expected non-NULL method for '",
                stringify!($function),
                "'"
            ))(drv)
        }
    };
    ($ptr:expr,$function:ident, $($args:tt),+) => {
        unsafe {
            let drv=$ptr.as_ptr();
            (*drv).$function.expect(concat!(
                "Expected non-NULL method for '",
                stringify!($function),
                "'"
            ))(drv, $($args:tt),+)
        }
    };
}

impl TestPvcamDriver {
    fn new() -> Self {
        let dylib_path = test_cdylib::build_current_project();
        debug!("Loading lib path: \"{}\"", dylib_path.display());

        let lib = unsafe { libloading::Library::new(dylib_path) }
            .expect("Could not find acquire-driver-pvcam");

        let entry: libloading::Symbol<extern "C" fn(ReporterCallback) -> *mut Driver> =
            unsafe { lib.get(b"acquire_driver_init_v0") }
                .expect("Expected to find entry point: acquire_driver_init_v0");

        let ptr = entry(Some(reporter));
        let driver = NonNull::new(ptr).expect("acquire_driver_init_v0() returned NULL");
        let out = Self { driver };
        info!("out: {:?} ptr {}", out.driver, ptr as usize);
        out
    }

    fn device_count(&self) -> u32 {
        let x = self.driver.as_ptr();
        debug!("XXX drop for ptr {:?}", x);
        debug!("XXX  device_count {:?}", unsafe { (*x).device_count });
        unsafe { (*x).device_count.unwrap()(x) }
    }
}

impl Drop for TestPvcamDriver {
    fn drop(&mut self) {
        call!(self.driver, shutdown);
        // let x = self.driver.as_ptr();
        // unsafe { (*x).shutdown.unwrap()(x) };
        // unsafe {
        //     (self.driver)
        //         .as_ref()
        //         .shutdown
        //         .expect("Expected non-NULL method for 'shutdown'")(
        //         (self.driver).as_ptr()
        //     );
        // }
        //assert_eq!(call!(self.driver, shutdown), DeviceStatusCode_Device_Ok);
    }
}

#[test]
fn init_and_shutdown() {
    setup();
    {
        let _driver = TestPvcamDriver::new();
    }
    info!("After drop");
}

#[test]
fn device_count_is_greater_than_zero() {
    setup();
    let driver = TestPvcamDriver::new();
    assert_gt!(driver.device_count(), 0)
}
