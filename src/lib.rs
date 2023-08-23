use std::ffi::{c_char, c_int};
use crate::capi::acquire::{DeviceIdentifier, DeviceStatusCode, Driver};

mod capi;

#[no_mangle]
pub extern "C" fn acquire_driver_init_v0(
    reporter: Option<
        unsafe extern "C" fn(
            is_error: c_int,
            file: *const c_char,
            line: c_int,
            function: *const c_char,
            msg: *const c_char,
        ),
    >,
) -> *mut Driver {
    // TODO: use the reporter for rust logging output
    let driver = Box::new(Driver {
        device_count: Some(|driver| -> u32 {
            todo!()
        }),
        describe: Some(|driver,
                        identifier,
                        i| -> DeviceStatusCode {
            todo!()
        }),
        open: Some(|driver,device_id,out|->DeviceStatusCode{
            todo!()
        }),
        close: Some(|driver, device|->DeviceStatusCode{
            todo!()
        }),
        shutdown: Some(|driver|->DeviceStatusCode {
            unsafe{Box::from_raw(driver)};
            todo!()
        }),
    });
    Box::leak(driver)
}
