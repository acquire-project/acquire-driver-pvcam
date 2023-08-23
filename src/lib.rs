use crate::capi::acquire::{
    DeviceIdentifier, DeviceStatusCode, DeviceStatusCode_Device_Err, DeviceStatusCode_Device_Ok,
    Driver,
};
use log::{error, warn};
use std::ffi::{c_char, c_int};
use std::mem::MaybeUninit;

mod capi;
mod logger;

#[repr(C)]
struct PVCamDriver {
    driver: Driver,
    logger: logger::AcquireLogger,
}

impl PVCamDriver {
    fn new(reporter: logger::ReporterCallback) -> Self {
        Self {
            driver: Driver {
                device_count: None, //Some(|driver| -> u32 { todo!() }),
                describe: None,     //Some(|driver, identifier, i| -> DeviceStatusCode { todo!() }),
                open: None, //Some(|driver, device_id, out| -> DeviceStatusCode { todo!() }),
                close: None, //Some(|driver, device| -> DeviceStatusCode { todo!() }),

                shutdown: Some(aq_pvcam_shutdown),
            },
            logger: logger::AcquireLogger::new(reporter),
        }
    }

    /// `driver` *must* point to the `PVCamDriver.driver` field.
    unsafe fn from_driver(driver: *const Driver) -> Option<&'static Self> {
        const OFFSET: isize = {
            let d = std::mem::MaybeUninit::uninit();
            let d_ptr: *const PVCamDriver = d.as_ptr();
            let d_u8_ptr = d_ptr as *const u8;
            unsafe {
                let x_u8_ptr = std::ptr::addr_of!((*d_ptr).driver) as *const u8;
                x_u8_ptr.offset_from(d_u8_ptr)
            }
        };
        if driver.is_null() {
            error!("Expected non-NULL Driver pointer");
            None
        } else {
            ((driver as *const u8).offset(-OFFSET) as *const PVCamDriver).as_ref()
        }
    }

    /// `driver` *must* point to the `PVCamDriver.driver` field.
    unsafe fn from_driver_mut(driver: *mut Driver) -> Option<&'static mut Self> {
        if let Some(ptr) = Self::from_driver(driver) {
            (ptr as *const PVCamDriver as *mut PVCamDriver).as_mut()
        } else {
            None
        }
    }
}

extern "C" fn aq_pvcam_shutdown(driver: *mut Driver) -> DeviceStatusCode {
    if let Some(ctx) = unsafe { PVCamDriver::from_driver_mut(driver) } {
        unsafe { Box::from_raw(ctx) }; // take ownership to free the pointer
        DeviceStatusCode_Device_Ok
    } else {
        DeviceStatusCode_Device_Err
    }
}
#[no_mangle]
pub extern "C" fn acquire_driver_init_v0(reporter: logger::ReporterCallback) -> *mut Driver {
    let context = Box::new(PVCamDriver::new(reporter));

    // This follows the pattern used on the C side.
    // For this to work:
    // - `PVCAMDriver` must be `repr(C)`
    // - `shutdown` must correctly recover the `PVCAMDriver` pointer and deallocate it.
    // - similarly other `Driver` callbacks must correctly recover the `PVCAMDriver` pointer.
    let ptr = &context.driver as *const Driver as *mut Driver;
    Box::leak(context);
    ptr
}
