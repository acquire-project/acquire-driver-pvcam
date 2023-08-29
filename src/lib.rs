use std::ffi::CStr;

use crate::capi::acquire::{
    DeviceStatusCode, DeviceStatusCode_Device_Err, DeviceStatusCode_Device_Ok, Driver,
};
use crate::logger::AcquireLogger;

use log::{error, info, warn, LevelFilter};

pub(crate) mod capi;
mod logger;
mod pvcam;

#[repr(C)]
struct PVCamDriver {
    driver: Driver,
    tag: u64,
}

impl PVCamDriver {
    fn new(reporter: logger::ReporterCallback) -> Self {
        log::set_boxed_logger(Box::new(AcquireLogger::new(reporter)))
            .map(|()| log::set_max_level(LevelFilter::Trace))
            .or_else(|_| -> Result<(), ()> {
                warn!("Logger already set. Ignoring.");
                Ok(())
            })
            .unwrap();

        Self {
            driver: Driver {
                device_count: Some(device_count), //Some(|driver| -> u32 { todo!() }),
                describe: None, //Some(|driver, identifier, i| -> DeviceStatusCode { todo!() }),
                open: None,     //Some(|driver, device_id, out| -> DeviceStatusCode { todo!() }),
                close: None,    //Some(|driver, device| -> DeviceStatusCode { todo!() }),

                shutdown: Some(shutdown),
            },
            tag: 81680085,
        }
    }

    fn device_count(&self) -> u32 {
        todo!()
    }

    /// Returns None if `driver` is null.
    /// Safety: `driver` *must* point to the `PVCamDriver.driver` field.
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

    /// Returns None if `driver` is null.
    /// Safety: `driver` *must* point to the `PVCamDriver.driver` field.
    unsafe fn from_driver_mut(driver: *mut Driver) -> Option<&'static mut Self> {
        if let Some(ptr) = Self::from_driver(driver) {
            (ptr as *const PVCamDriver as *mut PVCamDriver).as_mut()
        } else {
            None
        }
    }
}

extern "C" fn device_count(driver: *mut Driver) -> u32 {
    match std::panic::catch_unwind(|| {
        info!("HERE in device_count");
        let ctx = unsafe { PVCamDriver::from_driver_mut(driver) }.unwrap();
        ctx.device_count()
    }) {
        Ok(out) => out,
        Err(_) => {
            maybe_log_last_pvcam_error();
            error!("🔥Panic🔥");
            0
        }
    }
}

extern "C" fn shutdown(driver: *mut Driver) -> DeviceStatusCode {
    match std::panic::catch_unwind(|| {
        info!("HERE in shutdown");
        let ctx = unsafe { PVCamDriver::from_driver_mut(driver) }.unwrap();
        drop(unsafe { Box::from_raw(ctx) });
    }) {
        Ok(()) => DeviceStatusCode_Device_Ok,
        Err(_) => {
            maybe_log_last_pvcam_error();
            error!("🔥Panic🔥");
            DeviceStatusCode_Device_Err
        }
    }
}

#[no_mangle]
pub extern "C" fn acquire_driver_init_v0(reporter: logger::ReporterCallback) -> *mut Driver {
    match std::panic::catch_unwind(|| {
        let context = Box::leak(Box::new(PVCamDriver::new(reporter)));
        info!("HERE in init 😅");
        // This follows the pattern used elsewhere for the C driver adapters.
        //
        // For this to work:
        // - `PVCAMDriver` must be `repr(C)`
        // - `shutdown` must correctly recover the `PVCAMDriver` pointer and deallocate it.
        // - similarly other `Driver` callbacks must correctly recover the `PVCAMDriver` pointer.
        &context.driver as *const Driver as *mut Driver
    }) {
        Ok(ptr) => ptr,
        Err(_) => {
            maybe_log_last_pvcam_error();
            error!("🔥Panic🔥");
            std::ptr::null_mut() as _
        }
    }
}

#[cfg(test)]
mod test {
    use crate::capi::pvcam::{pl_pvcam_init, pl_pvcam_uninit, PV_OK};
    use crate::maybe_log_last_pvcam_error;
    use log::debug;

    #[test]
    fn test_init_and_uninit_cycle() {
        // In PVCAM, init should be followed by a shutdown. Similarly, shutdown
        // must be preceded by an init. This test establishes the correct calling
        // order works.
        if let Err(e) = std::panic::catch_unwind(|| {
            debug!("First init and uninit");
            assert_eq!(unsafe { pl_pvcam_init() } as i32, PV_OK);
            assert_eq!(unsafe { pl_pvcam_uninit() } as i32, PV_OK);
            debug!("Second init and uninit");
            assert_eq!(unsafe { pl_pvcam_init() } as i32, PV_OK);
            assert_eq!(unsafe { pl_pvcam_uninit() } as i32, PV_OK);
        }) {
            maybe_log_last_pvcam_error();
            std::panic::resume_unwind(e)
        }
    }
}
