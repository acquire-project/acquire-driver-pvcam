use capi::acquire::{Device, DeviceIdentifier};
use log::{debug, error, info, trace, warn, LevelFilter};

use memoffset::offset_of;

use crate::capi::acquire::{
    DeviceStatusCode, DeviceStatusCode_Device_Err, DeviceStatusCode_Device_Ok, Driver,
};
use crate::logger::AcquireLogger;

pub(crate) mod capi;
mod logger;
mod pvcam;

#[repr(C, align(8))]
struct PVCamDriver {
    driver: Driver,
}

impl PVCamDriver {
    fn new() -> Self {
        Self {
            driver: Driver {
                device_count: Some(aq_pvcam_device_count),
                describe: Some(aq_pvcam_describe),
                open: Some(aq_pvcam_open),
                close: Some(aq_pvcam_close),
                shutdown: Some(aq_pvcam_shutdown),
            },
        }
    }

    /// Returns None if `driver` is null.
    /// Safety: `driver` *must* point to the `PVCamDriver.driver` field.
    unsafe fn from_driver(driver: *const Driver) -> Option<&'static Self> {
        const OFFSET: isize = offset_of!(PVCamDriver, driver) as _;
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

extern "C" fn aq_pvcam_device_count(_driver: *mut Driver) -> u32 {
    match std::panic::catch_unwind(|| pvcam::api().lock().device_count().unwrap()) {
        Ok(out) => out as u32,
        Err(_) => {
            error!("🔥Panic🔥");
            0
        }
    }
}

extern "C" fn aq_pvcam_describe(
    _driver: *const Driver,
    identifier: *mut DeviceIdentifier,
    i_camera: u64,
) -> u32 {
    match std::panic::catch_unwind(|| {
        pvcam::api()
            .lock()
            .describe(i_camera as _, &mut unsafe { *identifier })
            .unwrap()
    }) {
        Ok(out) => DeviceStatusCode_Device_Ok,
        Err(_) => {
            error!("🔥Panic🔥");
            DeviceStatusCode_Device_Err
        }
    }
}

extern "C" fn aq_pvcam_open(
    _: *mut Driver,
    device_id: u64,
    device: *mut *mut Device,
) -> DeviceStatusCode {
    match std::panic::catch_unwind(|| {
        debug!("HERE in aq_pvcam_open");
        let ptr = pvcam::api().lock().open(device_id).unwrap();
        unsafe { *device = ptr };
    }) {
        Ok(()) => DeviceStatusCode_Device_Ok,
        Err(_) => {
            error!("🔥Panic🔥");
            DeviceStatusCode_Device_Err
        }
    }
}

extern "C" fn aq_pvcam_close(_: *mut Driver, device: *mut Device) -> DeviceStatusCode {
    match std::panic::catch_unwind(|| {
        debug!("HERE in aq_pvcam_close");
        unimplemented!()
    }) {
        Ok(()) => DeviceStatusCode_Device_Ok,
        Err(_) => {
            error!("🔥Panic🔥");
            DeviceStatusCode_Device_Err
        }
    }
}

extern "C" fn aq_pvcam_shutdown(driver: *mut Driver) -> DeviceStatusCode {
    match std::panic::catch_unwind(|| {
        debug!("HERE in shutdown");
        let ctx = unsafe { PVCamDriver::from_driver_mut(driver) }.unwrap();
        drop(unsafe { Box::from_raw(ctx) });
    }) {
        Ok(()) => DeviceStatusCode_Device_Ok,
        Err(_) => {
            error!("🔥Panic🔥");
            DeviceStatusCode_Device_Err
        }
    }
}

#[no_mangle]
pub extern "C" fn acquire_driver_init_v0(reporter: logger::ReporterCallback) -> *mut Driver {
    match std::panic::catch_unwind(|| {
        // 1. Setup logger
        log::set_boxed_logger(Box::new(AcquireLogger::new(reporter)))
            .map(|()| log::set_max_level(LevelFilter::Trace))
            .or_else(|_| -> Result<(), ()> {
                warn!("Logger already set. Ignoring.");
                Ok(())
            })
            .unwrap();
        debug!("HERE in init 😅");

        // 2. Allocate context and leak the pointer. Callee will manage.
        // This follows the pattern used elsewhere for the C driver adapters.
        //
        // For this to work:
        // - `PVCAMDriver` must be `repr(C)`
        // - `shutdown` must correctly recover the `PVCAMDriver` pointer and deallocate it.
        // - similarly other `Driver` callbacks must correctly recover the `PVCAMDriver` pointer.
        let context = Box::leak(Box::new(PVCamDriver::new()));
        assert_eq!(
            context as *const _ as *const u8,
            &context.driver as *const _ as *const u8
        );
        let ptr = &context.driver as *const Driver as *mut Driver;
        debug!("returning ptr {:?} with shutdown {:?}", ptr, unsafe {
            (*ptr).shutdown
        });
        // debug!("\tdevice_count {:?}", unsafe { (*ptr).device_count });
        // debug!("\t*** call device_count: {}", unsafe {
        //     (*ptr).device_count.unwrap()(ptr)
        // });
        ptr
    }) {
        Ok(ptr) => ptr,
        Err(_) => {
            error!("🔥Panic🔥");
            std::ptr::null_mut() as _
        }
    }
}
