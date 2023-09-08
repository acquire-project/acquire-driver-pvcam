use std::ffi::{c_char, CStr};
use std::fmt::{Debug, Display, Formatter};
use std::ptr::{null, null_mut};
use std::sync::{Arc, Once};

use crate::capi::acquire::{
    Camera, CameraProperties, CameraPropertyMetadata, Device, DeviceIdentifier,
    DeviceKind_DeviceKind_Camera, DeviceState_DeviceState_AwaitingConfiguration, DeviceStatusCode,
    ImageInfo, ImageShape,
};
use log::{error, info};
use memoffset::offset_of;
use parking_lot::Mutex;
use static_assertions::const_assert;

use crate::capi::pvcam::{
    pl_cam_close, pl_cam_get_name, pl_cam_get_total, pl_cam_open, pl_pvcam_get_ver, rs_bool,
    CAM_NAME_LEN, PL_OPEN_MODES_OPEN_EXCLUSIVE,
};

use super::capi::pvcam::{
    pl_error_code, pl_error_message, pl_pvcam_init, pl_pvcam_uninit, ERROR_MSG_LEN, PV_OK,
};

static mut API: Option<Arc<Mutex<PvcamApiInner>>> = None;
static INIT: Once = Once::new();

pub(crate) fn api() -> Arc<Mutex<PvcamApiInner>> {
    INIT.call_once(|| unsafe {
        API = Some(Arc::new(Mutex::new(PvcamApiInner::new().unwrap())));
    });
    unsafe { API.clone().unwrap() }
}

pub(crate) struct ApiError(i16);

impl ApiError {
    /// Return `None` when `is_ok` is true, otherwise use `pl_error_code()` to retrieve the last
    /// error code when `is_ok` is false.  
    ///
    /// There's a possible race condition here. The last error code might get replaced
    /// by a failure from another call.
    fn from_bool_racy(is_ok: rs_bool) -> Self {
        Self(if is_ok == PV_OK as u16 {
            0
        } else {
            unsafe { pl_error_code() }
        })
    }

    fn check_last() -> Self {
        Self(unsafe { pl_error_code() })
    }

    fn is_ok(&self) -> bool {
        self.0 == 0
    }

    fn into_result<T>(self, t: T) -> Result<T, Self> {
        if self.is_ok() {
            Ok(t)
        } else {
            Err(self)
        }
    }
}

impl Debug for ApiError {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "PVCAM ({}): {}", self.0, self)
    }
}

impl Display for ApiError {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        let mut buf = [0i8; ERROR_MSG_LEN as usize];
        unsafe { pl_error_message(self.0, &mut buf[0]) };
        let msg = unsafe { CStr::from_ptr(&buf[0]) };
        write!(f, "{}", msg.to_string_lossy())
    }
}

impl std::error::Error for ApiError {}

#[derive(Default)]
struct Version {
    /// see pl_pvcam_get_ver() for version encoding
    inner: u16,
}

impl Display for Version {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        let (major, minor, rev) = (self.inner >> 8, (self.inner & 0xff) >> 4, self.inner & 0xf);
        write!(f, "{major}.{minor}.{rev}")
    }
}

pub(crate) struct PvcamApiInner;

impl PvcamApiInner {
    fn new() -> Result<Self, ApiError> {
        let out = ApiError::from_bool_racy(unsafe { pl_pvcam_init() }).into_result(Self)?;
        info!("PVCAM v{} - initialized", out.version()?);
        Ok(out)
    }

    fn version(&self) -> Result<Version, ApiError> {
        let mut ver = Version::default();
        ApiError::from_bool_racy(unsafe { pl_pvcam_get_ver(&mut ver.inner) }).into_result(ver)
    }

    pub(crate) fn device_count(&self) -> Result<usize, ApiError> {
        info!("HERE in device_count");
        let mut n = 0;
        ApiError::from_bool_racy(unsafe { pl_cam_get_total(&mut n) }).into_result(n as usize)
    }

    pub fn describe(&self, i_camera: i16) -> Result<DeviceIdentifier, ApiError> {
        assert!(i_camera >= 0);
        let mut descriptor = DeviceIdentifier {
            driver_id: i_camera as _,
            device_id: 0, // will get filled in by runtime
            kind: DeviceKind_DeviceKind_Camera,
            name: [0; 256],
        };
        ApiError::from_bool_racy(unsafe { pl_cam_get_name(i_camera, &mut descriptor.name[0]) })
            .into_result(descriptor)
    }

    pub fn open(&self, device_id: u64) -> Result<Box<PvcamCamera>, ApiError> {
        // FIXME: lifetime of device? should be bounded by driver? pedantic
        Ok(Box::new(PvcamCamera::new(device_id as _)?))

        // // Need to check for errors even after a successful open
        // ApiError::check_last().into_result(())?;
        // let camera = Box::leak(camera);
        // dbg!(camera as *const PvcamCamera);
        // Ok(&camera.api.device as *const Device as *mut _)
    }

    pub fn close(&self, device: &mut Device) -> Result<(), ApiError> {
        let o = offset_of!(PvcamCamera, api) + offset_of!(Camera, device);
        let ptr = unsafe { (device as *mut Device as *mut u8).offset(-(o as isize)) };
        dbg!(ptr as *const PvcamCamera);
        drop(unsafe { Box::from_raw(ptr as *mut PvcamCamera) });
        Ok(())
    }
}

impl Drop for PvcamApiInner {
    fn drop(&mut self) {
        ApiError::from_bool_racy(unsafe { pl_pvcam_uninit() })
            .into_result(())
            .unwrap_or_else(|e| {
                error!("pl_pvcam_uninit() failed {}", e);
            });
    }
}

extern "C" fn aq_pvcam_set(
    camera: *mut Camera,
    settings: *mut CameraProperties,
) -> DeviceStatusCode {
    let camera = PvcamCamera::from_device_ptr_mut(camera);
    // exposure time

    todo!()
}

extern "C" fn aq_pvcam_get(
    camera: *const Camera,
    settings: *mut CameraProperties,
) -> DeviceStatusCode {
    todo!()
}

extern "C" fn aq_pvcam_get_meta(
    camera: *const Camera,
    meta: *mut CameraPropertyMetadata,
) -> DeviceStatusCode {
    todo!()
}

extern "C" fn aq_pvcam_get_shape(
    camera: *const Camera,
    shape: *mut ImageShape,
) -> DeviceStatusCode {
    todo!()
}

extern "C" fn aq_pvcam_start(camera: *mut Camera) -> DeviceStatusCode {
    todo!()
}

extern "C" fn aq_pvcam_stop(camera: *mut Camera) -> DeviceStatusCode {
    todo!()
}

extern "C" fn aq_pvcam_execute_trigger(camera: *mut Camera) -> DeviceStatusCode {
    todo!()
}

extern "C" fn aq_pvcam_get_frame(
    camera: *mut Camera,
    im: *mut ::std::os::raw::c_void,
    nbytes: *mut usize,
    info: *mut ImageInfo,
) -> DeviceStatusCode {
    todo!()
}

#[repr(C)]
pub(crate) struct PvcamCamera {
    api: Camera,
    hcam: i16,
}

impl PvcamCamera {
    /// Opens a camera via the PVCAM Api and initializes the Acquire Camera
    /// interface.
    fn new(device_id: i16) -> Result<Self, ApiError> {
        let api = Camera {
            // This gets filled in by the acquire runtime
            device: Device {
                identifier: DeviceIdentifier {
                    driver_id: 0,
                    device_id: 0,
                    kind: DeviceKind_DeviceKind_Camera,
                    name: [0; 256],
                },
                driver: null_mut(),
            },
            state: DeviceState_DeviceState_AwaitingConfiguration,
            set: Some(aq_pvcam_set),
            get: Some(aq_pvcam_get),
            get_meta: Some(aq_pvcam_get_meta),
            get_shape: Some(aq_pvcam_get_shape),
            start: Some(aq_pvcam_start),
            stop: Some(aq_pvcam_stop),
            execute_trigger: Some(aq_pvcam_execute_trigger),
            get_frame: Some(aq_pvcam_get_frame),
        };

        // get name
        let mut camera_name = [0; CAM_NAME_LEN as _];
        ApiError::from_bool_racy(unsafe { pl_cam_get_name(device_id as _, &mut camera_name[0]) })
            .into_result(())?;

        // call open
        let mut hcam = 0;
        ApiError::from_bool_racy(unsafe {
            pl_cam_open(
                &mut camera_name[0],
                &mut hcam,
                PL_OPEN_MODES_OPEN_EXCLUSIVE as _,
            )
        })
        .into_result(Self { api, hcam })
    }

    fn from_camera_ptr(device: *const Device) -> &'static Self {
        assert!(!device.is_null());
        let o = offset_of!(PvcamCamera, api);
        let ptr = unsafe { (device as *const Device as *const u8).offset(-(o as isize)) };
        unsafe { (ptr as *const PvcamCamera).as_ref().unwrap() }
    }

    fn from_camera_ptr_mut(device: *mut Device) -> &'static mut Self {
        assert!(!device.is_null());
        let o = offset_of!(PvcamCamera, api);
        let ptr = unsafe { (device as *mut Device as *mut u8).offset(-(o as isize)) };
        unsafe { (ptr as *mut PvcamCamera).as_mut().unwrap() }
    }

    pub(crate) fn as_device_ptr_mut(&mut self) -> *mut Device {
        dbg!(self as *const PvcamCamera);
        &self.api.device as *const Device as *mut _
    }
}

impl Drop for PvcamCamera {
    fn drop(&mut self) {
        ApiError::from_bool_racy(unsafe { pl_cam_close(self.hcam) })
            .into_result(())
            .unwrap_or_else(|e| error!("{}", e));
    }
}

#[derive(Clone)]
pub(crate) struct PvcamApi(Arc<Mutex<PvcamApiInner>>);

impl PvcamApi {
    fn new() -> Result<Self, ApiError> {
        Ok(Self(Arc::new(Mutex::new(PvcamApiInner::new()?))))
    }
}

#[cfg(test)]
mod test {
    use std::ffi::CStr;

    use log::{debug, error};

    use crate::capi::pvcam::{
        pl_error_code, pl_error_message, pl_pvcam_init, pl_pvcam_uninit, rs_bool, ERROR_MSG_LEN,
        PV_OK,
    };
    use crate::pvcam::PvcamApi;

    fn maybe_log_last_pvcam_error() {
        match unsafe { pl_error_code() } {
            x if x == PV_OK as i16 => { /* no op */ }
            ecode => {
                let mut buf = [0i8; ERROR_MSG_LEN as usize];
                unsafe { pl_error_message(ecode, &mut buf[0]) };
                let msg = unsafe { CStr::from_ptr(&buf[0]) };
                error!("PVCAM: {}", msg.to_string_lossy());
            }
        }
    }

    #[test]
    fn test_init_and_uninit_cycle() {
        // In PVCAM, init should be followed by a shutdown. Similarly, shutdown
        // must be preceded by an init. This test establishes the correct calling
        // order works.
        if let Err(e) = std::panic::catch_unwind(|| {
            debug!("First init and uninit");
            assert_eq!(unsafe { pl_pvcam_init() }, PV_OK as rs_bool);
            assert_eq!(unsafe { pl_pvcam_uninit() }, PV_OK as rs_bool);
            debug!("Second init and uninit");
            assert_eq!(unsafe { pl_pvcam_init() }, PV_OK as rs_bool);
            assert_eq!(unsafe { pl_pvcam_uninit() }, PV_OK as rs_bool);
        }) {
            maybe_log_last_pvcam_error();
            std::panic::resume_unwind(e)
        }
    }

    #[test]
    fn test_managed_api_access() {
        let _api = PvcamApi::new().unwrap();
    }
}
