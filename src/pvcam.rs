use std::ffi::{c_char, CStr};
use std::fmt::{Debug, Display, Formatter};
use std::sync::{Arc, Once};

use crate::capi::acquire::DeviceIdentifier;
use log::{error, info};
use parking_lot::Mutex;
use static_assertions::const_assert;

use crate::capi::pvcam::{
    pl_cam_get_name, pl_cam_get_total, pl_pvcam_get_ver, rs_bool, CAM_NAME_LEN,
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

    pub fn describe(
        &self,
        i_camera: i16,
        descriptor: &mut DeviceIdentifier,
    ) -> Result<(), ApiError> {
        let name = {
            const_assert!(CAM_NAME_LEN<std::mem::size_of())
            let mut buf = [0; CAM_NAME_LEN as usize];
            ApiError::from_bool_racy(unsafe { pl_cam_get_name(i_camera, &mut buf[0]) })
                .into_result(())?;
            unsafe { CStr::from_ptr(&buf[0]) }
        };
        todo!()
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
