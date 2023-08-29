use super::capi::pvcam::{
    pl_error_code, pl_error_message, pl_pvcam_init, pl_pvcam_uninit, ERROR_MSG_LEN, PV_OK,
};
use crate::capi::pvcam::rs_bool;
use log::error;
use parking_lot::Mutex;
use std::ffi::CStr;
use std::fmt::{Debug, Display, Formatter};
use thiserror::__private::DisplayAsDisplay;

struct ApiError(i16);

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

    fn into_result<T>(self, t: T) -> Result<T, Self> {
        if self.0 == PV_OK as i16 {
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

struct PvcamApiInner;

struct PvcamApi(Mutex<PvcamApiInner>);

impl PvcamApiInner {
    fn new() -> Result<Self, ApiError> {
        ApiError::from_bool_racy(unsafe { pl_pvcam_init() }).into_result(Self)
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

impl PvcamApi {
    fn new() -> Self {
        Self(Mutex::new(PvcamApiInner::new()))
    }
}

pub(crate) fn maybe_log_last_pvcam_error() {
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
