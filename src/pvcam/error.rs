use std::{
    ffi::CStr,
    fmt::{Debug, Display, Formatter},
};

use crate::capi::pvcam::{pl_error_code, pl_error_message, rs_bool, ERROR_MSG_LEN, PV_OK};

pub(crate) struct PvcamError(i16);

impl PvcamError {
    /// Return `None` when `is_ok` is true, otherwise use `pl_error_code()` to retrieve the last
    /// error code when `is_ok` is false.  
    ///
    /// There's a possible race condition here. The last error code might get replaced
    /// by a failure from another call.
    pub(super) fn from_bool_racy(is_ok: rs_bool) -> Self {
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

    pub(super) fn into_result<T>(self, t: T) -> Result<T, Self> {
        if self.is_ok() {
            Ok(t)
        } else {
            Err(self)
        }
    }
}

impl Debug for PvcamError {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "PVCAM ({}): {}", self.0, self)
    }
}

impl Display for PvcamError {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        let mut buf = [0i8; ERROR_MSG_LEN as usize];
        unsafe { pl_error_message(self.0, &mut buf[0]) };
        let msg = unsafe { CStr::from_ptr(&buf[0]) };
        write!(f, "{}", msg.to_string_lossy())
    }
}

impl std::error::Error for PvcamError {}
