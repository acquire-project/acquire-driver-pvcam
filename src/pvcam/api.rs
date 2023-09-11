use std::{
    ffi::{c_void, CStr},
    sync::Arc,
};

use log::{info, warn};
use memoffset::offset_of;
use parking_lot::{Mutex, Once};

use crate::capi::{
    acquire::{Device, DeviceIdentifier, DeviceKind_DeviceKind_Camera},
    pvcam::{
        pl_cam_close, pl_cam_get_name, pl_cam_get_total, pl_cam_open, pl_get_param,
        pl_pvcam_get_ver, pl_pvcam_init, pl_pvcam_uninit, CAM_NAME_LEN, PARAM_EXPOSURE_TIME,
        PARAM_EXP_RES, PL_EXP_RES_MODES_EXP_RES_ONE_MICROSEC,
        PL_EXP_RES_MODES_EXP_RES_ONE_MILLISEC, PL_EXP_RES_MODES_EXP_RES_ONE_SEC,
        PL_OPEN_MODES_OPEN_EXCLUSIVE, PL_PARAM_ATTRIBUTES_ATTR_AVAIL,
        PL_PARAM_ATTRIBUTES_ATTR_CURRENT,
    },
};

use super::{camera::PvcamCamera, error::PvcamError, Version};

static mut API: Option<Arc<Mutex<PvcamApiInner>>> = None;
static INIT: Once = Once::new();

pub(crate) fn api() -> Arc<Mutex<PvcamApiInner>> {
    INIT.call_once(|| unsafe {
        API = Some(Arc::new(Mutex::new(PvcamApiInner::new().unwrap())));
    });
    unsafe { API.clone().unwrap() }
}

#[derive(Clone)]
pub(super) struct PvcamApi(Arc<Mutex<PvcamApiInner>>);

impl PvcamApi {
    fn new() -> Result<Self, PvcamError> {
        Ok(Self(Arc::new(Mutex::new(PvcamApiInner::new()?))))
    }
}

pub(super) struct PvcamApiInner;

impl PvcamApiInner {
    fn new() -> Result<Self, PvcamError> {
        let out = PvcamError::from_bool_racy(unsafe { pl_pvcam_init() }).into_result(Self)?;
        info!("PVCAM v{} - initialized", out.version()?);
        Ok(out)
    }

    pub(crate) fn version(&self) -> Result<Version, PvcamError> {
        let mut ver = Version::default();
        PvcamError::from_bool_racy(unsafe { pl_pvcam_get_ver(&mut ver.inner) }).into_result(ver)
    }

    pub(crate) fn device_count(&self) -> Result<usize, PvcamError> {
        info!("HERE in device_count");
        let mut n = 0;
        PvcamError::from_bool_racy(unsafe { pl_cam_get_total(&mut n) }).into_result(n as usize)
    }

    pub(crate) fn describe(&self, i_camera: i16) -> Result<DeviceIdentifier, PvcamError> {
        assert!(i_camera >= 0);
        let mut descriptor = DeviceIdentifier {
            driver_id: i_camera as _,
            device_id: 0, // will get filled in by runtime
            kind: DeviceKind_DeviceKind_Camera,
            name: [0; 256],
        };
        PvcamError::from_bool_racy(unsafe { pl_cam_get_name(i_camera, &mut descriptor.name[0]) })
            .into_result(descriptor)
    }

    pub(crate) fn open(&self, device_id: u64) -> Result<Box<PvcamCamera>, PvcamError> {
        // get name
        let mut camera_name = [0; CAM_NAME_LEN as _];
        PvcamError::from_bool_racy(unsafe { pl_cam_get_name(device_id as _, &mut camera_name[0]) })
            .into_result(())?;

        // call open
        let mut hcam = 0;
        PvcamError::from_bool_racy(unsafe {
            pl_cam_open(
                &mut camera_name[0],
                &mut hcam,
                PL_OPEN_MODES_OPEN_EXCLUSIVE as _,
            )
        })
        .into_result(())?;

        // FIXME: lifetime of device? should be bounded by driver? pedantic
        Ok(Box::new(PvcamCamera::new(hcam)))
    }

    /// Closes and deallocates `device`.
    pub(crate) fn close(&self, device: &mut Device) -> Result<(), PvcamError> {
        // Invoke pl_cam_close. If there's an error, log it and keep going.
        let mut camera = PvcamCamera::from_device_ptr_mut(device);
        PvcamError::from_bool_racy(unsafe { pl_cam_close(camera.hcam) })
            .into_result(())
            .unwrap_or_else(|e| warn!("{}", e));

        drop(unsafe { Box::from_raw(camera as *mut PvcamCamera) });
        Ok(())
    }

    pub(crate) fn camera_name(&self, i_camera: usize) -> Result<String, PvcamError> {
        let mut camera_name = [0; CAM_NAME_LEN as _];
        PvcamError::from_bool_racy(unsafe { pl_cam_get_name(i_camera as _, &mut camera_name[0]) })
            .into_result(())?;
        let camera_name = unsafe { CStr::from_ptr(&camera_name[0]) };
        Ok(camera_name.to_string_lossy().into())
    }

    pub(crate) fn get_exposure_time_us(&self, hcam: i16) -> Result<f32, PvcamError> {
        let mut v = 0u64;
        let exposure = PvcamError::from_bool_racy(unsafe {
            pl_get_param(
                hcam,
                PARAM_EXPOSURE_TIME,
                PL_PARAM_ATTRIBUTES_ATTR_CURRENT as _,
                &mut v as *mut _ as *mut c_void,
            )
        })
        .into_result(v)?;

        // get unit used for current exposure setting
        let resolution = {
            let is_available = PvcamError::from_bool_racy(unsafe {
                pl_get_param(
                    hcam,
                    PARAM_EXP_RES,
                    PL_PARAM_ATTRIBUTES_ATTR_AVAIL as _,
                    &mut v as *mut _ as *mut c_void,
                )
            })
            .into_result(v != 0)?;

            if !is_available {
                //For some older cameras this parameter might not be available
                //(ATTR_AVAIL returns FALSE). In this case camera uses
                //EXP_RES_ONE_MILLISEC resolution.
                PL_EXP_RES_MODES_EXP_RES_ONE_MILLISEC
            } else {
                PvcamError::from_bool_racy(unsafe {
                    pl_get_param(
                        hcam,
                        PARAM_EXP_RES,
                        PL_PARAM_ATTRIBUTES_ATTR_CURRENT as _,
                        &mut v as *mut _ as *mut c_void,
                    )
                })
                .into_result(v as i32)?
            }
        };

        // translate to microseconds
        let scale = match resolution {
            PL_EXP_RES_MODES_EXP_RES_ONE_SEC => 1e6,
            PL_EXP_RES_MODES_EXP_RES_ONE_MILLISEC => 1e3,
            PL_EXP_RES_MODES_EXP_RES_ONE_MICROSEC => 1.0,
            _ => todo!(),
        };

        Ok(exposure as f32 * scale)
    }
}

impl Drop for PvcamApiInner {
    fn drop(&mut self) {
        PvcamError::from_bool_racy(unsafe { pl_pvcam_uninit() })
            .into_result(())
            .unwrap_or_else(|e| {
                warn!("pl_pvcam_uninit() failed {}", e);
            });
    }
}
