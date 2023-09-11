use std::ptr::{null_mut, NonNull};

use memoffset::offset_of;

use crate::capi::{
    acquire::{
        Camera, CameraProperties, CameraProperties_camera_properties_input_triggers_s,
        CameraProperties_camera_properties_offset_s,
        CameraProperties_camera_properties_output_triggers_s,
        CameraProperties_camera_properties_shape_s, Device, DeviceIdentifier,
        DeviceKind_DeviceKind_Camera, DeviceState_DeviceState_AwaitingConfiguration, ImageShape,
        SignalIOKind_Signal_Input, Trigger, TriggerEdge_TriggerEdge_Rising,
    },
    pvcam::CAM_NAME_LEN,
};

use super::{api, error::PvcamError};

#[repr(C)]
pub(super) struct PvcamCamera {
    acquire_camera: Camera,
    pub(super) hcam: i16,
}

impl PvcamCamera {
    /// Opens a camera via the PVCAM Api and initializes the Acquire Camera
    /// interface.
    pub(super) fn new(hcam: i16) -> Self {
        let acquire_camera = Camera {
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
            set: Some(acquire_camera_api::aq_pvcam_set),
            get: Some(acquire_camera_api::aq_pvcam_get),
            get_meta: Some(acquire_camera_api::aq_pvcam_get_meta),
            get_shape: Some(acquire_camera_api::aq_pvcam_get_shape),
            start: Some(acquire_camera_api::aq_pvcam_start),
            stop: Some(acquire_camera_api::aq_pvcam_stop),
            execute_trigger: Some(acquire_camera_api::aq_pvcam_execute_trigger),
            get_frame: Some(acquire_camera_api::aq_pvcam_get_frame),
        };

        Self {
            acquire_camera,
            hcam,
        }
    }

    pub(super) fn from_device_ptr(device: *const Device) -> &'static Self {
        Self::from_device_ptr_mut(device as *mut _)
    }

    pub(super) fn from_device_ptr_mut(device: *mut Device) -> &'static mut Self {
        assert!(!device.is_null());
        assert_eq!(
            unsafe { *device }.identifier.kind,
            DeviceKind_DeviceKind_Camera
        );
        let o = offset_of!(PvcamCamera, acquire_camera) + offset_of!(Camera, device);
        let mut ptr = unsafe {
            NonNull::new_unchecked(
                (device as *mut Device as *mut u8).offset(-(o as isize)) as *mut PvcamCamera
            )
        };
        unsafe { ptr.as_mut() }
    }

    pub(super) fn from_camera_ptr(camera: *const Camera) -> &'static Self {
        Self::from_camera_ptr_mut(camera as *mut _)
    }

    pub(super) fn from_camera_ptr_mut(camera: *mut Camera) -> &'static mut Self {
        assert!(!camera.is_null());
        let o = offset_of!(PvcamCamera, acquire_camera);
        let ptr = unsafe { (camera as *mut Device as *mut u8).offset(-(o as isize)) };
        unsafe { (ptr as *mut PvcamCamera).as_mut().unwrap() }
    }

    pub(crate) fn as_device_ptr_mut(&mut self) -> *mut Device {
        dbg!(self as *const PvcamCamera);
        &self.acquire_camera.device as *const Device as *mut _
    }

    pub fn get_config(&self) -> Result<CameraProperties, PvcamError> {
        let mut props = CameraProperties::default();

        let pvcam = api();
        let pvcam = pvcam.lock();

        let exposure_time_us = pvcam.get_exposure_time_us(self.hcam)?;

        Ok(CameraProperties {
            exposure_time_us,
            line_interval_us: todo!(),
            readout_direction: todo!(),
            binning: Default::default(),
            pixel_type: Default::default(),
            offset: CameraProperties_camera_properties_offset_s { x: 0, y: 0 },
            shape: CameraProperties_camera_properties_shape_s { x: 0, y: 0 },
            input_triggers: CameraProperties_camera_properties_input_triggers_s {
                acquisition_start: Trigger::default(),
                frame_start: Trigger::default(),
                exposure: Trigger::default(),
            },
            output_triggers: CameraProperties_camera_properties_output_triggers_s {
                exposure: Trigger::default(),
                frame_start: Trigger::default(),
                trigger_wait: Trigger::default(),
            },
        })
    }
}

impl Default for Trigger {
    fn default() -> Self {
        Self {
            enable: Default::default(),
            line: Default::default(),
            kind: Default::default(),
            edge: Default::default(),
        }
    }
}

impl Default for CameraProperties {
    fn default() -> Self {
        Self {
            exposure_time_us: Default::default(),
            line_interval_us: Default::default(),
            readout_direction: Default::default(),
            binning: Default::default(),
            pixel_type: Default::default(),
            offset: CameraProperties_camera_properties_offset_s { x: 0, y: 0 },
            shape: CameraProperties_camera_properties_shape_s { x: 0, y: 0 },
            input_triggers: CameraProperties_camera_properties_input_triggers_s {
                acquisition_start: Trigger::default(),
                frame_start: Trigger::default(),
                exposure: Trigger::default(),
            },
            output_triggers: CameraProperties_camera_properties_output_triggers_s {
                exposure: Trigger::default(),
                frame_start: Trigger::default(),
                trigger_wait: Trigger::default(),
            },
        }
    }
}

mod acquire_camera_api {
    use std::ptr::null_mut;

    use super::PvcamCamera;

    use crate::capi::acquire::{
        Camera, CameraProperties, CameraPropertyMetadata, DeviceStatusCode, ImageInfo, ImageShape,
    };

    pub(crate) extern "C" fn aq_pvcam_set(
        camera: *mut Camera,
        settings: *mut CameraProperties,
    ) -> DeviceStatusCode {
        let camera = PvcamCamera::from_camera_ptr_mut(camera);
        // exposure time

        todo!()
    }

    pub(crate) extern "C" fn aq_pvcam_get(
        camera: *const Camera,
        settings: *mut CameraProperties,
    ) -> DeviceStatusCode {
        todo!()
    }

    pub(crate) extern "C" fn aq_pvcam_get_meta(
        camera: *const Camera,
        meta: *mut CameraPropertyMetadata,
    ) -> DeviceStatusCode {
        todo!()
    }

    pub(crate) extern "C" fn aq_pvcam_get_shape(
        camera: *const Camera,
        shape: *mut ImageShape,
    ) -> DeviceStatusCode {
        todo!()
    }

    pub(crate) extern "C" fn aq_pvcam_start(camera: *mut Camera) -> DeviceStatusCode {
        todo!()
    }

    pub(crate) extern "C" fn aq_pvcam_stop(camera: *mut Camera) -> DeviceStatusCode {
        todo!()
    }

    pub(crate) extern "C" fn aq_pvcam_execute_trigger(camera: *mut Camera) -> DeviceStatusCode {
        todo!()
    }

    pub(crate) extern "C" fn aq_pvcam_get_frame(
        camera: *mut Camera,
        im: *mut ::std::os::raw::c_void,
        nbytes: *mut usize,
        info: *mut ImageInfo,
    ) -> DeviceStatusCode {
        todo!()
    }
}
