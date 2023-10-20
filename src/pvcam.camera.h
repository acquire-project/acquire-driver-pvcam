#ifndef H_ACQUIRE_DRIVER_PVCAM_PVCAM_CAMERA_V0
#define H_ACQUIRE_DRIVER_PVCAM_PVCAM_CAMERA_V0

#include "device/props/camera.h"
#include "device/kit/camera.h"
#include "device/kit/driver.h"
#include "platform.h"

#include "pvcam.prelude.h"

#ifdef __cplusplus
extern "C"
{
#endif

struct PVCamCamera {
    struct Camera camera;
};

struct PVCamDriver {
    struct Driver driver;
};

#ifdef __cplusplus
};
#endif

#endif // H_ACQUIRE_DRIVER_PVCAM_PVCAM_CAMERA_V0