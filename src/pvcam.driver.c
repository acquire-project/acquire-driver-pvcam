/// @file Driver wrapping the PVCAM SDK.
///
/// This implements PVCamCamera and PVCamDriver structs in C++
/// since the Spinnaker SDK is C++. The methods of those classes are
/// then bound to the Camera and Driver C structs in the Acquire API.

#include "pvcam.camera.h"

#include "device/kit/driver.h"
#include "device/hal/camera.h"
#include "logger.h"

acquire_export
struct Driver *
acquire_driver_init_v0(acquire_reporter_t reporter) {

    Error:
    return NULL;
}