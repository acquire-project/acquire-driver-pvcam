#include "device/kit/driver.h"
#include "device/hal/camera.h"
#include "logger.h"

#include <map>
#include <memory>
#include <mutex>
#include <unordered_map>

#include "master.h" // must come before pvcam.h
#include "pvcam.h"

#define L (aq_logger)
#define LOG(...) L(0, __FILE__, __LINE__, __FUNCTION__, __VA_ARGS__)
#define LOGE(...) L(1, __FILE__, __LINE__, __FUNCTION__, __VA_ARGS__)

// #define TRACE(...) LOG(__VA_ARGS__)
#define TRACE(...)

#define EXPECT(e, ...)                                                         \
    do {                                                                       \
        if (!(e)) {                                                            \
            LOGE(__VA_ARGS__);                                                 \
            throw std::runtime_error("Expression was false: " #e);             \
        }                                                                      \
    } while (0)
#define CHECK(e) EXPECT(e, "Expression was false:\n\t%s\n", #e)

#define PVCAM_INNER(e, mtx, logger, action)                                    \
    do {                                                                       \
        std::scoped_lock<std::mutex> lock((mtx));                              \
        if (PV_FAIL == (e)) {                                                  \
            const int16 code = pl_error_code();                                \
            char msg[ERROR_MSG_LEN];                                           \
            pl_error_message(code, msg);                                       \
            logger(msg);                                                       \
            action;                                                            \
        }                                                                      \
    } while (0)

#define PVCAM(e, mtx)                                                          \
    PVCAM_INNER(                                                               \
      e, mtx, LOGE, throw std::runtime_error("PVCAM API call failed."))
#define PVWARN(e, mtx) PVCAM_INNER(e, mtx, LOG, )

namespace {
/// Utilities

// Maps Acquire SampleType to GenICam pixel format strings.
const std::unordered_map<SampleType, PL_IMAGE_FORMATS>
  sample_type_to_pixel_format{
      { SampleType_u8, PL_IMAGE_FORMAT_MONO8 },
      { SampleType_u16, PL_IMAGE_FORMAT_MONO16 },
  };

/**
 * @brief Checks that the given camera id is within the range of a signed
 * 16-bit.
 * @param id The camera id to check.
 * @throws std::runtime_error if the id is out of range.
 */
void
check_pvcam_camera_id(uint64_t id)
{
    constexpr uint64_t limit = std::numeric_limits<int16>::max();
    EXPECT(id <= limit, "Expected a value at most %llu. Got %llu.", limit, id);
}

/// Camera declaration

struct PVCamCamera final : public Camera
{
  public:
    PVCamCamera(int16 hcam, std::shared_ptr<std::mutex> api_mutex);
    ~PVCamCamera() noexcept;

    void set(CameraProperties* properties);
    void get(CameraProperties* properties);
    void get_meta(CameraPropertyMetadata* meta) const;
    void get_shape(ImageShape* shape) const;
    void start();
    void stop();
    void execute_trigger() const;
    void get_frame(void* im, size_t* nbytes, struct ImageInfo* info);

  private:
    // Guards access to the PVCAM API.
    std::shared_ptr<std::mutex> pvcam_api_mutex_;

    // The PVCAM camera handle.
    int16 hcam;

    // Setting properties on the device may be expensive, so these are
    // used to avoid doing so when a value is unchanged.
    struct CameraProperties last_known_settings_;

    /// Property setters
    void maybe_set_exposure_time_us_(float exposure_time_us);
    void maybe_set_roi_(uint8_t binning,
                        CameraProperties::camera_properties_offset_s offset,
                        CameraProperties::camera_properties_shape_s shape);
    void maybe_set_pixel_type_(SampleType pixel_type);

    /// Property getters
    void maybe_get_exposure_time_(CameraProperties* properties) const;
    void maybe_get_binning_(CameraProperties* properties) const;
    void maybe_get_pixel_type_(CameraProperties* properties) const;

    /// Metadata getters
    void query_exposure_time_capabilities_(CameraPropertyMetadata* meta) const;
    void query_binning_capabilities(CameraPropertyMetadata* meta) const;

    /// Helpers
    bool is_param_available(uns32 param_id) const;
    bool is_param_writable(uns32 param_id) const;
    template<typename T>
    [[nodiscard]] bool query_param_min_max(uns32 param_id,
                                           T* min,
                                           T* max) const;
    float get_exposure_time_resolution_to_microseconds_() const;
};

/// Non-throwing camera implementation

DeviceStatusCode
pvcam_set(Camera* self, CameraProperties* settings)
{
    try {
        CHECK(self);
        ((PVCamCamera*)self)->set(settings);
        return Device_Ok;
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return Device_Err;
}

DeviceStatusCode
pvcam_get(const Camera* self_, CameraProperties* settings)
{
    try {
        CHECK(self_);
        ((PVCamCamera*)self_)->get(settings);
        return Device_Ok;
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return Device_Err;
}

DeviceStatusCode
pvcam_get_meta(const Camera* self_, CameraPropertyMetadata* meta)
{
    try {
        CHECK(self_);
        ((PVCamCamera*)self_)->get_meta(meta);
        return Device_Ok;
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return Device_Err;
}

DeviceStatusCode
pvcam_get_shape(const Camera* self_, ImageShape* shape)
{
    try {
        CHECK(self_);
        ((PVCamCamera*)self_)->get_shape(shape);
        return Device_Ok;
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return Device_Err;
}

DeviceStatusCode
pvcam_start(Camera* self_)
{
    try {
        CHECK(self_);
        ((PVCamCamera*)self_)->start();
        return Device_Ok;
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return Device_Err;
}

DeviceStatusCode
pvcam_stop(Camera* self_)
{
    try {
        CHECK(self_);
        ((PVCamCamera*)self_)->stop();
        return Device_Ok;
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return Device_Err;
}

DeviceStatusCode
pvcam_execute_trigger(Camera* self_)
{
    try {
        CHECK(self_);
        ((PVCamCamera*)self_)->execute_trigger();
        return Device_Ok;
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return Device_Err;
}

DeviceStatusCode
pvcam_get_frame(Camera* self_, void* im, size_t* nbytes, ImageInfo* info)
{
    try {
        CHECK(self_);
        ((PVCamCamera*)self_)->get_frame(im, nbytes, info);
        return Device_Ok;
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return Device_Err;
}

/// PVCamCamera implementation

PVCamCamera::PVCamCamera(int16 hcam, std::shared_ptr<std::mutex> api_mutex)
  : Camera{
      .set = ::pvcam_set,
      .get = ::pvcam_get,
      .get_meta = ::pvcam_get_meta,
      .get_shape = ::pvcam_get_shape,
      .start = ::pvcam_start,
      .stop = ::pvcam_stop,
      .execute_trigger = ::pvcam_execute_trigger,
      .get_frame = ::pvcam_get_frame,
  }
  , hcam{ hcam }
  , pvcam_api_mutex_{ api_mutex }
  , last_known_settings_{ 0 }
{
    get(&last_known_settings_);
}

PVCamCamera::~PVCamCamera() noexcept
{
    try {
        stop();
        PVCAM(pl_cam_close(hcam), *pvcam_api_mutex_);
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown).");
    }
}

void
PVCamCamera::set(CameraProperties* properties)
{
    CHECK(properties);
    std::scoped_lock<std::mutex> lock(*pvcam_api_mutex_);

    maybe_set_roi_(properties->binning, properties->offset, properties->shape);
    maybe_set_exposure_time_us_(properties->exposure_time_us);
    maybe_set_pixel_type_(properties->pixel_type);
}

void
PVCamCamera::get(CameraProperties* properties)
{
    CHECK(properties);
    std::scoped_lock<std::mutex> lock(*pvcam_api_mutex_);

    maybe_get_exposure_time_(properties);
    maybe_get_binning_(properties);
    maybe_get_pixel_type_(properties);
}

void
PVCamCamera::get_meta(CameraPropertyMetadata* meta) const
{
    CHECK(meta);
    std::scoped_lock<std::mutex> lock(*pvcam_api_mutex_);

    query_exposure_time_capabilities_(meta);
    query_binning_capabilities(meta);
}

void
PVCamCamera::get_shape(ImageShape* shape) const
{
}

void
PVCamCamera::start()
{
}

void
PVCamCamera::stop()
{
}

void
PVCamCamera::execute_trigger() const
{
}

void
PVCamCamera::get_frame(void* im, size_t* nbytes, struct ImageInfo* info)
{
}

void
PVCamCamera::maybe_set_exposure_time_us_(float exposure_time_us)
{
    // We assume here that the API mutex is already locked.
    if (exposure_time_us == last_known_settings_.exposure_time_us ||
        !is_param_writable(PARAM_EXPOSURE_TIME)) {
        return;
    }

    auto exposure_time =
      (ulong64)(exposure_time_us /
                get_exposure_time_resolution_to_microseconds_());

    if (PV_FAIL == pl_set_param(hcam, PARAM_EXPOSURE_TIME, &exposure_time)) {
        LOGE("Failed to set exposure time.");
    }
}

void
PVCamCamera::maybe_set_roi_(uint8_t binning_,
                            CameraProperties::camera_properties_offset_s offset,
                            CameraProperties::camera_properties_shape_s shape)
{
    // We assume here that the API mutex is already locked.
    if (binning_ != last_known_settings_.binning &&
        is_param_writable(PARAM_BINNING_PAR) &&
        is_param_writable(PARAM_BINNING_SER)) {
        auto binning = (int32)binning_;
        if (PV_FAIL == pl_set_param(hcam, PARAM_BINNING_PAR, &binning)) {
            LOGE("Failed to set parallel part of binning.");
        }
        if (PV_FAIL == pl_set_param(hcam, PARAM_BINNING_SER, &binning)) {
            LOGE("Failed to set serial part of binning.");
        }
    }

    //
    {
    }
}

void
PVCamCamera::maybe_set_pixel_type_(SampleType pixel_type)
{
    CHECK(pixel_type < SampleTypeCount);

    // We assume here that the API mutex is already locked.
    if (pixel_type == last_known_settings_.pixel_type ||
        !is_param_writable(PARAM_IMAGE_FORMAT_HOST)) {
        return;
    }

    if (!sample_type_to_pixel_format.contains(pixel_type)) {
        LOGE("Unsupported pixel type: %d", pixel_type);
        return;
    }

    auto pixel_format = sample_type_to_pixel_format.at(pixel_type);
    if (PV_FAIL == pl_set_param(hcam, PARAM_IMAGE_FORMAT_HOST, &pixel_format)) {
        LOGE("Failed to set pixel format.");
    }
}

void
PVCamCamera::maybe_get_exposure_time_(CameraProperties* properties) const
{
    // We assume here that the API mutex is already locked.
    properties->exposure_time_us = 0;

    if (!is_param_available(PARAM_EXPOSURE_TIME)) {
        return;
    }

    ulong64 exposure_time;
    if (PV_FAIL ==
        pl_get_param(hcam, PARAM_EXPOSURE_TIME, ATTR_CURRENT, &exposure_time)) {
        LOGE("Failed to get exposure time.");
        return;
    }

    // convert the resolution to microseconds
    properties->exposure_time_us =
      (float)exposure_time * get_exposure_time_resolution_to_microseconds_();
}

void
PVCamCamera::maybe_get_binning_(CameraProperties* properties) const
{
    // We assume here that the API mutex is already locked.
    properties->binning = 1;

    if (!is_param_available(PARAM_BINNING_PAR) ||
        !is_param_available(PARAM_BINNING_SER)) {
        return;
    }

    int32 binning;
    if (PV_FAIL ==
        pl_get_param(hcam, PARAM_BINNING_PAR, ATTR_CURRENT, &binning)) {
        LOGE("Failed to get binning.");
        return;
    }

    properties->binning = (uint8_t)binning;
}

void
PVCamCamera::maybe_get_pixel_type_(CameraProperties* properties) const
{
    // We assume here that the API mutex is already locked.
    properties->pixel_type = SampleType_u8;

    if (!is_param_available(PARAM_IMAGE_FORMAT_HOST)) {
        return;
    }

    int32 pixel_format;
    if (PV_FAIL ==
        pl_get_param(
          hcam, PARAM_IMAGE_FORMAT_HOST, ATTR_CURRENT, &pixel_format)) {
        LOGE("Failed to get pixel format.");
        return;
    }

    switch (pixel_format) {
        case PL_IMAGE_FORMAT_MONO8:
            properties->pixel_type = SampleType_u8;
            break;
        case PL_IMAGE_FORMAT_MONO16:
            properties->pixel_type = SampleType_u16;
            break;
        default:
            LOGE("Unsupported pixel format: %d", pixel_format);
            break;
    }
}

void
PVCamCamera::query_exposure_time_capabilities_(
  CameraPropertyMetadata* meta) const
{
    // We assume here that the API mutex is already locked.
    meta->exposure_time_us = { 0 };
    meta->exposure_time_us.type = PropertyType_FixedPrecision;

    if (!is_param_available(PARAM_EXPOSURE_TIME)) {
        return;
    }

    meta->exposure_time_us.writable =
      (int)is_param_writable(PARAM_EXPOSURE_TIME);

    ulong64 min, max;
    if (!query_param_min_max(PARAM_EXPOSURE_TIME, &min, &max)) {
        min = max = 0;
    }

    auto res = get_exposure_time_resolution_to_microseconds_();

    // convert the resolution to microseconds
    meta->exposure_time_us.low = (float)min * res;
    meta->exposure_time_us.high = (float)max * res;
}

void
PVCamCamera::query_binning_capabilities(CameraPropertyMetadata* meta) const
{
    // We assume here that the API mutex is already locked.
    meta->binning = { 0 };
    meta->binning.type = PropertyType_FixedPrecision;

    if (!is_param_available(PARAM_BINNING_PAR) ||
        !is_param_available(PARAM_BINNING_SER)) {
        meta->binning.low = meta->binning.high = 1.f;
        return;
    }

    meta->binning.writable = (int)(is_param_writable(PARAM_BINNING_PAR) &&
                                   is_param_writable(PARAM_BINNING_SER));

    int32 par_min, par_max;
    if (!query_param_min_max(PARAM_BINNING_PAR, &par_min, &par_max)) {
        par_min = par_max = 1;
    }

    int32 ser_min, ser_max;
    if (!query_param_min_max(PARAM_BINNING_SER, &ser_min, &ser_max)) {
        ser_min = ser_max = 1;
    }

    meta->binning.low = (float)(std::max(par_min, ser_min));
    meta->binning.high = (float)(std::min(par_max, ser_max));
}

bool
PVCamCamera::is_param_available(uns32 param_id) const
{
    // We assume here that the API mutex is already locked.
    rs_bool is_avail;
    if (PV_FAIL == pl_get_param(hcam, param_id, ATTR_AVAIL, &is_avail)) {
        return false;
    }

    return is_avail == TRUE;
}

bool
PVCamCamera::is_param_writable(uns32 param_id) const
{
    // We assume here that the API mutex is already locked.
    uns16 access;
    if (PV_OK != pl_get_param(hcam, param_id, ATTR_ACCESS, &access)) {
        LOGE("Failed to get access.");
        return false;
    }

    return access == ACC_READ_WRITE || access == ACC_WRITE_ONLY;
}

template<typename T>
bool
PVCamCamera::query_param_min_max(uns32 param_id, T* min, T* max) const
{
    // We assume here that the API mutex is already locked.
    *min = std::numeric_limits<T>::min();
    *max = std::numeric_limits<T>::max();

    if (!is_param_available(param_id)) {
        return false;
    }

    if (PV_FAIL == pl_get_param(hcam, param_id, ATTR_MIN, min)) {
        LOGE("Failed to get min.");
        return false;
    }

    if (PV_FAIL == pl_get_param(hcam, param_id, ATTR_MAX, max)) {
        LOGE("Failed to get max.");
        return false;
    }

    return true;
}

float
PVCamCamera::get_exposure_time_resolution_to_microseconds_() const
{
    // get the exposure time resolution
    int32 res;
    if (!is_param_available(PARAM_EXP_RES) ||
        PV_FAIL == pl_get_param(hcam, PARAM_EXP_RES, ATTR_CURRENT, &res)) {
        // For some older cameras this parameter might not be available
        // (ATTR_AVAIL returns FALSE). In this case camera uses
        // EXP_RES_ONE_MILLISEC resolution.
        res = EXP_RES_ONE_MILLISEC;
    }

    switch (res) {
        case EXP_RES_ONE_MILLISEC:
            return 1e3f;
            break;
        case EXP_RES_ONE_SEC:
            return 1e6f;
            break;
        case EXP_RES_ONE_MICROSEC:
        default:
            break;
    }

    return 1.f;
}

/// Driver declaration

struct PVCamDriver final : public Driver
{
  public:
    PVCamDriver();
    ~PVCamDriver() noexcept;

    uint32_t device_count();
    void describe(DeviceIdentifier* identifier, uint64_t id);
    void open(uint64_t device_id, Device** out);
    static void close(Device* in);
    void shutdown();

  private:
    // Guards access to the PVCAM API.
    std::shared_ptr<std::mutex> pvcam_api_mutex_;

    std::map<int16, PVCamCamera*> cameras_;
};

/// Non-throwing driver implementation

uint32_t
pvcam_device_count(Driver* self)
{
    try {
        CHECK(self);
        return ((PVCamDriver*)self)->device_count();
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return 0;
}

DeviceStatusCode
pvcam_describe(const Driver* self, DeviceIdentifier* identifier, uint64_t i)
{
    try {
        CHECK(self);
        CHECK(identifier);
        ((PVCamDriver*)self)->describe(identifier, i);
        return Device_Ok;
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return Device_Err;
}

DeviceStatusCode
pvcam_open(Driver* self, uint64_t device_id, Device** out)
{
    try {
        CHECK(self);
        CHECK(out);
        ((PVCamDriver*)self)->open(device_id, out);
        return Device_Ok;
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return Device_Err;
}

DeviceStatusCode
pvcam_close(Driver* self, Device* in)
{
    try {
        CHECK(self);
        CHECK(in);
        ((PVCamDriver*)self)->close(in);
        return Device_Ok;
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return Device_Err;
}

DeviceStatusCode
pvcam_shutdown(Driver* self)
{
    try {
        CHECK(self);
        ((PVCamDriver*)self)->shutdown();
        return Device_Ok;
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }

    return Device_Err;
}

/// PVCamDriver implementation

PVCamDriver::PVCamDriver()
  : Driver{
      .device_count = ::pvcam_device_count,
      .describe = ::pvcam_describe,
      .open = ::pvcam_open,
      .close = ::pvcam_close,
      .shutdown = ::pvcam_shutdown,
  }
  , pvcam_api_mutex_{ std::make_shared<std::mutex>() }
{
    PVCAM(pl_pvcam_init(), *pvcam_api_mutex_);
}

PVCamDriver::~PVCamDriver() noexcept
{
    try {
        shutdown();
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown).");
    }
}

uint32_t
PVCamDriver::device_count()
{
    uint32_t n_cameras = 0;
    PVCAM(pl_cam_get_total((int16*)&n_cameras), *pvcam_api_mutex_);
    return n_cameras;
}

void
PVCamDriver::describe(DeviceIdentifier* identifier, uint64_t id)
{
    CHECK(identifier);
    check_pvcam_camera_id(id);

    char model[CAM_NAME_LEN];
    char sn[MAX_ALPHA_SER_NUM_LEN];

    PVCAM(pl_cam_get_name(id, model), *pvcam_api_mutex_);

    *identifier = {
        .device_id = (uint8_t)id,
        .kind = DeviceKind_Camera,
    };

    int16 hcam;
    if (cameras_.contains((int16)id)) {
        hcam = (int16)id;
    } else {
        PVCAM(pl_cam_open(model, &hcam, OPEN_EXCLUSIVE), *pvcam_api_mutex_);
    }

    PVCAM(pl_get_param(hcam, PARAM_HEAD_SER_NUM_ALPHA, ATTR_CURRENT, sn),
          *pvcam_api_mutex_);

    snprintf(identifier->name,
             sizeof(identifier->name),
             "Teledyne Photometrics %s S/N %s",
             model,
             sn);

    if (!cameras_.contains((int16)id)) {
        PVCAM(pl_cam_close(hcam), *pvcam_api_mutex_);
    }
}

void
PVCamDriver::open(uint64_t device_id, Device** out)
{
    CHECK(out);
    check_pvcam_camera_id(device_id);

    char model[CAM_NAME_LEN];
    PVCAM(pl_cam_get_name((int16)device_id, model), *pvcam_api_mutex_);

    int16 hcam;

    try {
        PVCAM(pl_cam_open(model, &hcam, OPEN_EXCLUSIVE), *pvcam_api_mutex_);
    } catch (...) {
        // clean up on error
        if (PV_FAIL == pl_cam_close(hcam)) {
            LOGE("Failed to close camera.");
        }
        return;
    }

    cameras_.emplace(hcam, new PVCamCamera(hcam, pvcam_api_mutex_));

    *out = (Device*)cameras_.at(hcam);
}

void
PVCamDriver::close(Device* in)
{
    CHECK(in);
    delete (PVCamCamera*)in; // stop and close handled in the destructor
}

void
PVCamDriver::shutdown()
{
    PVCAM(pl_pvcam_uninit(), *pvcam_api_mutex_);
}
} // namespace ::{anonymous}

acquire_export struct Driver*
acquire_driver_init_v0(acquire_reporter_t reporter)
{
    try {
        logger_set_reporter(reporter);
        return new PVCamDriver();
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
    return nullptr;
}

#ifndef NO_UNIT_TESTS
#ifdef _WIN32
#define acquire_export __declspec(dllexport)
#else
#define acquire_export
#endif

extern "C"
{
    acquire_export int unit_test__init()
    {
        try {
            PVCamDriver driver;
        } catch (const std::exception& exc) {
            LOGE("Failed to init: %s", exc.what());
            return 0;
        } catch (...) {
            LOGE("Failed to init: (unknown).");
            return 0;
        }

        return 1;
    }
}

#endif