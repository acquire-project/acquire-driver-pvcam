#include "device/kit/driver.h"
#include "device/hal/camera.h"
#include "logger.h"

#include <memory>
#include <mutex>
#include <unordered_map>
#include <unordered_set>

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
        if (PV_OK != (e)) {                                                    \
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

const std::unordered_map<PL_IMAGE_FORMATS, SampleType>
  pixel_format_to_sample_type{
      { PL_IMAGE_FORMAT_MONO8, SampleType_u8 },
      { PL_IMAGE_FORMAT_MONO16, SampleType_u16 },
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

void
camera_properties_to_rgn_type(CameraProperties* properties, rgn_type& roi)
{
    auto binning = (uns16)properties->binning;
    roi = {
        .s1 = (uns16)properties->offset.x,
        .s2 = (uns16)(properties->offset.x + properties->shape.x * binning - 1),
        .sbin = binning,
        .p1 = (uns16)properties->offset.y,
        .p2 = (uns16)(properties->offset.y + properties->shape.y * binning - 1),
        .pbin = binning,
    };
}

void
rgn_type_to_camera_properties(rgn_type& roi, CameraProperties* properties)
{
    properties->binning = (uint8_t)roi.pbin;

    properties->offset = {
        .x = roi.s1,
        .y = roi.p1,
    };
    properties->shape = {
        .x = (uint32_t)((roi.s2 - roi.s1 + 1) / properties->binning),
        .y = (uint32_t)((roi.p2 - roi.p1 + 1) / properties->binning),
    };
}

// forward
struct PVCamCamera;

struct CallbackContext
{
    int16 hcam;

    std::shared_ptr<std::mutex> api_mutex;
    std::shared_ptr<std::mutex> frame_mutex;
    std::condition_variable cv;
    std::atomic<int> frames_available;

    uint8_t** frame;
    FRAME_INFO info;
};

void
callback_handler(FRAME_INFO* frame_info, void* context)
{
    auto* ctx = (CallbackContext*)context;
    std::unique_lock<std::mutex> frame_lock(*ctx->frame_mutex);

    PVCAM(pl_exp_get_oldest_frame_ex(ctx->hcam, (void**)ctx->frame, frame_info),
          *ctx->api_mutex);

    ctx->info = *frame_info;
    ++ctx->frames_available;
    ctx->cv.notify_one();
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
    void get_frame(void* im, size_t* nbytes, ImageInfo* info);

  private:
    // Guards access to the PVCAM API.
    std::shared_ptr<std::mutex> pvcam_api_mutex_;

    // The PVCAM camera handle.
    int16 hcam;

    // Setting properties on the device may be expensive, so these are
    // used to avoid doing so when a value is unchanged.
    struct CameraProperties last_known_settings_;
    mutable rgn_type roi_;
    mutable rgn_type roi_max_;
    /// exposure time in the camera's resolution
    mutable uns32 exposure_time_;
    /// factor to multiply exposure time to get value in microseconds
    mutable uns32 exposure_time_to_us_;

    // Whether or not the camera has been started.
    bool started_;

    // The number of bytes for a single ROI.
    uns32 bytes_per_frame_;

    // The number of frames in the circular buffer.
    uint32_t nframes_buf_;

    // Guards access to the frame buffer and count.
    std::shared_ptr<std::mutex> frame_mutex_;

    // Buffer where the frames are stored.
    std::vector<uint8_t> pvcam_internal_buffer_;
    uint8_t* pl_buffer_;
    ulong64 pl_buffer_size_;

    // Number of frames acquired.
    uint64_t frame_count_;

    CallbackContext callback_context_;

    /// Property setters
    void maybe_set_exposure_time_(CameraProperties* properties,
                                  uns32& exposure_time);
    void maybe_set_roi_and_binning_(CameraProperties* properties,
                                    rgn_type& roi);
    void maybe_set_pixel_type_(CameraProperties* properties);
    void maybe_set_input_trigger_(CameraProperties* properties);
    void maybe_set_output_triggers_(CameraProperties* properties);

    /// Property getters
    void maybe_get_exposure_time_(CameraProperties* properties) const;
    void maybe_get_roi_and_binning_(CameraProperties* properties) const;
    void maybe_get_pixel_type_(CameraProperties* properties) const;
    void maybe_get_input_triggers_(CameraProperties* properties) const;
    void maybe_get_output_triggers_(CameraProperties* properties) const;

    /// Metadata getters
    void query_exposure_time_capabilities_(CameraPropertyMetadata* meta) const;
    void query_binning_capabilities(CameraPropertyMetadata* meta) const;
    void query_roi_capabilities(CameraPropertyMetadata* meta) const;
    void query_pixel_type_capabilities(CameraPropertyMetadata* meta) const;
    void query_triggering_capabilities(CameraPropertyMetadata* meta) const;

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
  , frame_mutex_{ std::make_shared<std::mutex>() }
  , last_known_settings_{ 0 }
  , bytes_per_frame_{ 0 }
  , nframes_buf_{ 0 }
  , frame_count_{ 0 }
  , started_{ false }
  , pl_buffer_{ nullptr }
  , pl_buffer_size_{ 0 }
  , callback_context_ {
      .hcam = hcam,
      .api_mutex = pvcam_api_mutex_,
      .frame_mutex = frame_mutex_,
      .cv = {},
      .frames_available = 0,
      .frame = new uint8_t*(),
      .info = {},
  }
{
    get(&last_known_settings_);

    // register the callback
    PVCAM(pl_cam_register_callback_ex3(this->hcam,
                                       PL_CALLBACK_EOF,
                                       (void*)callback_handler,
                                       (void*)&callback_context_),
          *pvcam_api_mutex_);
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

    delete callback_context_.frame;
}

void
PVCamCamera::set(CameraProperties* properties)
{
    CHECK(properties);

    uns32 exposure_time;
    maybe_set_exposure_time_(properties, exposure_time);

    rgn_type roi;
    maybe_set_roi_and_binning_(properties, roi);

    maybe_set_pixel_type_(properties);
    maybe_set_input_trigger_(properties);
    maybe_set_output_triggers_(properties);

    // set up acquisition
    PVCAM(pl_exp_setup_cont(hcam,
                            1,
                            &roi,
                            TIMED_MODE,
                            exposure_time,
                            &bytes_per_frame_,
                            CIRC_NO_OVERWRITE),
          *pvcam_api_mutex_);

    get(&last_known_settings_);
}

void
PVCamCamera::get(CameraProperties* properties)
{
    CHECK(properties);

    maybe_get_exposure_time_(properties);
    maybe_get_roi_and_binning_(properties);
    maybe_get_pixel_type_(properties);
    maybe_get_input_triggers_(properties);
    maybe_get_output_triggers_(properties);
}

void
PVCamCamera::get_meta(CameraPropertyMetadata* meta) const
{
    CHECK(meta);

    query_exposure_time_capabilities_(meta);
    query_binning_capabilities(meta);
    query_roi_capabilities(meta);
    query_pixel_type_capabilities(meta);
    query_triggering_capabilities(meta);
}

void
PVCamCamera::get_shape(ImageShape* shape) const
{
    CHECK(shape);

    const auto width =
      last_known_settings_.shape.x / last_known_settings_.binning;
    const auto height =
      last_known_settings_.shape.y / last_known_settings_.binning;

    *shape = {
                .dims = {
                  .channels = 1,
                  .width = (uint32_t)width,
                  .height = (uint32_t)height,
                  .planes = 1,
                },
                .strides = {
                  .channels = 1,
                  .width = 1,
                  .height = width,
                  .planes = width * height,
                },
                .type = last_known_settings_.pixel_type,
            };
}

void
PVCamCamera::start()
{
    using namespace std::chrono_literals;

    CHECK(!started_);

    // allocate internal frame buffers, counters, and cursors
    ulong64 frame_buffer_size;
    PVCAM(pl_get_param(
            hcam, PARAM_FRAME_BUFFER_SIZE, ATTR_DEFAULT, &frame_buffer_size),
          *pvcam_api_mutex_);

    pl_buffer_size_ = frame_buffer_size;
    pl_buffer_ = new uint8_t[pl_buffer_size_];

    nframes_buf_ = (uint32_t)(frame_buffer_size / bytes_per_frame_);
    frame_count_ = 0;

    // start acquisition
    PVCAM(pl_exp_start_cont(hcam, (void*)pl_buffer_, (uns32)pl_buffer_size_),
          *pvcam_api_mutex_);

    int16 status;
    uns32 byte_cnt, buffer_cnt;
    PVCAM(pl_exp_check_cont_status(hcam, &status, &byte_cnt, &buffer_cnt),
          *pvcam_api_mutex_);

    started_ = true;
}

void
PVCamCamera::stop()
{
    if (!started_) {
        return;
    }

    LOG("Stopping");
    PVCAM(pl_exp_stop_cont(hcam, CCS_HALT), *pvcam_api_mutex_);
    started_ = false;

    delete[] pl_buffer_;
    pl_buffer_ = nullptr;
    pl_buffer_size_ = 0;
}

void
PVCamCamera::execute_trigger() const
{
    // In current implementation the input flags should be set to 0. On output,
    // the flags will contain one of the values defined in PL_SW_TRIG_STATUSES
    // enumeration.
    uns32 flags = 0;
    PVCAM(pl_exp_trigger(hcam, &flags, 0), *pvcam_api_mutex_);

    if (flags != PL_SW_TRIG_STATUS_TRIGGERED) {
        LOGE("The camera was unable to accept the trigger due to an ongoing "
             "exposure.");
    }
}

void
PVCamCamera::get_frame(void* im, size_t* nbytes, struct ImageInfo* info)
{
    CHECK(im);
    CHECK(nbytes);
    CHECK(info);

    int16 status;
    uns32 byte_cnt, buffer_cnt;
    PVCAM(pl_exp_check_cont_status(hcam, &status, &byte_cnt, &buffer_cnt),
          *pvcam_api_mutex_);

    std::unique_lock<std::mutex> frame_lock(*frame_mutex_);

    // callback will increment frames_available
    callback_context_.cv.wait_for(
      frame_lock,
      std::chrono::microseconds(exposure_time_ * exposure_time_to_us_),
      [&] { return callback_context_.frames_available > 0; });

    if (0 == callback_context_.frames_available) {
        *nbytes = 0;
        return;
    }
    --callback_context_.frames_available;

    uint32_t width = last_known_settings_.shape.x,
             height = last_known_settings_.shape.y;
    SampleType pixel_type = last_known_settings_.pixel_type;

    memcpy(
      im, *callback_context_.frame, width * height * bytes_of_type(pixel_type));

    *info = {
            .shape = {
              .dims = {
                .channels = 1,
                .width = width,
                .height = height,
                .planes = 1,
              },
              .strides = {
                .channels = 1,
                .width = 1,
                .height = width,
                .planes = width * height,
              },
              .type = pixel_type,
            },
            .hardware_timestamp = (uint64_t)callback_context_.info.TimeStamp,
            .hardware_frame_id = (uint64_t)callback_context_.info.FrameNr,
        };

    PVCAM(pl_exp_unlock_oldest_frame(hcam), *pvcam_api_mutex_);

    ++frame_count_;

    rgn_type roi;
    PVCAM(pl_get_param(hcam, PARAM_ROI, ATTR_CURRENT, &roi), *pvcam_api_mutex_);
}

void
PVCamCamera::maybe_set_exposure_time_(CameraProperties* properties,
                                      uns32& exposure_time)
{
    CHECK(properties);

    const auto resolution = get_exposure_time_resolution_to_microseconds_();
    exposure_time = (uns32)(properties->exposure_time_us / resolution);
}

void
PVCamCamera::maybe_set_roi_and_binning_(CameraProperties* properties,
                                        rgn_type& roi)
{
    CHECK(properties);

    // a couple of edge cases to handle
    if (properties->shape.x == 0 || properties->shape.y == 0 ||
        properties->binning == 0) {
        return;
    }

    camera_properties_to_rgn_type(properties, roi);

    // validate roi against max
    EXPECT(roi.s2 <= roi_max_.s2,
           "Specified x extent (%d) past sensor limit (%d).",
           roi.s2,
           roi_max_.s2);
    EXPECT(roi.p2 <= roi_max_.p2,
           "Specified y extent (%d) past sensor limit (%d).",
           roi.p2,
           roi_max_.p2);
}

void
PVCamCamera::maybe_set_pixel_type_(CameraProperties* properties)
{
    CHECK(properties && properties->pixel_type < SampleTypeCount);

    if (properties->pixel_type == last_known_settings_.pixel_type ||
        !is_param_writable(PARAM_IMAGE_FORMAT_HOST)) {
        return;
    }

    if (!sample_type_to_pixel_format.contains(properties->pixel_type)) {
        LOGE("Unsupported pixel type: %d", properties->pixel_type);
        return;
    }

    auto pixel_format = sample_type_to_pixel_format.at(properties->pixel_type);

    // As a general rule, the application should always rely on the
    // '_HOST'-specific parameters when identifying the output data format. The
    // native parameters should be used only for informational purposes, e.g. to
    // show the camera native format in the GUI.
    PVWARN(pl_set_param(hcam, PARAM_IMAGE_FORMAT_HOST, &pixel_format),
           *pvcam_api_mutex_);
    last_known_settings_.pixel_type = properties->pixel_type;
}

void
PVCamCamera::maybe_set_input_trigger_(CameraProperties* properties)
{
    CHECK(properties);
}

void
PVCamCamera::maybe_set_output_triggers_(CameraProperties* properties)
{
    CHECK(properties);
}

void
PVCamCamera::maybe_get_exposure_time_(CameraProperties* properties) const
{
    const auto resolution = get_exposure_time_resolution_to_microseconds_();
    properties->exposure_time_us = (float)exposure_time_ * resolution;

    if (!is_param_available(PARAM_EXPOSURE_TIME)) {
        return;
    }

    try {
        PVCAM(pl_get_param(
                hcam, PARAM_EXPOSURE_TIME, ATTR_CURRENT, &exposure_time_),
              *pvcam_api_mutex_);
    } catch (...) {
        // Failed to get the exposure time.
        return;
    }

    // convert the resolution to microseconds
    properties->exposure_time_us = (float)exposure_time_ * resolution;
}

void
PVCamCamera::maybe_get_roi_and_binning_(CameraProperties* properties) const
{
    if (!is_param_available(PARAM_ROI)) {
        return;
    }

    PVWARN(pl_get_param(hcam, PARAM_ROI, ATTR_CURRENT, &roi_),
           *pvcam_api_mutex_);
    PVWARN(pl_get_param(hcam, PARAM_ROI, ATTR_MAX, &roi_max_),
           *pvcam_api_mutex_);

    rgn_type_to_camera_properties(roi_, properties);
}

void
PVCamCamera::maybe_get_pixel_type_(CameraProperties* properties) const
{
    properties->pixel_type = SampleType_u8;

    if (!is_param_available(PARAM_IMAGE_FORMAT_HOST)) {
        return;
    }

    int32 pixel_format;
    PVWARN(
      pl_get_param(hcam, PARAM_IMAGE_FORMAT_HOST, ATTR_CURRENT, &pixel_format),
      *pvcam_api_mutex_);

    if (const auto sample_type =
          pixel_format_to_sample_type.find((PL_IMAGE_FORMATS)pixel_format);
        sample_type != pixel_format_to_sample_type.end()) {
        properties->pixel_type = sample_type->second;
    } else {
        LOGE("Unsupported pixel format: %d", pixel_format);
    }
}

void
PVCamCamera::maybe_get_input_triggers_(CameraProperties* properties) const
{
    properties->input_triggers = { 0 };
}

void
PVCamCamera::maybe_get_output_triggers_(CameraProperties* properties) const
{
    properties->input_triggers = { 0 };
}

void
PVCamCamera::query_exposure_time_capabilities_(
  CameraPropertyMetadata* meta) const
{
    meta->exposure_time_us = { 0 };
    meta->exposure_time_us.type = PropertyType_FixedPrecision;

    if (!is_param_available(PARAM_EXPOSURE_TIME)) {
        return;
    }

    meta->exposure_time_us.writable = 1;

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
    meta->binning = { 0 };
    meta->binning.type = PropertyType_FixedPrecision;

    if (!is_param_available(PARAM_BINNING_PAR) ||
        !is_param_available(PARAM_BINNING_SER)) {
        meta->binning.low = meta->binning.high = 1.f;
        return;
    }

    meta->binning.writable = 1;

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

void
PVCamCamera::query_roi_capabilities(CameraPropertyMetadata* meta) const
{
    meta->offset = { 0 };
    meta->offset.x.type = PropertyType_FixedPrecision;
    meta->offset.y.type = PropertyType_FixedPrecision;
    meta->shape.x.type = PropertyType_FixedPrecision;
    meta->shape.y.type = PropertyType_FixedPrecision;

    if (!is_param_available(PARAM_ROI)) {
        return;
    }

    rgn_type roi_min, roi_max;
    if (!query_param_min_max(PARAM_ROI, &roi_min, &roi_max)) {
        return;
    }

    meta->offset.x = {
        .writable = 1,
        .low = (float)roi_min.s1,
        .high = (float)roi_max.s1,
        .type = PropertyType_FixedPrecision,
    };

    meta->offset.y = {
        .writable = 1,
        .low = (float)roi_min.p1,
        .high = (float)roi_max.p1,
        .type = PropertyType_FixedPrecision,
    };

    meta->shape.x = {
        .writable = 1,
        .low = (float)(roi_min.s2 - roi_max.s1 + 1),
        .high = (float)(roi_max.s2 - roi_min.s1 + 1),
    };

    meta->shape.y = {
        .writable = 1,
        .low = (float)(roi_min.p2 - roi_max.p1 + 1),
        .high = (float)(roi_max.p2 - roi_min.p1 + 1),
    };
}

void
PVCamCamera::query_pixel_type_capabilities(CameraPropertyMetadata* meta) const
{
    meta->supported_pixel_types = 0;

    uns32 count;
    PVCAM(pl_get_param(hcam, PARAM_IMAGE_FORMAT_HOST, ATTR_COUNT, &count),
          *pvcam_api_mutex_);

    char text[256];

    for (auto i = 0; i < count; ++i) {
        uns32 str_length;
        PVCAM(pl_enum_str_length(hcam, PARAM_IMAGE_FORMAT_HOST, i, &str_length),
              *pvcam_api_mutex_);

        int32 val;
        PVCAM(pl_get_enum_param(
                hcam, PARAM_IMAGE_FORMAT_HOST, i, &val, text, str_length),
              *pvcam_api_mutex_);

        auto it = pixel_format_to_sample_type.find((PL_IMAGE_FORMATS)val);
        if (it != pixel_format_to_sample_type.end()) {
            meta->supported_pixel_types |= (1U << it->second);
        }
    }
}

void
PVCamCamera::query_triggering_capabilities(CameraPropertyMetadata* meta) const
{
    char model[MAX_PRODUCT_NAME_LEN];
    PVCAM(pl_get_param(hcam, PARAM_PRODUCT_NAME, ATTR_CURRENT, &model),
          *pvcam_api_mutex_);

    if (strstr(model, "Express")) {
        meta->digital_lines = {
            .line_count = 5,
            .names = {
              "Trigger In",
              "Trigger Ready Out",
              "Read Out",
              "Expose Out",
              "Software",
            },
        };

        meta->triggers = {
            .acquisition_start = {
                .input = 0,
                .output = 0,
              },
            .exposure = {
              .input = 0,
              .output = 0
            },
            .frame_start = {
              .input = 0,
              .output = 0
            }
        };

        meta->triggers = {
            .acquisition_start = {
              .input = 0b0001'0001,
              .output = 0b0000'0010,
            },
            .exposure = {
              .input = 0b0000'0001,
              .output = 0b0000'1000
            },
            .frame_start = {
              .input = 0b0001'0001,
              .output = 0b0000'1000
            }
        };
    } else {
        meta->digital_lines = {
            .line_count = 8,
            .names = {
              "Trigger In",
              "Trigger Ready Out",
              "Read Out",
              "Expose Out 1",
              "Expose Out 2",
              "Expose Out 3",
              "Expose Out 4",
              "Software",
            },
        };

        meta->triggers = {
            .acquisition_start = {
              .input = 0b1000'0001,
              .output = 0b0000'0010,
            },
            .exposure = {
              .input = 0b0000'0001,
              .output = 0b0111'1000
            },
            .frame_start = {
              .input = 0b1000'0001,
              .output = 0b0111'1000
            }
        };
    }
}

bool
PVCamCamera::is_param_available(uns32 param_id) const
{
    rs_bool is_avail;
    if (PV_OK != pl_get_param(hcam, param_id, ATTR_AVAIL, &is_avail)) {
        return false;
    }

    return is_avail == TRUE;
}

bool
PVCamCamera::is_param_writable(uns32 param_id) const
{
    uns16 access = 0;
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

    *min = std::numeric_limits<T>::min();
    *max = std::numeric_limits<T>::max();

    if (!is_param_available(param_id)) {
        return false;
    }

    if (PV_OK != pl_get_param(hcam, param_id, ATTR_MIN, min)) {
        LOGE("Failed to get min.");
        return false;
    }

    if (PV_OK != pl_get_param(hcam, param_id, ATTR_MAX, max)) {
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
        PV_OK != pl_get_param(hcam, PARAM_EXP_RES, ATTR_CURRENT, &res)) {
        // For some older cameras this parameter might not be available
        // (ATTR_AVAIL returns FALSE). In this case camera uses
        // EXP_RES_ONE_MILLISEC resolution.
        res = EXP_RES_ONE_MILLISEC;
    }

    switch (res) {
        case EXP_RES_ONE_MILLISEC:
            exposure_time_to_us_ = 1000;
            break;
        case EXP_RES_ONE_SEC:
            exposure_time_to_us_ = 1000000;
            break;
        case EXP_RES_ONE_MICROSEC:
        default:
            exposure_time_to_us_ = 1;
    }

    return (float)exposure_time_to_us_;
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

    std::unordered_set<int16> cameras_;

    void close_camera(int16 hcam) noexcept;
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

    char name[CAM_NAME_LEN];          // PVCAM-assigned name
    char model[MAX_PRODUCT_NAME_LEN]; // Product name, e.g., "BSI Express"
    char sn[MAX_ALPHA_SER_NUM_LEN];   // Serial number

    PVCAM(pl_cam_get_name((int16)id, name), *pvcam_api_mutex_);

    *identifier = {
        .device_id = (uint8_t)id,
        .kind = DeviceKind_Camera,
    };

    int16 hcam;
    if (cameras_.contains((int16)id)) {
        hcam = (int16)id;
    } else {
        PVCAM(pl_cam_open(name, &hcam, OPEN_EXCLUSIVE), *pvcam_api_mutex_);
    }

    PVCAM(pl_get_param(hcam, PARAM_PRODUCT_NAME, ATTR_CURRENT, model),
          *pvcam_api_mutex_);
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

    char name[CAM_NAME_LEN];
    int16 hcam;

    PVCAM(pl_cam_get_name((int16)device_id, name), *pvcam_api_mutex_);

    try {
        PVCAM(pl_cam_open(name, &hcam, OPEN_EXCLUSIVE), *pvcam_api_mutex_);
    } catch (...) {
        // clean up on error
        close_camera(hcam);
        throw;
    }

    cameras_.emplace(hcam);
    *out = (Device*)(new PVCamCamera(hcam, pvcam_api_mutex_));
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

void
PVCamDriver::close_camera(int16 hcam) noexcept
{
    try {
        PVCAM(pl_cam_close(hcam), *pvcam_api_mutex_);
    } catch (const std::exception& exc) {
        LOGE("Exception: %s\n", exc.what());
    } catch (...) {
        LOGE("Exception: (unknown)");
    }
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

/**
 * NOTE (aliddell): I observed that API calls between `start` and `get_frame`
 * seemed to cause the API to hang, resulting in no callbacks being fired. In
 * particular, it manifested in an earlier version of `get_shape`, which made
 * two calls to `pl_get_param` to query the current image ROI and pixel type. I
 * have changed it to use cached values, and I tested querying for the ROI at
 * the end of 100 calls to `get_frame`, and it appears to work fine.
 */
