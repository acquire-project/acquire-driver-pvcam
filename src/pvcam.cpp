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

#define PVCAM_INNER(e, logger, action)                                         \
    do {                                                                       \
        if (PV_FAIL == (e)) {                                                  \
            const int16 code = pl_error_code();                                \
            char msg[ERROR_MSG_LEN];                                           \
            pl_error_message(code, msg);                                       \
            logger(msg);                                                       \
            action;                                                            \
        }                                                                      \
    } while (0)

#define PVCAM(e)                                                               \
    PVCAM_INNER(e, LOGE, throw std::runtime_error("PVCAM API call failed."))
#define PVWARN(e) PVCAM_INNER(e, LOG, )

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
callback_handler(FRAME_INFO* frame_info, void* context);

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

    void push_frame(uint8_t* im, size_t nbytes, FRAME_INFO* info);

  private:
    // Guards access to the PVCAM API.
    std::shared_ptr<std::mutex> pvcam_api_mutex_;

    // The PVCAM camera handle.
    int16 hcam;

    // Setting properties on the device may be expensive, so these are
    // used to avoid doing so when a value is unchanged.
    struct CameraProperties last_known_settings_;

    // Whether or not the camera has been started.
    bool started_;

    // The number of bytes for a single ROI.
    uns32 bytes_per_frame_;

    // The number of frames in the circular buffer.
    uint32_t nframes_buf_;

    // Guards access to the frame buffer and count.
    std::mutex frame_buffer_mutex_;

    // Buffer where the frames are stored.
    std::vector<uint8_t> pvcam_internal_buffer_;
    std::vector<uint8_t> camera_frame_buffer_;
    std::vector<FRAME_INFO> frame_info_buffer_;

    // Number of frames currently stored in the buffer.
    uint32_t frame_count_;

    // Location in the frame buffer where the next frame will be read from.
    uint64_t read_cursor_;

    // Location in the frame buffer where the next frame will be stored.
    uint64_t write_cursor_;

    /// Property setters
    void maybe_set_pixel_type_(SampleType pixel_type);
    void maybe_set_input_triggers_(CameraProperties* properties);
    void maybe_set_output_triggers_(CameraProperties* properties);

    /// Property getters
    void maybe_get_exposure_time_(CameraProperties* properties) const;
    void maybe_get_binning_(CameraProperties* properties) const;
    void maybe_get_roi_(CameraProperties* properties) const;
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

    friend void callback_handler(FRAME_INFO* frame_info, void* context);
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
  , bytes_per_frame_{ 0 }
  , nframes_buf_{ 0 }
  , frame_count_{ 0 }
  , started_{ false }
  , read_cursor_{ 0 }
  , write_cursor_{ 0 }
{
    get(&last_known_settings_);
    PVCAM(pl_cam_register_callback_ex3(
      hcam, PL_CALLBACK_EOF, (void*)callback_handler, this));
}

PVCamCamera::~PVCamCamera() noexcept
{
    try {
        stop();
        std::scoped_lock lock(*pvcam_api_mutex_);
        PVCAM(pl_cam_close(hcam));
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

    maybe_set_pixel_type_(properties->pixel_type);
    maybe_set_input_triggers_(properties);
    maybe_set_output_triggers_(properties);

    // set shape, offset, binning, and exposure time
    if (0 != memcmp(&properties->shape,
                    &last_known_settings_.shape,
                    sizeof(properties->shape)) ||
        0 != memcmp(&properties->offset,
                    &last_known_settings_.offset,
                    sizeof(properties->offset)) ||
        properties->binning != last_known_settings_.binning ||
        properties->exposure_time_us != last_known_settings_.exposure_time_us) {
        rgn_type roi = {
            .s1 = (uns16)properties->offset.x,
            .s2 = (uns16)(properties->offset.x + properties->shape.x - 1),
            .sbin = (uns16)properties->binning,
            .p1 = (uns16)properties->offset.y,
            .p2 = (uns16)(properties->offset.y + properties->shape.y - 1),
            .pbin = (uns16)properties->binning,
        };

        const auto resolution = get_exposure_time_resolution_to_microseconds_();
        auto exposure_time = (uns32)(properties->exposure_time_us / resolution);

        // set up acquisition
        PVCAM(pl_exp_setup_cont(hcam,
                                1,
                                &roi,
                                TIMED_MODE,
                                exposure_time,
                                &bytes_per_frame_,
                                CIRC_NO_OVERWRITE));

        PVCAM(pl_get_param(hcam, PARAM_ROI, ATTR_CURRENT, &roi));
        PVCAM(pl_get_param(
          hcam, PARAM_EXPOSURE_TIME, ATTR_CURRENT, &exposure_time));

        last_known_settings_.offset.x = roi.s1;
        last_known_settings_.offset.y = roi.p1;
        last_known_settings_.shape.x = roi.s2 - roi.s1 + 1;
        last_known_settings_.shape.y = roi.p2 - roi.p1 + 1;
        last_known_settings_.binning = (uint8_t)roi.pbin;

        last_known_settings_.exposure_time_us =
          (float)exposure_time * resolution;

        // The size of the circular buffer - the number of frames the buffer can
        // hold - is completely under the control of the application, however,
        // since the buffer acts as a load leveling entity, it is recommended
        // that the buffer is large enough to hold at least 500ms of
        // acquisition.
        nframes_buf_ =
          (uint32_t)(500.f / (last_known_settings_.exposure_time_us / 1000.f));
    }

    pvcam_internal_buffer_.resize(nframes_buf_ * bytes_per_frame_);
    camera_frame_buffer_.resize(nframes_buf_ * bytes_per_frame_);
    frame_info_buffer_.resize(nframes_buf_);
}

void
PVCamCamera::get(CameraProperties* properties)
{
    CHECK(properties);
    std::scoped_lock<std::mutex> lock(*pvcam_api_mutex_);

    maybe_get_exposure_time_(properties);
    maybe_get_binning_(properties);
    maybe_get_roi_(properties);
    maybe_get_pixel_type_(properties);
    maybe_get_input_triggers_(properties);
    maybe_get_output_triggers_(properties);
}

void
PVCamCamera::get_meta(CameraPropertyMetadata* meta) const
{
    CHECK(meta);
    std::scoped_lock<std::mutex> lock(*pvcam_api_mutex_);

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
    std::scoped_lock<std::mutex> lock(*pvcam_api_mutex_);

    rgn_type roi;
    PVCAM(pl_get_param(hcam, PARAM_ROI, ATTR_CURRENT, &roi));

    int32 pixel_format;
    PVCAM(
      pl_get_param(hcam, PARAM_IMAGE_FORMAT_HOST, ATTR_CURRENT, &pixel_format));

    const auto width = (uint32_t)(roi.s2 - roi.s1 + 1) / roi.sbin;
    const auto height = (uint32_t)(roi.p2 - roi.p1 + 1) / roi.pbin;

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
        .type = pixel_format_to_sample_type.at((PL_IMAGE_FORMATS)pixel_format),
    };
}

void
PVCamCamera::start()
{
    CHECK(!started_);

    std::scoped_lock<std::mutex> api_lock(*pvcam_api_mutex_);
    std::scoped_lock<std::mutex> frame_buffer_lock(frame_buffer_mutex_);

    uint8_t* data = pvcam_internal_buffer_.data();
    PVCAM(pl_exp_start_cont(
      hcam, (void*)data, (uns32)pvcam_internal_buffer_.size()));
    started_ = true;

    frame_count_ = 0;
    write_cursor_ = read_cursor_ = 0;
}

void
PVCamCamera::stop()
{
    CHECK(started_);

    std::scoped_lock<std::mutex> lock(*pvcam_api_mutex_);
    PVCAM(pl_exp_stop_cont(hcam, CCS_HALT));
    started_ = false;
}

void
PVCamCamera::execute_trigger() const
{
    std::scoped_lock<std::mutex> lock(*pvcam_api_mutex_);

    // In current implementation the input flags should be set to 0. On output,
    // the flags will contain one of the values defined in PL_SW_TRIG_STATUSES
    // enumeration.
    uns32 flags = 0;
    PVCAM(pl_exp_trigger(hcam, &flags, 0));

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

    std::scoped_lock<std::mutex> api_lock(*pvcam_api_mutex_);
    std::scoped_lock<std::mutex> frame_buffer_lock(frame_buffer_mutex_);

    if (0 == frame_count_) {
        // No frames available.
        *nbytes = 0;
        return;
    }

    uint32_t width = last_known_settings_.shape.x,
             height = last_known_settings_.shape.y;
    SampleType pixel_type = last_known_settings_.pixel_type;

    const FRAME_INFO& frame_info = frame_info_buffer_[read_cursor_];

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
        .hardware_timestamp = (uint64_t)frame_info.TimeStamp,
        .hardware_frame_id = (uint64_t)frame_info.FrameNr,
    };

    *nbytes = bytes_per_frame_;
    size_t offset = read_cursor_ * bytes_per_frame_;
    std::copy(camera_frame_buffer_.data() + offset,
              camera_frame_buffer_.data() + offset + bytes_per_frame_,
              (uint8_t*)im);

    PVCAM(
      pl_release_frame_info_struct(frame_info_buffer_.data() + read_cursor_));
    PVCAM(pl_exp_unlock_oldest_frame(hcam));

    ++read_cursor_;
    --frame_count_;
}

void
PVCamCamera::push_frame(uint8_t* im, size_t nbytes, FRAME_INFO* info)
{
    if (0 == nbytes) {
        return;
    }

    CHECK(im);
    CHECK(info);

    std::scoped_lock<std::mutex> frame_lock_(frame_buffer_mutex_);

    if (write_cursor_ == read_cursor_ && frame_count_ > 0) {
        // The buffer is full. Drop the oldest frame.
        PVCAM(pl_exp_unlock_oldest_frame(hcam));
        --frame_count_;
        read_cursor_ = (read_cursor_ + 1) % nframes_buf_;
    }

    // Copy the frame into the buffer.
    std::copy(im,
              im + nbytes,
              pvcam_internal_buffer_.data() + write_cursor_ * bytes_per_frame_);
    frame_info_buffer_[write_cursor_] = *info;

    write_cursor_ = (write_cursor_ + 1) % nframes_buf_;

    ++frame_count_;
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

    // As a general rule, the application should always rely on the
    // '_HOST'-specific parameters when identifying the output data format. The
    // native parameters should be used only for informational purposes, e.g. to
    // show the camera native format in the GUI.
    PVWARN(pl_set_param(hcam, PARAM_IMAGE_FORMAT_HOST, &pixel_format));
}

void
PVCamCamera::maybe_set_input_triggers_(CameraProperties* properties)
{
}

void
PVCamCamera::maybe_set_output_triggers_(CameraProperties* properties)
{
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
    PVWARN(
      pl_get_param(hcam, PARAM_EXPOSURE_TIME, ATTR_CURRENT, &exposure_time));

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
    PVWARN(pl_get_param(hcam, PARAM_BINNING_PAR, ATTR_CURRENT, &binning));

    properties->binning = (uint8_t)binning;
}

void
PVCamCamera::maybe_get_roi_(CameraProperties* properties) const
{
    // We assume here that the API mutex is already locked.
    properties->offset = { 0 };
    properties->shape = { 0 };
    properties->binning = 1;

    rgn_type roi;
    PVWARN(pl_get_param(hcam, PARAM_ROI, ATTR_CURRENT, &roi));

    properties->offset.x = roi.s1;
    properties->offset.y = roi.p1;
    properties->shape.x = roi.s2 - roi.s1 + 1;
    properties->shape.y = roi.p2 - roi.p1 + 1;
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
    PVWARN(
      pl_get_param(hcam, PARAM_IMAGE_FORMAT_HOST, ATTR_CURRENT, &pixel_format));

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
PVCamCamera::maybe_get_input_triggers_(CameraProperties* properties) const
{
    // We assume here that the API mutex is already locked.
    properties->input_triggers = { 0 };
}

void
PVCamCamera::maybe_get_output_triggers_(CameraProperties* properties) const
{
    // We assume here that the API mutex is already locked.
    properties->input_triggers = { 0 };
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

void
PVCamCamera::query_roi_capabilities(CameraPropertyMetadata* meta) const
{
    // We assume here that the API mutex is already locked.
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

    auto is_writable = (uint8_t)is_param_writable(PARAM_ROI);
    meta->offset.x = {
        .writable = is_writable,
        .low = (float)roi_min.s1,
        .high = (float)roi_max.s1,
        .type = PropertyType_FixedPrecision,
    };

    meta->offset.y = {
        .writable = is_writable,
        .low = (float)roi_min.p1,
        .high = (float)roi_max.p1,
        .type = PropertyType_FixedPrecision,
    };

    meta->shape.x = {
        .writable = is_writable,
        .low = (float)(roi_min.s2 - roi_max.s1 + 1),
        .high = (float)(roi_max.s2 - roi_min.s1 + 1),
    };

    meta->shape.y = {
        .writable = is_writable,
        .low = (float)(roi_min.p2 - roi_max.p1 + 1),
        .high = (float)(roi_max.p2 - roi_min.p1 + 1),
    };
}

void
PVCamCamera::query_pixel_type_capabilities(CameraPropertyMetadata* meta) const
{
}

void
PVCamCamera::query_triggering_capabilities(CameraPropertyMetadata* meta) const
{
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
        case EXP_RES_ONE_SEC:
            return 1e6f;
        case EXP_RES_ONE_MICROSEC:
        default:
            return 1.f;
    }
}

void PV_DECL
callback_handler(FRAME_INFO* frame_info, void* context)
{
    CHECK(context);

    LOG("callback_handler");

    auto camera = (PVCamCamera*)context;
    std::vector<uint8_t> frame_buffer(camera->bytes_per_frame_);

    try {
        PVCAM(pl_exp_get_latest_frame(camera->hcam, (void**)&frame_buffer[0]));
        camera->push_frame(
          frame_buffer.data(), frame_buffer.size(), frame_info);
    } catch (...) {
        return;
    }
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
    std::scoped_lock<std::mutex> lock(*pvcam_api_mutex_);
    PVCAM(pl_pvcam_init());
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
    std::scoped_lock<std::mutex> lock(*pvcam_api_mutex_);

    uint32_t n_cameras = 0;
    PVCAM(pl_cam_get_total((int16*)&n_cameras));
    return n_cameras;
}

void
PVCamDriver::describe(DeviceIdentifier* identifier, uint64_t id)
{
    CHECK(identifier);
    check_pvcam_camera_id(id);
    std::scoped_lock<std::mutex> lock(*pvcam_api_mutex_);

    char model[CAM_NAME_LEN];
    char sn[MAX_ALPHA_SER_NUM_LEN];

    PVCAM(pl_cam_get_name((int16)id, model));

    *identifier = {
        .device_id = (uint8_t)id,
        .kind = DeviceKind_Camera,
    };

    int16 hcam;
    if (cameras_.contains((int16)id)) {
        hcam = (int16)id;
    } else {
        PVCAM(pl_cam_open(model, &hcam, OPEN_EXCLUSIVE));
    }

    PVCAM(pl_get_param(hcam, PARAM_HEAD_SER_NUM_ALPHA, ATTR_CURRENT, sn));

    snprintf(identifier->name,
             sizeof(identifier->name),
             "Teledyne Photometrics %s S/N %s",
             model,
             sn);

    if (!cameras_.contains((int16)id)) {
        PVCAM(pl_cam_close(hcam));
    }
}

void
PVCamDriver::open(uint64_t device_id, Device** out)
{
    CHECK(out);
    check_pvcam_camera_id(device_id);

    char model[CAM_NAME_LEN];
    int16 hcam;

    {
        std::scoped_lock<std::mutex> lock(*pvcam_api_mutex_);
        PVCAM(pl_cam_get_name((int16)device_id, model));
    }

    try {
        std::scoped_lock<std::mutex> lock(*pvcam_api_mutex_);
        PVCAM(pl_cam_open(model, &hcam, OPEN_EXCLUSIVE));
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
    std::scoped_lock<std::mutex> lock(*pvcam_api_mutex_);
    PVCAM(pl_pvcam_uninit());
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