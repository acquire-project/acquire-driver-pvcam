// Checks various property setting manipulation
#include "acquire.h"

#include "device/hal/device.manager.h"
#include "device/props/camera.h"
#include "logger.h"
#include <cstdio>
#include <stdexcept>

/// Helper for passing size static strings as function args.
/// For a function: `f(char*,size_t)` use `f(SIZED("hello"))`.
/// Expands to `f("hello",5)`.
#define SIZED(str) str, sizeof(str)

#define L (aq_logger)
#define LOG(...) L(0, __FILE__, __LINE__, __FUNCTION__, __VA_ARGS__)
#define ERR(...) L(1, __FILE__, __LINE__, __FUNCTION__, __VA_ARGS__)
#define EXPECT(e, ...)                                                         \
    do {                                                                       \
        if (!(e)) {                                                            \
            char buf[1 << 8] = { 0 };                                          \
            ERR(__VA_ARGS__);                                                  \
            snprintf(buf, sizeof(buf) - 1, __VA_ARGS__);                       \
            throw std::runtime_error(buf);                                     \
        }                                                                      \
    } while (0)
#define CHECK(e) EXPECT(e, "Expression evaluated as false: %s", #e)
#define DEVOK(e) CHECK(Device_Ok == (e))
#define OK(e) CHECK(AcquireStatus_Ok == (e))

/// example: `ASSERT_EQ(int, "%d", 42, meaning_of_life())`
#define ASSERT_EQ(T, fmt, a, b)                                                \
    do {                                                                       \
        T a_ = (T)(a);                                                         \
        T b_ = (T)(b);                                                         \
        EXPECT(a_ == b_, "Expected %s==%s but " fmt "!=" fmt, #a, #b, a_, b_); \
    } while (0)

void
reporter(int is_error,
         const char* file,
         int line,
         const char* function,
         const char* msg)
{
    auto stream = is_error ? stderr : stdout;
    fprintf(stream,
            "%s%s(%d) - %s: %s\n",
            is_error ? "ERROR " : "",
            file,
            line,
            function,
            msg);
    fflush(stream);
}

void
setup(AcquireRuntime* runtime)
{
    CHECK(runtime);

    const auto* dm = acquire_device_manager(runtime);
    CHECK(dm);

    AcquireProperties props = {};
    OK(acquire_get_configuration(runtime, &props));

    DEVOK(device_manager_select(
                                dm, 
                                DeviceKind_Camera, 
                                SIZED(".*Kinetix.*") - 1, 
                                &props.video[0].camera.identifier));

    DEVOK(device_manager_select(dm,
                                DeviceKind_Storage,
                                SIZED("Trash") - 1,
                                &props.video[0].storage.identifier));

    OK(acquire_configure(runtime, &props));
}

void
check_metadata(AcquireRuntime* runtime)
{
    AcquirePropertyMetadata metadata = { 0 };
    OK(acquire_get_configuration_metadata(runtime, &metadata));
    const CameraPropertyMetadata& meta = metadata.video[0].camera;

    // Expected values determined by inspecting Prime BSI metadata in PVCamTest.

    ASSERT_EQ(uint8_t, "%d", meta.exposure_time_us.writable, 1);
    ASSERT_EQ(
      int, "%d", meta.exposure_time_us.type, PropertyType_FixedPrecision);

    ASSERT_EQ(uint8_t, "%d", meta.line_interval_us.writable, 0);
    ASSERT_EQ(uint8_t, "%d", meta.readout_direction.writable, 0);

    ASSERT_EQ(uint8_t, "%d", meta.binning.writable, 1);
    ASSERT_EQ(float, "%g", meta.binning.low, 1);
    ASSERT_EQ(float, "%g", meta.binning.high, 4);
    ASSERT_EQ(int, "%d", meta.binning.type, PropertyType_FixedPrecision);

    ASSERT_EQ(uint8_t, "%d", meta.shape.x.writable, 1);
    ASSERT_EQ(float, "%g", meta.shape.x.low, 1);
    ASSERT_EQ(float, "%g", meta.shape.x.high, 3200);
    ASSERT_EQ(int, "%d", meta.shape.x.type, PropertyType_FixedPrecision);

    ASSERT_EQ(uint8_t, "%d", meta.shape.y.writable, 1);
    ASSERT_EQ(float, "%g", meta.shape.y.low, 1);
    ASSERT_EQ(float, "%g", meta.shape.y.high, 3200);
    ASSERT_EQ(int, "%d", meta.shape.y.type, PropertyType_FixedPrecision);

    ASSERT_EQ(uint8_t, "%d", meta.offset.x.writable, 1);
    ASSERT_EQ(float, "%g", meta.offset.x.low, 0);
    ASSERT_EQ(float, "%g", meta.offset.x.high, 3199);
    ASSERT_EQ(int, "%d", meta.offset.x.type, PropertyType_FixedPrecision);

    ASSERT_EQ(uint8_t, "%d", meta.offset.y.writable, 1);
    ASSERT_EQ(float, "%g", meta.offset.y.low, 0);
    ASSERT_EQ(float, "%g", meta.offset.y.high, 3199);
    ASSERT_EQ(int, "%d", meta.offset.y.type, PropertyType_FixedPrecision);

    ASSERT_EQ(unsigned int,
              "0x%x",
              (unsigned int)meta.supported_pixel_types,
              (1U << SampleType_u8) | (1U << SampleType_u16));
}

int
main()
{
    auto* runtime = acquire_init(reporter);
    try {
        setup(runtime);
        check_metadata(runtime);

        OK(acquire_shutdown(runtime));
        LOG("OK");
        return 0;
    } catch (const std::runtime_error& e) {
        ERR("Runtime error: %s", e.what());
    } catch (...) {
        ERR("Uncaught exception");
    }
    acquire_shutdown(runtime);
    return 1;
}