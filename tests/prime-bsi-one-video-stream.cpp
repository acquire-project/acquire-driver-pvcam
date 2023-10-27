#include "acquire.h"
#include "device/hal/device.manager.h"
#include "platform.h"
#include "logger.h"

#include <cstdio>
#include <stdexcept>

/// Helper for passing size static strings as function args.
/// For a function: `f(char*,size_t)` use `f(SIZED("hello"))`.
/// Expands to `f("hello",5)`.
#define SIZED(str) str, sizeof(str)

#define L (aq_logger)
#define LOG(...) L(0, __FILE__, __LINE__, __FUNCTION__, __VA_ARGS__)
#define LOGE(...) L(1, __FILE__, __LINE__, __FUNCTION__, __VA_ARGS__)
#define EXPECT(e, ...)                                                         \
    do {                                                                       \
        if (!(e)) {                                                            \
            char buf[1 << 8] = { 0 };                                          \
            LOGE(__VA_ARGS__);                                                 \
            snprintf(buf, sizeof(buf) - 1, __VA_ARGS__);                       \
            throw std::runtime_error(buf);                                     \
        }                                                                      \
    } while (0)
#define CHECK(e) EXPECT(e, "Expression evaluated as false: %s", #e)
#define DEVOK(e) CHECK(Device_Ok == (e))
#define OK(e) CHECK(AcquireStatus_Ok == (e))

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

    auto dm = acquire_device_manager(runtime);
    CHECK(dm);

    AcquireProperties props = {};
    OK(acquire_get_configuration(runtime, &props));

    DEVOK(device_manager_select(dm,
                                DeviceKind_Camera,
                                SIZED(".*PMPCIECam.*") - 1,
                                &props.video[0].camera.identifier));
    DEVOK(device_manager_select(dm,
                                DeviceKind_Storage,
                                SIZED("trash") - 1,
                                &props.video[0].storage.identifier));

    storage_properties_init(&props.video[0].storage.settings,
                            0,
                            nullptr,
                            0,
                            nullptr,
                            0,
                            { .x = 1, .y = 1 });

    props.video[0].camera.settings.binning = 1;
    props.video[0].camera.settings.pixel_type = SampleType_u8;
    props.video[0].camera.settings.shape = { .x = 1920, .y = 1200 };
    props.video[0].camera.settings.exposure_time_us = 1e4;
    props.video[0].max_frame_count = 10;

    OK(acquire_configure(runtime, &props));

    AcquirePropertyMetadata meta = { 0 };
    OK(acquire_get_configuration_metadata(runtime, &meta));
}

void
acquire(AcquireRuntime* runtime)
{
    OK(acquire_start(runtime));
    OK(acquire_abort(runtime));
    LOG("OK");
}

int
main()
{
    auto runtime = acquire_init(reporter);
    try {
        setup(runtime);
        acquire(runtime);

        OK(acquire_shutdown(runtime));
    } catch (const std::exception& exc) {
        LOGE("Exception: %s", exc.what());
        return 1;
    } catch (...) {
        LOGE("Uncaught exception");
        return 1;
    }

    return 0;
}