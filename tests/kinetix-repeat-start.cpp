#include "acquire.h"
#include "device/hal/device.manager.h"
#include "platform.h"
#include "logger.h"

#include <algorithm> // std::sort
#include <cstdio>
#include <exception>
#include <stdexcept>
#include <vector>

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

static void
reporter(int is_error,
         const char* file,
         int line,
         const char* function,
         const char* msg)
{
    printf("%s%s(%d) - %s: %s\n",
           is_error ? "ERROR " : "",
           file,
           line,
           function,
           msg);
}

void
setup(AcquireRuntime* runtime)
{
    CHECK(runtime);

    const auto* dm = acquire_device_manager(runtime);
    CHECK(dm);

    AcquireProperties props = {};
    OK(acquire_get_configuration(runtime, &props));

    DEVOK(device_manager_select(dm,
                                DeviceKind_Camera,
                                SIZED(".*Kinetix.*") - 1,
                                &props.video[0].camera.identifier));
    DEVOK(device_manager_select(dm,
                                DeviceKind_Storage,
                                SIZED("Trash") - 1,
                                &props.video[0].storage.identifier));

    props.video[0].max_frame_count = 10;
    OK(acquire_configure(runtime, &props));
}

double
acquire(AcquireRuntime* runtime)
{
    struct clock clock = {};
    clock_init(&clock);
    OK(acquire_start(runtime));
    OK(acquire_stop(runtime));

    return clock_toc_ms(&clock);
}

int
main()
{
    auto* runtime = acquire_init(reporter);
    try {
        setup(runtime);

        std::vector<double> run_times;
        for (auto i = 0; i < 10; ++i) {
            run_times.push_back(acquire(runtime));
        }
        OK(acquire_shutdown(runtime));

        std::sort(run_times.begin(), run_times.end());
        LOG("Start/stop cycle times: %f ms (min); %f ms (median); %f ms (max)",
            run_times.at(0),
            run_times.at(run_times.size() / 2),
            run_times.at(run_times.size() - 1));
        LOG("OK");
        return 0;
    } catch (const std::exception& e) {
        ERR("Exception: %s", e.what());
    } catch (...) {
        ERR("Exception: (unknown)");
    }

    acquire_shutdown(runtime);
    return 1;
}
