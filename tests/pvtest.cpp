#include "master.h" // must come before pvcam.h
#include "pvcam.h"

#include <atomic>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <stdexcept>

#define PVCAM_INNER(e, action)                                                 \
    do {                                                                       \
        if (PV_FAIL == (e)) {                                                  \
            const int16 code = pl_error_code();                                \
            char msg[ERROR_MSG_LEN];                                           \
            pl_error_message(code, msg);                                       \
            std::cerr << "PVCAM error: " << msg << std::endl;                  \
            action;                                                            \
        }                                                                      \
    } while (0)

#define PVCAM(e)                                                               \
    PVCAM_INNER(e, throw std::runtime_error("PVCAM API call failed."))

uint8_t* buf;

struct CameraContext
{
    char model[CAM_NAME_LEN]{ '\0' };
    int16 hcam{ -1 };
    rgn_type region{ 0, 0, 0, 0, 0, 0 };
    uns32 exp_bytes;
    std::mutex api_mtx;
    std::mutex frame_mtx;
    std::atomic<ulong64> frames_acquired{ 0 };
    std::condition_variable cv;
    uint8_t** frame;
};

void PV_DECL
callback_handler(FRAME_INFO* frame_info, void* context)
{
    auto* ctx = (CameraContext*)context;
    {
        std::scoped_lock<std::mutex> lock(ctx->api_mtx);
        PVCAM(pl_exp_get_oldest_frame(ctx->hcam, (void**)(ctx->frame)));
        PVCAM(pl_exp_unlock_oldest_frame(ctx->hcam));
    }
    {
        std::unique_lock<std::mutex> lock(ctx->frame_mtx);
        std::cout << "callback_handler (" << (int)(++ctx->frames_acquired)
                  << " frames acquired)" << std::endl;
        ctx->cv.notify_one();
    }
}

void
open_camera(CameraContext* ctx)
{
    std::scoped_lock<std::mutex> lock(ctx->api_mtx);
    PVCAM(pl_cam_get_name(0, ctx->model));
    PVCAM(pl_cam_open(ctx->model, &ctx->hcam, OPEN_EXCLUSIVE));
}

void
close_camera(CameraContext* ctx)
{
    std::scoped_lock<std::mutex> lock(ctx->api_mtx);
    PVCAM(pl_cam_close(ctx->hcam));
}

void
start_camera(CameraContext* ctx)
{
    std::scoped_lock<std::mutex> lock(ctx->api_mtx);
    PVCAM(pl_cam_register_callback_ex3(
      ctx->hcam, PL_CALLBACK_EOF, (void*)callback_handler, (void*)ctx));
    PVCAM(pl_get_param(ctx->hcam, PARAM_ROI, ATTR_CURRENT, &ctx->region));
    PVCAM(pl_exp_setup_cont(ctx->hcam,
                            1,
                            &ctx->region,
                            TIMED_MODE,
                            10,
                            &ctx->exp_bytes,
                            CIRC_NO_OVERWRITE));

    ulong64 frame_buffer_size;
    PVCAM(pl_get_param(
      ctx->hcam, PARAM_FRAME_BUFFER_SIZE, ATTR_DEFAULT, &frame_buffer_size));

    buf = new uint8_t[frame_buffer_size];

    PVCAM(pl_exp_start_cont(ctx->hcam, buf, (uns32)frame_buffer_size));
    std::cout << "after start" << std::endl;
}

void
stop_camera(CameraContext* ctx)
{
    std::cout << "before stop" << std::endl;
    std::scoped_lock<std::mutex> lock(ctx->api_mtx);
    PVCAM(pl_exp_stop_cont(ctx->hcam, CCS_HALT));
}

int
main()
{
    PVCAM(pl_pvcam_init());

    CameraContext ctx{
        .frame = new uint8_t*(),
    };
    open_camera(&ctx);

    start_camera(&ctx);

    int16 status;
    uns32 bytes_arrived, buffer_cnt;
    PVCAM(
      pl_exp_check_cont_status(ctx.hcam, &status, &bytes_arrived, &buffer_cnt));

    std::unique_lock<std::mutex> lock(ctx.frame_mtx);
    ctx.cv.wait(lock, [&ctx] { return ctx.frames_acquired >= 100; });

    stop_camera(&ctx);

    close_camera(&ctx);

    PVCAM(pl_pvcam_uninit());

    std::cout << ctx.frames_acquired << " frames acquired" << std::endl;

    delete[] ctx.frame;
    delete[] buf;
}
