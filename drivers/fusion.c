#include "fusion.h"
#include "public_cfg.h"



void* fusion_thread(void* arg) {
    thread_context_t* ctx = (thread_context_t*)arg;
    // int argc = ctx->thread_args.argc;
    // char **argv = ctx->thread_args.argv;


    while (1) {
        pthread_mutex_lock(&ctx->thermal_buf.mutex);
        while (!ctx->thermal_buf.updated)
            pthread_cond_wait(&ctx->thermal_buf.cond, &ctx->thermal_buf.mutex);
        pthread_mutex_unlock(&ctx->thermal_buf.mutex);

        pthread_mutex_lock(&ctx->yuv_buf.mutex);
        while (!ctx->yuv_buf.updated)
            pthread_cond_wait(&ctx->yuv_buf.cond, &ctx->yuv_buf.mutex);
        pthread_mutex_unlock(&ctx->yuv_buf.mutex);

        pthread_mutex_lock(&ctx->fusion_buf.mutex);

        //对两份数据进行处理，
        // memcpy(ctx->fusion_buf.fused_thermal, ctx->thermal_buf.thermal_data, sizeof(ctx->fusion_buf.fused_thermal));
        // memcpy(ctx->fusion_buf.fused_yuv, ctx->yuv_buf.yuv_data, sizeof(ctx->fusion_buf.fused_yuv));
        ctx->fusion_buf.updated = 1;
        pthread_cond_signal(&ctx->fusion_buf.cond);
        pthread_mutex_unlock(&ctx->fusion_buf.mutex);
    }
    return NULL;
}
