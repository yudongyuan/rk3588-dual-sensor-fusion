#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <pthread.h>
#include <readline/readline.h>
#include <readline/history.h>

#include "heimann_drv.h"
#include "mix415_drv.h"
#include "public_cfg.h"
#include "opencv_draw.h"
#include "websocket_server.h"
#include "cmd_shell.h"

int main(int argc, char *argv[]) 
{
    thread_context_t *ctx = malloc(sizeof(thread_context_t));
    ctx->thread_args.argc = argc;
    ctx->thread_args.argv = argv;

    pthread_t therm_tid, cam_tid, opencv_tid, cmd_tid, ws_tid;
    // thread_args_t *args = malloc(sizeof(thread_args_t));
    // args->argc = argc;
    // args->argv = argv;
    ctx->cmd_req.exit_req = 0;
    ctx->cmd_req.colormap_ctrl = COLORMAP_TURBO;

    if (pthread_create(&therm_tid, NULL, thermal_thread, ctx) != 0) {
        perror("Failed to create thermal_thread");
        return -1;
    }

    if (pthread_create(&cam_tid, NULL, camera_thread, ctx) != 0) {
        perror("Failed to create camera_thread");
        return -1;
    }

    if (pthread_create(&opencv_tid, NULL, opencv_thread, ctx) != 0) {
        perror("Failed to create opencv_thread");
        return -1;
    }

    if (pthread_create(&ws_tid, NULL, websocket_thread, ctx) != 0) {
        perror("Failed to create camera_thread");
        return -1;
    }

    if (pthread_create(&cmd_tid, NULL, cmd_thread, ctx) != 0) {
        perror("Failed to create camera_thread");
        return -1;
    }
    


    pthread_join(therm_tid, NULL);
    pthread_join(cam_tid, NULL);
    pthread_join(opencv_tid, NULL);
    pthread_join(ws_tid, NULL);
    pthread_join(cmd_tid, NULL);

    pthread_mutex_destroy(&ctx->fusion_buf.mutex);
    pthread_cond_destroy(&ctx->fusion_buf.cond);

    free(ctx);  // 线程结束后释放内存
    printf("main:资源释放\n");

    return 0;
}
