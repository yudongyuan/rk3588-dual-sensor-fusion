#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <time.h>
#include <sys/time.h>

#include "mix415_drv.h"
#include "opencv_draw.h"
#include "public_cfg.h"

#define TARGET_FPS 30
#define MAX_DEQUEUE_FAIL 10
#define BUFFER_COUNT 8 // 调大缓冲区数量，避免丢帧

struct buffer {
    void *start;
    size_t length;
};

static struct buffer *buffers;

void* camera_thread(void *arg) {
    thread_context_t* ctx = (thread_context_t*)arg;
    int argc = ctx->thread_args.argc;
    char **argv = ctx->thread_args.argv;

    const char *device = (argc == 4) ? argv[3] : CAM_DEVICE;
    // int fd = open(device, O_RDWR | O_NONBLOCK);
    int fd = open(device, O_RDWR);
    if (fd == -1) {
        perror("Opening video device failed!");
        return (void*)-1;
    }

    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    fmt.fmt.pix_mp.width = MIX_WIDTH;
    fmt.fmt.pix_mp.height = MIX_HEIGHT;
    fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
    fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;

    if (ioctl(fd, VIDIOC_S_FMT, &fmt) == -1) {
        perror("Setting Pixel Format");
        close(fd);
        return (void*)-1;
    }

    struct v4l2_requestbuffers req = {0};
    req.count = BUFFER_COUNT;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd, VIDIOC_REQBUFS, &req) == -1) {
        perror("Requesting Buffers");
        close(fd);
        return (void*)-1;
    }

    buffers = calloc(req.count, sizeof(*buffers));
    for (int i = 0; i < req.count; i++) {
        struct v4l2_buffer buf;
        struct v4l2_plane planes[VIDEO_MAX_PLANES];
        memset(&buf, 0, sizeof(buf));
        memset(planes, 0, sizeof(planes));

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        buf.length = VIDEO_MAX_PLANES;
        buf.m.planes = planes;

        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) == -1) {
            perror("Querying Buffer");
            close(fd);
            return (void*)-1;
        }

        buffers[i].length = buf.m.planes[0].length;
        buffers[i].start = mmap(NULL, buf.m.planes[0].length,
                                PROT_READ | PROT_WRITE, MAP_SHARED,
                                fd, buf.m.planes[0].m.mem_offset);
    }

    for (int i = 0; i < req.count; i++) {
        struct v4l2_buffer buf;
        struct v4l2_plane planes[VIDEO_MAX_PLANES];
        memset(&buf, 0, sizeof(buf));
        memset(planes, 0, sizeof(planes));

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        buf.length = VIDEO_MAX_PLANES;
        buf.m.planes = planes;

        if (ioctl(fd, VIDIOC_QBUF, &buf) == -1) {
            perror("Queue Buffer");
            close(fd);
            return (void*)-1;
        }
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) == -1) {
        perror("Stream On");
        close(fd);
        return (void*)-1;
    }

    int dequeue_fail_count = 0;

    size_t frame_size = MIX_WIDTH * MIX_HEIGHT * 3 / 2; // NV12大小
    uint8_t *local_frame_buffer = malloc(frame_size); // 本地拷贝缓冲区

    while (!ctx->cmd_req.exit_req) {

        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd, &fds);

        struct timeval tv;
        tv.tv_sec = 2;
        tv.tv_usec = 0;

        int r = select(fd + 1, &fds, NULL, NULL, &tv);
        if (r == -1) {
            if (errno == EINTR)
                continue;
            perror("Select failed");
            break;
        }
        else if (r == 0) {
            fprintf(stderr, "Camera select timeout!\n");
            continue;
        }

        struct v4l2_buffer buf;
        struct v4l2_plane planes[VIDEO_MAX_PLANES];
        memset(&buf, 0, sizeof(buf));
        memset(planes, 0, sizeof(planes));

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.length = VIDEO_MAX_PLANES;
        buf.m.planes = planes;

        if (ioctl(fd, VIDIOC_DQBUF, &buf) == -1) {
            // perror("Dequeue Buffer failed");
            // if (++dequeue_fail_count > MAX_DEQUEUE_FAIL) {
            //     fprintf(stderr, "Dequeue failed too many times, exiting...\n");
            //     break;
            // }
            // continue; //对 V4L2 设备做非阻塞打开后，如果这时 buffer 还没准备好，ioctl(fd, VIDIOC_DQBUF, &buf) 就会立刻返回 -EAGAIN continue 直接跳回去，不会做任何延时，导致 tight‐loop。
            perror("Dequeue Buffer failed");
            break;  // 阻塞模式下一般不会走到这里，直接退出更安全
        }
        dequeue_fail_count = 0;

        pthread_mutex_lock(&ctx->yuv_buf.mutex);

        memcpy(local_frame_buffer, buffers[buf.index].start, frame_size); // 拷贝一份，避免撕裂
        ctx->yuv_buf.yuv_data = local_frame_buffer; 
        ctx->yuv_buf.updated = 1;

        pthread_cond_signal(&ctx->yuv_buf.cond);
        pthread_mutex_unlock(&ctx->yuv_buf.mutex);

        // 重新入队
        if (ioctl(fd, VIDIOC_QBUF, &buf) == -1) {
            perror("Requeue Buffer failed");
            break;
        }

        usleep(100);
    }
    
    for (int i = 0; i < req.count; i++) {
        munmap(buffers[i].start, buffers[i].length);
    }
    free(buffers);
    free(local_frame_buffer);

    ioctl(fd, VIDIOC_STREAMOFF, &type);
    close(fd);

    return NULL;
}
