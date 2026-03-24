#include "websocket_server.h"
#include <libwebsockets.h>
#include <pthread.h>
#include <opencv2/opencv.hpp>
#include <string.h>
#include <stdlib.h>
#include "public_cfg.h"

#define MAX_FRAME_SIZE (1024 * 1024)  // 1MB 图像缓冲区
static uint8_t image_buffer[MAX_FRAME_SIZE];
static size_t image_size = 0;
static pthread_mutex_t buffer_mutex = PTHREAD_MUTEX_INITIALIZER;

static struct lws *global_wsi = NULL;
static struct lws_context *context;

// base64编码函数（用你已有的或库）
// extern int base64_encode(const uint8_t *src, size_t srclen, char *dst, size_t dstlen);

static const char base64_chars[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

int base64_encode(const unsigned char* input, unsigned long input_len, char* output, unsigned long output_len) {
    int i = 0, j = 0;
    unsigned char char_array_3[3], char_array_4[4];
    unsigned long pos = 0;

    while (input_len--) {
        char_array_3[i++] = *(input++);
        if (i == 3) {
            char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
            char_array_4[1] = ((char_array_3[0] & 0x03) << 4) +
                              ((char_array_3[1] & 0xf0) >> 4);
            char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) +
                              ((char_array_3[2] & 0xc0) >> 6);
            char_array_4[3] = char_array_3[2] & 0x3f;

            for (i = 0; i < 4 && pos < output_len; i++) {
                output[pos++] = base64_chars[char_array_4[i]];
            }
            i = 0;
        }
    }

    if (i) {
        for (j = i; j < 3; j++) char_array_3[j] = '\0';

        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] = ((char_array_3[0] & 0x03) << 4) +
                          ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) +
                          ((char_array_3[2] & 0xc0) >> 6);
        char_array_4[3] = char_array_3[2] & 0x3f;

        for (j = 0; j < i + 1 && pos < output_len; j++)
            output[pos++] = base64_chars[char_array_4[j]];

        while ((i++ < 3) && pos < output_len)
            output[pos++] = '=';
    }

    if (pos < output_len) output[pos] = '\0';

    return pos;
}


enum protocols {
    PROTOCOL_IMAGE = 0,
    PROTOCOL_COUNT
};

enum  {
    COLORMAP_CLASSIC,
    COLORMAP_TURBO,
    COLORMAP_HOT,
    COLORMAP_VIRIDIS,
    COLORMAP_INFERNO,
    COLORMAP_GRAYSR, // 白热
    COLORMAP_GRAYS   // 黑热
}; // 颜色映射表类型

static thread_context_t* ctx = NULL;

static int callback_image(struct lws *wsi, enum lws_callback_reasons reason,
                          void *user, void *in, size_t len)
{
    // // 注意：user 是指向 per_session_data 的指针，这里定义为 ctx 指针的指针
    // thread_context_t **pctx = (thread_context_t **)user;
    switch (reason) {
    case LWS_CALLBACK_ESTABLISHED:
        global_wsi = wsi;
        *((thread_context_t **)user) = ctx; // 把当前线程的 ctx 保存到 user 中
        break;
    case LWS_CALLBACK_SERVER_WRITEABLE:
        pthread_mutex_lock(&buffer_mutex);
        if (image_size > 0) {
            // base64编码图像数据为文本
            static char base64_out[MAX_FRAME_SIZE * 2];
            int out_len = base64_encode(image_buffer, image_size, base64_out, sizeof(base64_out));
            if (out_len > 0) {
                unsigned char* buf = (unsigned char*)malloc(LWS_PRE + out_len);
                memcpy(buf + LWS_PRE, base64_out, out_len);
                lws_write(wsi, buf + LWS_PRE, out_len, LWS_WRITE_TEXT);
                free(buf);
            }
            image_size = 0;
        }
        pthread_mutex_unlock(&buffer_mutex);
        break;
    case LWS_CALLBACK_RECEIVE:
        ((char *)in)[len] = '\0'; // 确保消息以 null 结尾
        printf("Received: %s\n", (char *)in);
    
        if (strcmp((char *)in, "toggle_yolo") == 0) {
            if(ctx->cmd_req.yolo_req == 1) ctx->cmd_req.yolo_req = 0;
            else if(ctx->cmd_req.yolo_req == 0) ctx->cmd_req.yolo_req = 1;
        } else if (strcmp((char *)in, "toggle_edge") == 0) {
            if(ctx->cmd_req.edge_req == 1) ctx->cmd_req.edge_req = 0;
            else if(ctx->cmd_req.edge_req == 0) ctx->cmd_req.edge_req = 1;
        } else if (strcmp((char *)in, "colormap_next") == 0) {
            if(ctx->cmd_req.colormap_ctrl == COLORMAP_GRAYS){
                ctx->cmd_req.colormap_ctrl = COLORMAP_CLASSIC;
            } else {
                ctx->cmd_req.colormap_ctrl += 1;
            }
            
        }
        break;
    
    default:
        break;
    }
    return 0;
}

static struct lws_protocols protocols[] = {
    {
        .name = "image-protocol",
        .callback = callback_image,
        .per_session_data_size = sizeof(thread_context_t*), // 传 ctx 指针,
        .rx_buffer_size = 0,
    },
    { NULL, NULL, 0, 0 } // terminator
};


void* websocket_thread(void* arg)
{
    // thread_context_t* ctx = (thread_context_t*)arg;
    ctx = (thread_context_t*)arg;
    int argc = ctx->thread_args.argc;
    char **argv = ctx->thread_args.argv;

    struct lws_context_creation_info info;
    memset(&info, 0, sizeof(info));
    info.port = 9002;
    info.protocols = protocols;

    context = lws_create_context(&info);
    if (!context) {
        fprintf(stderr, "lws init failed\n");
        return NULL;
    }

    while (!ctx->cmd_req.exit_req) {
        lws_service(context, 50);
        if (global_wsi) {
            lws_callback_on_writable(global_wsi);
        }
    }

    pthread_mutex_destroy(&buffer_mutex);
    lws_context_destroy(context);
    
    return NULL;
}

void send_fusion_frame(const cv::Mat& fusion_img)
{
    std::vector<uchar> buf;
    cv::imencode(".jpg", fusion_img, buf);

    pthread_mutex_lock(&buffer_mutex);
    if (buf.size() < MAX_FRAME_SIZE) {
        memcpy(image_buffer, buf.data(), buf.size());
        image_size = buf.size();
    }
    pthread_mutex_unlock(&buffer_mutex);
}
