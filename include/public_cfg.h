#ifndef _PUBLIC_CFG_H
#define _PUBLIC_CFG_H


#include <pthread.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>

#define TH_THERMAL_ROWS 32
#define TH_THERMAL_COLS 32
#define TH_MIX_WIDTH 640
#define TH_MIX_HEIGHT 360
#define TH_YUV_FRAME_SIZE (TH_MIX_WIDTH * TH_MIX_HEIGHT *3/2)

typedef struct {
    uint16_t thermal_data[TH_THERMAL_ROWS][TH_THERMAL_COLS];
    pthread_mutex_t mutex; //互斥量
    pthread_cond_t cond;   //条件变量
    int updated;
} thermal_buffer_t;

typedef struct {
    uint8_t *yuv_data;//[TH_YUV_FRAME_SIZE];
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    int updated;
} yuv_buffer_t;

typedef struct {
    uint16_t fusion_data[256][256];
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    int updated;
} fusion_buffer_t;

typedef struct {
    int argc;
    char **argv;
} thread_args_t;

typedef struct {
    volatile sig_atomic_t snapshot_request;
    volatile sig_atomic_t print_eeprom_header_req;
    volatile sig_atomic_t print_eeprom_hex_req;
    volatile sig_atomic_t exit_req;
    volatile sig_atomic_t colormap_ctrl;
    volatile sig_atomic_t yolo_req;
    volatile sig_atomic_t edge_req;

} cmd_request_t;

typedef struct {
    thread_args_t thread_args;
    thermal_buffer_t thermal_buf;
    yuv_buffer_t yuv_buf;
    fusion_buffer_t fusion_buf;
    cmd_request_t cmd_req;
} thread_context_t;

#endif