#ifndef __OPENCV_DRAW_H
#define __OPENCV_DRAW_H



#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "heimann_inter.h"
#include "heimann_drv.h"
#include "colormap.h"

#include "mix415_drv.h"
#include "public_cfg.h"


#define WINDOW_WIDTH 1024
#define WINDOW_HEIGHT 640
#define THERMAL_COLS 32
#define THERMAL_ROWS 30
#define BLOCK_SIZE 2
#define PROB_SCALE 8

#define IMAGE_WIDTH (THERMAL_COLS * PROB_SCALE * BLOCK_SIZE)
#define IMAGE_HEIGHT (THERMAL_ROWS * PROB_SCALE * BLOCK_SIZE)
#define OFFSET_X ((WINDOW_WIDTH - IMAGE_WIDTH) / 2)
#define OFFSET_Y ((WINDOW_HEIGHT - IMAGE_HEIGHT) / 2)

int cv_show_heimann_classic(const uint16_t* data_pixel);
void draw_roi_frame(const uint8_t* y_plane);
void* opencv_thread(void *arg);

#ifdef __cplusplus
}
#endif


#endif
