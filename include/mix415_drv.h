#ifndef __MIX415_DRV_H
#define __MIX415_DRV_H

#define CAM_DEVICE "/dev/video11"
#define MIX_WIDTH 640
#define MIX_HEIGHT 360

#define ROI_X 0
#define ROI_Y 0
#define ROI_W 640
#define ROI_H 360

void* camera_thread(void *arg);

#endif // 