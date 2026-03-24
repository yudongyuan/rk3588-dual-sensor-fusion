#ifndef __HEIMANN_DRV_H
#define __HEIMANN_DRV_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <math.h>

#include "heimann_reg.h"

#define I2C_BUFFER_LENGTH 300
// STRUCT WITH ALL SENSOR CHARACTERISTICS
typedef struct {
    uint16_t NumberOfPixel;
    uint8_t NumberOfBlocks;
    uint8_t RowPerBlock;
    uint16_t PixelPerBlock;
    uint16_t PixelPerColumn;
    uint16_t PixelPerRow;
    uint8_t AllowedDeadPix;
    uint16_t TableNumber;
    uint16_t TableOffset;
    uint8_t PTATPos;
    uint8_t VDDPos;
    uint8_t PTATVDDSwitch;
    uint8_t CyclopsActive;
    uint8_t CyclopsPos;
    uint8_t DataPos;
} mlx_characteristics_t;

enum  {
    PROB_CONNECTING,
    PROB_INITIALIZING,
    PROB_PREPARING,
    PROB_READY
}; // 探头目前的启动状态码

enum  {
    COLORMAP_CLASSIC,
    COLORMAP_TURBO,
    COLORMAP_HOT,
    COLORMAP_VIRIDIS,
    COLORMAP_INFERNO,
    COLORMAP_GRAYSR, // 白热
    COLORMAP_GRAYS   // 黑热
}; // 颜色映射表类型

extern uint16_t T_max, T_min;// 温度
extern uint32_t  T_avg; // 温度平均值
// int sensor_init(int sensor_fd, int eeprom_fd);
// int sensor_main(int argc,char *argv[]);
void* thermal_thread(void *arg);





#endif