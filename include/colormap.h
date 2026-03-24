#ifndef __COLORMAP_H
#define __COLORMAP_H

#include "heimann_drv.h"

void get_rgb888_from_rgb565(uint16_t val, uint8_t* r8, uint8_t* g8, uint8_t* b8);
uint16_t load_colormap(uint8_t colormap, uint8_t index);


#endif