#ifndef __HEIMANN_INTER_H
#define __HEIMANN_INTER_H


#define SRC_W 30
#define SRC_H 32

#define DST_W SRC_W * 8
#define DST_H SRC_H * 8

// float bio_linear_interpolation(int dst_x, int dst_y, float *src_data);
int bio_linear_interpolation(int dst_x, int dst_y, unsigned short *src_data);


#endif