#include <math.h>
#include <stdio.h>


// #include "heimann_drv.h"
#include "heimann_inter.h"

static const float scale_x = 1. / 9.;
static const float scale_y = 1. / 9.;

// float min(float a, float b){
//     return a < b ? a : b;
// }

// x, y : 0 ~ 30 | 0 ~ 32
// 数组中的位置 x, y : 0 ~ 30 | 0 ~ 32
int getValue(int y, int x, unsigned short *datas){
    // printf("x=%d, y=%d\n",x,y);
    return (int)datas[x*SRC_H+y];
}
// inline float getValue(int y, int x, float *datas){
//     printf("x=%d, y=%d, data=%d\n",x,y,x + (23 - y) * SRC_W);
//     return datas[x + (23 - y) * SRC_W];
// }

int min(int a, int b){
    return a < b ? a : b;
}

int max(int a, int b){
    return a > b ? a : b;
}

int bio_linear_interpolation(int dst_x, int dst_y, unsigned short *src_data)
{
    int src_x, src_y;
    int src_x0, src_y0, src_x1, src_y1;
    int value00, value01, value10, value11, v0, v1, frac_x, frac_y;
    int res=0;
            
    // 目标在源数据上的坐标, 当作保留一位小数处理
    src_x = (dst_x*1024 + 512) / 8 - 512;
    src_y = (dst_y*1024 + 512) / 8 - 512;

    // 找到四个最近邻点的位置
    src_x0 = src_x / 1024;
    src_y0 = src_y / 1024;
    src_x1 = src_x0+1;
    src_y1 = src_y0+1;

    // 确保不超出源图像边界
    src_x1 = min(src_x1, SRC_W-1);
    src_y1 = min(src_y1, SRC_H-1);
    src_x1 = max(src_x1, 0);
    src_y1 = max(src_y1, 0);

    // 计算分数部分
    frac_x = src_x - src_x0 * 1024;
    frac_y = src_y - src_y0 * 1024;

    // 获取四个最近邻点的值
    value00 = getValue(src_y0, src_x0, src_data);
    value01 = getValue(src_y0, src_x1, src_data);
    value10 = getValue(src_y1, src_x0, src_data);
    value11 = getValue(src_y1, src_x1, src_data);
    // Serial.printf("%d, %d, %d, %d, %.2f\n", src_x0, src_y0, src_x1, src_y1, frac_x);
    // 沿x轴的线性插值
    v0 = value00 * (1024 - frac_x) + value01 * frac_x;
    v1 = value10 * (1024 - frac_x) + value11 * frac_x;

    v0 /= 1024;
    v1 /= 1024;
    // 沿y轴的线性插值
    res = (v0 * (1024 - frac_y) + v1 * frac_y) / 1024;
    res = min(res, 179);
    res = max(res, 0);

    return res;
    // return (v0 * (1024 - frac_y) + v1 * frac_y) / 1024;
}