#ifndef _YOLOV5_RKNN_H
#define _YOLOV5_RKNN_H

#include <opencv2/opencv.hpp>

int yolov5_detect(const cv::Mat& roi_img, cv::Mat& yolo_img);

#endif
