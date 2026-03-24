
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "rknn_api.h"

#include "yolov5_rknn.h"

#define OBJ_THRESH 0.25
#define NMS_THRESH 0.45
#define IMG_WIDTH 640
#define IMG_HEIGHT 640
#define NUM_CLASSES 80

using namespace std;
using namespace cv;

const char* class_names[NUM_CLASSES] = {
    "person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
    "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
    "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana",
    "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza",
    "donut", "cake", "chair", "sofa", "pottedplant", "bed", "diningtable",
    "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
    "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock",
    "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
};

struct Object {
    cv::Rect bbox;
    int label;
    float score;
};

float iou(const cv::Rect& a, const cv::Rect& b) {
    int xx1 = max(a.x, b.x);
    int yy1 = max(a.y, b.y);
    int xx2 = min(a.x + a.width, b.x + b.width);
    int yy2 = min(a.y + a.height, b.y + b.height);
    int w = max(0, xx2 - xx1);
    int h = max(0, yy2 - yy1);
    float inter = w * h;
    float union_area = a.area() + b.area() - inter;
    return inter / union_area;
}

void nms(std::vector<Object>& objects, float nms_thresh) {
    std::sort(objects.begin(), objects.end(),
              [](const Object& a, const Object& b) { return a.score > b.score; });
    std::vector<bool> removed(objects.size(), false);
    for (size_t i = 0; i < objects.size(); ++i) {
        if (removed[i]) continue;
        for (size_t j = i + 1; j < objects.size(); ++j) {
            if (removed[j]) continue;
            if (objects[i].label == objects[j].label && iou(objects[i].bbox, objects[j].bbox) > nms_thresh)
                removed[j] = true;
        }
    }
    std::vector<Object> keep;
    for (size_t i = 0; i < objects.size(); ++i)
        if (!removed[i]) keep.push_back(objects[i]);
    objects = std::move(keep);
}

std::vector<Object> decode_outputs(float* output, int output_size, float scale_w, float scale_h, int img_w, int img_h) {
    std::vector<Object> objects;
    int num_anchors = output_size / (NUM_CLASSES + 5);
    for (int i = 0; i < num_anchors; ++i) {
        float* ptr = output + i * (NUM_CLASSES + 5);
        float obj_conf = ptr[4];
        if (obj_conf < OBJ_THRESH) continue;

        int class_id = -1;
        float max_prob = 0;
        for (int j = 0; j < NUM_CLASSES; ++j) {
            if (ptr[5 + j] > max_prob) {
                max_prob = ptr[5 + j];
                class_id = j;
            }
        }

        float conf = obj_conf * max_prob;
        if (conf < OBJ_THRESH) continue;

        float cx = ptr[0], cy = ptr[1], w = ptr[2], h = ptr[3];
        int left = (cx - w / 2.f) * scale_w;
        int top = (cy - h / 2.f) * scale_h;
        int box_w = w * scale_w;
        int box_h = h * scale_h;

        Object obj;
        obj.bbox = cv::Rect(left, top, box_w, box_h) & cv::Rect(0, 0, img_w, img_h);
        obj.label = class_id;
        obj.score = conf;
        objects.push_back(obj);
    }
    return objects;
}

void draw_objects(cv::Mat& image, const std::vector<Object>& objects, cv::Mat& yolo_image) {
    for (const auto& obj : objects) {
        cv::rectangle(image, obj.bbox, cv::Scalar(0, 255, 0), 2);
        std::string label = cv::format("%s: %.2f", class_names[obj.label], obj.score);
        int baseLine = 0;
        cv::Size label_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        int x = obj.bbox.x;
        int y = obj.bbox.y - label_size.height < 0 ? 0 : obj.bbox.y - label_size.height;
        cv::rectangle(image, cv::Rect(x, y, label_size.width, label_size.height + baseLine), cv::Scalar(0, 255, 0), cv::FILLED);
        cv::putText(image, label, cv::Point(x, y + label_size.height), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    }
    yolo_image = image.clone();
    // cv::imshow("YOLOv5 Detection", image);
    // cv::waitKey(0);
}

std::vector<uint8_t> load_model(const std::string& filename) {
    // std::cout << "Loading model file: " << filename << std::endl;
    std::ifstream file(filename, std::ios::binary | std::ios::ate);
    size_t size = file.tellg();
    // std::cout << "Model file size: " << size << std::endl;
    std::vector<uint8_t> buffer(size);
    file.seekg(0, std::ios::beg);
    file.read((char*)buffer.data(), size);
    return buffer;
}

int yolov5_detect(const cv::Mat& roi_img, cv::Mat& yolo_img) {
    const char* model_path = "./yolov5/yolov5n.rknn";
    const char* image_path = "./snapshot_rgb_0.png";
    rknn_context ctx;

    // 1. 加载模型
    auto model = load_model(model_path);
    int ret = rknn_init(&ctx, model.data(), model.size(), 0, NULL);
    if (ret != RKNN_SUCC) {
        std::cerr << "rknn_init failed! ret=" << ret << std::endl;
        return -1;
    }

    // 2. 加载图片
    cv::Mat orig_img = roi_img.clone();
    if (orig_img.empty()) {
        std::cerr << "Image read failed!" << std::endl;
        return -1;
    }

    cv::Mat resized_img;
    cv::resize(orig_img, resized_img, cv::Size(IMG_WIDTH, IMG_HEIGHT));
    cv::cvtColor(resized_img, resized_img, cv::COLOR_BGR2RGB);

    // 3. 设置输入
    rknn_input inputs[1];
    memset(inputs, 0, sizeof(inputs));
    inputs[0].index = 0;
    inputs[0].type = RKNN_TENSOR_UINT8;
    inputs[0].size = IMG_WIDTH * IMG_HEIGHT * 3;
    inputs[0].fmt = RKNN_TENSOR_NHWC;
    inputs[0].buf = resized_img.data;

    ret = rknn_inputs_set(ctx, 1, inputs);
    if (ret != RKNN_SUCC) {
        std::cerr << "rknn_inputs_set failed! ret=" << ret << std::endl;
        return -1;
    }

    // 4. 推理
    ret = rknn_run(ctx, nullptr);
    if (ret != RKNN_SUCC) {
        std::cerr << "rknn_run failed! ret=" << ret << std::endl;
        return -1;
    }

    // 5. 获取输出
    rknn_input_output_num io_num;
    ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    if (ret != RKNN_SUCC) {
        std::cerr << "rknn_query failed! ret=" << ret << std::endl;
        return -1;
    }

    int output_num = io_num.n_output;
    rknn_output outputs[output_num];
    memset(outputs, 0, sizeof(outputs));
    for (int i = 0; i < output_num; ++i) {
        outputs[i].want_float = 1;
    }

    // std::cout << "Output size: " << outputs[0].size << std::endl;


    ret = rknn_outputs_get(ctx, output_num, outputs, nullptr);
    if (ret != RKNN_SUCC) {
        std::cerr << "rknn_outputs_get failed! ret=" << ret << std::endl;
        return -1;
    }

    // 6. 后处理
    float scale_w = (float)orig_img.cols / IMG_WIDTH;
    float scale_h = (float)orig_img.rows / IMG_HEIGHT;

    float* output_data = reinterpret_cast<float*>(outputs[0].buf);
    int output_size = outputs[0].size / sizeof(float);

    std::vector<Object> objs = decode_outputs(output_data, output_size, scale_w, scale_h, orig_img.cols, orig_img.rows);
    nms(objs, NMS_THRESH);
    draw_objects(orig_img, objs, yolo_img);

    // 7. 释放
    rknn_outputs_release(ctx, output_num, outputs);
    rknn_destroy(ctx);

    // std::cout << "Done." << std::endl;
    return 0;
}





// int main() {
//     const char* model_path = "./yolov5s.rknn";
//     const char* image_path = "./snapshot_rgb_0.png";
//     rknn_context ctx;

//     // 1. 加载模型
//     auto model = load_model(model_path);
//     int ret = rknn_init(&ctx, model.data(), model.size(), 0, NULL);
//     if (ret != RKNN_SUCC) {
//         std::cerr << "rknn_init failed! ret=" << ret << std::endl;
//         return -1;
//     }

//     // 2. 加载图片
//     cv::Mat orig_img = cv::imread(image_path);
//     if (orig_img.empty()) {
//         std::cerr << "Image read failed!" << std::endl;
//         return -1;
//     }

//     cv::Mat resized_img;
//     cv::resize(orig_img, resized_img, cv::Size(IMG_WIDTH, IMG_HEIGHT));
//     cv::cvtColor(resized_img, resized_img, cv::COLOR_BGR2RGB);

//     // 3. 设置输入
//     rknn_input inputs[1];
//     memset(inputs, 0, sizeof(inputs));
//     inputs[0].index = 0;
//     inputs[0].type = RKNN_TENSOR_UINT8;
//     inputs[0].size = IMG_WIDTH * IMG_HEIGHT * 3;
//     inputs[0].fmt = RKNN_TENSOR_NHWC;
//     inputs[0].buf = resized_img.data;

//     ret = rknn_inputs_set(ctx, 1, inputs);
//     if (ret != RKNN_SUCC) {
//         std::cerr << "rknn_inputs_set failed! ret=" << ret << std::endl;
//         return -1;
//     }

//     // 4. 推理
//     ret = rknn_run(ctx, nullptr);
//     if (ret != RKNN_SUCC) {
//         std::cerr << "rknn_run failed! ret=" << ret << std::endl;
//         return -1;
//     }

//     // 5. 获取输出
//     rknn_input_output_num io_num;
//     ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
//     if (ret != RKNN_SUCC) {
//         std::cerr << "rknn_query failed! ret=" << ret << std::endl;
//         return -1;
//     }

//     int output_num = io_num.n_output;
//     rknn_output outputs[output_num];
//     memset(outputs, 0, sizeof(outputs));
//     for (int i = 0; i < output_num; ++i) {
//         outputs[i].want_float = 1;
//     }

//     ret = rknn_outputs_get(ctx, output_num, outputs, nullptr);
//     if (ret != RKNN_SUCC) {
//         std::cerr << "rknn_outputs_get failed! ret=" << ret << std::endl;
//         return -1;
//     }

//     // 6. 后处理
//     float scale_w = (float)orig_img.cols / IMG_WIDTH;
//     float scale_h = (float)orig_img.rows / IMG_HEIGHT;

//     float* output_data = reinterpret_cast<float*>(outputs[0].buf);
//     int output_size = outputs[0].size / sizeof(float);

//     std::vector<Object> objs = decode_outputs(output_data, output_size, scale_w, scale_h, orig_img.cols, orig_img.rows);
//     nms(objs, NMS_THRESH);
//     // draw_objects(orig_img, objs);

//     // 7. 释放
//     rknn_outputs_release(ctx, output_num, outputs);
//     rknn_destroy(ctx);

//     std::cout << "Done." << std::endl;
//     return 0;
// }