#include "opencv2/opencv.hpp"

#include "opencv_draw.h"
#include "yolov5_rknn.h"
#include "websocket_server.h"

using namespace cv;
using namespace std;

extern void send_fusion_frame(const cv::Mat& fusion_img);

uint8_t colormap = 1;
uint8_t yolo_flag = 0, edge_flag = 0;

struct AdjustParams {
    float shift_x;  // X方向微调（像素）
    float shift_y;  // Y方向微调（像素）
    float scale;    // 缩放（比例，比如1.0不变，1.01放大1%）
    float angle;    // 旋转角度（单位度°）
    AdjustParams() : shift_x(0), shift_y(0), scale(1.0), angle(0.0) {}
};

// 进行图像配准并返回配准后的热成像图
Mat register_thermal_to_visible(const Mat& thermal_img, const AdjustParams& adjust = AdjustParams()) {
    // 热成像的4个点
    vector<Point2f> pts_thermal = {
        Point2f(50, 73),
        Point2f(232, 81),
        Point2f(46, 178),
        Point2f(229, 176)
    };

    // 可见光ROI图像中的4个对应点
    vector<Point2f> pts_visible = {
        Point2f(25, 45),
        Point2f(535, 22),
        Point2f(27, 333),
        Point2f(526, 335)
    };

    // 计算单应矩阵
    Mat H = findHomography(pts_thermal, pts_visible);

    // 初步配准
    Mat registered_img;
    warpPerspective(thermal_img, registered_img, H, Size(ROI_W, ROI_H));

    // ========== 追加微调 ==========
    if (adjust.shift_x != 0 || adjust.shift_y != 0 || adjust.scale != 1.0 || adjust.angle != 0.0) {
        // 以图像中心为基准
        Point2f center(registered_img.cols / 2.0f, registered_img.rows / 2.0f);

        // 生成仿射矩阵（旋转+缩放）
        Mat M = getRotationMatrix2D(center, adjust.angle, adjust.scale);

        // 加上平移
        M.at<double>(0, 2) += adjust.shift_x;
        M.at<double>(1, 2) += adjust.shift_y;

        // 应用微调
        warpAffine(registered_img, registered_img, M, registered_img.size());
    }

    return registered_img;
}

// 使用sobel提取边缘
Mat extract_edges_sobel(const Mat& roi_img) {
    Mat gray, blurred, grad_x, grad_y, abs_grad_x, abs_grad_y, edge_map, edge_colored, result;

    // 1. 转换为灰度图
    cvtColor(roi_img, gray, COLOR_BGR2GRAY);

    // 2. 可选：高斯滤波去噪
    GaussianBlur(gray, blurred, Size(3, 3), 1.5);

    // 3. 计算 Sobel 梯度
    Sobel(blurred, grad_x, CV_16S, 1, 0, 3); // x 方向梯度
    Sobel(blurred, grad_y, CV_16S, 0, 1, 3); // y 方向梯度

    // 4. 计算梯度幅值（取绝对值并转换到 8-bit）
    convertScaleAbs(grad_x, abs_grad_x);
    convertScaleAbs(grad_y, abs_grad_y);
    addWeighted(abs_grad_x, 1, abs_grad_y, 1, 0, edge_map);  //组合梯度,可以调整边缘强度

    bitwise_not(edge_map, edge_map);

    cvtColor(edge_map, edge_colored, COLOR_GRAY2BGR);
    // addWeighted(roi_img, 0.3, edge_colored, 0.7, 0, result);

    return edge_colored;
}

Mat estimate_intrinsic_matrix(int w, int h, double fov_deg) {
    double fov_rad = fov_deg * CV_PI / 180.0;
    double fx = w / (2.0 * tan(fov_rad / 2.0));
    double fy = fx;
    double cx = w / 2.0;
    double cy = h / 2.0;
    return (Mat_<double>(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
}

void correct_image(const Mat& input, Mat& output, const Mat& K_src, const Mat& K_dst) {
    // Mat map1, map2;
    // initUndistortRectifyMap(K_src, Mat::zeros(5,1,CV_64F),
    //                         Mat(), K_dst, src.size(), CV_32FC1, map1, map2);
    // Mat dst;
    // remap(src, dst, map1, map2, INTER_LINEAR);  //似乎有问题
    // return dst;
    Mat D_src = Mat::zeros(5,1,CV_64F);  // 假设无畸变
    undistort(input, output, K_src, D_src, K_dst);
}

// int w_th = 640, h_th = 360;
double fov_th = 94.0, fov_vis = 119.8;
Mat K_th, K_vis; 

int cv_show_fusion_display(const uint16_t* thermal_pixel, const uint8_t* yuv_data, Mat& out_bgr_img, Mat& out_thermal_img) {
    
    //--------STEP1: 处理摄像头图像 YUV → BGR -------------//
    // Mat cam_yuv_img(MIX_HEIGHT * 3 / 2, MIX_WIDTH, CV_8UC1, (void*)yuv_data);  // 假设 NV12 格式
    // Mat cam_bgr_img;
    // cvtColor(cam_yuv_img, cam_bgr_img, COLOR_YUV2BGR_NV12);       // 转换成 BGR 格式


    //cvtColor 是 OpenCV 中一个计算密集型函数，对于 NV12 → BGR 转换，它会对每个像素进行 YUV 到 RGB 的颜色空间变换，涉及浮点矩阵变换和插值操作；
    //static  全局静态对象或线程静态对象，只分配一次，避免内存频繁申请释放和Cache miss 增多
    static Mat cam_yuv_img(MIX_HEIGHT * 3 / 2, MIX_WIDTH, CV_8UC1);  // 固定内存
    static Mat cam_bgr_img(MIX_HEIGHT, MIX_WIDTH, CV_8UC3);          // 输出图像
    memcpy(cam_yuv_img.data, yuv_data, MIX_WIDTH * MIX_HEIGHT * 3 / 2);
    cvtColor(cam_yuv_img, cam_bgr_img, COLOR_YUV2BGR_NV12);



    //--------STEP2: 处理热成像 -------------//
    unsigned short draw_pixel[32][32] = {{0}};
    int temp_inter = 0;
    const int disp_rows = 32 * PROB_SCALE;  // 128
    const int disp_cols = 32 * PROB_SCALE;  // 128
    
    static Mat thermal_ori_img(32, 32, CV_8UC1, Scalar(0));
    static Mat thermal_gauss_img, thermal_lanczos_img;
    static Mat thermal_color_img(disp_rows, disp_cols, CV_8UC3, Scalar(0, 0, 0));
    static Mat lanczos_img(32, 32, CV_8UC1, Scalar(0));

    for (int i = 0; i < 32; i++) {
        for (int j = 0; j < 32; j++) {
            int val = (180.0 * (thermal_pixel[i * THERMAL_COLS + j] - T_min) / (T_max - T_min));
            val = (val < 0) ? 0 : (val > 179 ? 179 : val);
            draw_pixel[i][j] = val;
            thermal_ori_img.at<uchar>(j, i) = draw_pixel[i][j];
            // printf("thermal_pixel[%d][%d]=%d; ",i, j, thermal_pixel[i * THERMAL_COLS + j]);
            // printf("T_min=%d, T_max=%d; ", T_min, T_max);
            // printf("draw_pixel=%d\n", draw_pixel[i][j]);
            
        }
    }

    //高斯降噪
    // GaussianBlur(thermal_ori_img, thermal_gauss_img, Size(3, 3), 0);
    blur(thermal_ori_img, thermal_gauss_img, Size(3, 3));

    //LANCZOS插值
    resize(thermal_gauss_img, thermal_lanczos_img, Size(256, 256), 0, 0, INTER_LANCZOS4); 

    //色彩映射
    for (int y = 0; y < disp_rows; y++) {
        for (int x = 0; x < disp_cols; x++) {
            temp_inter = thermal_lanczos_img.at<uchar>(y, x);
            // printf("temp_inter=[%d][%d]=%d\n", y, x, temp_inter);
            uint8_t r, g, b;
            // get_rgb888_from_rgb565(turbo[temp_inter], &r, &g, &b);
            get_rgb888_from_rgb565(load_colormap(colormap, temp_inter), &r, &g, &b);
            thermal_color_img.at<Vec3b>(y, x) = Vec3b(b, g, r);
        }
    }
 
    float ft_point = (float)(thermal_pixel[16*32+16] / 10) - 273.15;
    // printf("ft_point[16][16]=%.2f\n", ft_point);

        // 计算图像中心点
    int center_x = disp_cols / 2;
    int center_y = disp_rows / 2;

    // 绘制十字（水平线和垂直线）
    line(thermal_color_img, Point(center_x - 10, center_y), Point(center_x + 10, center_y), Scalar(0, 255, 255), 1); // 黄线
    line(thermal_color_img, Point(center_x, center_y - 10), Point(center_x, center_y + 10), Scalar(0, 255, 255), 1);

    // 构造温度字符串，保留一位小数
    char temp_text[32];
    snprintf(temp_text, sizeof(temp_text), "%.1fC", ft_point);

    // 设置文本显示位置（可以根据实际效果调整）
    Point text_pos(center_x + 15, center_y - 10);

    // 显示文本
    putText(thermal_color_img, temp_text, text_pos, FONT_HERSHEY_SIMPLEX, 0.3, Scalar(255, 255, 255), 1);

    //--------STEP3: 处理cam畸变 -------------//
    static Mat cam_corrected_img;

    // correct_image(cam_bgr_img, cam_corrected_img, K_vis, K_th);
    undistort(cam_bgr_img, cam_corrected_img, K_vis, Mat::zeros(5,1,CV_64F), K_th);


    /*****  cam、ther原始图片暂存 *******/
    // out_bgr_img = cam_corrected_img.clone();   
    // out_thermal_img = thermal_color_img.clone();

    //--------STEP3.1: yolov5检测 -------------//
    static Mat cam_yolo_img;
    if(yolo_flag) yolov5_detect(cam_corrected_img, cam_yolo_img);

    //--------STEP3.2: edge检测 -------------//
    static Mat cam_edge_img;
    if(edge_flag) cam_edge_img = extract_edges_sobel(cam_corrected_img);

    // //--------STEP4: 双光融合-------------//
    AdjustParams adjust;
    adjust.shift_x = 50;   // 往右移动10个像素
    adjust.shift_y = 10;    
    adjust.scale = 1;    // 不缩放
    adjust.angle = 0.0;    // 不旋转
    Mat thermal_corrected_img = register_thermal_to_visible(thermal_color_img, adjust); //矫正

    Mat fusion_img;
    if(yolo_flag) {
        addWeighted(cam_yolo_img, 0.2, thermal_corrected_img,0.8, 0, fusion_img);
    } else {
        addWeighted(cam_corrected_img, 0.2, thermal_corrected_img,0.8, 0, fusion_img);
    }
    
    if(edge_flag) {
        Mat tmp_img;
        addWeighted(cam_edge_img, 0.2, fusion_img, 0.8, 0, tmp_img);
        //addWeighted(cam_edge_img, 0.4, fusion_img, 0.6, 0, fusion_img); 这种方式会导致程序退出后，CPU高负载100%
        tmp_img.copyTo(fusion_img);
    } 
    // else if(pure_edge_flag) {
    //     addWeighted(cam_edge_img, 0.2, thermal_corrected_img,0.8, 0, fusion_img);
    // }

    //--------STEP5: CV显示-------------//
    Mat final_img(640, 360, CV_8UC3, Scalar(0, 0, 0)); // 创建黑底画布
    
    fusion_img.copyTo(final_img);

    send_fusion_frame(final_img);  //socket转发
    // cam_corrected_img.copyTo(final_img);


    imshow("Fusion Display", final_img);
    waitKey(1); //调整为30ms间隔，如果是waitKey(10)，会100%占用CPU

    return 0;

}


void* opencv_thread(void *arg){
    thread_context_t* ctx = (thread_context_t*)arg;
    // int argc = ctx->thread_args.argc;
    // char **argv = ctx->thread_args.argv;
    // printf("opencv_thread!!!\n");
    Mat save_bgr, save_thermal;

    /*OpenCV 默认使用的线程池是静态的（线程池是懒加载的，第一次用才创建，最后退出也不一定销毁），比如：
        OpenMP 或 TBB 线程池可能是进程级别的；
        如果 OpenCV 在某些平台被编译为使用动态线程池，线程并不会随 cvtColor() 等函数结束而销毁；
        线程池内部的某些线程可能已经阻塞或睡眠，但仍会被唤醒；
        如果某些线程卡在 IO 或 pthread_cond_wait()，还可能因为 race condition 没有正确退出。
    这时，虽然主线程退出了，但这些辅助线程仍存活，并可能触发大量的 wake_up_process() 内核软中断，导致 ksoftirqd 持续高占用。 */
    cv::setNumThreads(1);  //限制 OpenCV 使用线程数, 防止 OpenCV 多线程影响调度

    K_th = estimate_intrinsic_matrix(640, 360, fov_th);
    K_vis = estimate_intrinsic_matrix(640, 360, fov_vis);

    struct timespec ts;
    while(!ctx->cmd_req.exit_req){
        // 设置100ms超时,似乎解决了问题
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_nsec += 100000000;
        if (ts.tv_nsec >= 1000000000) {
            ts.tv_sec++;
            ts.tv_nsec -= 1000000000;
        }

        pthread_mutex_lock(&ctx->thermal_buf.mutex);
        while (!ctx->thermal_buf.updated) {
            if (pthread_cond_timedwait(&ctx->thermal_buf.cond, &ctx->thermal_buf.mutex, &ts) == ETIMEDOUT) {
                pthread_mutex_unlock(&ctx->thermal_buf.mutex);
                continue;
            }
        }
        pthread_mutex_unlock(&ctx->thermal_buf.mutex);

        pthread_mutex_lock(&ctx->yuv_buf.mutex);
        while (!ctx->yuv_buf.updated) {
            if (pthread_cond_timedwait(&ctx->yuv_buf.cond, &ctx->yuv_buf.mutex, &ts) == ETIMEDOUT) {
                pthread_mutex_unlock(&ctx->yuv_buf.mutex);
                continue;
            }
        }
        pthread_mutex_unlock(&ctx->yuv_buf.mutex);

        // size_t frame_size = MIX_WIDTH * MIX_HEIGHT * 3 / 2;
        // static uint8_t* local_yuv = (uint8_t*)malloc(frame_size);
        // pthread_mutex_lock(&ctx->yuv_buf.mutex);
        // while (!ctx->yuv_buf.updated) {
        //     pthread_cond_wait(&ctx->yuv_buf.cond, &ctx->yuv_buf.mutex);
        // }
        // // 拷贝数据
        // memcpy(local_yuv, ctx->yuv_buf.yuv_data, frame_size);
        // ctx->yuv_buf.updated = 0;
        // pthread_mutex_unlock(&ctx->yuv_buf.mutex);

        // static uint16_t local_thermal[TH_THERMAL_ROWS][TH_THERMAL_COLS];
        // pthread_mutex_lock(&ctx->thermal_buf.mutex);
        // while (!ctx->thermal_buf.updated) {
        //     pthread_cond_wait(&ctx->thermal_buf.cond, &ctx->thermal_buf.mutex);
        // }
        // memcpy(local_thermal, ctx->thermal_buf.thermal_data, sizeof(local_thermal));
        // ctx->thermal_buf.updated = 0;
        // pthread_mutex_unlock(&ctx->thermal_buf.mutex);


        pthread_mutex_lock(&ctx->fusion_buf.mutex);
        // printf("opencv!!!\n");
        // draw_roi_frame(ctx->yuv_buf.yuv_data);
        // cv_show_heimann_classic(&ctx->thermal_buf.thermal_data[0][0]);
        colormap = uint8_t(ctx->cmd_req.colormap_ctrl);
        yolo_flag = uint8_t(ctx->cmd_req.yolo_req);
        edge_flag = uint8_t(ctx->cmd_req.edge_req);

        // printf("colomap: %d\n", colormap);
        cv_show_fusion_display(&ctx->thermal_buf.thermal_data[0][0], ctx->yuv_buf.yuv_data, save_bgr, save_thermal);
        // cv_show_fusion_display(&local_thermal[0][0], local_yuv, save_bgr, save_thermal);


        if (ctx->cmd_req.snapshot_request) {
            // 保存热成像和可见光图像
            static int snapshot_count = 0;
            char filename_rgb[128];
            char filename_thermal[128];
            snprintf(filename_rgb, sizeof(filename_rgb), "snapshot_rgb_%d.png", snapshot_count);
            snprintf(filename_thermal, sizeof(filename_thermal), "snapshot_thermal_%d.png", snapshot_count);
        
            // 保存
            imwrite(filename_rgb, save_bgr);
            imwrite(filename_thermal, save_thermal);
        
            printf("Saved snapshot %d\n", snapshot_count);
            snapshot_count++;
            if (snapshot_count >= 4) {
                printf("Already captured 4 snapshots, ignoring further requests.\n");
            }
        
            ctx->cmd_req.snapshot_request = 0; // 重置请求标志
        }

        pthread_mutex_unlock(&ctx->fusion_buf.mutex);
    }

    cv::destroyAllWindows();    
    
    return NULL;

}
