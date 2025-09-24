//
// Created by 10415 on 2025/9/15.
//

#ifndef LOW_C_FILTER_H
#define LOW_C_FILTER_H

// 定义卡尔曼滤波器结构??
typedef struct {
    float Q;         // 过程噪声的方??
    float R;         // 测量噪声的方??
    float x_last;    // 上一时刻的状态估�???????
    float p_last;    // 上一时刻的估计误差协方差
} KalmanFilter;

// 初始化卡尔曼滤波??
void kalman_filter_init(KalmanFilter *filter, float Q, float R);

// 卡尔曼滤波函??
float kalman_filter_update(KalmanFilter *filter, float measurement);

#endif //LOW_C_FILTER_H