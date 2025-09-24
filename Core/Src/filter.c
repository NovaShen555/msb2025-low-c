//
// Created by 10415 on 2025/9/15.
//
#include "filter.h"

// 初始化卡尔曼滤波??
void kalman_filter_init(KalmanFilter *filter, float Q, float R) {
    filter->Q = Q;
    filter->R = R;
    filter->x_last = 0.0f;
    filter->p_last = 1.0f;
}

// 卡尔曼滤波函??
float kalman_filter_update(KalmanFilter *filter, float measurement) {
    float x_pred = filter->x_last;      // 当前时刻的状态预�???????
    float p_pred = filter->p_last + filter->Q;  // 当前时刻的预测误差协方差

    float kg = p_pred / (p_pred + filter->R);  // 卡尔曼增??

    float x_now = x_pred + kg * (measurement - x_pred);  // 当前时刻的状态更�???????
    float p_now = (1 - kg) * p_pred;                      // 当前时刻的更新误差协方差

    filter->x_last = x_now;  // 更新�???????估计???
    filter->p_last = p_now;  // 更新估计误差协方??

    return x_now;
}