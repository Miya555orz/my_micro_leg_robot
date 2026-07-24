#include "Filter.h"

/*对水平速度的滤波*/

KalmanFilter_t vaEstimateKF; // 卡尔曼滤波器结构体

float vaEstimateKF_F[4] = {1.0f, 0.001f,

                           0.0f, 1.0f}; // 状态转移矩阵，控制周期为0.001s

float vaEstimateKF_P[4] = {1.0f, 0.0f,

                           0.0f, 1.0f}; // 后验估计协方差初始值

float vaEstimateKF_Q[4] = {0.1f, 0.0f,

                           0.0f, 0.1f}; // Q矩阵初始值

float vaEstimateKF_R[4] = {200.0f, 0.0f,

                           0.0f, 200.0f};

float vaEstimateKF_K[4];

const float vaEstimateKF_H[4] = {1.0f, 0.0f,

                                 0.0f, 1.0f}; // 设置矩阵H为常量

void xvEstimateKF_Init(KalmanFilter_t *EstimateKF)

{

    Kalman_Filter_Init(EstimateKF, 2, 0, 2); // 状态向量2维 没有控制量 测量向量2维

    memcpy(EstimateKF->F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));

    memcpy(EstimateKF->P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));

    memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));

    memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));

    memcpy(EstimateKF->H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));
}

void xvEstimateKF_Update(KalmanFilter_t *EstimateKF, float acc, float vel)

{

    // 卡尔曼滤波器测量值更新

    EstimateKF->MeasuredVector[0] = vel; // 测量速度

    EstimateKF->MeasuredVector[1] = acc; // 测量加速度

    // 卡尔曼滤波器更新函数

    Kalman_Filter_Update(EstimateKF);
}

/*对水平位移的滤波*/

KalmanFilter_t XEstimateKF; // 卡尔曼滤波器结构体

float XEstimateKF_F[4] = {1.0f, 0.001f,

                          0.0f, 1.0f}; // 状态转移矩阵，控制周期为0.001s

float XEstimateKF_P[4] = {1.0f, 0.0f,

                          0.0f, 1.0f}; // 后验估计协方差初始值

float XEstimateKF_Q[4] = {0.1f, 0.0f,

                          0.0f, 0.1f}; // Q矩阵初始值

float XEstimateKF_R[4] = {200.0f, 0.0f,

                          0.0f, 100.0f};

float XEstimateKF_K[4];

const float XEstimateKF_H[4] = {1.0f, 0.0f,

                                0.0f, 1.0f}; // 设置矩阵H为常量

void XEstimateKF_Init(KalmanFilter_t *EstimateKF)

{

    Kalman_Filter_Init(EstimateKF, 2, 0, 2); // 状态向量2维 没有控制量 测量向量2维

    memcpy(EstimateKF->F_data, XEstimateKF_F, sizeof(XEstimateKF_F));

    memcpy(EstimateKF->P_data, XEstimateKF_P, sizeof(XEstimateKF_P));

    memcpy(EstimateKF->H_data, XEstimateKF_H, sizeof(XEstimateKF_H));

    memcpy(EstimateKF->K_data, XEstimateKF_K, sizeof(XEstimateKF_K));
}

void XEstimateKF_Clear(KalmanFilter_t *EstimateKF)

{

    memcpy(EstimateKF->F_data, XEstimateKF_F, sizeof(XEstimateKF_F));

    memcpy(EstimateKF->P_data, XEstimateKF_P, sizeof(XEstimateKF_P));

    memcpy(EstimateKF->Q_data, XEstimateKF_Q, sizeof(XEstimateKF_Q));

    memcpy(EstimateKF->R_data, XEstimateKF_R, sizeof(XEstimateKF_R));

    memcpy(EstimateKF->H_data, XEstimateKF_H, sizeof(XEstimateKF_H));

    memset(EstimateKF->Pminus_data, 0, sizeof_float * 4);
}

void XEstimateKF_Update(KalmanFilter_t *EstimateKF, float vel, float s)

{

    // 卡尔曼滤波器测量值更新

    EstimateKF->MeasuredVector[0] = s; // 测量位移

    EstimateKF->MeasuredVector[1] = vel; // 测量速度

    // 卡尔曼滤波器更新函数

    Kalman_Filter_Update(EstimateKF);
}
