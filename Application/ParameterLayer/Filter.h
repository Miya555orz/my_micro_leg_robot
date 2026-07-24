#ifndef __FILTER_H
#define __FILTER_H

#include "stm32h7xx_hal.h"
#include "kalman_filter.h"

extern KalmanFilter_t vaEstimateKF;
void xvEstimateKF_Init(KalmanFilter_t *EstimateKF);
void xvEstimateKF_Update(KalmanFilter_t *EstimateKF ,float acc,float vel);
extern KalmanFilter_t XEstimateKF;
void XEstimateKF_Init(KalmanFilter_t *EstimateKF);
void XEstimateKF_Update(KalmanFilter_t *EstimateKF ,float vel,float s);
void XEstimateKF_Clear(KalmanFilter_t *EstimateKF);
#endif



