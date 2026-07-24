/**
 * @file        rp_math.h
 * @author      RobotPilots
 * @Version     v1.1
 * @brief       RobotPilots Robots' Math Libaray.
 * @update
 *              v1.0(11-September-2020)
 *              v1.1(13-November-2021)
 *                  1.增加位操作函数
 */

#ifndef __RP_MATH_H
#define __RP_MATH_H

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Exported macro ------------------------------------------------------------*/
#define ANGLE_TO_RAD 0.0174532922222222222222222222f

typedef enum jugde_logical_e // 逻辑判断
{
    Flase = 0,
    True  = 1,

} jugde_logical_e;
/* Exported types ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* 位操作函数 */
#define SET_EVENT(EVENT, FLAG)   ((EVENT) |= FLAG)
#define CLEAR_EVENT(EVENT, FLAG) ((EVENT) &= ~(FLAG))
#define GET_EVENT(EVENT, FLAG)   ((EVENT) & (FLAG))
/* 数值函数 */
#define constrain(x, min, max)     ((x > max) ? max : (x < min ? min : x))
#define max(a, b)                  ((a) > (b) ? (a) : (b))
#define min(a, b)                  ((a) < (b) ? (a) : (b))
#define my_abs(x)                  ((x) > 0 ? (x) : (-(x)))
#define one(x)                     ((x) > 0 ? (1) : (-1))
#define sgn(x)                     (((x) > 0) ? 1 : ((x) < 0 ? -1 : 0))
#define within_or_not(x, min, max) (x > max) ? Flase : ((x < min) ? Flase : True) // 在规定范围外返回0

#define angle2rad(ANGLE)           ((ANGLE) * ANGLE_TO_RAD)

/* 半圈处理*/
float half_cycle(float angle, float max);
/* 步进限幅滤波*/
float step_limit_filter(float new_value, float last_value, float max_step);
/* 斜坡函数 */
int16_t RampInt(int16_t final, int16_t now, int16_t ramp);
/* 获取下一个周期数值，用于拨盘 */
float get_next_periodic_value(float init_value, float current_value, float period);
/* 获取离current_value最近的周期值 */
float get_nearest_periodic_value(float init_value, float current_value, float period);

float RampFloat(float final, float now, float ramp);
/* 死区函数 */
float DeathZoom(float input, float center, float death);
/* 低通滤波 */
float Lowpass(float X_last, float X_new, float K);
/*浮点数线性映射成整数*/
int float_to_uint(float x, float x_min, float x_max, int bits);

/*整数线性映射成浮点数*/
float uint_to_float(int x_int, float x_min, float x_max, int bits);
/*整数线性映射成浮点数（指定输入输出范围）*/
float int16_to_float(int16_t x_int, int16_t x_min, int16_t x_max, float y_min, float y_max);
float my_sqrt(float num);
#endif
