/**
 * @file        rp_math.c
 * @author      RobotPilots
 * @Version     v1.1
 * @brief       RobotPilots Robots' Math Libaray.
 * @update
 *              v1.0(11-September-2020)
 *              v1.1(13-November-2021)
 *                  1.增加位操作函数
 */

/* Includes ------------------------------------------------------------------*/
#include "rp_math.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
 *	@brief	过半圈处理 angle：源数据 cycle:数据范围
 */
float half_cycle(float angle, float max)
{
    if (my_abs(angle) > (max / 2.f))
    {
        if (angle >= 0)
            angle += -max;
        else
            angle += max;
    }
    return angle;
}

/**
 * @brief 步进式限幅滤波函数
 * @param new_value 当前采样值
 * @param last_value 上一次的滤波输出值
 * @param max_step 最大步进值（死区阈值）
 * @return 本次滤波后的值
 new50,last0,max10,dif50>10,fil=0+10     new0,last20,max10,dif20>10,fil=20-10    new-50,last20,max10,dif70>10,fil=20-10   new10,last
 */
float step_limit_filter(float new_value, float last_value, float max_step)
{
    float filtered_value;
    float difference = new_value - last_value;

    // 如果变化量超过最大步进值，则进行限幅步进处理
    if (fabsf(difference) > max_step)
    {
        filtered_value = last_value + sgn(difference) * max_step;
    }
    else
    {
        // 变化量在允许范围内，直接采用新采样值
        filtered_value = new_value;
    }

    return filtered_value;
}

/*浮点数线性映射成整数*/
int float_to_uint(float x, float x_min, float x_max, int bits)
{
    /// Converts a float to an unsigned int, given range and number of bits
    ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/*整数线性映射成浮点数*/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief  将int16_t整数线性映射到浮点数（指定输入输出范围）
 * @param  x_int: 输入整数
 * @param  x_min, x_max: 输入整数范围
 * @param  y_min, y_max: 输出浮点数范围
 * @retval 映射后的浮点数
 */
float int16_to_float(int16_t x_int, int16_t x_min, int16_t x_max, float y_min, float y_max)
{
    if (x_max == x_min)
        return (y_max + y_min) / 2.0f;

    float ratio = (float)(x_int - x_min) / (float)(x_max - x_min);
    return ratio * (y_max - y_min) + y_min;
}

/**
 * @brief  快速开方
 * @param
 * @retval
 */
float my_sqrt(float num)
{
    float halfnum = 0.5f * num;
    float y = num;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfnum * y * y));
    return y;
}

/**
 * @brief  低通滤波,k越小滤波越好
 */
float Lowpass(float X_last, float X_new, float K)
{
    return (X_last + (X_new - X_last) * K);
}

int16_t RampInt(int16_t final, int16_t now, int16_t ramp)
{
    int32_t buffer = 0;

    buffer = final - now;
    if (buffer > 0)
    {
        if (buffer > ramp)
            now += ramp;
        else
            now += buffer;
    }
    else
    {
        if (buffer < -ramp)
            now += -ramp;
        else
            now += buffer;
    }

    return now;
}

/**
 * @brief  angleum获取下一个周期的值，方向为正
 */
float get_next_periodic_value(float init_value, float current_value, float period)
{
    // 计算当前值相对于初始值已经经历了多少个完整的周期
    // 注意: (int32_t)截断取整对负数行为不正确，必须用floor
    int32_t cycles_completed = (int32_t)floorf((current_value - init_value) / period);

    // 计算下一个周期的值
    float next_value = init_value + (float)(cycles_completed + 1) * period;

    return next_value;
}

/**
 * @brief  angle获取下一个周期的值，方向为正
 */
float get_next_periodic_circle_value(float min, float max, float init_value, float current, float period)
{
    if (max <= min)
        return current;

    float range = max - min;
    int32_t cycles_completed = (int32_t)((current - init_value) / period);
    // 计算下一个周期的值
    float next_value = init_value + (cycles_completed + 1) * period;

    if (next_value > max)
    {
        next_value -= range;
    }
    else if (next_value < min)
    {
        next_value += range;
    }
    return next_value;
}

float RampFloat(float final, float now, float ramp)
{
    float buffer = 0;

    buffer = final - now;
    if (buffer > 0)
    {
        if (buffer > ramp)
            now += ramp;
        else
            now += buffer;
    }
    else
    {
        if (buffer < -ramp)
            now += -ramp;
        else
            now += buffer;
    }

    return now;
}

float DeathZoom(float input, float center, float death)
{
    if (my_abs(input - center) < death)
        return center;
    return input;
}

/**
 * @brief  获取离current_value最近的周期值
 * @param  init_value: 初始周期起点
 * @param  current_value: 当前值
 * @param  period: 周期长度
 * @retval 离current_value最近的周期值（init_value + n*period 中的某一个）
 */
float get_nearest_periodic_value(float init_value, float current_value, float period)
{
    float err = current_value - init_value;

    /* 计算相对于init_value走过的完整周期数（向零取整） */
    int32_t cycles_completed = (int32_t)(err / period);

    /* 以init_value + cycles_completed * period为基准 */
    float nearest = init_value + cycles_completed * period;

    /* 如果当前值偏向下一个周期，则选下一个周期值 */
    if (err - cycles_completed * period > period * 0.5f)
    {
        nearest += period;
    }
    /* 如果当前值偏向上一个周期，则选上一个周期值 */
    else if (err - cycles_completed * period < -period * 0.5f)
    {
        nearest -= period;
    }

    return nearest;
}
