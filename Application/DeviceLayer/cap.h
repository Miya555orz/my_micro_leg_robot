/**
 ******************************************************************************
 * @file    cap.h
 * @brief   电容管理 - 包含无线充电支持
 ******************************************************************************
 */

#ifndef __CAP_H
#define __CAP_H

#include "stm32h7xx_hal.h"
#include "judge.h"

#define MASTER_TO_CAP_ID         0x222
#define CAP_TO_MASTER_ID         0x211
#define WIRELESS_ID    0x212

/* 发送数据结构体（发送给电容板） */
typedef struct __attribute__((packed)) tx_info_struct
{
    uint8_t  chassis_power_buffer;          // 底盘能量缓冲
    uint16_t chassis_power_limit;           // 机器人底盘功率限制上限
    int16_t  cap_power_out_limit;           // 电容放电功率限制（负值）
    uint16_t cap_power_in_limit;            // 电容充电功率限制（正值）

    struct __attribute__((packed)) bit_control_struct
    {
        uint8_t cap_switch : 1;             // 电容开关，1为开，0为关
        uint8_t turbo_mode : 1;             // 是否使用缓冲能量充电，0为不用，1为用
        uint8_t pre_charge_mode_en : 1;    // 是否使能预充电模式
        uint8_t unuse : 5;                 // 暂时未使用
    } bit_control;
} capboard_tx_info_t;

/* 接收数据结构体（从电容板接收） */
typedef struct __attribute__((packed)) rx_info_struct
{
    int16_t now_chassis_power;              // 当前底盘消耗功率
    int16_t now_cap_V;                      // 当前电容组电压
    int16_t now_cap_I;                      // 当前电容组电流

    struct __attribute__((packed)) bit_state_struct
    {
        uint8_t ability : 1;                // 电容是否有放电能力，0为无，1为有
        uint8_t is_in_pre_charge_mode : 1; // 是否在预充电模式
        uint8_t unuse : 6;                 // 暂时未使用
    } bit_state;
} capboard_rx_info_t;

/* 无线充电接收数据结构体 */
typedef struct __attribute__((packed)) wireless_rx_info_struct
{
    int16_t charging_power;                 // 充电功率
    uint8_t is_charging;                    // 是否在充电，1为充电，0为不充电
    uint8_t reserved1;
    uint16_t reserved2;
    uint16_t reserved3;
} wireless_rx_info_t;

/* 电容状态枚举 */
typedef enum
{
    CAP_ONLINE,
    CAP_OFFLINE,
} cap_state_e;

/* 无线充电状态枚举 */
typedef enum
{
    WIRELESS_ONLINE,
    WIRELESS_OFFLINE,
} wireless_state_e;

/* 电容信息结构体 */
typedef struct cap_info_struct
{
    capboard_tx_info_t tx;
    capboard_rx_info_t rx;
    wireless_rx_info_t wireless;

    float cap_u;           // 电容电压（浮点）
    float cap_i;           // 电容电流（浮点）
    float wireless_p;      // 无线充电功率（浮点）

    uint8_t offline_cnt;       // 电容离线计数
    uint8_t offline_max_cnt;   // 电容离线最大计数

    uint8_t w_offline_cnt;     // 无线充电离线计数
    uint8_t w_offline_max_cnt; // 无线充电离线最大计数
} cap_info_t;

/* 电容对象结构体（OOP风格） */
typedef struct cap_struct
{
    cap_state_e state;         // 电容状态
    wireless_state_e w_state;   // 无线充电状态

    cap_info_t info;

    void (*setdata)(struct cap_struct *self);     // 仅更新tx数据，不发CAN
    void (*txdata)(struct cap_struct *self);     // 发送CAN数据
    void (*update)(struct cap_struct *self, uint8_t *rxBuf, uint32_t can_id);
    void (*heartbeat)(struct cap_struct *self);
} cap_t;

/* Exported variables */
extern cap_t cap;

#endif
