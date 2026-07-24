/**
  ******************************************************************************
  * File Name          : shoot.h
  * Description        : 发射机构模块 — 7状态机 + 弹速自适应 + 热量限制
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 Liang YQ.
  ******************************************************************************
    *
  ==============================================================================
                        ##### How To Use #####
  ==============================================================================
  (#) 配置参数: shoot_cfg_t / shoot_heat_limit_info_t / shoot_speed_adapt_cfg_t
      重点: stuck_cfg.block_time_max, reset_cfg.reset_timeout,
            shoot_cfg.single_shoot_timeout, shoot_cfg.handle_stuck_reverse_timeout,
            shoot_cfg.handle_stuck_reload_timeout

  (#) ①初始化
      Shoot_Init(&shoot);

  (#) ②主循环 (每1ms调用一次)
      shoot.work(&shoot);

  (#) 状态流说明:
      S_SLEEP →(拨轮上拨rising edge)→ S_INIT →(堵转/超时)→ S_WAITING
      S_WAITING →(连发)→ S_REPEAT_SHOOT →(ctrl_flag=0)→ S_WAITING
      任意状态 →(关控/拨轮上拨)→ S_SLEEP (全局最高优先级)

  (#) 外部数据解耦:
      所有外部数据通过 shoot->extern_input 统一访问
      Shoot_Extern_Update() 在 Shoot_Work 开头被调用，注入:
      - dial: 拨盘电机反馈(encoder_sum/speed/current/work_state)
      - fric: 摩擦轮在线状态(L_online/R_online)
      - shoot_flag: 发射命令标志位(Enable_Shoot_Flag/Shoot_Ctrl_Flag/Shoot_Mode)
  *
  ==============================================================================
                        ##### 弹速自适应配置 #####
  ==============================================================================
  (#) BULLET_SPEED_ADAPT 宏使能弹速自适应功能
      摩擦轮转速根据实际弹速自动调整，使弹速维持在 ideal_speed_min~max 区间

  (#) 热量限制说明:
      仅 S_REPEAT_SHOOT 状态下生效，根据裁判系统热量分级设置拨盘射频
  *
  ==============================================================================
                        ##### 输出接口 #####
  ==============================================================================
  (#) shoot.dail_info.target_anglesum  // 角度环目标角度(encoder_sum)
  (#) shoot.dail_info.target_speed     // 速度环目标转速(dps)
  (#) shoot.dail_info.ctrl_mode        // DIAL_SLEEP / DIAL_ANGLE / DIAL_SPEED
  (#) shoot.dail_info.dail_output      // 拨盘最终电流输出
    (#) shoot.adapt_info.final_fric_target_speed  // 摩擦轮最终目标转速
  *
  ******************************************************************************
*/

#ifndef __shoot_H_
#define __shoot_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "motor_def.h"
#include "rc_sensor.h"
#include "KT_motor.h"
#include "shoot_motor.h"
#include "rp_math.h"
/* Export define ------------------------------------------------------------------*/
#define ALL_SHOOT_MOTOR_ONLINE (shoot.fric_online_flag == true && shoot.dail_offline_flag == false)
/*==============================================================================
 * 外部输入数据结构体 — 从外部模块注入，所有计算通过此结构体访问
 *============================================================================*/
/*拨盘电机外部反馈数据*/
typedef struct {
    uint8_t work_state;  // 电机工作状态 (M_OFFLINE 等)
    int32_t encoder_sum; // 编码器累计角度值
    int16_t speed;       // 转速(dps)
    int16_t current;     // 电流
} shoot_dial_extern_t;

/*摩擦轮在线状态*/
typedef struct {
    uint8_t L_online; // 左摩擦轮在线标志
    uint8_t R_online; // 右摩擦轮在线标志
} shoot_fric_extern_t;

/*发射命令标志位（从 Balance 注入）*/
typedef struct {
    bool Enable_Shoot_Flag; // 是否允许发射(关控=false，开控=true)
    bool Shoot_Ctrl_Flag;   // 开火触发标志(单发上升沿/连发电平)
    uint8_t Shoot_Mode;     // 发射模式 (0=单发, 1=连发)
} shoot_flag_extern_t;

/*外部输入数据总结构体*/
typedef struct {
    shoot_dial_extern_t dial;       // 拨盘电机反馈
    shoot_fric_extern_t fric;       // 摩擦轮在线状态
    shoot_flag_extern_t shoot_flag; // 发射命令标志位
} shoot_extern_input_t;

/* Public types ------------------------------------------------------------*/
/*==============================================================================
 * 枚举定义 — 发射机构状态机状态 (7状态)
 *============================================================================*/
typedef enum {
    S_SLEEP = 0,            // 睡眠状态：摩擦轮/拨盘完全卸力
    S_INIT,                 // 初始化状态：摩擦轮开转，拨盘速度环反转复位
    S_WAITING,              // 等待就绪状态：拨盘角度环，待命中
    S_SINGLE_SHOOT,         // 单发发射状态：拨盘角度环，执行单次拨弹
    S_REPEAT_SHOOT,         // 连发发射状态：拨盘速度环，执行连续拨弹
    S_HANDLE_STUCK_REVERSE, // 卡弹反转处理状态：拨盘角度环反转退弹
    S_HANDLE_STUCK_RELOAD,  // 卡弹复位重载状态：拨盘角度环复位重载上弹
} shoot_state_e;

// 拨盘控制模式
typedef enum {
    DIAL_SLEEP = 0, // 卸力
    DIAL_ANGLE,     // 角度环控制
    DIAL_SPEED      // 速度环控制
} dial_ctrl_mode_e;

/*==============================================================================
 * 配置结构体
 *============================================================================*/
// 堵转检测配置
typedef struct {
    float speed_max;         // 堵转判定速度阈值(dps)，速度低于此值认为堵转
    float current_min;       // 堵转判定电流阈值，电流大于此值认为堵转
    uint16_t block_time_max; // 确认堵转所需连续 tick 数
    int8_t current_dir;      // 堵转电流方向 (1 或 -1)
} dial_stuck_cfg_t;

// 拨盘复位配置
typedef struct {
    float reset_speed;      // 复位速度(dps)，负值为反转方向
    uint16_t reset_timeout; // 复位最大超时时间(ms)
    float oneshot_angle;    // 单发拨弹角度(encoder_sum单位)
} dial_reset_cfg_t;

// 拨盘发射配置
typedef struct {
    float stop_angle_err_max;              // 判断拨盘到达目标角度的误差阈值
    uint16_t single_shoot_timeout;         // 单发发射(SINGLE_SHOOT)最大运行时间(ms)
    uint16_t handle_stuck_reverse_timeout; // 卡弹反转处理最大运行时间(ms)
    uint16_t handle_stuck_reload_timeout;  // 卡弹复位重载最大运行时间(ms)
} dial_shoot_cfg_t;

// 发射机构总配置结构体
typedef struct {
    dial_stuck_cfg_t stuck_cfg; // 堵转检测配置
    dial_reset_cfg_t reset_cfg; // 复位配置
    dial_shoot_cfg_t shoot_cfg; // 发射配置
    // float fric_target_speed;    // 摩擦轮初始目标转速
} shoot_cfg_t;

// 弹速自适应配置
typedef struct {
    float init_fric_target_speed; // 初始摩擦轮目标转速
    float ideal_speed_min;        // 理想弹速下限
    float ideal_speed_max;        // 理想弹速上限
    float speed_max;              // 弹速上限(超速阈值)
    float overspeed_adjust_speed; // 超速时直接减小的速度量
    uint16_t less_cnt_max;        // 低速连续容忍次数
    float low_adjust_speed;       // 低速调整增量
    uint16_t more_cnt_max;        // 高速连续容忍次数
    float high_adjust_speed;      // 高速调整减量
} shoot_speed_adapt_cfg_t;

// 弹速自适应调试信息结构体
#define SHOOT_ADAPT_RECENT_SPEED_COUNT 10
typedef struct {
    shoot_speed_adapt_cfg_t cfg;                               // 弹速自适应配置参数
    uint16_t final_fric_target_speed;                          // 摩擦轮最终目标转速
    float recent_bullet_speed[SHOOT_ADAPT_RECENT_SPEED_COUNT]; // 最近10颗弹丸初速度
    uint8_t recent_speed_write_idx;                            // 最近弹速写入索引
    uint8_t recent_speed_count;                                // 已记录弹速数量
    uint32_t last_shoot_count;                                 // 上次处理的弹丸计数
    uint16_t more_cnt;                                         // 高速连续计数
    uint16_t less_cnt;                                         // 低速连续计数
    float current_speed;                                       // 当前用于自适应的弹速
    float current_speed_over;                                  // 当前超理想上限速度差
    float current_divide;                                      // 高速减速计算分母
    uint16_t add_fric_speed;                                   // 本次自适应增加的摩擦轮速度
    uint16_t dec_fric_speed;                                   // 本次自适应减少的摩擦轮速度
} shoot_speed_adapt_info_t;

// 热量限制配置
typedef struct {
    float k_from_shooting_freq_to_dail_speed; // 射频→拨盘速度系数
    float high_heat_shooting_freq;            // 高热量时射频(Hz)
    uint16_t low_heat_value;                  // 低热量判断阈值
    float low_heat_shooting_freq;             // 低热量时射频(Hz)
    uint16_t very_low_heat_value;             // 极低热量判断阈值
    float very_low_heat_shooting_freq;        // 极低热量时射频(Hz)
    uint16_t no_shoot_heat_value;             // 停止发射热量阈值
} shoot_heat_limit_info_t;

/*==============================================================================
 * 拨盘输出接口
 *============================================================================*/
// 拨盘信息结构体 — 状态机输出接口
typedef struct {
    KT_motor_t *dail_motor;                // 拨盘电机实例
    dial_ctrl_mode_e ctrl_mode;            // 当前控制模式
    float target_anglesum;                 // 角度环目标角度(encoder_sum as float)
    float target_speed;                    // 速度环目标转速(dps)
    int16_t dail_output;                   // 拨盘最终电流输出
    struct dail_pid_info_struct *dail_pid; // 拨盘PID结构体
} shoot_dail_info_t;

/*==============================================================================
 * 弹速统计
 *============================================================================*/
// 弹速统计结构体(最近50发，剔除22~27范围外数据)
#define SPEED_STAT_COUNT 50
typedef struct {
    float speed_buf[SPEED_STAT_COUNT]; // 弹速环形缓存
    uint8_t high_speed_count;          // 最近50发中弹速超过24.9的数量
    float latest_speed;                // 最近一颗弹丸初速度
    float std_dev;                     // 标准差
    float average;                     // 平均弹速
    float range;                       // 极差(最大值-最小值)
    float variance;                    // 方差
    uint8_t speed_count;               // 当前有效弹速数量
    uint8_t abnormal_count;            // 异常(剔除)弹速计数
    bool is_valid;                     // 统计是否有效
    uint8_t head_index;                // 环形缓冲区最旧数据索引

} shoot_speed_stats_t;

// 首弹射出时间结构体(最近10发 ms)
#define FIRST_BULLET_TIME_COUNT 10
typedef struct {
    uint32_t dail_start_time[FIRST_BULLET_TIME_COUNT];      // 拨弹开始时刻(ms)
    uint32_t first_shot_time[FIRST_BULLET_TIME_COUNT];      // 首弹射出时刻(ms)
    float first_bullet_delta_time[FIRST_BULLET_TIME_COUNT]; // 时间差(ms)
    uint8_t count;                                          // 当前记录数量
    uint8_t index;                                          // 环形缓冲区写入索引
    bool is_recording;                                      // 当前是否处于拨弹记录状态
    uint8_t current_dail_start_index;                       // 当前拨弹对应缓冲区索引
} shoot_first_bullet_info_t;

/*==============================================================================
 * 发射机构主结构体
 *============================================================================*/
typedef struct shoot_struct_t {
    /*--- 输出接口 ---*/
    shoot_dail_info_t dail_info; // 拨盘输出接口

    /*--- 配置 ---*/
    shoot_cfg_t cfg;                         // 总配置(堵转/复位/发射)
    shoot_heat_limit_info_t heat_limit_info; // 热量限制配置
    shoot_speed_adapt_info_t adapt_info;     // 弹速自适应配置与调试信息

    /*--- PID ---*/

    /*--- 统计 ---*/
    shoot_speed_stats_t speed_stats;             // 弹速统计
    shoot_first_bullet_info_t first_bullet_info; // 首弹射出时间

    /*--- 状态机内部状态 ---*/
    shoot_state_e state;           // 当前状态机状态
    uint32_t state_tick;           // 当前状态运行 tick 数
    uint16_t stuck_block_tick;     // 堵转连续确认 tick 计数
    uint16_t stuck_statistics_cnt; // 每从单发或连发状态进入堵转处理状态+1
    float last_target_anglesum;    // 上一次目标角度(用于比较)
    float dail_init_angle;         // S_INIT退出时记录的拨盘角度，作为连发计算基准
    bool dial_arrived_flag;        // 拨盘到达目标角度标志
    bool dial_block_flag;          // 本周期检测到堵转标志
    bool dail_offline_flag;        // 拨盘电机离线标志
    bool fric_online_flag;         // 摩擦轮在线标志(左或右任一离线则置0)

    /*--- 外部输入(每周期从外部注入，统一通过 extern_input 访问) ---*/
    shoot_extern_input_t extern_input; // 外部输入数据（解耦用）

    /*--- 输出至摩擦轮驱动 ---*/
    /*--- 成员函数 ---*/
    void (*init)(struct shoot_struct_t *shoot); // 初始化函数
    void (*work)(struct shoot_struct_t *shoot); // 主工作函数
} shoot_t;

/* Public functions --------------------------------------------------------*/

extern shoot_t shoot; // 发射机构全局实例

#endif /* __shoot_H_ */
