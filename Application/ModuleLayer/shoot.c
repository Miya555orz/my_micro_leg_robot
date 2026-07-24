/**
  ******************************************************************************
  * File Name          : shoot.c
  * Description        : 发射机构模块实现 — 状态机 + 弹速自适应 + 热量限制
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 Liang YQ.
  ******************************************************************************
  *
  ==============================================================================
                        ##### 调用顺序 (每1ms周期) #####
  ==============================================================================
  (#) Shoot_Extern_Update()        — 外部数据更新（注入外部输入）
  (#) Shoot_State_Machine()          — 状态机逻辑，写入 target_anglesum / target_speed / ctrl_mode
  (#) Shoot_Heat_Limit()            — 热量限制，写入 dial target_speed (仅S_REPEAT_SHOOT)
    (#) Shoot_Fric_Ctrl()             — 弹速自适应，写入 adapt_info.final_fric_target_speed
  (#) Shoot_Speed_Statistics_Update() — 弹速统计(debug)
  (#) Shoot_Dial_Pid_Cal()          — 拨盘PID计算，写入 dail_output 并下发电机
  *
  ==============================================================================
                        ##### 输出接口 #####
  ==============================================================================
  (#) shoot.dail_info.target_anglesum / target_speed / ctrl_mode / dail_output
    (#) shoot.adapt_info.final_fric_target_speed
  *
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "shoot.h"
#include "Balance.h"
#include "communicate.h"
#include "judge.h"
#include "rp_config.h"

#include <math.h>

/* Private macros ------------------------------------------------------------*/
#define DIAL_MAX_ENCODER_VALUE 65536.0f // 拨盘编码器一圈数值
#define DIAL_REDUCTION_RATIO 10.0f      // 拨盘减速比
#define DIAL_HOLD_BULLET_NUM 10.0f      // 拨盘整盘容纳弹丸数量
/* Private functions -------------------------------------------------------*/
static void Shoot_Extern_Update(shoot_t *shoot);                                                             // 外部数据更新（从外部模块注入）
static void Shoot_State_Machine(shoot_t *shoot);                                                             // 状态机状态更新
static bool Shoot_Dial_Block_Check(shoot_t *shoot);                                                          // 拨盘堵转检测
static void Shoot_Heat_Limit(shoot_t *shoot);                                                                // 热量限制控制
static void Shoot_Fric_Ctrl(shoot_t *shoot);                                                                 // 摩擦轮控制与弹速自适应
static void Shoot_Speed_Statistics_Update(shoot_t *shoot);                                                   // 弹速统计更新
static void Shoot_First_Bullet_Time_Update(shoot_t *shoot);                                                  // 首弹射出时间更新
static void Shoot_Stuck_Statistics_Update(shoot_t *shoot, shoot_state_e pre_state, shoot_state_e cur_state); // 堵转处理进入次数统计
static void Shoot_Dial_Pid_Cal(shoot_t *shoot);                                                              // 拨盘PID计算
static void Shoot_Init(shoot_t *shoot);                                                                      // 发射机构初始化
static void Shoot_Work(shoot_t *shoot);                                                                      // 发射机构主工作函数(每1ms调用)
/* Private variables ---------------------------------------------------------*/
// 发射机构全局实例
shoot_t shoot = {
    .dail_info.dail_motor = &dail_motor, // 绑定拨盘电机实例
    .dail_info.dail_pid = &dail_pid,     // 绑定拨盘PID结构体

    /* 配置参数 — 堵转检测 */
    .cfg = {
        .stuck_cfg = {
            .speed_max = 10.0f,    // 速度低于此值认为堵转
            .current_min = 150.0f, // 电流大于此值认为堵转
            .block_time_max = 100, // 确认堵转所需连续 tick 数
            .current_dir = 1,      // 堵转电流方向
        },
        .reset_cfg = {
            .reset_speed = -6000.0f, // 复位速度(负值为反转)
            .reset_timeout = 1000,   // 复位最大超时时间(ms)
            // 单发拨弹角度 = 一圈编码器值 / 减速比 / 弹丸数
            .oneshot_angle = DIAL_MAX_ENCODER_VALUE,
        },
        .shoot_cfg = {
            .stop_angle_err_max = 500.0f, // 判断到位误差阈值
            .single_shoot_timeout = 300,
            // 单发发射超时(ms)
            .handle_stuck_reverse_timeout = 200, // 卡弹反转处理超时(ms)
            .handle_stuck_reload_timeout = 200,  // 卡弹复位重载超时(ms)
        },
    },

    /* 配置参数 — 弹速自适应 */
    .adapt_info = {
        .cfg = {
            .init_fric_target_speed = 6300,  // 初始摩擦轮目标转速
            .ideal_speed_min = 24.3f,        // 理想弹速下限
            .ideal_speed_max = 24.5f,        // 理想弹速上限
            .speed_max = 24.7f,              // 超速阈值
            .overspeed_adjust_speed = 20.0f, // 超速时直接减小量
            .less_cnt_max = 2,               // 低速连续容忍次数
            .low_adjust_speed = 1.0f,        // 低速增加转速量
            .more_cnt_max = 0,               // 高速连续容忍次数
            .high_adjust_speed = 5.0f,       // 高速降低转速量
        },
    },

    /* 配置参数 — 热量限制 */
    .heat_limit_info = {
        // 射频→拨盘速度系数，KT返回的速度单位是°/s
        .k_from_shooting_freq_to_dail_speed = 360.0f * DIAL_REDUCTION_RATIO / DIAL_HOLD_BULLET_NUM,
        .high_heat_shooting_freq = 18.f, // 高热量时射频(Hz)

        .low_heat_value = 100.f,        // 低热量判断阈值
        .low_heat_shooting_freq = 18.f, // 低热量时射频(Hz)

        .very_low_heat_value = 90.f,        // 极低热量判断阈值
        .very_low_heat_shooting_freq = 2.f, // 极低热量时射频(Hz)

        .no_shoot_heat_value = 30.f, // 停止发射热量阈值
    },
    .init = Shoot_Init, // 绑定初始化函数
};

/* Private function prototypes (static) --------------------------------------*/
static void Shoot_Extern_Update(shoot_t *shoot);                                                             // 外部数据更新（从外部模块注入）
static void Shoot_State_Machine(shoot_t *shoot);                                                             // 7状态机状态更新
static bool Shoot_Dial_Block_Check(shoot_t *shoot);                                                          // 拨盘堵转检测
static void Shoot_Heat_Limit(shoot_t *shoot);                                                                // 热量限制控制
static void Shoot_Fric_Ctrl(shoot_t *shoot);                                                                 // 摩擦轮控制与弹速自适应
static void Shoot_Speed_Statistics_Update(shoot_t *shoot);                                                   // 弹速统计更新
static void Shoot_First_Bullet_Time_Update(shoot_t *shoot);                                                  // 首弹射出时间更新
static void Shoot_Stuck_Statistics_Update(shoot_t *shoot, shoot_state_e pre_state, shoot_state_e cur_state); // 堵转处理进入次数统计
static void Shoot_Dial_Pid_Cal(shoot_t *shoot);                                                              // 拨盘PID计算

/* Public member functions ---------------------------------------------------*/
/**
 * @brief  发射机构初始化 — 绑定函数指针，初始化状态机
 * @param  shoot: 发射机构结构体指针
 * @retval None
 */
void Shoot_Init(shoot_t *shoot)
{
    // 绑定成员函数到具体实现 (OOP函数指针初始化)

    shoot->work = Shoot_Work;

    // 状态机初始化
    shoot->state = S_SLEEP;
    shoot->state_tick = 0;
    shoot->stuck_block_tick = 0;
    shoot->stuck_statistics_cnt = 0;
    shoot->dial_arrived_flag = false;
    shoot->dial_block_flag = false;
    shoot->dail_offline_flag = false;
    shoot->fric_online_flag = false;
    shoot->last_target_anglesum = 0.0f;

    // 输出接口初始化
    shoot->dail_info.ctrl_mode = DIAL_SLEEP;
    shoot->dail_info.target_anglesum = 0.0f;
    shoot->dail_info.target_speed = 0.0f;
    shoot->dail_info.dail_output = 0;
    shoot->adapt_info.final_fric_target_speed = 0;
    shoot->adapt_info.add_fric_speed = 0;
    shoot->adapt_info.dec_fric_speed = 0;
}

/* Private member functions --------------------------------------------------*/
/**
 * @brief  检测拨盘/摩擦轮离线状态
 * @param  shoot: 发射机构结构体指针
 * @retval None
 * @note   拨盘电机离线或任一摩擦轮离线则 fric_online_flag=false
 */
/**
 * @brief  外部数据更新 — 从外部模块注入数据到 extern_input
 * @param  shoot: 发射机构结构体指针
 * @retval None
 * @note   所有外部数据统一在此函数中更新，后续计算全部通过 shoot->extern_input 访问
 */
static void Shoot_Extern_Update(shoot_t *shoot)
{
    // ======================== 拨盘电机外部反馈 ========================
    // 获取拨盘电机反馈数据指针
    KT_motor_rx_info_t *rx = &shoot->dail_info.dail_motor->KT_motor_info.rx_info;
    uint8_t dail_work_state = shoot->dail_info.dail_motor->KT_motor_info.state_info.work_state;

    // 写入拨盘外部数据
    shoot->extern_input.dial.work_state = dail_work_state;
    shoot->extern_input.dial.encoder_sum = rx->encoder_sum;
    shoot->extern_input.dial.speed = rx->speed;
    shoot->extern_input.dial.current = rx->current;

    // ======================== 摩擦轮在线状态 ========================
    shoot->extern_input.fric.L_online = Board_Rx_Info.flag.L_fric_online;
    shoot->extern_input.fric.R_online = Board_Rx_Info.flag.R_fric_online;

    // ======================== 发射命令标志位 ========================
    shoot->extern_input.shoot_flag.Enable_Shoot_Flag = Balance.Shoot_Flag_struct.Enable_Shoot_Flag;
    shoot->extern_input.shoot_flag.Shoot_Ctrl_Flag = Balance.Shoot_Flag_struct.Shoot_Ctrl_Flag;
    shoot->extern_input.shoot_flag.Shoot_Mode = Balance.Shoot_Flag_struct.Shoot_Mode;

    // ======================== 内部状态计算 ========================
    // 拨盘电机离线检测
    if (dail_work_state == M_OFFLINE)
    {
        shoot->dail_offline_flag = true;
    }
    else
    {
        shoot->dail_offline_flag = false;
    }

    // 摩擦轮离线检测: 左或右任一离线则视为离线
    if (Board_Rx_Info.flag.L_fric_online == 0 || Board_Rx_Info.flag.R_fric_online == 0)
    {
        // shoot->fric_online_flag = false;
        shoot->fric_online_flag = true; // 拆头时调试拨盘用
    }
    else
    {
        shoot->fric_online_flag = true;
    }
}

/**
 * @brief  拨盘堵转检测 — 速度+电流+时间三条件联合判定
 * @param  shoot: 发射机构结构体指针
 * @retval true=堵转, false=正常
 */
static bool Shoot_Dial_Block_Check(shoot_t *shoot)
{
    // 从外部输入数据获取拨盘电机反馈
    KT_motor_rx_info_t *rx = (KT_motor_rx_info_t *)&shoot->extern_input.dial;
    dial_stuck_cfg_t *cfg = &shoot->cfg.stuck_cfg;

    // 速度低于阈值 且 电流方向正确且大于阈值 → 疑似堵转
    float abs_speed = fabsf((float)shoot->extern_input.dial.speed);
    float current_with_dir = (float)shoot->extern_input.dial.current * cfg->current_dir;
    if (abs_speed < cfg->speed_max && current_with_dir > cfg->current_min)
    {
        // 堵转计数递增，达到阈值则确认堵转
        if (shoot->stuck_block_tick >= cfg->block_time_max)
        {
            return true;
        }
        shoot->stuck_block_tick++;
    }
    else
    {
        // 正常状态，计数清零
        shoot->stuck_block_tick = 0;
    }
    return false;
}

/**
 * @brief  状态机状态更新 — 写入 ctrl_mode / target_anglesum / target_speed / state
 * @param  shoot: 发射机构结构体指针
 * @note   全局最高优先级: enable_shoot_flag==false 时无条件切入 S_SLEEP
 *         输入: rx->encoder_sum / speed / current
 */
static void Shoot_State_Machine(shoot_t *shoot)
{
    // 获取拨盘电机反馈
    KT_motor_rx_info_t *rx = (KT_motor_rx_info_t *)&shoot->extern_input.dial;
    float current_angle = (float)shoot->extern_input.dial.encoder_sum; // 当前角度
    float oneshot = shoot->cfg.reset_cfg.oneshot_angle;                // 单发拨弹角度

    // ============================ 全局优先级: 关控 → S_SLEEP ============================
    if (shoot->extern_input.shoot_flag.Enable_Shoot_Flag == false ||
        RC_OFFLINE)
    {
        // 关控时强制进入睡眠状态，清除所有状态标志
        shoot->state = S_SLEEP;
        shoot->state_tick = 0;
        shoot->stuck_block_tick = 0;
        shoot->dial_arrived_flag = false;
        shoot->dial_block_flag = false;
        shoot->dail_info.ctrl_mode = DIAL_SLEEP;
        shoot->dail_info.target_speed = 0;
        shoot->adapt_info.final_fric_target_speed = 0;
        return;
    }

    // 状态运行 tick 递增
    shoot->state_tick++;

    // 状态机主 switch
    switch (shoot->state)
    {

    /*--------------------------------------- S_SLEEP ---------------------------------------*/
    case S_SLEEP:
        // 摩擦轮/拨盘完全卸力
        shoot->dail_info.ctrl_mode = DIAL_SLEEP;
        shoot->dail_info.target_speed = 0;
        shoot->dail_info.target_anglesum = 0;
        shoot->adapt_info.final_fric_target_speed = 0;
        shoot->adapt_info.add_fric_speed = 0;
        shoot->adapt_info.dec_fric_speed = 0;
        shoot->state_tick = 0;
        shoot->stuck_block_tick = 0;
        // S_SLEEP → S_INIT 的跳转由 Shoot_Work 在检测到 rising edge 时处理
        break;

    /*--------------------------------------- S_INIT ---------------------------------------*/
    case S_INIT:
        // 摩擦轮开转，拨盘速度环反转进行初始化复位
        shoot->dail_info.ctrl_mode = DIAL_SPEED;
        shoot->dail_info.target_speed = shoot->cfg.reset_cfg.reset_speed;

        // 堵转检测 → 初始化完成，进入等待就绪状态
        if (Shoot_Dial_Block_Check(shoot))
        {
            shoot->dail_init_angle = current_angle; // 记录退出角度作为后续连发基准
            shoot->dail_info.target_anglesum = current_angle;
            shoot->dial_arrived_flag = false;
            shoot->state = S_WAITING;
            shoot->state_tick = 0;
            shoot->stuck_block_tick = 0;
        }
        // 复位超时检测 → 初始化超时，进入等待就绪状态
        else if (shoot->state_tick >= shoot->cfg.reset_cfg.reset_timeout)
        {
            shoot->dail_init_angle = current_angle; // 记录退出角度作为后续连发基准
            shoot->dail_info.target_anglesum = current_angle;
            shoot->dial_arrived_flag = false;
            shoot->state = S_WAITING;
            shoot->state_tick = 0;
            shoot->stuck_block_tick = 0;
        }
        break;

    /*--------------------------------------- S_WAITING ---------------------------------------*/
    case S_WAITING:
        // 拨盘角度环控制，待触发发射命令
        shoot->dail_info.ctrl_mode = DIAL_ANGLE;
        shoot->state_tick = 0;
        shoot->dial_arrived_flag = false;

        // 优先级1: 堵转检测 → 进入卡弹反转处理
        if (Shoot_Dial_Block_Check(shoot))
        {
            shoot->dial_block_flag = true;
            shoot->dail_info.target_anglesum = current_angle - oneshot; // 反转退弹
            shoot->state = S_HANDLE_STUCK_REVERSE;
            shoot->state_tick = 0;
            shoot->stuck_block_tick = 0;
            break;
        }

        // 到位检测: 计算当前角度与目标角度误差
        float angle_err = fabsf(shoot->dail_info.target_anglesum - current_angle);
        if (angle_err <= shoot->cfg.shoot_cfg.stop_angle_err_max)
        {
            shoot->dial_arrived_flag = true;
        }

        // 优先级2: 摩擦轮离线 → 禁止拨弹，直接返回等待
        if (shoot->fric_online_flag == false)
        {
            break;
        }

        // 优先级3: 单发发射 (shoot_mode==0 且 shoot_ctrl_flag上升沿)
        if (shoot->extern_input.shoot_flag.Shoot_Mode == 0 && shoot->extern_input.shoot_flag.Shoot_Ctrl_Flag == true)
        {
#ifndef TEST_NO_LIMIT_SHOOT
            uint16_t heat_remain = My_Judge.info->shooter_cooling_limit - My_Judge.info->shooter_cooling_heat;
            uint8_t is_in_match = (My_Judge.org_info->game_status.game_progress == 4);
            uint8_t single_shoot_blocked = 0;

            if (is_in_match)
            {
                if (heat_remain <= shoot->heat_limit_info.no_shoot_heat_value ||
                    My_Judge.org_info->projectile_allowance.projectile_allowance_17mm <= 0)
                {
                    single_shoot_blocked = 1;
                }
            }
            else
            {
                if (heat_remain <= shoot->heat_limit_info.no_shoot_heat_value)
                {
                    single_shoot_blocked = 1;
                }
            }

            if (single_shoot_blocked)
            {
                break;
            }
#endif
            shoot->dial_arrived_flag = false;
            shoot->dail_info.target_anglesum += oneshot; // 目标角度增加一弹
            shoot->state = S_SINGLE_SHOOT;
            shoot->state_tick = 0;
            shoot->stuck_block_tick = 0;
        }
        // 优先级4: 连发发射 (shoot_mode==1 且 shoot_ctrl_flag==1)
        else if (shoot->extern_input.shoot_flag.Shoot_Mode == 1 && shoot->extern_input.shoot_flag.Shoot_Ctrl_Flag == true)
        {
            shoot->state = S_REPEAT_SHOOT;
            shoot->state_tick = 0;
            shoot->stuck_block_tick = 0;
        }
        break;

    /*--------------------------------------- S_SINGLE_SHOOT ---------------------------------------*/
    case S_SINGLE_SHOOT:
        shoot->dail_info.ctrl_mode = DIAL_ANGLE;

        // 堵转检测 → 进入卡弹反转处理
        if (Shoot_Dial_Block_Check(shoot))
        {
            shoot->dial_block_flag = true;
            shoot->dail_info.target_anglesum = current_angle - oneshot;
            shoot->state = S_HANDLE_STUCK_REVERSE;
            shoot->state_tick = 0;
            shoot->stuck_block_tick = 0;
            break;
        }

        // 到位 → 返回等待就绪状态
        float single_angle_err = fabsf(shoot->dail_info.target_anglesum - current_angle);
        if (single_angle_err <= shoot->cfg.shoot_cfg.stop_angle_err_max)
        {
            shoot->state = S_WAITING;
            shoot->state_tick = 0;
            shoot->stuck_block_tick = 0;
        }

        // 超时 → 返回等待就绪状态
        if (shoot->state_tick >= shoot->cfg.shoot_cfg.single_shoot_timeout)
        {
            shoot->state = S_WAITING;
            shoot->state_tick = 0;
            shoot->stuck_block_tick = 0;
        }
        break;

    /*--------------------------------------- S_REPEAT_SHOOT ---------------------------------------*/
    case S_REPEAT_SHOOT:
        // 目标速度由 Shoot_Heat_Limit 每周期更新
        shoot->dail_info.ctrl_mode = DIAL_SPEED;

        // 堵转检测 → 进入卡弹反转处理
        if (Shoot_Dial_Block_Check(shoot))
        {
            shoot->dial_block_flag = true;
            shoot->dail_info.target_anglesum = current_angle - oneshot;
            shoot->dail_info.ctrl_mode = DIAL_ANGLE; // 切换为角度模式反转
            shoot->state = S_HANDLE_STUCK_REVERSE;
            shoot->state_tick = 0;
            shoot->stuck_block_tick = 0;
            break;
        }

        // 连发停止: ctrl_flag==0 → 计算下一个周期角度，切换角度环返回等待
        if (shoot->extern_input.shoot_flag.Shoot_Ctrl_Flag == false)
        {
            float next_grid = get_next_periodic_value(shoot->dail_init_angle, current_angle, oneshot);
            shoot->dail_info.target_anglesum = next_grid;
            shoot->dail_info.ctrl_mode = DIAL_ANGLE;
            shoot->state = S_WAITING;
            shoot->state_tick = 0;
            shoot->stuck_block_tick = 0;
        }
        break;

    /*--------------------------------------- S_HANDLE_STUCK_REVERSE ---------------------------------------*/
    case S_HANDLE_STUCK_REVERSE:
        shoot->dail_info.ctrl_mode = DIAL_ANGLE;

        // 任一条件满足 → 进入卡弹复位重载状态
        float stuck_reverse_err = fabsf(shoot->dail_info.target_anglesum - current_angle);
        if (stuck_reverse_err <= shoot->cfg.shoot_cfg.stop_angle_err_max ||
            Shoot_Dial_Block_Check(shoot) ||
            shoot->state_tick >= shoot->cfg.shoot_cfg.handle_stuck_reverse_timeout)
        {
            // 目标角度加一弹丸角度，准备重载
            shoot->dail_info.target_anglesum = current_angle + oneshot;
            shoot->state = S_HANDLE_STUCK_RELOAD;
            shoot->state_tick = 0;
            shoot->stuck_block_tick = 0;
        }
        break;

    /*--------------------------------------- S_HANDLE_STUCK_RELOAD ---------------------------------------*/
    case S_HANDLE_STUCK_RELOAD:
        shoot->dail_info.ctrl_mode = DIAL_ANGLE;

        // 任一条件满足 → 返回等待就绪状态
        float stuck_reload_err = fabsf(shoot->dail_info.target_anglesum - current_angle);
        if (stuck_reload_err <= shoot->cfg.shoot_cfg.stop_angle_err_max ||
            Shoot_Dial_Block_Check(shoot) ||
            shoot->state_tick >= shoot->cfg.shoot_cfg.handle_stuck_reload_timeout)
        {
            shoot->state = S_WAITING;
            shoot->state_tick = 0;
            shoot->stuck_block_tick = 0;
        }
        break;

    /*--------------------------------------- default ---------------------------------------*/
    default:
        shoot->state = S_SLEEP;
        break;
    }
}

/**
 * @brief  热量限制控制
 * @param  shoot: 发射机构结构体指针
 * @note   仅在 S_REPEAT_SHOOT 状态下更新 dial target_speed
 *         根据裁判系统剩余热量分级设置拨盘发射射频
 */
static void Shoot_Heat_Limit(shoot_t *shoot)
{
    // 仅在连发状态下限制拨盘转速
    if (shoot->state != S_REPEAT_SHOOT)
    {
        return;
    }

#ifdef TEST_NO_LIMIT_SHOOT
    // TEST_NO_LIMIT_SHOOT 模式: 跳过热量限制，使用最高射频
    shoot->dail_info.target_speed =
        shoot->heat_limit_info.high_heat_shooting_freq * shoot->heat_limit_info.k_from_shooting_freq_to_dail_speed;
#else
    // 计算剩余热量
    uint16_t heat_remain = My_Judge.info->shooter_cooling_limit - My_Judge.info->shooter_cooling_heat;
    uint8_t is_in_match = (My_Judge.org_info->game_status.game_progress == 4);
    uint8_t stop_shoot = 0;

    // 比赛中: 热量低于阈值或弹丸用完则停止发射
    if (is_in_match)
    {
        if (heat_remain <= shoot->heat_limit_info.no_shoot_heat_value ||
            My_Judge.org_info->projectile_allowance.projectile_allowance_17mm <= 0)
        {
            stop_shoot = 1;
        }
    }
    else
    {
        // 赛外: 仅热量限制
        if (heat_remain <= shoot->heat_limit_info.no_shoot_heat_value)
        {
            stop_shoot = 1;
        }
    }

    // 根据是否停止发射设置目标速度
    if (stop_shoot)
    {
        shoot->dail_info.target_speed = 0;
    }
    else
    {
        // 根据剩余热量分级设置射频(拨盘转速)
        if (heat_remain <= shoot->heat_limit_info.very_low_heat_value)
        {
            // 极低热量: 最低射频
            shoot->dail_info.target_speed =
                shoot->heat_limit_info.very_low_heat_shooting_freq * shoot->heat_limit_info.k_from_shooting_freq_to_dail_speed;
        }
        else if (heat_remain <= shoot->heat_limit_info.low_heat_value)
        {
            // 低热量: 中等射频
            shoot->dail_info.target_speed =
                shoot->heat_limit_info.low_heat_shooting_freq * shoot->heat_limit_info.k_from_shooting_freq_to_dail_speed;
        }
        else
        {
            // 正常热量: 最高射频
            shoot->dail_info.target_speed =
                shoot->heat_limit_info.high_heat_shooting_freq * shoot->heat_limit_info.k_from_shooting_freq_to_dail_speed;
        }
    }
#endif
}

/**
 * @brief  摩擦轮控制与弹速自适应
 * @param  shoot: 发射机构结构体指针
 * @note   基于摩擦轮当前转速，自适应加减偏移量使弹速处于设定区间
 *         S_SLEEP 状态下目标转速强制为0
 */
static void Shoot_Fric_Ctrl(shoot_t *shoot)
{
    // 绑定调试信息结构，统一承载配置和中间变量
    shoot_speed_adapt_info_t *adapt_info = &shoot->adapt_info;
    int32_t final_speed_calc = 0;
    uint16_t step_adjust = 0;

    // S_SLEEP: 摩擦轮完全停止
    if (shoot->state == S_SLEEP)
    {
        shoot->adapt_info.final_fric_target_speed = 0;
        shoot->adapt_info.add_fric_speed = 0;
        shoot->adapt_info.dec_fric_speed = 0;
        return;
    }

#ifdef BULLET_SPEED_ADAPT
    // 仅在新弹丸出现时更新自适应累计量
    if (My_Judge.shoot_count != adapt_info->last_shoot_count)
    {
        adapt_info->last_shoot_count = My_Judge.shoot_count;
        adapt_info->current_speed = My_Judge.org_info->shoot_data.initial_speed;

        // 保存最近10颗弹丸初速度
        adapt_info->recent_bullet_speed[adapt_info->recent_speed_write_idx] = adapt_info->current_speed;
        adapt_info->recent_speed_write_idx = (adapt_info->recent_speed_write_idx + 1) % SHOOT_ADAPT_RECENT_SPEED_COUNT;
        if (adapt_info->recent_speed_count < SHOOT_ADAPT_RECENT_SPEED_COUNT)
        {
            adapt_info->recent_speed_count++;
        }

        // 弹速在理想区间内: 清除计数，不改累计调整量
        if (adapt_info->current_speed >= adapt_info->cfg.ideal_speed_min && adapt_info->current_speed <= adapt_info->cfg.ideal_speed_max)
        {
            adapt_info->more_cnt = 0;
            adapt_info->less_cnt = 0;
        }
        // 超速: 累计减速量
        else if (adapt_info->current_speed >= adapt_info->cfg.speed_max)
        {
            step_adjust = (uint16_t)adapt_info->cfg.overspeed_adjust_speed;
            adapt_info->dec_fric_speed += step_adjust;
            adapt_info->more_cnt = 0;
            adapt_info->less_cnt = 0;
        }
        // 低速: 累计增速量
        else if (adapt_info->current_speed < adapt_info->cfg.ideal_speed_min)
        {
            adapt_info->less_cnt++;
            adapt_info->more_cnt = 0;
            if (adapt_info->less_cnt >= adapt_info->cfg.less_cnt_max)
            {
                adapt_info->less_cnt = 0;
                step_adjust = (uint16_t)adapt_info->cfg.low_adjust_speed;

                adapt_info->add_fric_speed += step_adjust;
            }
        }
        // 高速但不超速: 按比例累计减速量
        else
        {
            adapt_info->more_cnt++;
            adapt_info->less_cnt = 0;
            if (adapt_info->more_cnt >= adapt_info->cfg.more_cnt_max)
            {
                adapt_info->more_cnt = 0;
                adapt_info->current_divide = adapt_info->cfg.speed_max - adapt_info->cfg.ideal_speed_max;
                if (adapt_info->current_divide == 0.0f)
                {
                    adapt_info->current_divide = 1.0f; // 防止除零
                }

                adapt_info->current_speed_over = adapt_info->current_speed - adapt_info->cfg.ideal_speed_max;
                step_adjust = (uint16_t)(adapt_info->current_speed_over / adapt_info->current_divide * adapt_info->cfg.high_adjust_speed);

                adapt_info->dec_fric_speed += step_adjust;
            }
        }
    }

    // 最终转速统一由 初始转速 + 增速累计 - 减速累计 回算
    final_speed_calc = (int32_t)adapt_info->cfg.init_fric_target_speed +
                       (int32_t)adapt_info->add_fric_speed -
                       (int32_t)adapt_info->dec_fric_speed;
    if (final_speed_calc < 0)
    {
        final_speed_calc = 0;
    }
    else if (final_speed_calc > 65535)
    {
        final_speed_calc = 65535;
    }
    shoot->adapt_info.final_fric_target_speed = (uint16_t)final_speed_calc;
#else
    // 无弹速自适应: 使用初始目标转速
    shoot->adapt_info.final_fric_target_speed = (uint16_t)adapt_info->cfg.init_fric_target_speed;
#endif
}

/**
 * @brief  弹速统计更新
 * @param  shoot: 发射机构结构体指针
 * @note   每收到一发新弹丸时调用，计算最近50发的均值/极差/标准差/方差
 *         剔除22~27m/s范围外的异常数据
 */
static void Shoot_Speed_Statistics_Update(shoot_t *shoot)
{
    static uint32_t last_shoot_count = 0;
    shoot_speed_stats_t *stats = &shoot->speed_stats;

    // 无新弹丸不更新
    if (My_Judge.shoot_count == last_shoot_count)
    {
        return;
    }

    last_shoot_count = My_Judge.shoot_count;
    float current_speed = My_Judge.org_info->shoot_data.initial_speed;

    // 记录最近一颗弹丸初速度，便于调试观察
    stats->latest_speed = current_speed;

    // 剔除不在[22, 27]范围内的弹速
    if (current_speed < 22.0f || current_speed > 27.0f)
    {
        stats->abnormal_count++;
        return;
    }

    // 环形缓冲区写入
    uint8_t write_idx = stats->speed_count % SPEED_STAT_COUNT;
    stats->speed_buf[write_idx] = current_speed;
    stats->speed_count++;
    // 更新最旧数据索引
    if (stats->speed_count > SPEED_STAT_COUNT)
    {
        stats->head_index = (write_idx + 1) % SPEED_STAT_COUNT;
    }

    // 有效数据少于2个时统计无效
    uint8_t valid_count = (stats->speed_count < SPEED_STAT_COUNT) ? stats->speed_count : SPEED_STAT_COUNT;
    if (valid_count < 2)
    {
        stats->is_valid = false;
        return;
    }

    // 第一遍遍历: 计算均值、找最大最小值
    float sum = 0.0f;
    float min_val = stats->speed_buf[(stats->head_index + valid_count - 1) % SPEED_STAT_COUNT];
    float max_val = min_val;
    for (uint8_t i = 0; i < valid_count; i++)
    {
        uint8_t idx = (stats->head_index + i) % SPEED_STAT_COUNT;
        float val = stats->speed_buf[idx];
        sum += val;
        if (val < min_val)
            min_val = val;
        if (val > max_val)
            max_val = val;
    }
    stats->average = sum / (float)valid_count;
    stats->range = max_val - min_val;

    // 第二遍遍历: 计算方差
    float variance_sum = 0.0f;
    for (uint8_t i = 0; i < valid_count; i++)
    {
        uint8_t idx = (stats->head_index + i) % SPEED_STAT_COUNT;
        float diff = stats->speed_buf[idx] - stats->average;
        variance_sum += diff * diff;
    }
    stats->variance = variance_sum / (float)valid_count;
    stats->std_dev = sqrtf(stats->variance);
    stats->is_valid = true;

    // 统计最近50发中弹速超过24.9的数量
    stats->high_speed_count = 0;
    for (uint8_t i = 0; i < valid_count; i++)
    {
        uint8_t idx = (stats->head_index + i) % SPEED_STAT_COUNT;
        if (stats->speed_buf[idx] > 24.9f)
        {
            stats->high_speed_count++;
        }
    }
}

/**
 * @brief  首弹射出时间更新
 * @param  shoot: 发射机构结构体指针
 * @note   检测 shoot_ctrl_flag 上升沿记录拨弹开始时间，
 *         检测到新弹丸射出时记录首弹射出时间，仅记录最近10发(ms)
 */
static void Shoot_First_Bullet_Time_Update(shoot_t *shoot)
{
    static bool last_shoot_flag = 0;
    shoot_first_bullet_info_t *info = &shoot->first_bullet_info;

    // 检测 shoot_ctrl_flag 上升沿 → 拨弹开始
    if (Balance.Shoot_Flag_struct.Shoot_Ctrl_Flag == 1 && last_shoot_flag == 0)
    {
        info->current_dail_start_index = info->count % FIRST_BULLET_TIME_COUNT;
        info->dail_start_time[info->current_dail_start_index] = HAL_GetTick();
        info->is_recording = true;
    }
    last_shoot_flag = Balance.Shoot_Flag_struct.Shoot_Ctrl_Flag;

    // 有新弹丸射出且正在记录 → 记录首弹射出时间
    if (info->is_recording)
    {
        static uint32_t last_shoot_count = 0;
        if (My_Judge.shoot_count != last_shoot_count)
        {
            last_shoot_count = My_Judge.shoot_count;
            // 记录首弹射出时刻
            info->first_shot_time[info->current_dail_start_index] = HAL_GetTick();
            // 计算时间差(ms)
            info->first_bullet_delta_time[info->current_dail_start_index] =
                (float)(info->first_shot_time[info->current_dail_start_index] -
                        info->dail_start_time[info->current_dail_start_index]);
            info->count++;
            info->is_recording = false;
        }
    }
}

/**
 * @brief  拨盘PID计算
 * @param  shoot: 发射机构结构体指针
 * @note   根据 ctrl_mode 选择角度环(串级PID)或速度环(单级PID)，
 *         计算结果写入 dail_output 并通过 W_iqControl 下发至电机
 */
static void Shoot_Dial_Pid_Cal(shoot_t *shoot)
{
    // 获取电机反馈值
    float measure_angle = (float)shoot->extern_input.dial.encoder_sum;
    float measure_speed = (float)shoot->extern_input.dial.speed;

    if (shoot->dail_info.ctrl_mode == DIAL_ANGLE)
    {
        // 角度环串级PID: 外环角度 → 内环速度 → 电流输出
        shoot->dail_info.dail_pid->position_outer->target = shoot->dail_info.target_anglesum;
        shoot->dail_info.dail_pid->position_outer->measure = measure_angle;
        pid_err_cal(shoot->dail_info.dail_pid->position_outer);
        single_pid_ctrl(shoot->dail_info.dail_pid->position_outer);

        // 内环速度目标值 = 外环PID输出
        shoot->dail_info.dail_pid->position_inner->target = shoot->dail_info.dail_pid->position_outer->out;
        shoot->dail_info.dail_pid->position_inner->measure = measure_speed;
        pid_err_cal(shoot->dail_info.dail_pid->position_inner);
        single_pid_ctrl(shoot->dail_info.dail_pid->position_inner);

        // 最终输出 = 内环PID输出
        shoot->dail_info.dail_output = shoot->dail_info.dail_pid->position_inner->out;
    }
    else if (shoot->dail_info.ctrl_mode == DIAL_SPEED)
    {
        // 速度环单级PID
        shoot->dail_info.dail_pid->speed->target = shoot->dail_info.target_speed;
        shoot->dail_info.dail_pid->speed->measure = measure_speed;
        pid_err_cal(shoot->dail_info.dail_pid->speed);
        single_pid_ctrl(shoot->dail_info.dail_pid->speed);
        shoot->dail_info.dail_output = shoot->dail_info.dail_pid->speed->out;
    }
    else
    {
        // DIAL_SLEEP: 卸力
        shoot->dail_info.dail_output = 0;
    }

    // S_SLEEP 状态强制输出为0
    if (shoot->state == S_SLEEP)
    {
        shoot->dail_info.dail_output = 0;
    }
}

/**
 * @brief  堵转处理进入次数统计
 * @param  shoot: 发射机构结构体指针
 * @param  pre_state: 状态机更新前状态
 * @param  cur_state: 状态机更新后状态
 * @retval None
 * @note   仅统计从 S_SINGLE_SHOOT / S_REPEAT_SHOOT 进入堵转处理状态的跳变
 */
static void Shoot_Stuck_Statistics_Update(shoot_t *shoot, shoot_state_e pre_state, shoot_state_e cur_state)
{
    // 判断更新前是否处于需要统计的发射状态
    bool from_shoot_state = (pre_state == S_SINGLE_SHOOT || pre_state == S_REPEAT_SHOOT);

    // 判断更新后是否进入堵转处理状态
    bool to_stuck_handle_state = (cur_state == S_HANDLE_STUCK_REVERSE || cur_state == S_HANDLE_STUCK_RELOAD);

    // 仅在状态跳变满足条件时累计堵转处理次数
    if (from_shoot_state && to_stuck_handle_state)
    {
        shoot->stuck_statistics_cnt++;
    }
}

/* Public functions --------------------------------------------------------*/
/**
 * @brief  发射机构总控入口 — 每1ms调用一次
 * @param  shoot: 发射机构结构体指针
 * @note   读取外部输入 → 边沿检测 → 状态机 → 各功能模块 → PID计算
 */
void Shoot_Work(shoot_t *shoot)
{
    Shoot_Extern_Update(shoot);
    shoot_state_e pre_state = shoot->state;

    // 边沿检测用的静态变量
    static bool last_enable_shoot_flag = false;
    static bool last_shoot_ctrl_flag = false;

    // ============================ 读取外部输入 ============================
    // (发射标志已由 Shoot_Extern_Update() 统一写入 shoot->extern_input.shoot_flag)

    // ============================ 边沿检测 ============================
    // enable_shoot_flag 上升沿检测: 用于触发 S_SLEEP → S_INIT 跳转
    bool enable_rising = (shoot->extern_input.shoot_flag.Enable_Shoot_Flag == true && last_enable_shoot_flag == false);

    // S_SLEEP → S_INIT: 检测到拨轮上拨上升沿，启动初始化
    if (shoot->state == S_SLEEP && enable_rising)
    {
        shoot->adapt_info.final_fric_target_speed = (uint16_t)shoot->adapt_info.cfg.init_fric_target_speed; // 启动摩擦轮
        shoot->adapt_info.add_fric_speed = 0;
        shoot->adapt_info.dec_fric_speed = 0;
        shoot->state = S_INIT;
        shoot->state_tick = 0;
        shoot->stuck_block_tick = 0;
    }

    // ============================ 功能模块调度 ============================
    // 1. 外部数据更新: 注入外部数据到 extern_input

    // 2. 状态机更新: 写入 ctrl_mode / target_anglesum / target_speed / state
    Shoot_State_Machine(shoot);

    // 2.1 堵转处理进入次数统计: 检测单发/连发切入堵转处理状态的跳变
    Shoot_Stuck_Statistics_Update(shoot, pre_state, shoot->state);

    // 3. 热量限制: 根据裁判系统热量设置连发时拨盘目标转速
    Shoot_Heat_Limit(shoot);

    // 4. 摩擦轮控制与弹速自适应: 设置摩擦轮目标转速
    Shoot_Fric_Ctrl(shoot);

    // 5. 弹速统计: 更新均值/极差/标准差/方差 (debug用途)
    Shoot_Speed_Statistics_Update(shoot);

    // 6. 首弹时间更新: 记录拨弹到射出时间差
    Shoot_First_Bullet_Time_Update(shoot);

    // 7. 拨盘PID计算: 根据 ctrl_mode 计算电流输出并下发电机
    Shoot_Dial_Pid_Cal(shoot);

    // ============================ 保存边沿检测值 ============================
    // 保存本周期值供下一周期边沿检测
    last_enable_shoot_flag = shoot->extern_input.shoot_flag.Enable_Shoot_Flag;
    last_shoot_ctrl_flag = shoot->extern_input.shoot_flag.Shoot_Ctrl_Flag;
}