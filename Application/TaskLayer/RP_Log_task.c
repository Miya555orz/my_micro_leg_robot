#include "RP_Log_task.h"
#include "Balance.h"
#include "RP_Log.h"
#include "Straight_Leg_Calc.h"
#include "buzzer.h"
#include "cap.h"

#include "usart.h"

void StartRP_LogTask(void const *argument)
{
    for (;;)
    {
        /*---------------- 日志 begin-------------------- */

        My_Log_Check_Vision_Flag_Edge_Beep();

        //  PowerWarning_Buzzer_Check();
        // CapLowVoltage_Buzzer_Check();
        /*---------------- 日志 end-------------------- */
        g_rp_log.work(&g_rp_log);
        buzzer.work(&buzzer);
        osDelay(1);
    }
}

void My_Log_Check_Vision_Flag_Edge_Beep(void)
{
    const uint32_t beep_on_time_ms = 400U;
    const uint32_t beep_off_time_ms = 400U;

    static uint8_t is_inited = 0U;
    static uint8_t last_auto_catch_flag = 0U;
    static uint8_t last_outpost_flag = 0U;
    static uint8_t last_engi_flag = 0U;
    static uint8_t pending_beep_cnt = 0U;
    static uint8_t is_beep_on = 0U;
    static uint32_t stage_start_tick = 0U;

    uint32_t now_tick = HAL_GetTick();

    uint8_t current_auto_catch_flag = (uint8_t)Balance.Vision.Auto_Catch_Flag;
    uint8_t current_outpost_flag = (uint8_t)Balance.Vision.Auto_Catch_Outpost_Flag;
    uint8_t current_engi_flag = (uint8_t)Balance.Vision.Auto_Catch_Engi_Flag;

    // 首次进入仅同步历史值，避免上电后误判为跳变
    if (is_inited == 0U)
    {
        is_inited = 1U;
        last_auto_catch_flag = current_auto_catch_flag;
        last_outpost_flag = current_outpost_flag;
        last_engi_flag = current_engi_flag;
        return;
    }

    // Auto_Catch_Flag 上升沿短鸣一声，下降沿短鸣两声
    if (current_auto_catch_flag != last_auto_catch_flag)
    {
        if (current_auto_catch_flag != 0U)
        {
            pending_beep_cnt += 1U;
        }
        else
        {
            pending_beep_cnt += 2U;
        }
        last_auto_catch_flag = current_auto_catch_flag;
    }

    // Auto_Catch_Outpost_Flag 上升沿短鸣一声，下降沿短鸣两声
    if (current_outpost_flag != last_outpost_flag)
    {
        if (current_outpost_flag != 0U)
        {
            pending_beep_cnt += 1U;
        }
        else
        {
            pending_beep_cnt += 2U;
        }
        last_outpost_flag = current_outpost_flag;
    }

    // Auto_Catch_Engi_Flag 上升沿短鸣一声，下降沿短鸣两声
    if (current_engi_flag != last_engi_flag)
    {
        if (current_engi_flag != 0U)
        {
            pending_beep_cnt += 1U;
        }
        else
        {
            pending_beep_cnt += 2U;
        }
        last_engi_flag = current_engi_flag;
    }

    // 没有待播放短鸣并且当前不在发声时，直接返回
    if (pending_beep_cnt == 0U && is_beep_on == 0U)
    {
        return;
    }

    // 空闲且有待播放请求时，立即开启一声短鸣
    if (is_beep_on == 0U && pending_beep_cnt > 0U)
    {
        is_beep_on = 1U;
        stage_start_tick = now_tick;
        Buzzer_Normal_On();
        return;
    }

    // 发声阶段到时后关闭蜂鸣器并进入间隔阶段
    if (is_beep_on == 1U)
    {
        if ((now_tick - stage_start_tick) >= beep_on_time_ms)
        {
            is_beep_on = 0U;
            stage_start_tick = now_tick;
            Buzzer_Normal_Off();

            // 一次短鸣播放完成后减少待播放计数
            if (pending_beep_cnt > 0U)
            {
                pending_beep_cnt--;
            }
        }
        return;
    }

    // 间隔到时后，如果仍有待播放请求则开启下一声短鸣
    if ((now_tick - stage_start_tick) >= beep_off_time_ms)
    {
        if (pending_beep_cnt > 0U)
        {
            is_beep_on = 1U;
            stage_start_tick = now_tick;
            Buzzer_Normal_On();
        }
    }
}

void My_Log_print_leg_spring_compensation(void)
{
    RP_LOG_INFO("L_Front: %f, L_Back: %f, \r\nR_Front: %f, R_Back: %f",
                Chassis.Leg_Unit[L_Leg]->force->T_Spring_Compensation_Front,
                Chassis.Leg_Unit[L_Leg]->force->T_Spring_Compensation_Back,
                Chassis.Leg_Unit[R_Leg]->force->T_Spring_Compensation_Front,
                Chassis.Leg_Unit[R_Leg]->force->T_Spring_Compensation_Back);
}

void My_Log_print_leg_gravity_torque(void)
{
    RP_LOG_INFO("L_Leg_Gravity_Torque: %f, R_Leg_Gravity_Torque: %f",
                Chassis.Leg_Unit[L_Leg]->force->Leg_Gravity_Torque,
                Chassis.Leg_Unit[R_Leg]->force->Leg_Gravity_Torque);
}

void My_Log_print_posture_degree(void)
{
    RP_LOG_INFO("pitch: %f, roll: %f, yaw: %f, pitch_v: %f, roll_v: %f, yaw_v: %f",
                Chassis.Posture->info->pitch_degree,
                Chassis.Posture->info->roll_degree,
                Chassis.Posture->info->yaw_degree,
                Chassis.Posture->info->pitch_v_degree,
                Chassis.Posture->info->roll_v_degree,
                Chassis.Posture->info->yaw_v_degree);
}

void My_Log_print_posture_world(void)
{
    RP_LOG_INFO("x_world: %f, y_world: %f, z_world: %f",
                Chassis.Posture->info->x_world,
                Chassis.Posture->info->y_world,
                Chassis.Posture->info->z_world);
}

void My_Log_print_phi1_phi4(void)
{
    RP_LOG_INFO("L_phi1_degree: %f,L_phi4_degree: %f,L_phi1_motor_angle: %f,L_phi4_motor_angle: %f ",
                Chassis.Leg_Unit[L_Leg]->Link->info->angle->phi1_degree,
                Chassis.Leg_Unit[L_Leg]->Link->info->angle->phi4_degree,
                Chassis.Sd->motor[L_F_Sd_M]->rx_info->motor_angle,
                Chassis.Sd->motor[L_B_Sd_M]->rx_info->motor_angle);
}

void My_Log_print_leg_state_info(void)
{
    State_info_t *L_info = Chassis.Leg_Unit[L_Leg]->Straight->info;
    State_info_t *R_info = Chassis.Leg_Unit[R_Leg]->Straight->info;
    RP_LOG_INFO("L_thetal: %f, L_thetald1: %f, L_s: %f, L_sd1: %f, L_thetab: %f, L_thetabd1: %f",
                L_info->thetal, L_info->thetald1, L_info->s, L_info->sd1, L_info->thetab, L_info->thetabd1);
    RP_LOG_INFO("R_thetal: %f, R_thetald1: %f, R_s: %f, R_sd1: %f, R_thetab: %f, R_thetabd1: %f",
                R_info->thetal, R_info->thetald1, R_info->s, R_info->sd1, R_info->thetab, R_info->thetabd1);
}

void My_Log_print_leg_state_err(void)
{
    State_info_t *L_info = Chassis.Leg_Unit[L_Leg]->Straight->info;
    State_info_t *R_info = Chassis.Leg_Unit[R_Leg]->Straight->info;
    RP_LOG_INFO("L_thetal_err: %f, L_thetald1_err: %f, L_s_err: %f, L_sd1_err: %f, L_thetab_err: %f, L_thetabd1_err: %f",
                L_info->thetal_err, L_info->thetald1_err, L_info->s_err, L_info->sd1_err, L_info->thetab_err, L_info->thetabd1_err);
    RP_LOG_INFO("R_thetal_err: %f, R_thetald1_err: %f, R_s_err: %f, R_sd1_err: %f, R_thetab_err: %f, R_thetabd1_err: %f",
                R_info->thetal_err, R_info->thetald1_err, R_info->s_err, R_info->sd1_err, R_info->thetab_err, R_info->thetabd1_err);
}

void My_Log_print_leg_F_support(void)
{
    RP_LOG_INFO("L_F_support: %f, R_F_support: %f",
                Chassis.Leg_Unit[L_Leg]->force->F_support,
                Chassis.Leg_Unit[R_Leg]->force->F_support);
}

void My_Log_print_leg_sd1(Leg_e leg)
{
    State_info_t *info = Chassis.Leg_Unit[leg]->Straight->info;
    const char *leg_name = (leg == L_Leg) ? "L" : "R";
    RP_LOG_INFO("%s_sd1: %f, %s_target_sd1: %f", leg_name, info->sd1, leg_name, info->target_sd1);
}

void My_Log_print_power_cap(void)
{
    RP_LOG_INFO("chassis_power: %d, cap_v: %f, cap_i: %f, power_buffer: %d",
                cap.info.rx.now_chassis_power, cap.info.cap_u, cap.info.cap_i, My_Judge.info->chassis_power_buffer);
}

void My_Log_print_rescue_state(void)
{
    Rescue_state_machine_e state = Chassis.rescue_info->rescue_state_mac;
    const char *state_name;
    switch (state)
    {
    case Reset:
        state_name = "Reset";
        break;
    case ForwardFlip_and_L_Rollover:
        state_name = "ForwardFlip_and_L_Rollover";
        break;
    case ForwardFlip_and_R_Rollover:
        state_name = "ForwardFlip_and_R_Rollover";
        break;
    case BackwardFlip_and_L_Rollover:
        state_name = "BackwardFlip_and_L_Rollover";
        break;
    case BackwardFlip_and_R_Rollover:
        state_name = "BackwardFlip_and_R_Rollover";
        break;
    case L_Rollover:
        state_name = "L_Rollover";
        break;
    case R_Rollover:
        state_name = "R_Rollover";
        break;
    case ForwardFlip:
        state_name = "ForwardFlip";
        break;
    case BackwardFlip:
        state_name = "BackwardFlip";
        break;
    case PRNormalBackwardLeg:
        state_name = "PRNormalBackwardLeg";
        break;
    case RetractLegs:
        state_name = "RetractLegs";
        break;
    default:
        state_name = "Unknown";
        break;
    }
    RP_LOG_INFO("Rescue_state: %s, L_Tp: %f, R_Tp: %f",
                state_name,
                Chassis.Leg_Unit[L_Leg]->force->Tp_target,
                Chassis.Leg_Unit[R_Leg]->force->Tp_target);
}

/**
 * @brief  功率缓冲警告蜂鸣器检测
 * @note  当chassis_power_buffer <= 40时:
 *        - 蜂鸣器响5秒后停止
 *        - 如果缓冲仍然不足，则以200ms间隔持续蜂鸣
 */
void PowerWarning_Buzzer_Check(void)
{
    /* 功率警告蜂鸣器状态变量 */
    static uint8_t power_warning_active = 0;          // 0=空闲, 1=发声中
    static uint32_t power_warning_start_tick = 0;     // 开始发声时HAL_GetTick()记录的时间
    static uint8_t power_warning_in_gap = 0;          // 0=发声中, 1=5s后的200ms间隙期
    static uint32_t power_warning_gap_start_tick = 0; // 间隙期开始时HAL_GetTick()记录的时间

    uint32_t current_tick = HAL_GetTick();

    if (My_Judge.info->chassis_power_buffer <= 5)
    {
        /* 缓冲不足 -> 处理蜂鸣器状态机 */

        if (power_warning_active == 0 && power_warning_in_gap == 0 && My_Judge.status->status == DEV_ONLINE)
        {
            /* 当前未发声且不在间隙期 -> 开始蜂鸣 */
            power_warning_active = 1;
            power_warning_start_tick = current_tick;
            Buzzer_Normal_On();
        }
        else if (power_warning_active == 1)
        {
            /* 正在发声 -> 检查是否已达5秒 */
            if (current_tick - power_warning_start_tick >= 5000U)
            {
                /* 5秒到达 -> 停止并进入间隙期 */
                power_warning_active = 0;
                power_warning_in_gap = 1;
                power_warning_gap_start_tick = current_tick;
                Buzzer_Normal_Off();
            }
        }
        else if (power_warning_in_gap == 1)
        {
            /* 处于间隙期 -> 检查是否已达200ms */
            if (current_tick - power_warning_gap_start_tick >= 1U)
            {
                /* 间隙结束 -> 如果缓冲仍不足则重新开始蜂鸣 */
                power_warning_in_gap = 0;
                /* 由外层if判断是否需要重新启动蜂鸣 */
            }
        }
    }
    else
    {
        /* 缓冲恢复正常 -> 停止任何警告 */
        if (power_warning_active || power_warning_in_gap)
        {
            power_warning_active = 0;
            power_warning_in_gap = 0;
            Buzzer_Normal_Off();
        }
    }
}

/**
 * @brief  超级电容低压警告蜂鸣器检测
 * @note  当超级电容电压 < 19V时:
 *        - 蜂鸣器响5秒后停止
 *        - 如果电压仍然不足，则以200ms间隔持续蜂鸣
 */
void CapLowVoltage_Buzzer_Check(void)
{
    /* 低压警告蜂鸣器状态变量 */
    static uint8_t cap_low_warning_active = 0;          // 0=空闲, 1=发声中
    static uint32_t cap_low_warning_start_tick = 0;     // 开始发声时HAL_GetTick()记录的时间
    static uint8_t cap_low_warning_in_gap = 0;          // 0=发声中, 1=5s后的200ms间隙期
    static uint32_t cap_low_warning_gap_start_tick = 0; // 间隙期开始时HAL_GetTick()记录的时间

    uint32_t current_tick = HAL_GetTick();

    if (cap.info.cap_u < 19.0f)
    {
        /* 电容低压 -> 处理蜂鸣器状态机 */

        if (cap_low_warning_active == 0 && cap_low_warning_in_gap == 0)
        {
            /* 当前未发声且不在间隙期 -> 开始蜂鸣 */
            cap_low_warning_active = 1;
            cap_low_warning_start_tick = current_tick;
            Buzzer_Normal_On();
        }
        else if (cap_low_warning_active == 1)
        {
            /* 正在发声 -> 检查是否已达5秒 */
            if (current_tick - cap_low_warning_start_tick >= 5000U)
            {
                /* 5秒到达 -> 停止并进入间隙期 */
                cap_low_warning_active = 0;
                cap_low_warning_in_gap = 1;
                cap_low_warning_gap_start_tick = current_tick;
                Buzzer_Normal_Off();
            }
        }
        else if (cap_low_warning_in_gap == 1)
        {
            /* 处于间隙期 -> 检查是否已达200ms */
            if (current_tick - cap_low_warning_gap_start_tick >= 200U)
            {
                /* 间隙结束 -> 如果电压仍低则重新开始蜂鸣 */
                cap_low_warning_in_gap = 0;
            }
        }
    }
    else
    {
        /* 电压恢复正常 -> 停止任何警告 */
        if (cap_low_warning_active || cap_low_warning_in_gap)
        {
            cap_low_warning_active = 0;
            cap_low_warning_in_gap = 0;
            Buzzer_Normal_Off();
        }
    }
}