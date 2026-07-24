/* Includes ------------------------------------------------------------------*/

#include "judge.h"

#include "cap.h"

#include "drv_tick.h"

#include "imu_sensor.h"

Judge_Info_t Judge_Info;

Judge_Org_Info_t Judge_Org_Info;

Judge_Status_t Judge_Status =

    {

        .offline_cnt_max = JUDGE_OFFLINE_CNT_MAX,

        .status = DEV_OFFLINE,

        .offline_cnt = 0,

};

My_Judge_t My_Judge =

    {

        .org_info = &Judge_Org_Info,

        .info = &Judge_Info,

        .status = &Judge_Status,

        .shoot_count = 0, // 中断累加而来的发弹计数

};

/**

 * @brief  裁判系统实时任务，检测离线与状态更新

 * @param  My_Judge_t * my_judge

 * @retval None

 */

void My_Judge_Realtime_Task(My_Judge_t *my_judge)

{

    my_judge->status->offline_cnt++;

    if (my_judge->status->offline_cnt >= my_judge->status->offline_cnt_max)

    {

        my_judge->status->offline_cnt = my_judge->status->offline_cnt_max;

        my_judge->status->status = DEV_OFFLINE;
    }

    My_Judge_Update(my_judge);
}

/**

 * @brief  裁判系统数据分析后更新状态

 * @param  My_Judge_t * my_judge

 * @retval None

 */

void My_Judge_Update(My_Judge_t *my_judge)

{

    // 血量层层更迭并更新
    my_judge->info->remain_HP = my_judge->org_info->game_robot_status.current_HP;

    if (my_judge->org_info->game_robot_status.robot_id < 10) // 红方
    {

        my_judge->info->my_color = 0; //
    }

    else

    {

        my_judge->info->my_color = 1;
    }

    // 缓冲能量

    my_judge->info->chassis_power_buffer = my_judge->org_info->power_heat_data.buffer_energy;

    // 功率上限

    my_judge->info->chassis_power_limit = my_judge->org_info->game_robot_status.chassis_power_limit;

    // 热量上限

    my_judge->info->shooter_cooling_limit = my_judge->org_info->game_robot_status.shooter_barrel_heat_limit;

    // 弹丸初速度

    my_judge->info->shooting_speed = my_judge->org_info->shoot_data.initial_speed;

    // 现在热量

    my_judge->info->shooter_cooling_heat = my_judge->org_info->power_heat_data.shooter_17mm_barrel_heat;

    // 比赛状态

    my_judge->info->game_progress = my_judge->org_info->game_status.game_progress;

    // 伤害种类

    my_judge->info->hurt_type = my_judge->org_info->ext_robot_hurt.HP_deduction_reason;

    // RFID状态

    my_judge->info->rfid = my_judge->org_info->rfid_status.rfid_status;
}

uint16_t shoot_cnt11 = 0;

uint8_t warning;

void judge_update(uint16_t id, uint8_t *rxBuf)

{

    My_Judge.status->status = DEV_ONLINE;

    switch (id)

    {

    case ID_rfid_status:

        memcpy(&My_Judge.org_info->rfid_status, rxBuf, LEN_rfid_status);

        My_Judge.status->offline_cnt = 0;

        My_Judge.status->status = DEV_ONLINE;

        break;

    case ID_game_state:

        memcpy(&My_Judge.org_info->game_status, rxBuf, LEN_game_state);

        My_Judge.status->offline_cnt = 0;

        My_Judge.status->status = DEV_ONLINE;

        break;

    case ID_power_heat_data:

        memcpy(&My_Judge.org_info->power_heat_data, rxBuf, LEN_power_heat_data);

        My_Judge.status->offline_cnt = 0;

        My_Judge.status->status = DEV_ONLINE;

        break;

    case ID_game_robot_state:

        memcpy(&My_Judge.org_info->game_robot_status, rxBuf, LEN_game_robot_state);

        My_Judge.status->offline_cnt = 0;

        My_Judge.status->status = DEV_ONLINE;

        break;

    case ID_shoot_data:

        memcpy(&My_Judge.org_info->shoot_data, rxBuf, LEN_shoot_data);

        My_Judge.shoot_count++; // 每次收到射击数据，增加全局发弹计数

        My_Judge.status->offline_cnt = 0;

        My_Judge.status->status = DEV_ONLINE;

        break;

    case ID_game_robot_HP:

        memcpy(&My_Judge.org_info->ext_game_robot_HP, rxBuf, LEN_game_robot_HP);

        My_Judge.status->offline_cnt = 0;

        My_Judge.status->status = DEV_ONLINE;

        break;

    case ID_robot_hurt:

        memcpy(&My_Judge.org_info->ext_robot_hurt, rxBuf, LEN_robot_hurt);

        break;

    case ID_projectile_allowance:

        memcpy(&My_Judge.org_info->projectile_allowance, rxBuf, LEN_bullet_remaining);

        My_Judge.status->offline_cnt = 0;

        My_Judge.status->status = DEV_ONLINE;

        break;

    case ID_interactive_header_data:

        memcpy(&My_Judge.org_info->interactive_header_data, rxBuf, LEN_interactive_header_data);
        if (My_Judge.info->my_color == 0) // 红方
        {
            if (My_Judge.org_info->interactive_header_data.sender_id == 9)
            {
                if (My_Judge.org_info->interactive_header_data.data_cmd_id == Radio_information_data_id)
                {
                    memcpy(&My_Judge.org_info->radio_information_data, rxBuf, LEN_radio_information_data);
                }
                if (My_Judge.org_info->interactive_header_data.data_cmd_id == Radio_dart_state_data_id)
                {
                    memcpy(&My_Judge.org_info->radio_dart_state_data, rxBuf, LEN_radio_dart_state_data);
                }
            }
        }
        if (My_Judge.info->my_color == 1) // 蓝方
        {
            if (My_Judge.org_info->interactive_header_data.sender_id == 109)
            {
                if (My_Judge.org_info->interactive_header_data.data_cmd_id == Radio_information_data_id)
                {
                    memcpy(&My_Judge.org_info->radio_information_data, rxBuf, LEN_radio_information_data);
                }
                if (My_Judge.org_info->interactive_header_data.data_cmd_id == Radio_dart_state_data_id)
                {
                    memcpy(&My_Judge.org_info->radio_dart_state_data, rxBuf, LEN_radio_dart_state_data);
                }
            }
        }

        My_Judge.status->offline_cnt = 0;
        My_Judge.status->status = DEV_ONLINE;

        break;

    default:

        break;
    }
}
