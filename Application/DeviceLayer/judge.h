#ifndef __JUDGE_H

#define __JUDGE_H

/* Includes ------------------------------------------------------------------*/

#include "stm32h7xx_hal.h"

#include "rp_config.h"

#include "judge_protocol.h"

#include "communicate.h"

/* Exported macro ------------------------------------------------------------*/

#define JUDGE_OFFLINE_CNT_MAX 1000

typedef struct Judge_Org_Info_struct_t {

    ext_rfid_status_t rfid_status;

    ext_game_status_t game_status;

    ext_game_robot_status_t game_robot_status;

    ext_power_heat_data_t power_heat_data;

    ext_shoot_data_t shoot_data;

    ext_game_robot_pos_t game_robot_pos;

    ext_robot_hurt_t ext_robot_hurt;

    ext_game_robot_HP_t ext_game_robot_HP;

    ext_projectile_allowance_t projectile_allowance;

    robot_interaction_data_t interactive_header_data;

    radio_information_data_t radio_information_data;

    dart_state_data_t radio_dart_state_data;
} Judge_Org_Info_t; // 原始信息

typedef struct

{

    int16_t chassis_power_buffer; // 底盘缓存功率

    int32_t chassis_out_put_max; // 底盘最大输出

    uint16_t shooter_cooling_limit; // 机器人 42mm 枪口热量上限

    uint16_t shooter_cooling_heat; // 机器人 42mm 枪口热量

    uint8_t my_color; //  0红色 1蓝色

    uint8_t hurt_type; // 伤害种类

    uint16_t chassis_power_limit; // 底盘功率限制

    uint16_t shooter_id1_17mm_speed_limit; // 射速上限

    uint16_t remain_HP; // 剩余血量

    uint8_t game_progress; // 比赛状态

    uint8_t rfid;

    float shooting_speed;

} Judge_Info_t;

typedef struct

{

    uint16_t offline_cnt_max;

    dev_work_state_t status;

    uint16_t offline_cnt;

} Judge_Status_t;

typedef struct

{

    Judge_Org_Info_t *org_info;

    Judge_Info_t *info;

    Judge_Status_t *status;

    uint32_t shoot_count; // 全局发弹计数器

} My_Judge_t;

extern My_Judge_t My_Judge;

void My_Judge_Realtime_Task(My_Judge_t *my_judge);

void My_Judge_Update(My_Judge_t *my_judge);

void judge_update(uint16_t id, uint8_t *rxBuf);

#endif
