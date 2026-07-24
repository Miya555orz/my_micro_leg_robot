#ifndef __communicate_H_

#define __communicate_H_

#include "rp_device_config.h"

#include "rc_sensor.h"

#include "drv_can.h"

#include "can_protocol.h"

#include "shoot.h"

#include "rp_math.h"

// 上板给下板上传的数据结构体

typedef struct

{

    float yaw_imu_angle; // yaw 陀螺仪角度

    float yaw_imu_speed; // yaw 陀螺仪角速度

    float pitch_imu_angle; // pitch 陀螺仪角度

    float pitch_imu_speed; // pitch 陀螺仪角速度

    float vision_target_yaw; // 视觉目标 yaw

    float vision_target_pitch; // 视觉目标 pitch

    float pitch_mec_angle; // pitch 机械角

    __packed union {

        uint32_t realtime_flag;

        __packed struct

        {

            uint8_t pitch_motor_online : 1; // pitch 电机在线

            uint8_t L_fric_online : 1; // 左摩擦轮在线

            uint8_t R_fric_online : 1;   // 右摩擦轮在线
            uint8_t L_fric_spinning : 1; // 左摩擦轮在转
            uint8_t R_fric_spinning : 1; // 右摩擦轮在转

            uint8_t is_find_target : 1;    // 识别到目标
            uint8_t vision_detect_num : 4; // 锁到几号（0哨兵，6前哨，15为没锁到)
            uint8_t hit_enable : 1;        // 允许击打

            uint8_t is_keep_shoot : 1; // 连发模式

            uint8_t is_vision_online : 1; // 视觉在线

        } flag;
    };

} Board_Rx_Info_t;

// 下板给上板的数据结构体

typedef struct

{

    float yaw_mec_imu;

    float fric_target_speed;

    float pitch_output;

    __packed union {

        uint32_t realtime_flag;

        __packed struct

        {

            uint8_t our_color_flag : 1;              // 本方颜色
            uint8_t is_rc_online : 1;                // 遥控器在线
            uint8_t is_ready_shoot : 1;              // 可以马上发射
            uint8_t is_game_in_progress : 1;         // 比赛进行中
            uint8_t is_big_energy_engine_mode : 1;   // 大符模式
            uint8_t is_small_energy_engine_mode : 1; // 小符模式
            uint8_t is_outpost_mode : 1;             // 前哨模式
            uint8_t is_hero_mode : 1;                // 英雄模式

        } bit;

    } flag;

} Board_Tx_Info_t;

typedef struct
{

    dev_work_state_t status; // 工作状态

    uint32_t send_time; // 发包时间

    uint32_t rx_tick; // 收到数据时间戳

    uint8_t offline_cnt_pack_1; // 各包离线计数

    uint8_t offline_cnt_pack_2;

    uint8_t offline_cnt_pack_3;

    uint8_t offline_cnt_pack_4;

    uint8_t offline_cnt_max; // 离线计数上限

} Board_HeartBeat_t;

extern Board_Tx_Info_t Board_Tx_Info;

extern Board_Rx_Info_t Board_Rx_Info;

extern Board_HeartBeat_t Board_HeartBeat;

void D_Board_HeartBeat(void);

void Board_Rx_C1(uint8_t *rxbuf);

void Board_Rx_C2(uint8_t *rxbuf);

void Board_Rx_C3(uint8_t *rxbuf);

void Board_Rx_C4(uint8_t *rxbuf);

void Send_To_Up_Board(void);

#endif
