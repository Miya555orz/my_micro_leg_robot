#include "communicate.h"
#include "crc.h"
#include "drv_can.h"
#include "string.h"

Board_Tx_Info_t Board_Tx_Info;
Board_Rx_Info_t Board_Rx_Info;
Board_HeartBeat_t Board_HeartBeat =
    {
        .offline_cnt_max = 100,
};

uint8_t board_tx_buf_1[8];
uint8_t board_tx_buf_2[8];

static void Board_Tx_D1(void)
{
    memcpy(&board_tx_buf_1[0], &Board_Tx_Info.fric_target_speed, 4);
    memcpy(&board_tx_buf_1[4], &Board_Tx_Info.pitch_output, 4);
    CAN_SendData(&hfdcan3, 0xD1, board_tx_buf_1);
}

static void Board_Tx_D2(void)
{
    uint32_t reserve = 0;
    memcpy(&board_tx_buf_2[0], &Board_Tx_Info.flag.realtime_flag, 4);
    memcpy(&board_tx_buf_2[4], &reserve, 4);
    CAN_SendData(&hfdcan3, 0xD2, board_tx_buf_2);
}

void Board_Rx_C1(uint8_t *rxbuf)
{
    memcpy(&Board_Rx_Info.yaw_imu_angle, &rxbuf[0], 4);
    memcpy(&Board_Rx_Info.yaw_imu_speed, &rxbuf[4], 4);
    Board_HeartBeat.offline_cnt_pack_1 = 0;
}

void Board_Rx_C2(uint8_t *rxbuf)
{
    memcpy(&Board_Rx_Info.pitch_imu_angle, &rxbuf[0], 4);
    memcpy(&Board_Rx_Info.pitch_imu_speed, &rxbuf[4], 4);
    Board_HeartBeat.offline_cnt_pack_2 = 0;
}

void Board_Rx_C3(uint8_t *rxbuf)
{
    memcpy(&Board_Rx_Info.vision_target_yaw, &rxbuf[0], 4);
    memcpy(&Board_Rx_Info.vision_target_pitch, &rxbuf[4], 4);
    Board_HeartBeat.offline_cnt_pack_3 = 0;
}

void Board_Rx_C4(uint8_t *rxbuf)
{
    memcpy(&Board_Rx_Info.pitch_mec_angle, &rxbuf[0], 4);
    memcpy(&Board_Rx_Info.realtime_flag, &rxbuf[4], 4);
    Board_HeartBeat.offline_cnt_pack_4 = 0;
}

void Board_Tx_Update(Board_Tx_Info_t *Board_Tx_Info)
{
    Board_Tx_Info->fric_target_speed = shoot.adapt_info.final_fric_target_speed;
    Board_Tx_Info->pitch_output = gimbal.base_info.output_gimbal_p;
    // 我方颜色标志位更新
    Board_Tx_Info->flag.bit.our_color_flag = My_Judge.info->my_color;
    // 遥控器在线标志位更新
    if (RC_ONLINE)
    {
        Board_Tx_Info->flag.bit.is_rc_online = 1;
    }
    else
    {
        Board_Tx_Info->flag.bit.is_rc_online = 0;
    }

    // 可以马上发射标志位：发射相关电机在线 && 发射机构就绪 && 自瞄开启
    if (ALL_SHOOT_MOTOR_ONLINE &&
        (shoot.state == S_WAITING) &&
        Balance.Vision.Auto_Catch_Flag == true)
    {
        Board_Tx_Info->flag.bit.is_ready_shoot = 1;
    }
    else
    {
        Board_Tx_Info->flag.bit.is_ready_shoot = 0;
    }

    // 5s倒计时开比赛标志位
    if (My_Judge.info->game_progress == 3 ||
        My_Judge.info->game_progress == 4)
    {
        Board_Tx_Info->flag.bit.is_game_in_progress = 1;
    }
    else
    {
        Board_Tx_Info->flag.bit.is_game_in_progress = 0;
    }

    /*-------打符标志位更新-------*/
    if (Balance.Vision.Auto_Catch_Engi_Flag == true)
    {
        // #define TEST_BIG_ENERGY_ENGINE
        // #define TEST_SMALL_ENERGY_ENGINE
#ifdef TEST_BIG_ENERGY_ENGINE
        Board_Tx_Info->flag.bit.is_big_energy_engine_mode = 1;
#endif

#ifdef TEST_SMALL_ENERGY_ENGINE
        Board_Tx_Info->flag.bit.is_small_energy_engine_mode = 1
#endif

#if !defined(TEST_BIG_ENERGY_ENGINE) && !defined(TEST_SMALL_ENERGY_ENGINE)
            if (My_Judge.org_info->game_status.stage_remain_time < (60 * 7 - 60 * 3)) // 根据比赛剩余时间自动判断大小符
        {
            Board_Tx_Info->flag.bit.is_big_energy_engine_mode = 1;
            Board_Tx_Info->flag.bit.is_small_energy_engine_mode = 0;
        }
        else
        {
            Board_Tx_Info->flag.bit.is_big_energy_engine_mode = 0;
            Board_Tx_Info->flag.bit.is_small_energy_engine_mode = 1;
        }
#endif
    }
    else
    {
        Board_Tx_Info->flag.bit.is_big_energy_engine_mode = 0;
        Board_Tx_Info->flag.bit.is_small_energy_engine_mode = 0;
    }

    Board_Tx_Info->flag.bit.is_outpost_mode = Balance.Vision.Auto_Catch_Outpost_Flag; // 前哨模式标志位更新
    Board_Tx_Info->yaw_mec_imu = gimbal.base_info.yaw_imu_angle_target;
}

void Send_To_Up_Board(void)
{
    Board_Tx_Update(&Board_Tx_Info);
    Board_Tx_D1();
    Board_Tx_D2();
}

void D_Board_HeartBeat(void)
{
    Board_HeartBeat.offline_cnt_pack_1++;
    Board_HeartBeat.offline_cnt_pack_2++;
    Board_HeartBeat.offline_cnt_pack_3++;
    Board_HeartBeat.offline_cnt_pack_4++;
    if (Board_HeartBeat.offline_cnt_pack_1 > Board_HeartBeat.offline_cnt_max ||
        Board_HeartBeat.offline_cnt_pack_2 > Board_HeartBeat.offline_cnt_max ||
        Board_HeartBeat.offline_cnt_pack_3 > Board_HeartBeat.offline_cnt_max ||
        Board_HeartBeat.offline_cnt_pack_4 > Board_HeartBeat.offline_cnt_max)
    {
        Board_HeartBeat.offline_cnt_pack_1 = Board_HeartBeat.offline_cnt_max;
        Board_HeartBeat.offline_cnt_pack_2 = Board_HeartBeat.offline_cnt_max;
        Board_HeartBeat.offline_cnt_pack_3 = Board_HeartBeat.offline_cnt_max;
        Board_HeartBeat.offline_cnt_pack_4 = Board_HeartBeat.offline_cnt_max;
        Board_HeartBeat.status = DEV_OFFLINE;
    }
    else if (Board_HeartBeat.status == DEV_OFFLINE)
    {
        Board_HeartBeat.status = DEV_ONLINE;
    }
}
