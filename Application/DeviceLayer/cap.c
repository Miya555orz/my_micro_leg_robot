/**
 ******************************************************************************
 * @file    cap.c
 * @brief   电容管理 - OOP风格实现，包含无线充电和心跳检测
 ******************************************************************************
 */

#include "cap.h"
#include "drv_can.h"
#include "fdcan.h"
#include "rp_math.h"
#include "string.h"
#include "judge.h"

extern CAN_HandleTypeDef hfdcan3;

static void CAP_txMessage(cap_t *self);
static void CAP_setMessage(cap_t *self);
static void CAP_rxMessage(cap_t *self, uint8_t *rxBuf, uint32_t can_id);
static void CAP_Heartbeat(cap_t *self);

uint8_t cap_send_buf[8];

cap_t cap = {
    .state = CAP_OFFLINE,
    .w_state = WIRELESS_OFFLINE,

    .info.offline_cnt = 100,
    .info.offline_max_cnt = 100,

    .info.w_offline_cnt = 100,
    .info.w_offline_max_cnt = 100,

    .info.tx.chassis_power_buffer = 0,
    .info.tx.chassis_power_limit = 0,
    .info.tx.cap_power_out_limit = -300, // 放电功率
    .info.tx.cap_power_in_limit = +300,  // 充电功率

    #ifdef CAP_ENABLE
    .info.tx.bit_control.cap_switch = 1,
    #else
    .info.tx.bit_control.cap_switch = 0,
    #endif
    
    .info.tx.bit_control.turbo_mode = 0,
    .info.tx.bit_control.pre_charge_mode_en = 0,

    .setdata = CAP_setMessage,
    .txdata = CAP_txMessage,
    .update = CAP_rxMessage,
    .heartbeat = CAP_Heartbeat,
};

/**
 * @brief  超电心跳检测
 */
static void CAP_Heartbeat(cap_t *self)
{
    cap_info_t *info = &self->info;

    info->offline_cnt++;
    info->w_offline_cnt++;

    if (info->offline_cnt > info->offline_max_cnt)
    {
        info->offline_cnt = info->offline_max_cnt;
        self->state = CAP_OFFLINE;
    }
    else if (self->state == CAP_OFFLINE)
    {
        self->state = CAP_ONLINE;
    }

    if (info->w_offline_cnt > info->w_offline_max_cnt)
    {
        info->w_offline_cnt = info->w_offline_max_cnt;
        self->w_state = WIRELESS_OFFLINE;
    }
    else if (self->w_state == WIRELESS_OFFLINE)
    {
        self->w_state = WIRELESS_ONLINE;
    }
}

/**
 * @brief  更新发送数据（功率限制、缓冲能量），不执行CAN发送
 */
static void CAP_setMessage(cap_t *self)
{
    self->info.tx.chassis_power_limit = My_Judge.org_info->game_robot_status.chassis_power_limit;
    self->info.tx.chassis_power_buffer = My_Judge.org_info->power_heat_data.buffer_energy;
}

/**
 * @brief  CAN发送数据到电容板
 */
static void CAP_txMessage(cap_t *self)
{

    memcpy(cap_send_buf, &self->info.tx, sizeof(capboard_tx_info_t));
    CAN_SendData(&hfdcan3, MASTER_TO_CAP_ID, cap_send_buf);
}

/**
 * @brief  CAN接收数据处理（区分CAP_TO_MASTER_ID和WIRELESS_ID）
 */
static void CAP_rxMessage(cap_t *self, uint8_t *rxBuf, uint32_t can_id)
{
    if (can_id == CAP_TO_MASTER_ID)
    {
        memcpy(&self->info.rx, rxBuf, sizeof(capboard_rx_info_t));

        int16_t now_cap_V = self->info.rx.now_cap_V;
        int16_t now_cap_I = self->info.rx.now_cap_I;

        self->info.cap_u = int16_to_float(now_cap_V, 32000, -32000, 25.0f, 0.0f);
        self->info.cap_i = int16_to_float(now_cap_I, 32000, -32000, 16.0f, -16.0f);

        self->info.offline_cnt = 0;
    }
    else if (can_id == WIRELESS_ID)
    {
        memcpy(&self->info.wireless, rxBuf, sizeof(wireless_rx_info_t));

        int16_t wireless_P = self->info.wireless.charging_power;
        self->info.wireless_p = int16_to_float(wireless_P, 32000, -32000, 0.0f, 150.0f);

        self->info.w_offline_cnt = 0;
    }
}
