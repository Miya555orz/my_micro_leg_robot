#ifndef __CHASSIS_MOTOR_H
#define __CHASSIS_MOTOR_H

#include "DM_Motor.h"
#include "RM_Motor.h"
#include "drv_can.h"
#include "kalman_filter.h"

#define L_Wheel_ID_Index 0     // 0~3,对应一拖四的序号
#define R_Wheel_ID_Index 0     // 0~3,对应一拖四的序号
#define L_Wheel_RX_ID    0X201 // CAN接收ID
#define R_Wheel_RX_ID    0X201 // CAN接收ID

#define TXID_R_F_Sd_M    0x01 // 0x01
#define RXID_R_F_Sd_M    0x11 // 0x11
#define TXID_R_B_Sd_M    0x02 // 0x02
#define RXID_R_B_Sd_M    0x22 // 0x22

#define TXID_L_F_Sd_M    0x01 // 0x01
#define RXID_L_F_Sd_M    0x11 // 0x11
#define TXID_L_B_Sd_M    0x02 // 0x02
#define RXID_L_B_Sd_M    0x22 // 0x22

typedef enum {

    R_F_Sd_M,
    R_B_Sd_M,
    L_F_Sd_M,
    L_B_Sd_M,
    Sd_Num,

} Motor_Sd_e;

typedef enum {
    R_WHEEL_M,
    L_WHEEL_M,
    Wheel_Num,
} Motor_Wheel_e;
extern Motor_DM_t L_F_Sd;
extern Motor_DM_t L_B_Sd;
extern Motor_DM_t R_F_Sd;
extern Motor_DM_t R_B_Sd;

extern Motor_DM_Group_t Sd_Group;
extern Motor_RM_Group_t Wheel_Group;
extern Motor_RM_t Test_Motor;

#endif
