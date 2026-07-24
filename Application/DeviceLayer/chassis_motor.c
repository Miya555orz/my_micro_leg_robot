#include "Chassis_motor.h"

#include "RM_Motor.h"

#include "car_info.h"

/*........................轮毂电机 begin.......................*/

/*左驱动轮*/

Motor_RM_Born_Info_t L_Wheel_Born =

    {

        .rxId = L_Wheel_ID_Index,

        .hcan = &hfdcan2,

        .type = _3508_Reduction,

        .stdId = 0x200,

        .order_correction = L_W_ANGLESUM_ORDER_CORRECT,

};

Motor_RM_Tx_Info_t L_Wheel_Tx;

Motor_RM_State_t L_Wheel_State;

Motor_RM_Rx_Info_t L_Wheel_Rx;

Motor_RM_t L_Wheel =

    {

        .born_info = &L_Wheel_Born,

        .rx_info = &L_Wheel_Rx,

        .tx_info = &L_Wheel_Tx,

        .state = &L_Wheel_State,

        .single_init = RM_Motor_Init,

};

/*---------------------------- 右驱动轮 ---------------------------*/

Motor_RM_Born_Info_t R_Wheel_Born =

    {

        .rxId = R_Wheel_ID_Index,

        .hcan = &hfdcan1,

        .type = _3508_Reduction,

        .stdId = 0x200,

        .order_correction = R_W_ANGLESUM_ORDER_CORRECT,

};

Motor_RM_Tx_Info_t R_Wheel_Tx;

Motor_RM_State_t R_Wheel_State;

Motor_RM_Rx_Info_t R_Wheel_Rx;

Motor_RM_t R_Wheel =

    {

        .born_info = &R_Wheel_Born,

        .rx_info = &R_Wheel_Rx,

        .tx_info = &R_Wheel_Tx,

        .state = &R_Wheel_State,

        .single_init = RM_Motor_Init,

};

// 主要用作方便阅读和调用心跳函数，因不在一条can上不能使用groupset来执行can发送

Motor_RM_Group_t Wheel_Group =

    {

        .motor[R_Leg] = &R_Wheel,

        .motor[L_Leg] = &L_Wheel,

        .motor[2] = NULL,

        .motor[3] = NULL,

        .stdId = 0x200,

        .group_init = RM_Group_Motor_Init,

};

/*........................轮毂电机 end.......................*/

/*..........................................关节电机..........................................*/

/*右前关节*/

Motor_DM_Born_Info_t R_F_Sd_Born_Info =

    {

        .txId = TXID_R_F_Sd_M,

        .hcan = &hfdcan1,

        .order_correction = R_F_SD_ANGLESUM_ORDER_CORRECT,

};

Motor_DM_Rx_Info_t R_F_Sd_Rx_Info_t;

Motor_DM_Tx_Info_t R_F_Sd_Tx_Info_t;

Motor_DM_State_t R_F_Sd_State_t;

Motor_DM_t R_F_Sd =

    {

        .born_info = &R_F_Sd_Born_Info,

        .rx_info = &R_F_Sd_Rx_Info_t,

        .tx_info = &R_F_Sd_Tx_Info_t,

        .state = &R_F_Sd_State_t,

        .single_init = &DM_Single_Motor_Init,

        .type = leg_8009,

};

/*右后关节*/

Motor_DM_Born_Info_t R_B_Sd_Born_Info =

    {

        .txId = TXID_R_B_Sd_M,

        .hcan = &hfdcan1,

        .order_correction = R_B_SD_ANGLESUM_ORDER_CORRECT,

};

Motor_DM_Rx_Info_t R_B_Sd_Rx_Info_t;

Motor_DM_Tx_Info_t R_B_Sd_Tx_Info_t;

Motor_DM_State_t R_B_Sd_State_t;

Motor_DM_t R_B_Sd =

    {

        .born_info = &R_B_Sd_Born_Info,
        .rx_info = &R_B_Sd_Rx_Info_t,
        .tx_info = &R_B_Sd_Tx_Info_t,
        .state = &R_B_Sd_State_t,
        .single_init = &DM_Single_Motor_Init,
        .type = leg_8009,

};

/*左前关节*/
Motor_DM_Born_Info_t L_F_Sd_Born_Info =

    {

        .txId = TXID_L_F_Sd_M,
        .hcan = &hfdcan2,
        .order_correction = L_F_SD_ANGLESUM_ORDER_CORRECT,

};

Motor_DM_Rx_Info_t L_F_Sd_Rx_Info_t;

Motor_DM_Tx_Info_t L_F_Sd_Tx_Info_t;

Motor_DM_State_t L_F_Sd_State_t;

Motor_DM_t L_F_Sd =

    {

        .born_info = &L_F_Sd_Born_Info,

        .rx_info = &L_F_Sd_Rx_Info_t,

        .tx_info = &L_F_Sd_Tx_Info_t,

        .state = &L_F_Sd_State_t,

        .single_init = &DM_Single_Motor_Init,

        .type = leg_8009,

};

/*左后关节*/

Motor_DM_Born_Info_t L_B_Sd_Born_Info =

    {

        .txId = TXID_L_B_Sd_M,

        .hcan = &hfdcan2,

        .order_correction = L_B_SD_ANGLESUM_ORDER_CORRECT,

};

Motor_DM_Rx_Info_t L_B_Sd_Rx_Info_t;

Motor_DM_Tx_Info_t L_B_Sd_Tx_Info_t;

Motor_DM_State_t L_B_Sd_State_t;

Motor_DM_t L_B_Sd =

    {

        .born_info = &L_B_Sd_Born_Info,

        .rx_info = &L_B_Sd_Rx_Info_t,

        .tx_info = &L_B_Sd_Tx_Info_t,

        .state = &L_B_Sd_State_t,

        .single_init = &DM_Single_Motor_Init,

        .type = leg_8009,

};

/*关节电机组*/

Motor_DM_Group_t Sd_Group =

    {

        .motor[R_F_Sd_M] = &R_F_Sd,

        .motor[R_B_Sd_M] = &R_B_Sd,

        .motor[L_F_Sd_M] = &L_F_Sd,

        .motor[L_B_Sd_M] = &L_B_Sd,

        .group_init = Group_Motor_Init,

};