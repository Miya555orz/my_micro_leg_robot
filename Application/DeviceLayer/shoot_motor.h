#ifndef __MOTOR_H
#define __MOTOR_H

#include "rp_config.h"
#include "motor_def.h"
#include "KT_motor.h"
#include "can_protocol.h"
#include "drv_can.h"

/*电机ID宏定义------------------------------------------------*/
#define DAIL_MOTOR_ID 0x141
typedef struct dail_pid_info_struct {
    pid_ctrl_t *speed;          // 速度环
    pid_ctrl_t *position_outer; // 位置环外环
    pid_ctrl_t *position_inner; // 位置环内环
} dail_pid_info_t;

/* Exported functions --------------------------------------------------------*/
extern KT_motor_t dail_motor;
extern dail_pid_info_t dail_pid;

#endif
