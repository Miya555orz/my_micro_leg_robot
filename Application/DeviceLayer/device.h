#ifndef __DEVICE_H
#define __DEVICE_H

/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
#include "imu_sensor.h"
#include "RM_motor.h"
#include "rc_sensor.h"
#include "rc_protocol.h"
#include "judge.h"
#include "cap.h"
#include "command.h"
#include "gimbal_motor.h"
#include "shoot.h"
#include "Chassis_Motor.h"
#include "Chassis.h"
#include "Balance.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct dev_list_struct 
{
	rc_sensor_t 	*rc_sen;

} dev_list_t;

extern dev_list_t dev_list;
/* Exported functions --------------------------------------------------------*/
void DEVICE_Init(void);

/* Servo functions */

#endif
