/**
 * @file  device.c
 */

/* Includes ------------------------------------------------------------------*/
#include "device.h"
#include "buzzer.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
dev_list_t dev_list = {
    .rc_sen = &rc_sensor,
};

/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void DEVICE_Init(void)
{

    dev_list.rc_sen->init(dev_list.rc_sen);
    Board_Tx_Info.flag.bit.is_rc_online = 1;
    imu_sensor.init(&imu_sensor);
    buzzer.init(&buzzer);
    shoot.init(&shoot);
    /*只能放在imu初始化后面 begin*/
    Chassis.Init(&Chassis);
    Balance.init(&Balance);
    /*只能放在imu初始化后面 end*/
    /*软件层初始化*/
    Yaw_Motor.single_init(&Yaw_Motor);
    dail_motor.init(&dail_motor);
    Sd_Group.group_init(&Sd_Group);
    Wheel_Group.group_init(&Wheel_Group);
    Cmd_Init();
}
