/**
 ******************************************************************************
 * @file    monitor_task.c
 * @brief   监控任务
 *          1. 各模块心跳失联检测
 *          2. 监控遥控器状态，软件复位
 ******************************************************************************
 */
#include "monitor_task.h"
#include "Ui_Task.h"
#include "cap.h"
#include "device.h"
#include "usart.h"

uint16_t open_ui = 0;
uint32_t test_time;
uint32_t test_flag;

void StartMonitorTask(void const *argument)
{

    for (;;)
    {
        // 软件复位
        // if (rc_sensor.info->Z.value == true && rc_sensor.info->X.value == true && rc_sensor.info->C.value == true)
        // {
        //     HAL_NVIC_SystemReset();
        // }
        HAL_IWDG_Refresh(&hiwdg1); // 喂狗
        rc_sensor.heart_beat(&rc_sensor);
        imu_sensor.heart_beat(&imu_sensor.work_state);
        // Sd_Group.group_heartbeat(&Sd_Group);	  //在Chassis.heartbeat里
        // Wheel_Group.group_heartbeat(&Wheel_Group);//在Chassis.heartbeat里
        dail_motor.heartbeat(&dail_motor);
        Yaw_Motor.single_heart_beat(&Yaw_Motor);
        Chassis.heartbeat(&Chassis);
        Cmd_Heartbeat();

        D_Board_HeartBeat();
        cap.heartbeat(&cap);

        

        osDelay(1);
    }
}
