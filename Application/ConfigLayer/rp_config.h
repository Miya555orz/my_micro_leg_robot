/**
 * @file        rp_config.c
 * @author      RobotPilots
 * @Version     v1.1
 * @brief       RobotPilots Robots' Configuration.
 * @update
 *              v1.0(9-September-2020)
 *              v1.1(7-November-2021)
 *                  1.优化设备类信息与结构体的变量定义，增加volatile/const关键字
 *                  //2.将rp_config.h分成driver_config.h, device_config.h, user_config.h三个头文件
 */
#ifndef __RP_CONFIG_H
#define __RP_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "stdbool.h"
#include "string.h"
#include "RP_Log.h"

// 驱动层配置
#include "rp_driver_config.h"
// 设备层配置
#include "rp_device_config.h"
// 用户层配置
#include "rp_user_config.h"

/* Exported macro ------------------------------------------------------------*/
// TEST_开头的宏定义比赛时应该注释，测试模式之间可能会冲突，开启前请注意
/*测试基本控制算法模式，主要用来遥控器控腿长腿角，只控关节电机*/
//  #define TEST_MY_LEG
/*测试自救模式，自救完不进LQR*/
// #define TEST_RESCUE
/*自救完进机械模式*/
// #define TEST_MEC_MODE
/*主要用来离线模式在没有发弹量的时候打弹*/
// #define TEST_NO_LIMIT_SHOOT
/*不检测自救*/
// #define TEST_NO_RESCUE
/*测试底盘卸力模式*/
// #define TEST_SITDOWN_MODE
/*测试功率限制模式，超功率会底盘断电*/
#define TEST_POWER_LIMIT

/*是否开启超电*/
#define CAP_ENABLE

/*是否使用变速小陀螺*/
// #define IS_VARY_CYCLE

/*无裁判系统下测试，无裁判系统下开启此宏状态机才会更新*/
// #define NO_REFEREE_SYSTEM

/*是否有弹速自适应*/
#define BULLET_SPEED_ADAPT

/*不进行K矩阵拟合*/
// #define NO_K_Fitting

// 是否开启功率限制
#define Power_limit

/*选择IMU解算算法为Mahony*/
#define IMU_USE_MAHONY 0
/*选择IMU解算算法为EKF*/
#define IMU_USE_EKF 1

/* Exported types ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
// 以下为汇编函数
void WFI_SET(void);          // 执行WFI指令
void INTX_DISABLE(void);     // 关闭所有中断
void INTX_ENABLE(void);      // 开启所有中断
void MSR_MSP(uint32_t addr); // 设置堆栈地址

#endif
