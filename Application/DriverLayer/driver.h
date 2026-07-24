/**
 ******************************************************************************
 * @file        driver.h
 * @author      RobotPilots@2020
 * @brief       Drivers' Manager.
 ******************************************************************************
 * @attention   
 * 
 * Copyright 2020 RobotPilots
 *  
 * @Version     V1.0
 * @date        9-September-2020
 ****************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRIVER_H
#define __DRIVER_H

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "stdbool.h"

#include "drv_can.h"
#include "drv_flash.h"
#include "drv_tick.h"
#include "drv_tim.h"
#include "drv_uart.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void DRIVER_Init(void);

#endif
