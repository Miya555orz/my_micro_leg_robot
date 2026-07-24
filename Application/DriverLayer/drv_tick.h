/**
 ******************************************************************************
 * @file        drv_tick.h
 * @author      RobotPilots@2020
 * @brief       Haltick driver
 ******************************************************************************
 * @attention   
 * 
 * Copyright 2020 RobotPilots
 * 
 * @Version     V1.0
 * @date        15-September-2020
 ****************************************************************************
 */
#ifndef __TICK_DRV_H
#define __TICK_DRV_H

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
uint32_t micros(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

#endif
