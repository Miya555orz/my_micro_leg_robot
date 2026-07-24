/**
 ******************************************************************************
 * @file        driver.c
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
 
/* Includes ------------------------------------------------------------------*/
#include "driver.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

void DRIVER_Init(void)
{
	USART1_Init();
	USART5_Init();
	USART7_Init();
	/* USART10 is reserved for the STS3032 half-duplex servo bus. */
	USART8_Init();
	CAN1_Filter_Init();
	CAN2_Filter_Init();
	CAN3_Filter_Init();
}
