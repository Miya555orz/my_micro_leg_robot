/**
  ******************************************************************************
  * @file    drv_can.h
  * @brief   CAN driver header file
  ******************************************************************************
  * @attention
  * 
  * Copyright 2024 RobotPilots
  ******************************************************************************
  */
#ifndef __DRV_CAN_H
#define __DRV_CAN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "rp_driver_config.h"

/* Exported variables --------------------------------------------------------*/
extern uint8_t CAN1_200_DATA[8];
extern uint8_t CAN1_1FF_DATA[8];
extern uint8_t CAN2_200_DATA[8];
extern uint8_t CAN2_1FF_DATA[8];

/** @defgroup CAN_Tx_Mailboxes CAN Tx Mailboxes
  * @{
  */
#define CAN_TX_MAILBOX0             (0x00000001U)  /*!< Tx Mailbox 0  */
#define CAN_TX_MAILBOX1             (0x00000002U)  /*!< Tx Mailbox 1  */
#define CAN_TX_MAILBOX2             (0x00000004U)  /*!< Tx Mailbox 2  */

#define CAN_HandleTypeDef 		FDCAN_HandleTypeDef
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
/* Private typedef -----------------------------------------------------------*/
/**
  * @brief CAN1\CAN2
  */
typedef struct {
	FDCAN_RxHeaderTypeDef header;
	uint8_t				data[8];
} CAN_RxFrameTypeDef;

/* Exported functions --------------------------------------------------------*/
void CAN1_Filter_Init(void);
void CAN2_Filter_Init(void);
void CAN1_CMD_200(void);
void CAN1_CMD_1FF(void);
void CAN2_CMD_200(void);
void CAN2_CMD_1FF(void);
void int16_to_uint8(uint8_t *data, int16_t *dat);
void FDCAN1_Restart(void);
void FDCAN2_Restart(void);

void CAN1_SendData(uint32_t StdId, uint8_t *data);
void CAN2_SendData(uint32_t StdId, uint8_t *data);
HAL_StatusTypeDef CAN_SendData(FDCAN_HandleTypeDef *hcan, uint32_t stdId, const uint8_t *dat);

#endif
