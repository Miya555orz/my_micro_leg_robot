#include "fdcan.h"

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

static void fdcan_common_init(FDCAN_HandleTypeDef *hfdcan, FDCAN_GlobalTypeDef *instance, uint32_t ram_offset)
{
    hfdcan->Instance = instance;
    hfdcan->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    hfdcan->Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan->Init.AutoRetransmission = ENABLE;
    hfdcan->Init.TransmitPause = DISABLE;
    hfdcan->Init.ProtocolException = DISABLE;
    hfdcan->Init.NominalPrescaler = 1;
    hfdcan->Init.NominalSyncJumpWidth = 20;
    hfdcan->Init.NominalTimeSeg1 = 59;
    hfdcan->Init.NominalTimeSeg2 = 20;
    hfdcan->Init.DataPrescaler = 1;
    hfdcan->Init.DataSyncJumpWidth = 20;
    hfdcan->Init.DataTimeSeg1 = 59;
    hfdcan->Init.DataTimeSeg2 = 20;
    hfdcan->Init.MessageRAMOffset = ram_offset;
    hfdcan->Init.StdFiltersNbr = 1;
    hfdcan->Init.ExtFiltersNbr = 0;
    hfdcan->Init.RxFifo0ElmtsNbr = 16;
    hfdcan->Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
    hfdcan->Init.RxFifo1ElmtsNbr = 0;
    hfdcan->Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
    hfdcan->Init.RxBuffersNbr = 0;
    hfdcan->Init.RxBufferSize = FDCAN_DATA_BYTES_8;
    hfdcan->Init.TxEventsNbr = 0;
    hfdcan->Init.TxBuffersNbr = 0;
    hfdcan->Init.TxFifoQueueElmtsNbr = 8;
    hfdcan->Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    hfdcan->Init.TxElmtSize = FDCAN_DATA_BYTES_8;
    if (HAL_FDCAN_Init(hfdcan) != HAL_OK) {
        Error_Handler();
    }
}

void MX_FDCAN1_Init(void)
{
    fdcan_common_init(&hfdcan1, FDCAN1, 0U);
}

void MX_FDCAN2_Init(void)
{
    fdcan_common_init(&hfdcan2, FDCAN2, 1280U);
}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *fdcanHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    static uint32_t fdcan_clock_refcount;

    if (fdcanHandle->Instance != FDCAN1 && fdcanHandle->Instance != FDCAN2) {
        return;
    }

    if (fdcan_clock_refcount++ == 0U) {
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
        PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
            Error_Handler();
        }
        __HAL_RCC_FDCAN_CLK_ENABLE();
    }

    if (fdcanHandle->Instance == FDCAN1) {
        __HAL_RCC_GPIOD_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
        HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    } else {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
    }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef *fdcanHandle)
{
    if (fdcanHandle->Instance == FDCAN1) {
        HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0 | GPIO_PIN_1);
        HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
    } else if (fdcanHandle->Instance == FDCAN2) {
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5 | GPIO_PIN_6);
        HAL_NVIC_DisableIRQ(FDCAN2_IT0_IRQn);
    }
}
