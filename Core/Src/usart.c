#include "usart.h"

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart10;
DMA_HandleTypeDef hdma_uart7_rx;

static void uart_common_init(UART_HandleTypeDef *huart, USART_TypeDef *instance, uint32_t baud)
{
    huart->Instance = instance;
    huart->Init.BaudRate = baud;
    huart->Init.WordLength = UART_WORDLENGTH_8B;
    huart->Init.StopBits = UART_STOPBITS_1;
    huart->Init.Parity = UART_PARITY_NONE;
    huart->Init.Mode = UART_MODE_TX_RX;
    huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart->Init.OverSampling = UART_OVERSAMPLING_16;
    huart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart->Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(huart) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetTxFifoThreshold(huart, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(huart, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_DisableFifoMode(huart) != HAL_OK) {
        Error_Handler();
    }
}

void MX_USART1_UART_Init(void)
{
    uart_common_init(&huart1, USART1, 1000000U);
}

void MX_UART7_Init(void)
{
    uart_common_init(&huart7, UART7, 115200U);
}

void MX_USART10_UART_Init(void)
{
    uart_common_init(&huart10, USART10, 1000000U);
}

void HAL_UART_MspInit(UART_HandleTypeDef *uartHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    if (uartHandle->Instance == USART1) {
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
        PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16910CLKSOURCE_D2PCLK2;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
            Error_Handler();
        }

        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    } else if (uartHandle->Instance == UART7) {
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART7;
        PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
            Error_Handler();
        }

        __HAL_RCC_UART7_CLK_ENABLE();
        __HAL_RCC_GPIOE_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_8;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_UART7;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

        hdma_uart7_rx.Instance = DMA1_Stream7;
        hdma_uart7_rx.Init.Request = DMA_REQUEST_UART7_RX;
        hdma_uart7_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_uart7_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_uart7_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_uart7_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_uart7_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_uart7_rx.Init.Mode = DMA_NORMAL;
        hdma_uart7_rx.Init.Priority = DMA_PRIORITY_HIGH;
        hdma_uart7_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_uart7_rx) != HAL_OK) {
            Error_Handler();
        }
        __HAL_LINKDMA(uartHandle, hdmarx, hdma_uart7_rx);

        HAL_NVIC_SetPriority(UART7_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(UART7_IRQn);
    } else if (uartHandle->Instance == USART10) {
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART10;
        PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16910CLKSOURCE_D2PCLK2;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
            Error_Handler();
        }

        __HAL_RCC_USART10_CLK_ENABLE();
        __HAL_RCC_GPIOE_CLK_ENABLE();
        GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF11_USART10;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(USART10_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(USART10_IRQn);
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *uartHandle)
{
    if (uartHandle->Instance == USART1) {
        __HAL_RCC_USART1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);
    } else if (uartHandle->Instance == UART7) {
        __HAL_RCC_UART7_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOE, GPIO_PIN_7 | GPIO_PIN_8);
        HAL_DMA_DeInit(uartHandle->hdmarx);
        HAL_NVIC_DisableIRQ(UART7_IRQn);
    } else if (uartHandle->Instance == USART10) {
        __HAL_RCC_USART10_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOE, GPIO_PIN_2 | GPIO_PIN_3);
        HAL_NVIC_DisableIRQ(USART10_IRQn);
    }
}
