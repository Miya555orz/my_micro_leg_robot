#include "drv_uart.h"

#include "usart.h"
#include <string.h>

__attribute__((section(".AXI_SRAM"))) static uint8_t uart7_rx_buffer[UART7_RX_BUF_LEN];

HAL_StatusTypeDef DRV_UART7_StartRx(void)
{
    memset(uart7_rx_buffer, 0, sizeof(uart7_rx_buffer));
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart7, uart7_rx_buffer, sizeof(uart7_rx_buffer)) != HAL_OK) {
        return HAL_ERROR;
    }
    __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
    return HAL_OK;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart != &huart7) {
        return;
    }

    if (size > 0U) {
        UART7_rxDataHandler(uart7_rx_buffer, size);
    }
    (void)DRV_UART7_StartRx();
}

__WEAK void UART7_rxDataHandler(const uint8_t *rx_buf, uint16_t length)
{
    (void)rx_buf;
    (void)length;
}
