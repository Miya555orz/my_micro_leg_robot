#include "main.h"
#include "stm32h7xx_it.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern UART_HandleTypeDef huart7;
extern TIM_HandleTypeDef htim2;

void NMI_Handler(void)
{
    while (1) {
    }
}

void HardFault_Handler(void)
{
    while (1) {
    }
}

void MemManage_Handler(void)
{
    while (1) {
    }
}

void BusFault_Handler(void)
{
    while (1) {
    }
}

void UsageFault_Handler(void)
{
    while (1) {
    }
}

void DebugMon_Handler(void)
{
}

void FDCAN1_IT0_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&hfdcan1);
}

void FDCAN2_IT0_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&hfdcan2);
}

void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim2);
}

void DMA1_Stream7_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_uart7_rx);
}

void UART7_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart7);
}
