#include "gpio.h"

void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_USART10_GPIO_Port, LED_USART10_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_USART1_GPIO_Port, LED_USART1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, LED_CAN1_Pin | LED_HEARTBEAT_Pin | LED_CAN2_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = NRF_CE_Pin | NRF_CSN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LED_USART1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_USART1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LED_CAN1_Pin | LED_HEARTBEAT_Pin | LED_CAN2_Pin;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LED_USART10_Pin;
    HAL_GPIO_Init(LED_USART10_GPIO_Port, &GPIO_InitStruct);
}
