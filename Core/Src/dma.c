#include "dma.h"

void MX_DMA_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
}
