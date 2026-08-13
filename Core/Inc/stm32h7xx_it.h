#ifndef __STM32H7xx_IT_H
#define __STM32H7xx_IT_H

#ifdef __cplusplus
extern "C" {
#endif

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);
void FDCAN1_IT0_IRQHandler(void);
void FDCAN2_IT0_IRQHandler(void);
void TIM2_IRQHandler(void);
void DMA1_Stream7_IRQHandler(void);
void UART7_IRQHandler(void);
void USART10_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif
