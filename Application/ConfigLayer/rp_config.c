/**
 * @file        rp_config.c
 * @author      RobotPilots
 * @Version     v1.0
 * @brief       RobotPilots Robots' Configuration.
 * @update
 *              v1.0(9-September-2020)
 */
/* Includes ------------------------------------------------------------------*/
#include "rp_config.h"
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

// 使用 GCC 内联汇编执行 WFI 指令（适用于 ARM/Thumb 模式）
void WFI_SET(void)
{
    __asm volatile ("WFI");
}
//关闭所有中断(但是不包括fault和NMI中断)
void INTX_DISABLE(void)
{
    __asm volatile ("CPSID I");
}
//开启所有中断
void INTX_ENABLE(void)
{
    __asm volatile ("CPSIE I");
}
//设置栈顶地址
//addr:栈顶地址
void MSR_MSP(uint32_t addr)
{
    __asm volatile ("MSR MSP, %0" : : "r" (addr));
    __asm volatile ("BX LR");
}
