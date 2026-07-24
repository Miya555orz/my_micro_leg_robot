/**

 ******************************************************************************

 * @file        drv_tick.c

 * @author      RobotPilots@2020

 * @brief       Haltick driver

 ******************************************************************************

 * @attention   

 * 

 * Copyright 2020 RobotPilots

 * 

 * @note

 * 使用cubemx生成FREERTOS后会建议将SYS的时基切换成除SysTick之外的定时器

 * 从而系统会存在两套时基，①用于RTOS的SysTick ②用于HAL的HalTick

 * SysTick 使用cortex-m4内核的SysTick (SysTick->VAL会在启动任务调度器之后才更新)

 * HalTick 在本工程里面使用TIM2 (TIM2->CNT可提供微妙级延时)

 * # delay_us 和 delay_ms 不会引起任务调度(阻塞型)

 * 

 * @Version     V1.0

 * @date        15-September-2020

 ****************************************************************************

 */

 

/* Includes ------------------------------------------------------------------*/

#include "drv_tick.h"



/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

uint32_t haltick = 0;

/* Private functions ---------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**

 * @brief  获取当前时间

 * @param  None

 * @retval 当前时间

 */

uint32_t micros(void)

{

	register uint32_t ms, us;

	

	ms = HAL_GetTick();

	/* 选用定时器2作为HAL时基的TimeBase */

	/* Freq:1MHz => 1Tick = 1us */

	/* Period:1ms */

	us = TIM2->CNT;

	

    haltick = ms*1000 + us;

    

	return haltick;

}



void delay_us(uint32_t us)

{

	uint32_t now = micros();

	

	while((micros() - now) < us);

}



void delay_ms(uint32_t ms)

{

	while(ms--)

		delay_us(1000);

}



/* Define Exported Functions ---------------------------------------------------------------------------------------------------------------------------------------- */

/*!

 * @usage: 开启DWT模块

 */

void DWT_Init(void)

{

    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk))

    {

        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 启用跟踪功能

    }

    

    DWT->CYCCNT = 0;                        // 清零周期计数器

    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;    // 启用周期计数器

}



/*!

 * @usage: 获得时钟周期数

 */

uint32_t DWT_GetCycleCount(void)

{

    return DWT->CYCCNT;

}

