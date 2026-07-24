/**

  ******************************************************************************

  * @file    UI_task.c

  * @brief   UI更新任务

  ******************************************************************************

  */

#include "Ui_Task.h"



//float sin_value;

uint32_t t;

void StartUITask(void const * argument)

{

		for(;;)

	{

//		t++;

//		sin_value=sin(t/100.f);

//		WL_UART_printf(&huart7,"%f\n",sin_value);

//		WL_UART_printf(&huart1,"%f,%f\n",Chassis.Leg_Unit[R_Leg]->force->F_support,

//		Chassis.Leg_Unit[L_Leg]->force->F_support);

		//SCB_CleanDCache_by_Addr((uint32_t*)t_buf, sizeof(t_buf));

	//	HAL_UART_Transmit_DMA(&huart7, (uint8_t *)t_buf, sizeof(t_buf));


		
		osDelay(1);

	}

}

