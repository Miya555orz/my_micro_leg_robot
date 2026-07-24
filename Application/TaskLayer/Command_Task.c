/**

  ******************************************************************************

  * @file    Command_task.c

  * @brief   指令更新任务

  *          更新整车标志位和模式

  ******************************************************************************

  */

#include "Command_Task.h"

void StartCommandTask(void const *argument)

{

    for (;;)

    {

        keyboard_update(rc_sensor.info);

        rc_sensor_s_last_update(&rc_sensor);
        Command_Update();
#ifndef TEST_MY_LEG
        Balance.update(&Balance);
#endif
        //    #ifndef TEST_MY_LEG

        //        Balance.update(&Balance);

        //    #endif

        osDelay(1);
    }
}
