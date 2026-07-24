#include "observe_task.h"




void StartUpdataTask(void const *argument)

{

    for (;;)

    {



        if (imu_sensor.work_state.err_code == IMU_NONE_ERR ||

            imu_sensor.work_state.err_code == IMU_DATA_CALI)

        {

            imu_sensor.update(&imu_sensor);

        }

        Chassis.data_update(&Chassis); // 含目标值更新






        osDelay(1);

    }

}

