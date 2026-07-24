#ifndef __CHASSIS_POSTURE_H

#define __CHASSIS_POSTURE_H

/* Includes ------------------------------------------------------------------*/

#include "imu_sensor.h"

#include "arm_math.h"

/* Exported macro ------------------------------------------------------------*/

#define Degree_to_rad 0.017453f

/* Exported types ------------------------------------------------------------*/

/*底盘上层机体姿态信息结构体*/

typedef struct Chassis_Posture_info_struct_t

{
float pitch_degree;

    float yaw_degree;

    float roll_degree;
    float pitch;

    float yaw;

    float roll;

    float pitch_v;

    float yaw_v;

    float roll_v;
 float pitch_v_degree;

    float yaw_v_degree;

    float roll_v_degree;
    float a_x;

    float a_y;

    float a_z;

    float x_world;

    float y_world;

    float z_world;

} Chassis_Posture_info_t;

/*底盘上层机体结构体*/

typedef struct Chassis_Posture_struct_t

{

    Chassis_Posture_info_t *info;

    void (*data_update)(struct Chassis_Posture_struct_t *My_Chassis_Posture);

} Chassis_Posture_t;

/* Exported functions --------------------------------------------------------*/

/* Servo functions */

extern Chassis_Posture_t Chassis_Posture;

#endif
