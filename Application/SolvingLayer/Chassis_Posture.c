#include "Chassis_Posture.h"

static void Chassis_Posture_Update(Chassis_Posture_t *My_Chassis_Posture);

Chassis_Posture_info_t Chassis_Posture_info;

Chassis_Posture_t Chassis_Posture = {

    .data_update = Chassis_Posture_Update,

    .info = &Chassis_Posture_info,

};

float roll_offset = 0.0057f;

float pitch_offset = 0.0f;

static void Chassis_Posture_Update(Chassis_Posture_t *My_Chassis_Posture)

{

    Chassis_Posture_info_t *info = My_Chassis_Posture->info;

    info->pitch = imu_sensor.info->base_info.roll * Degree_to_rad + pitch_offset * Degree_to_rad;

    info->roll = -imu_sensor.info->base_info.pitch * Degree_to_rad + roll_offset;

    info->yaw = imu_sensor.info->base_info.yaw * Degree_to_rad;

    // 角速度更新
    info->pitch_v = imu_sensor.info->base_info.rate_roll * Degree_to_rad;
    info->roll_v = -imu_sensor.info->base_info.rate_pitch * Degree_to_rad;
    info->yaw_v = imu_sensor.info->base_info.rate_yaw * Degree_to_rad;

    // degree赋值
    info->pitch_degree = info->pitch / Degree_to_rad;
    info->roll_degree = info->roll / Degree_to_rad;
    info->yaw_degree = info->yaw / Degree_to_rad;
    info->pitch_v_degree = info->pitch_v / Degree_to_rad;
    info->roll_v_degree = info->roll_v / Degree_to_rad;
    info->yaw_v_degree = info->yaw_v / Degree_to_rad;

    // 加速度更新

    info->a_x = imu_sensor.info->raw_info.acc_y;

    info->a_y = imu_sensor.info->raw_info.acc_x;

    info->a_z = imu_sensor.info->raw_info.acc_z;

    // 世界加速度更新

    info->x_world = -imu_sensor.info->base_info.accx;

    info->y_world = -imu_sensor.info->base_info.accy;

    info->z_world = -imu_sensor.info->base_info.accz;
}
