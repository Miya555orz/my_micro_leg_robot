#ifndef __LINK_CALC_H

#define __LINK_CALC_H

/* Includes ------------------------------------------------------------------*/

#include "pid.h"

#include "car_info.h"

#include "arm_math.h"

#include "rp_math.h"

/* Exported types ------------------------------------------------------------*/

#define Rad2Angle 57.2957804f

/*连杆坐标信息*/

typedef struct Link_Coord_struct_t {

    float xa;

    float ya;

    float xe;

    float ye;

    float xb;

    float yb;

    float xb_d1;

    float yb_d1;

    float xc;

    float yc;

    float xc_d1;

    float yc_d1;

    float xd;

    float yd;

    float xd_d1;

    float yd_d1;

    float xp;

    float yp;

} Link_Coord_t;

/*四连杆角度信息*/

typedef struct Link_Angle_struct_t {

    /*弧度制*/

    float phi1;

    float phi1_d1;

    float phi2;

    float phi2_d1;

    float phi3;

    float phi4;

    float phi4_d1;

    float phi0;

    float phi0_d1;

    float good_phi0_d1;

    float vir_phi0;
    float vir_phi0_degree;

    float vir_phi0_d1;
    float vir_phi0_d1_degree;

    /*目标角度*/

    float target_phi1;

    float target_phi4;

    /*角度制*/

    float phi1_degree;

    float phi2_degree;

    float phi3_degree;

    float phi4_degree;

    float phi0_degree;

    float phi0_last_degree;



} Link_Angle_t;

typedef struct ex_link_data_struct_t {

    float phi1;

    float phi4;

    float phi1_d1;

    float phi4_d1;

    float torque_phi1;

    float torque_phi4;

} Ex_link_data_t;

typedef struct Link_Stator_Correction_struct_t {

    /*移除杆接定子端角速度影响使用变量*/

    float stator_angle_now;

    float stator_angle_last;

    float stator_angular_speed;

    float stator_bias; // 在角度和测量值中减去

} Link_Stator_Correction_t;

/*等效直腿腿长信息*/

typedef struct Link_Leg_Length_struct_t {

    float lbd;

    float l0;

    float l0_last;

    float l0_dot;

    float l0_dot_last;

    float l0_dot2;

    float l0_dot2_last;

    float good_l0_dot;

    float l_gravity; // 质心系数，需要自行调整

} Link_Leg_Length_t;

/*杆的质心位置信息*/

typedef struct Link_Centroid_struct_t {

    float mx_l1;

    float my_l1;

    float mx_l2;

    float my_l2;

    float mx_l3;

    float my_l3;

    float mx_l4;

    float my_l4;

    float centriod_coefficient;

} Link_Centroid_t;

/*等效直腿受力信息*/

typedef struct Link_Force_struct_t {

    //	float F_gravity;//重力补偿力

    //

    //  float F_inertial;//侧向惯性补偿力

    //

    //  float F_roll;//roll轴补偿力

    //

    //  float F;//保持腿长力,pid,伸腿为正

    float F_bl_target; // 合力,F+F_roll+F_inertial+F_gravity

    //

    //	float Tp_sync;//双腿协调

    float Tp_target;

    //	float F_support;//支持力

    float G_torque;

    float G_support;

    float F_bl_mea;

    float Tp_mea;

    float Sd_F_Pos_Tor_Fix;

    float Sd_B_Pos_Tor_Fix;

    float torque_phi1_mea;

    float torque_phi4_mea;

} Link_Force_t;

typedef struct Link_info_struct_t {

    float F_Sd_Output_Torque; // phi1，VMC后的前关节力矩,逆时针为正

    float B_Sd_Output_Torque; // phi4，VMC后的后关节力矩,逆时针为正

    Link_Coord_t *coord;

    Link_Angle_t *angle;

    Link_Leg_Length_t *length;

    Link_Force_t *force;

    Link_Stator_Correction_t *stator_correction;

    Link_Centroid_t *centroid;

    Ex_link_data_t *Ex_data;

} Link_info_t;

typedef struct Link_struct_t {

    Link_info_t *info;

    void (*init)(struct Link_struct_t *link);

    void (*mea_data_update)(struct Link_struct_t *Link, float phi1, float phi1_d1,

                            float phi4, float phi4_d1, float torque_phi1_mea, float torque_phi4_mea);

    void (*tar_data_update)(struct Link_struct_t *Link, float F_bl_target, float Tp_target);

    void (*link_update)(struct Link_struct_t *link);

    void (*torque_cal)(struct Link_struct_t *link);

    void (*Fb1_Tp_cal)(struct Link_struct_t *link);

} Link_t;

void Link_Init(Link_t *My_Link);

void My_Link_reverse_cal(Link_t *Link, float T1, float T2);

#endif
