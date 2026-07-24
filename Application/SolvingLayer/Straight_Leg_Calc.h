#ifndef __STRAIGHT_LEG_CALC_H

#define __STRAIGHT_LEG_CALC_H

/* Includes ------------------------------------------------------------------*/

#include "arm_math.h"

#include "stdbool.h"

#include "pid.h"

/* Exported macro ------------------------------------------------------------*/

#define mat              arm_matrix_instance_f32

#define Matrix_Init      arm_mat_init_f32

#define Matrix_Add       arm_mat_add_f32

#define Matrix_Subtract  arm_mat_sub_f32

#define Matrix_Multiply  arm_mat_mult_f32

#define Matrix_Transpose arm_mat_trans_f32

#define Matrix_Inverse   arm_mat_inverse_f32

#define STATE_NUM        6

#define u_NUM            2

#define sizeof_float     sizeof(float)

typedef struct State_info_struct_t

{

    float thetal;        // 杆和竖直方向夹角，顺时针为正，l表示杆
    float thetal_degree; // °为单位
    float thetald1;

    float thetald2;

    float s; // 位移，针对HGC模型的图，往右为正

    float sd1;

    float thetab; // 机体pitch角，往上为正，b表示机体

    float thetabd1;

    /*中间变量用 begin*/

    float s_now;

    float s_last;

    float thetal_now;

    float thetal_last;

    float thetald1_l_now;

    float thetald1_l_last;

    /*中间变量用 end*/

    float target_thetal; // 杆和竖直方向夹角，顺时针为正，l表示杆

    float target_thetald1;

    float target_s; // 位移，往右为正

    float target_sd1;

    float target_thetab; // 机体pitch角，往上为正，b表示机体

    float target_thetabd1;

    float thetal_err; // 目标-测量

    float thetald1_err; // 目标-测量

    float s_err; // 目标-测量

    float sd1_err; // 目标-测量

    float thetab_err; // 目标-测量

    float thetabd1_err; // 目标-测量

} State_info_t;

typedef struct X_Matrix_struct_t

{

    /*状态矩阵结构体的变量名

    X_State_mat = [thetal thetald1 s sd1 thetab thetabd1]T */

    float X_state_mat_storage[6];

    mat X_state_mat;

    /*目标矩阵结构体的变量名

    X_target_mat = [thetal thetald1 s sd1 thetab thetabd1]T*/

    float X_target_mat_storage[6];

    mat X_target_mat;

    /*err矩阵结构体的变量名

    X_err_mat = [thetal thetald1 s sd1 thetab thetabd1]T*/

    float X_err_mat_storage[6];

    mat X_err_mat;

} X_Matrix_t;

/*K矩阵、拟合系数矩阵*/

typedef struct K_Matrix_struct_t

{

    float K_coefficient[2][6]; // 2个控制量，6个状态变量

    mat K_mat;

    float K_coefficient_fit[2][6][4]; // 2个控制量，6个状态变量，4个多项式系数

    mat K_fit_Tw_mat;

    mat K_fit_Tp_mat;

} K_Matrix_t;

/*状态量枚举*/

typedef enum X_enum_e

{

    X_thetal,
    X_thetald1,

    X_s,
    X_sd1,

    X_thetab,
    X_thetabd1,

    X_Num,

} X_e;

/*控制量枚举*/

typedef enum u_enum_e

{

    Tw,

    Tp,

    u_Num,

} u_e;

/*控制量输出*/

typedef struct u_struct_t

{

    float s_part;

    float thetal_part;

    float thetab_part;

    float u_mat_storage[2]; // Tw（顺时针）、Tp（顺时针）,使用枚举获取

    mat u_mat;

} u_t;

typedef struct ex_leg_data_struct_t

{

    float l0; // 接收腿长

} Ex_leg_data_t;

typedef struct Straight_Leg_struct_t

{

    State_info_t *info;

    K_Matrix_t *K_info;

    X_Matrix_t *X_info;

    u_t *u;

    Ex_leg_data_t *Ex_leg_data;

    void (*init)(struct Straight_Leg_struct_t *straight_leg);

    void (*target_state_update)(struct Straight_Leg_struct_t *straight_leg, float tar_thetal, float tar_thetald1,

                                float tar_s, float tar_sd1,

                                float tar_thetab, float tar_thetabd1);

    void (*ex_data_update)(struct Straight_Leg_struct_t *straight_leg, float l0);

    void (*K_fitting)(struct Straight_Leg_struct_t *straight_leg);

    void (*LQR_cal)(struct Straight_Leg_struct_t *straight_leg);

    float (*get_Tw)(struct Straight_Leg_struct_t *straight_leg);

    float (*get_Tp)(struct Straight_Leg_struct_t *straight_leg);

} Straight_Leg_t;

/* Exported functions --------------------------------------------------------*/

void Straight_Leg_Init(Straight_Leg_t *My_Model);

#endif
