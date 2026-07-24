#ifndef __CAR_INFO_H
#define __CAR_INFO_H

/*************************** 《机体属性》 begin ****************************/
#define WHEEL_RADIUS 0.06f // 驱动轮半径，单位：m
// 腿杆长，如果为串联腿请将l5置零，l1=l2,l3=l4
#define l1 0.215f
#define l2 0.258f
#define l3 0.258f
#define l4 0.215f
#define l5 0.f
// 各杆质心系数
#define l1_cen 0.42f
#define l2_cen 0.f
#define l3_cen 0.4985f
#define l4_cen 0.42054f

// 各杆质量
#define m_l1 0.0558f
#define m_l2 0.f
#define m_l3 0.265f
#define m_l4 0.2977f

// 驱动轮质量（算上定子）
#define mw 0.55f
// 机体质量
#define mb 29.0f
#define g  9.81f

// 整车旋转半径
#define Rl 0.2574f
// 最长最短腿长
#define MAX_LEG_LENGTH 0.32f

#define MIN_LEG_LENGTH 0.13f
#define MID_LEG_LENGTH ((MAX_LEG_LENGTH + MIN_LEG_LENGTH) / 2.f)
// 单腿质量，四杆总和
#define m_l (m_l1 + m_l2 + m_l3 + m_l4)
// #define R_PHI1_UP_ANGLE (-170.96)
// #define R_PHI4_UP_ANGLE (-14.4)

// #define L_PHI1_UP_ANGLE (-164.0f)
// #define L_PHI4_UP_ANGLE (-13.5f)
/*************************** 机体属性 end ****************************/

/*************************** 控制配置 begin ****************************/

#define TAR_LEG_LENGTH_INITIAL    (0.15f) // 初始目标腿长
#define OFF_GROUND_SUPPORT        50.0f   // 离地支持力阈值，越小越难触发，单位：N
#define OFF_GROUND_TIME_THRESHOLD 5       // 离地检测触发时间阈值，单位ms
#define MAX_LIFT_SPEED            0.15f   // 单位：m/s  腿长改变最大速度
#define MAX_SPIN_SPEED            200.0f  // 单位：°/s 车体转向运动最大速度

/*软件限位相关，保护机械结构*/
// #define LIMIT_RANGE      (10.f)
// #define SD_POS_FIX_TOR_K (-0.f) // 关节限位力矩补偿系数 -0.1

/*卸力阻尼时间与阻尼系数*/
#define DAMPING_DELAY_MAX_CNT     3500 // 阻尼持续时间

#define Wheel_Damping_Coefficient 0.0001f //
#define Sd_Damping_Coefficient    3.f

// 机体最大前进速度
#define MAX_STRAIGHT_SPEED         2.5f
#define RC_INPUT_SD1_ORDER_CORRECT 1.f

/*************************** 控制配置 end ****************************/

/*************************** 零点、方向配置 begin ****************************/
/*
 * - 前进方向朝右，腿后躺摆放
 * - 机体右边电机零点(HORIZON_ANGLE)为：-《平躺时后伸腿姿态编码器值》+ 《水平向左与膝关节腿/大腿的夹角》
 * - 机体左边电机零点(HORIZON_ANGLE)为：-《-平躺时后伸腿姿态编码器值》+ 《水平向左与膝关节腿/大腿的夹角》
 * - 右边电机连杆解算角度为： 《实时编码器值》+ 零点
 * - 左边电机连杆解算角度为：-《实时编码器值》+ 零点
 * - 左边电机零点-《-平躺时后伸腿姿态编码器值》= +《平躺时后伸腿姿态编码器值》，故下面是+
 * - X_X_MOTOR_ZERO_ANGLE为平躺时后伸腿姿态原始编码器值
 */
// 水平向左与膝关节腿夹角：73.06°
// 水平向左与大腿夹角：-34.81°
#define Angle_Horizontal_Leftward_and_Knee_Joint_Leg 73.06f
#define Angle_Horizontal_Leftward_and_Big_Leg        -34.81f
#define R_F_MOTOR_ZERO_ANGLE                         1.23961902f
#define R_B_MOTOR_ZERO_ANGLE                         0.925723314f
#define L_F_MOTOR_ZERO_ANGLE                         1.72263885f
#define L_B_MOTOR_ZERO_ANGLE                         0.593420029f
#define R_F_HORIZON_ANGLE                            (-(R_F_MOTOR_ZERO_ANGLE) + Degree_to_rad * Angle_Horizontal_Leftward_and_Knee_Joint_Leg)
#define R_B_HORIZON_ANGLE                            (-(R_B_MOTOR_ZERO_ANGLE) + Degree_to_rad * Angle_Horizontal_Leftward_and_Big_Leg)
#define L_F_HORIZON_ANGLE                            (+(L_F_MOTOR_ZERO_ANGLE) + Degree_to_rad * Angle_Horizontal_Leftward_and_Knee_Joint_Leg)
#define L_B_HORIZON_ANGLE                            (+(L_B_MOTOR_ZERO_ANGLE) + Degree_to_rad * Angle_Horizontal_Leftward_and_Big_Leg)

/*关节电机零点运算方向校正*/
#define R_F_HORIZON_ANGLE_ORDER_CORRECT 1
#define R_B_HORIZON_ANGLE_ORDER_CORRECT 1
#define L_F_HORIZON_ANGLE_ORDER_CORRECT 1
#define L_B_HORIZON_ANGLE_ORDER_CORRECT 1

/*电机编码器值递增方向修正，逆时针为1，顺时针为-1*/
#define R_F_TIME 1
#define R_B_TIME 1
#define L_F_TIME -1
#define L_B_TIME -1

/*建模与VMC的Tp方向矫正*/
#define L_TP_LQR_ORDER_CORRECT -1
#define R_TP_LQR_ORDER_CORRECT -1

/*建模与VMC的vir_phi0方向矫正*/
#define L_VIR_PHI0_ORDER_CORRECT -1
#define R_VIR_PHI0_ORDER_CORRECT -1

/*建模与电机扭矩输出方向矫正*/
#define L_F_ORDER_CORRECT -1 // 关节电机--++
#define L_B_ORDER_CORRECT -1
#define R_F_ORDER_CORRECT 1
#define R_B_ORDER_CORRECT 1

#define L_W_ORDER_CORRECT -1 // 驱动轮
#define R_W_ORDER_CORRECT 1

// 氮气弹簧补偿力方向矫正
#define FRONT_SPRING_COMPENSATION_ORDER_CORRECT 1
#define BACK_SPRING_COMPENSATION_ORDER_CORRECT  -1

// 腿部重力补偿力方向矫正
#define LEG_GRAVITY_COMPENSATION_ORDER_CORRECT -1

/* 双腿协调Tp_sync方向矫正 */
#define L_SYNC_ORDER_CORRECT -1
#define R_SYNC_ORDER_CORRECT 1

/* Roll角控制Tp_roll方向矫正 */
#define L_TP_Roll_ORDER_CORRECT -1
#define R_TP_Roll_ORDER_CORRECT 1

/* 转向控制Tw_turn方向矫正 */
#define R_TURN_ORDER_CORRECT 1
#define L_TURN_ORDER_CORRECT -1

/* 侧向前馈竖直力F_inertial方向矫正 */
#define L_F_INERTIAL_ORDER_CORRECT -1
#define R_F_INERTIAL_ORDER_CORRECT 1

/* 消除电机定子转动对位移影响方向矫正 */
#define R_STATOR_ORDER_CORRECT 1
#define L_STATOR_ORDER_CORRECT -1

/* 用于求s、sd1的轮速、轮总角度方向矫正 */
#define R_W_SPEED_ORDER_CORRECT    1
#define L_W_SPEED_ORDER_CORRECT    1

#define R_W_ANGLESUM_ORDER_CORRECT 1
#define L_W_ANGLESUM_ORDER_CORRECT -1

/* 关节电机总角度方向矫正 */
#define R_F_SD_ANGLESUM_ORDER_CORRECT -1
#define R_B_SD_ANGLESUM_ORDER_CORRECT -1
#define L_F_SD_ANGLESUM_ORDER_CORRECT 1
#define L_B_SD_ANGLESUM_ORDER_CORRECT 1

/*************************** 零点、方向配置 end ****************************/

#define TIME_STEP 0.001f // 任务运行周期，单位：s

typedef enum {
    R_Leg,
    L_Leg,
    Leg_Num,
} Leg_e;

#endif
