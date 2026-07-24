/**
  ******************************************************************************
  * File Name          : Chassis.c
  * Description        : Code for Chassis applications
  * License            : MIT
  ******************************************************************************
  * @attention
  * Copyright (c) 2026 SZU RobotPilots.
  * @version
  * V1.0 October 2025 首次发布
  * V2.0 March 2026 添加了自救、氮气弹簧补偿、功率限制等功能，几乎是完整的串腿代码，并增加了注释
  * @author
  * Liang 741427745@qq.com
  ******************************************************************************
  *
  ==============================================================================
                       ##### How To Use #####
  ==============================================================================
  (#) 调用软件层初始化函数 Chassis_Init

  (#) 调用底盘电机心跳函数 Chassis_HeartBeat

  (#) 调用 Chassis_Status_React ，根据车模式来响应底盘状态，依赖Balance文件

  (#) 调用 Chassis_Data_Update ，包含整车状态更新和获取遥控器数据的更新

  (#) 调用 Chassis_Ctrl ，根据底盘状态来进行相应控制

  ==============================================================================
                      ##### How To Input Data #####
  ==============================================================================
  (#)   车体方向定义：从车的前进方向的右边来观测连杆；无论是左腿和右腿，观测方向都不变。
        例如有一个人站在你面前，你走到他的右手边来观测他的手的运动。
  (#)   以下为正方向：
  (#)	基于HGC模型:
        ①https://zhuanlan.zhihu.com/p/563048952
        ②https://zhuanlan.zhihu.com/p/613007726

  (#) Chassis_Posture 里输出的数据方向应该：
        以你的头为本体，来示意方向
        pitch	方向：抬头为正
        roll	方向：头往右边歪为正
        yaw 	方向：头往左扭为正

  (#) Chassis_Data_Update 里需要保证以下数据输入正确：
        phi1 						 单位：rad  ，方向：逆时针,零点：水平向左
        phi4 						 单位：rad  ，方向：逆时针，零点：水平向左
        phi1_d1 					 单位：rad/s  ，方向：逆时针
        phi4_d1 					 单位：rad/s  ，方向：逆时针
        torque_phi1_mea				 单位：N*m ，方向逆时针
        torque_phi4_mea				 单位：N*m ，方向逆时针


  (#) Chassis_State_Var_Update 里需要保证以下数据输入正确：
        vir_phi0 					 单位：rad  ，方向：逆时针 ，零点：竖直向下
        pitch 						 单位：rad  ，方向：逆时针 ，零点：水平向右
        vir_phi0_d1 				 单位：rad/s  ，方向：逆时针
        pitch_d1 					 单位：rad/s  ，方向：逆时针
        L_Wheel.rx_info->speed		 单位：rad/s  ，方向：顺时针
        R_Wheel.rx_info->speed		 单位：rad/s  ，方向：顺时针
        L_Wheel.rx_info->motor_angle_sum		 单位：rad  ，方向：顺时针
        R_Wheel.rx_info->motor_angle_sum		 单位：rad  ，方向：顺时针
        l0							 单位：m  ，方向：伸腿增大
        α							 -vir_phi0

        计算出来的变量方向：
        thetal				 		 单位：rad ，方向:顺时针，零点：水平向下
        thetal_d1					 单位：rad/s ，方向:顺时针
        stator_bias			    	 伸腿时相当于轮子后退（逆时针），具体请看Stator_Correction_Cal
        L_Wheel_speed_Transformed      单位：rad 方向：车前进时为正（顺时针）
        R_Wheel_speed_Transformed	   单位：rad 方向：车前进时为正（顺时针）
        L_Wheel_anglesum_Transformed   单位：rad 方向：车前进时为正（顺时针）
        R_Wheel_anglesum_Transformed   单位：rad 方向：车前进时为正（顺时针）

    (#) Chassis_Target_Update 这里的方向自己定

    ==============================================================================
                                ##### Quick Start #####
    ==============================================================================
  (#)	更改car_info的参数，使其符合车体建模

  (#)	连上关节电机，检查五连杆解算，l0，vir_phi0，及其导数是否正确

  (#)   连上轮毂电机，检查speed，anglesum是否正确

  (#)   连上板子，检查姿态输入是否正确

  (#)   自救时腿部Tp重力补偿

  (#)   氮气弹簧前馈

  (#)   检查s，sd1状态变量是否正确，检查状态变量误差是否正确，检查LQR能否正常输出Tp,Tw

  (#)   在Balance文件定义的 LEG_TEST_Mode 下检查 l0，phi0控制

  (#)	检查重力前馈

  (#)   定腿长调k矩阵，实现平衡，仅保留LQR，重力前馈。初始目标腿长在car_info里，k矩阵K_MATRIX_COEFFICIENT

  (#)	离地检测，仅保持thetal

  (#)	前进后退，旋转；需要测试双腿协调sync

  (#)	roll轴控制

  (#)	检查是否能防打滑

  (#)	根据腿长拟合k矩阵，实现平衡

  (#)   自救

  (#)	磕膝上台阶

  (#)	功率限制

  (#)	跳跃


    ==============================================================================
                                ##### Complete not yet #####
    ==============================================================================

    (#)



  */

#include "Chassis.h"
#include "Filter.h"
#include "RP_Log.h"
#include "car_info.h"
void Chassis_Init(Chassis_t *My_Chassis);
static void Chassis_HeartBeat(Chassis_t *My_Chassis);
/*任务调用函数 begin*/
static void Chassis_Data_Update(Chassis_t *My_Chassis);
static void Chassis_Status_React(Chassis_t *My_Chassis);
static void Chassis_Ctrl(Chassis_t *My_Chassis);
/*任务调用函数 end*/
static void Chassis_Work(Chassis_t *My_Chassis);

static float My_Phi1_Transform(Leg_e My_Leg_e, Motor_DM_t *my_motor);
static float My_Phi4_Transform(Leg_e My_Leg_e, Motor_DM_t *my_motor);
static void Chassis_State_Var_Update(Chassis_t *My_Chassis);
static void Stator_Correction_Cal(Chassis_t *My_Chassis);
static void Chassis_Offline_Process(Chassis_t *My_Chassis);

/* 目标设置函数begin */
static void Chassis_Rc_Input_Update(Chassis_t *My_Chassis);
static void Chassis_Key_Input_Update(Chassis_t *My_Chassis);
static void Chassis_sd1_Target_Update(Chassis_t *My_Chassis);
static void Chassis_Yaw_Target_Process_All(Chassis_t *My_Chassis);
static void Cycle_Target_Process(Chassis_t *My_Chassis); // 小陀螺目标旋转速度处理
static void Chassis_Leg_Length_Target_Process(Chassis_t *My_Chassis);
static void Chassis_Speed_Limit(Chassis_t *My_Chassis);
static void Chassis_Thetal_Target_By_Leg_Length_Update(Chassis_t *My_Chassis);
static void Chassis_Thetal_Target_Update(Chassis_t *My_Chassis);
static void Chassis_Thetab_Target_Update(Chassis_t *My_Chassis);
static void Chassis_Target_Update(Chassis_t *My_Chassis);
static void Chassis_thetab_err_constrain(Chassis_t *My_Chassis);
/* 目标设置函数end */

/*腿长相关力 begin*/
static void Chassis_Leg_Length_Strength_Cal(Chassis_t *My_Chassis);
static void Chassis_Roll_Control(Chassis_t *My_Chassis);
static void Chassis_Link_Feedforward_Cal(Chassis_t *My_Chassis);
static void Chassis_Leg_Fbl_Cal(Chassis_t *My_Chassis);
/*腿长相关力 end*/

/*-------- Tp相关力 begin --------*/
// 双腿协调
static void Cal_Leg_Gravity_Torque(Chassis_t *My_Chassis); // 自救时腿部重力补偿
static void Chassis_Leg_Sync_Cal(Chassis_t *My_Chassis);

// 用于自救
static void Chassis_Leg_vir_phi0_Cal(Chassis_t *My_Chassis);
static void Chassis_Leg_vir_phi0_d1_Cal(Chassis_t *My_Chassis); // 自救控速
/* -------- Tp相关力 end ---------*/

static void Chassis_Clean_Process(Chassis_t *My_Chassis);
static void Chassis_Wheel_Turn_Cal(Chassis_t *My_Chassis);
// static void Chassis_Wheel_Set_Zero_Detection(Chassis_t *My_Chassis);//在特殊情况设置轮子力矩为0，此处做全局检测

/******************** 计算chassis所有电机力矩 begin**********************/
static void Chassis_Tp_target_Cal(Chassis_t *My_Chassis);
static void Chassis_Fbl_target_Cal(Chassis_t *My_Chassis);
static void Chassis_Tw_target_Cal(Chassis_t *My_Chassis);
static void Chassis_Torque_Cal(Chassis_t *My_Chassis);
/******************** 计算chassis所有电机力矩 end*******************/

// 由控制层到电机层，将计算出来的扭矩赋给相应电机
static void Chassis_Set_Torque(Chassis_t *My_Chassis);
// 电机层目标扭矩设0
static void Chassis_Motor_Set_Sleep(Chassis_t *My_Chassis);

// 离地检测
static void Chassis_Takeoff_Detect(Chassis_t *My_Chassis);

// 阻尼卸力，真正调用的函数
static void Chassis_Damping_Sleep(Chassis_t *My_Chassis);
// Chassis_Damping_Sleep内部函数
static void Chassis_Stop_Damping(Chassis_t *My_Chassis);

// 功率限制
static void Chassis_Power_Limit(void); // 受力分析

/* 测试用功能 begin*/
static void Test_Basic_Control(Chassis_t *My_Chassis);
static void Test_Straight_Ctrl(Chassis_t *My_Chassis);
/* 测试用功能 end*/

static void Chassis_Init_Ctrl(Chassis_t *My_Chassis);
/********************** 特殊动作 begin **************************/
// 自救
static void Rescue_State_Process(Chassis_t *My_Chassis);           // 自救状态机,自救状态机检测在Balance.c的Rescue_Check里
static void Chassis_Manual_Rescue_Process(Chassis_t *My_Chassis);  // 手动自救模式处理
static void My_Chassis_Handle_Save_Process(Chassis_t *My_Chassis); // 操作手手动自救
// 跳跃过程处理
static void Jump_Target_Process(Chassis_t *My_Chassis);
// 撞膝上台阶
static void KNEE_STRIKE_Target_Process(Chassis_t *My_Chassis);
static void JUMP_THEN_KNEE_STRIKE_Target_Process(Chassis_t *My_Chassis);
static void JUMP_AND_MID_Target_Process(Chassis_t *My_Chassis);
static void Retract_down_Two_Step_Process(Chassis_t *My_Chassis);
/********************** 特殊动作 end **************************/

/********************** 功率限制 begin **************************/
static void Chassis_Power_Predict(Chassis_t *My_Chassis);
static void Chassis_Wheel_Torque_Limit_Calc(Chassis_t *My_Chassis);
static void Chassis_Model_Power_Limit_Update(Chassis_t *My_Chassis);
/********************** 功率限制 end **************************/

/********************** 工具函数 begin **************************/
static void Chassis_Motor_Group_Offline_Check(Chassis_t *My_Chassis);
/********************** 工具函数 end **************************/

// 氮气弹簧补偿配置
Spring_compensation_info_t spring_c_info = {
    .springForce = 300,
    .d_BigLeg = 0.04715f,
    .angle_cauchy = 70.f,

    .m_length = 0.06f,
    .n_length = 0.116f,
    .j_length = 0.04715f,
    .k_length = 0.06f,
    .k_compensation_knee_joint_leg = 7.f,
    .Knee_joint_leg_length = 0.09f};
// 自救配置
Chassis_Rescue_info_t rescue_info = {
    .rescue_state_mac = Reset,
    .rescue_config = {
        .pitchBackwardFlipThreshold = 90.f, // 后翻pitch判断阈值
        .pitchForwardFlipThreshold = -80.f, // 前翻pitch判断阈值
        .pitchNormalThreshold = 30.f,       // pitch正常判断阈值，主要用于前翻时从前摆腿切换到后摆腿 以及头归位
        .rollRightRolloverThreshold = 20.f, // 右侧翻roll判断阈值，左腿腾空
        .rollLeftRolloverThreshold = -20.f, // 左侧翻roll判断阈值，右腿腾空

        .thetalForwardThreshold = -60.f, // 腿前伸theta角度自救判断阈值
        .thetalBackwardThreshold = 85.f, // 腿后伸theta角度自救判断阈值

        // 腿后摆 收腿theta角度判断阈值，不用堵转判断的原因是这个可以控制收腿位置，比较灵活，其实都有利有弊
        .thetaBackwardRetractLegThreshold = 80.f,

        .phi0PRNormalBackwardSpeed = 360.f,            // 车身正常腿phi0后摆速度，°每秒
        .phi0ForwardFlipSpeed = -250.f,                // 前翻自救腿phi0前摆速度，°每秒
        .phi0BackwardFlipSpeed = 250.f,                // 后翻自救腿phi0后摆速度，°每秒
        .retractLegLengthTolerance = 0.08f,            // 收腿成功判断容忍度
        .RetractLegsTimeoutPeriod = 1000.f,            // *** 平躺收腿超时时间 (ms)
        .PRNormalBackwardLegTimeoutPeriod = 3000.f,    // *** Pitch正常，腿后摆，头归正超时时间 (ms)
        .CorrectGimbalDirectionTimeoutPeriod = 3000.f, // *** 等待云台yaw归正超时时间 (ms)，gimbal内部也有超时
        .RolloverTimeoutPeriod = 9000.f,               // roll翻超时时间 (ms)
        .ForwardFlipTimeoutPeriod = 9000.f,            // 前翻到pitch正常超时时间 (ms)
        .BackwardFlipTimeoutPeriod = 9000.f,           // 后翻到pitch正常超时时间 (ms)
    }};

// Manual Rescue配置
Chassis_manual_rescue_info_t manual_rescue_info = {
    .config = {
        .phi0d1_speed_abs = 150.0f, // °/s
        .F_change_speed = 350.0f,   // N/s
        .F_max = 300.0f,
        .F_min = -300.0f,
    },
    .var = {0},
};

// 将Instance导入
Leg_force_t Leg_force[Leg_Num];
Leg_Unit_t Leg_Unit[Leg_Num] =
    {
        [R_Leg].Straight = &Straight_Leg[R_Leg],
        [R_Leg].Link = &Link[R_Leg],

        [L_Leg].Straight = &Straight_Leg[L_Leg],
        [L_Leg].Link = &Link[L_Leg],

        [R_Leg].force = &Leg_force[R_Leg],
        [L_Leg].force = &Leg_force[L_Leg],

        [R_Leg].K_Leg_Gravity_Torque = 1,
        [L_Leg].K_Leg_Gravity_Torque = 1,
};

Chassis_Rc_Input_t Chassis_Rc_Input;
Chassis_state_t Chassis_state;
Chassis_Jump_t Chassis_Jump;
Chassis_Jump_And_Mid_t Chassis_Jump_And_Mid;
Chassis_Retract_to_Second_Step_Info_t Chassis_RTS_Info;
Chassis_Knee_Strike_t Chassis_Knee_Strike;
Chassis_Power_Limit_t Power_Limit;
Chassis_cycle_info_t Chassis_cycle_info;
Chassis_pid_init_parament_t Chassis_pid_init_parament;
Chassis_Model_Power_Limit_Info_t Model_PL[Leg_Num] = {
    [R_Leg].config.a = 2.f,
    [R_Leg].config.k_i2 = 1.93e-07f,
    [R_Leg].config.k_iw = 1.74e-06f,
    [R_Leg].config.k_w2 = 2.0e-07f,
    [R_Leg].config.k_torque_to_LSB = 1.f / _3508_REDUCT_TORQUE_COEFFICIENT / _3508_VALUE_TO_CURRENT_COEFFICIENT,

    [L_Leg].config.a = 2.f,
    [L_Leg].config.k_i2 = 1.93e-07f,
    [L_Leg].config.k_iw = 1.74e-06f,
    [L_Leg].config.k_w2 = 2.0e-07f,
    [L_Leg].config.k_torque_to_LSB = 1.f / _3508_REDUCT_TORQUE_COEFFICIENT / _3508_VALUE_TO_CURRENT_COEFFICIENT,
};
Chassis_etc_config_t Chassis_etc_config =
    {
        .thetal_length_linear_k = -0.35f,
        .thetal_length_linear_b = 0.1f,
        .cycle_thetal_l_target = -0.08f,
        .cycle_thetal_r_target = 0.2f,

        .cycle_length_l_target = 0.15f,
        .cycle_length_r_target = 0.15f,
        .Mid_Leg_Max_Speed = 2.1f,
        .cycle_thetab_target = 0.02f,
        .max_thetab_err = 20.f * Degree_to_rad,
};
Chassis_Target_t Chassis_Target =
    {
        .s = 0.f,
        .sd1 = 0.f,
        .yaw_v_degree = 0.f,
        .yaw_degree = 0.f,
        .roll = 0.f,
        .thetal_r = 0.1f,
        .thetal_l = 0.1f,
        .leg_length_l = TAR_LEG_LENGTH_INITIAL,
        .leg_length_r = TAR_LEG_LENGTH_INITIAL,

        .thetab = -0.0f,
        .thetabd1 = 0.f,
        .limit_v = 0.f,
};

Chassis_t Chassis = {
    .Init = Chassis_Init,

};

/**
 * @brief  底盘软件层初始化
 * @param  Chassis_t* My_Chassis
 * @retval None
 */
void Chassis_Init(Chassis_t *My_Chassis)
{
    /*结构体初始化*/
    My_Chassis->Posture = &Chassis_Posture;
    My_Chassis->Wheel = &Wheel_Group;
    My_Chassis->Sd = &Sd_Group;
    My_Chassis->state = &Chassis_state;
    My_Chassis->rc_input = &Chassis_Rc_Input;
    My_Chassis->damping_delay_cnt = DAMPING_DELAY_MAX_CNT;
    My_Chassis->target = &Chassis_Target;
    My_Chassis->chassis_PID = &chassis_PID;
    My_Chassis->etc_config = &Chassis_etc_config;
    My_Chassis->jump_info = &Chassis_Jump;
    My_Chassis->jump_and_mid_info = &Chassis_Jump_And_Mid;
    My_Chassis->rts_info = &Chassis_RTS_Info;
    My_Chassis->knee_strike_info = &Chassis_Knee_Strike;
    My_Chassis->cycle_info = &Chassis_cycle_info;
    My_Chassis->rescue_info = &rescue_info;
    My_Chassis->manual_rescue_info = &manual_rescue_info;
    My_Chassis->pid_init_parament = &Chassis_pid_init_parament;
    My_Chassis->Leg_Unit[R_Leg] = &Leg_Unit[R_Leg];
    My_Chassis->Leg_Unit[L_Leg] = &Leg_Unit[L_Leg];
    My_Chassis->spring_c_info = &spring_c_info;
    My_Chassis->Power_Limit_info = &Power_Limit;
    My_Chassis->Model_PL[R_Leg] = &Model_PL[R_Leg];
    My_Chassis->Model_PL[L_Leg] = &Model_PL[L_Leg];
    /*参数初始化*/
    My_Chassis->key_input->w_now = 0;
    My_Chassis->key_input->s_now = 0;
    My_Chassis->key_input->w_last = 0;
    My_Chassis->key_input->s_last = 0;
    My_Chassis->key_input->keyboard_forward_speed = 0;
    My_Chassis->key_input->keyboard_forward_speed_last = 0;
    My_Chassis->key_input->keyboard_lateral_speed = 0;
    My_Chassis->key_input->keyboard_lateral_speed_last = 0;

    // 小陀螺配置
    Chassis_cycle_info.high_cap_constant_speed = 700.0f; // 匀速小陀螺目标速度 (deg/s)
    Chassis_cycle_info.low_cap_constant_speed = 400.0f;
    Chassis_cycle_info.high_cap_threshold = 18.f;
    Chassis_cycle_info.vary_min_speed = 150.0f;   // 变速小陀螺最低速度 (deg/s)
    Chassis_cycle_info.vary_max_speed = 800.0f;   // 变速小陀螺最高速度 (deg/s)
    Chassis_cycle_info.vary_accel_duration = 600; // 变速小陀螺加速段时间 (ms)
    Chassis_cycle_info.vary_const_duration = 200; // 变速小陀螺恒速段时间 (ms)
    Chassis_cycle_info.vary_decel_duration = 600; // 变速小陀螺减速段时间 (ms)

    // 跳跃
    Chassis_Jump.Minimum_l0_range = 0.02f;
    Chassis_Jump.Max_l0_range = 0.02f;
    Chassis_Jump.Landing_l0_range = 0.02f;

    Chassis_Jump.Max_COMPRESS_tick = 300.f;
    Chassis_Jump.Max_EXTEND_tick = 500.f;  // 小跳，所以伸腿时间段短
    Chassis_Jump.Max_RETRACT_tick = 400.f; // 小跳，所以收腿时间短
    Chassis_Jump.Max_PRE_LANDING_tick = 500.f;
    Chassis_Jump.Max_LANDING_tick = 300.f;

    // 各阶段目标腿长
    Chassis_Jump.J_COMPRESS_length_target = TAR_LEG_LENGTH_INITIAL;             // 压缩阶段目标：初始腿长
    Chassis_Jump.J_EXTEND_length_target = MAX_LEG_LENGTH;                       // 伸腿阶段目标：最大腿长
    Chassis_Jump.J_RETRACT_length_target = MIN_LEG_LENGTH;                      // 收腿阶段目标：最小腿长
    Chassis_Jump.J_PRE_LANDING_length_target = TAR_LEG_LENGTH_INITIAL + 0.005f; // 预落地目标：初始腿长+5mm
    Chassis_Jump.J_LANDING_length_target = TAR_LEG_LENGTH_INITIAL;              // 落地目标：初始腿长

    Chassis_Jump.COMPRESS_length_kp = 60.f;
    Chassis_Jump.EXTEND_length_kp = 4000.f;
    Chassis_Jump.RETRACT_length_kp = 60.f;
    Chassis_Jump.PRE_LANDING_length_kp = 200.f;
    Chassis_Jump.LANDING_length_kp = 60.f;
    Chassis_Jump.LANDING_length_speed_kp = 1000.f;

    // 跳+中腿长（默认参数来自JUMP_AND_MID状态机当前标定值）
    Chassis_Jump_And_Mid.Minimum_l0_range = 0.02f;
    Chassis_Jump_And_Mid.Max_l0_range = 0.02f;
    Chassis_Jump_And_Mid.Landing_l0_range = 0.04f;
    Chassis_Jump_And_Mid.JAM_COMPRESS_target_length = MIN_LEG_LENGTH;
    Chassis_Jump_And_Mid.JAM_EXTEND_target_length = MIN_LEG_LENGTH + 0.8f;
    Chassis_Jump_And_Mid.Max_EXTEND_tick = 900;
    Chassis_Jump_And_Mid.JAM_RETRACT_target_length = MIN_LEG_LENGTH;
    Chassis_Jump_And_Mid.Max_RETRACT_tick = 400;
    Chassis_Jump_And_Mid.JAM_PRE_LANDING_target_length = MAX_LEG_LENGTH - 0.07f;
    Chassis_Jump_And_Mid.Max_PRE_LANDING_tick = 500;
    Chassis_Jump_And_Mid.JAM_LANDING_target_length = MID_LEG_LENGTH;
    Chassis_Jump_And_Mid.Max_COMPRESS_tick = 300;

    Chassis_Jump_And_Mid.Max_LANDING_tick = 800;
    Chassis_Jump_And_Mid.Max_Both_Onground_tick = 300; // 两腿同时在地面(不离地)的最长容忍时间
    Chassis_Jump_And_Mid.COMPRESS_length_kp = 70.f;
    Chassis_Jump_And_Mid.EXTEND_length_kp = 80.f;
    Chassis_Jump_And_Mid.RETRACT_length_kp = 90.f;
    Chassis_Jump_And_Mid.PRE_LANDING_length_kp = 50.f;
    Chassis_Jump_And_Mid.LANDING_length_kp = 50.f;
    Chassis_Jump_And_Mid.JUMP_AND_MID_step = JAM_IDLE;
    Chassis_Jump_And_Mid.JUMP_AND_MID_tick = 0;
    Chassis_Jump_And_Mid.Max_JUMP_AND_MID_tick = 10000;

    // 收腿下二级台阶RTS
    Chassis_RTS_Info.config.front_swing_leg_length = MAX_LEG_LENGTH;
    Chassis_RTS_Info.config.landing_leg_length = MID_LEG_LENGTH;
    Chassis_RTS_Info.config.front_angle = 10.0f;
    Chassis_RTS_Info.config.behind_angle = 10.0f;
    Chassis_RTS_Info.config.front_threshold = 5.0f;
    Chassis_RTS_Info.config.behind_threshold = 5.0f;
    Chassis_RTS_Info.config.sd1_speed_reduced = 0.8f;
    Chassis_RTS_Info.config.theta_offset = 15.0f;
    Chassis_RTS_Info.config.front_swing_timeout = 15000;
    Chassis_RTS_Info.config.min_front_swing_time = 600;
    Chassis_RTS_Info.config.landing_reset_time = 4000;
    Chassis_RTS_Info.config.encoder_speed_threshold = 100000.0f;
    Chassis_RTS_Info.config.theta_threshold_offset = 5.0f;

    Chassis_RTS_Info.var.rts_step = RTS_IDLE;
    Chassis_RTS_Info.var.rts_tick = 0;
    Chassis_RTS_Info.var.front_swing_tick = 0;
    Chassis_RTS_Info.var.landing_tick = 0;

    // 撞膝上台阶
    Chassis_Knee_Strike.Max_Stand_High_tick = 15000; // 最长站高时间
    Chassis_Knee_Strike.thetal_threshold = 42.f;     // 收腿摆脚阈值
    Chassis_Knee_Strike.thetal_diff_max = 5.f;       // 两腿thetal差值上限(°)，超过则不触发收腿
    Chassis_Knee_Strike.RETRACT_length_kp = 200.f;
    Chassis_Knee_Strike.Minimum_l0_range = 0.05f;
    Chassis_Knee_Strike.Max_l0_range = 0.03f;
    Chassis_Knee_Strike.Max_RETRACT_tick = 700;

    // 功率限制相关参数
    Power_Limit.a = 0.f;
    Power_Limit.kk = 0.f;
    Power_Limit.Nf = 0.f;
    Power_Limit.power_limit = 60.f;
    Power_Limit.Tp_Big = 0.f;
    Power_Limit.Tw_Enable = 2.3f;
    Power_Limit.turn_limit_high_buffer_threshold = 50;
    Power_Limit.turn_limit_mid_buffer_threshold = 30;
    Power_Limit.turn_limit_low_buffer_threshold = 20;
    Power_Limit.k_cap_capacity_scale_limit = 0.9f;
    // pid初始参数
    My_Chassis->pid_init_parament->l0_length_kp = My_Chassis->chassis_PID->length_cal[R_Leg]->kp;
    My_Chassis->pid_init_parament->l0_length_speed_kp = My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp;
    My_Chassis->pid_init_parament->l0_length_outmax = My_Chassis->chassis_PID->length_cal[R_Leg]->out_max;
    My_Chassis->pid_init_parament->l0_length_speed_outmax = My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max;

    /*电机初始化*/
    My_Chassis->Wheel->group_init(My_Chassis->Wheel);
    My_Chassis->Sd->group_init(My_Chassis->Sd);

    /*底盘函数初始化*/
    My_Chassis->heartbeat = Chassis_HeartBeat;
    My_Chassis->data_update = Chassis_Data_Update;
    My_Chassis->status_react = Chassis_Status_React;
    My_Chassis->work = Chassis_Work;
    My_Chassis->ctrl = Chassis_Ctrl;

    /*五连杆初始化*/
    My_Chassis->Leg_Unit[R_Leg]->Link->init(My_Chassis->Leg_Unit[R_Leg]->Link);
    My_Chassis->Leg_Unit[L_Leg]->Link->init(My_Chassis->Leg_Unit[L_Leg]->Link);
    My_Chassis->Leg_Unit[R_Leg]->Straight->init(My_Chassis->Leg_Unit[R_Leg]->Straight);
    My_Chassis->Leg_Unit[L_Leg]->Straight->init(My_Chassis->Leg_Unit[L_Leg]->Straight);

    /*卡尔曼滤波器初始化*/
    xvEstimateKF_Init(&vaEstimateKF);
    XEstimateKF_Init(&XEstimateKF);
}

/**
 * @brief  底盘心跳包
 * @param  Chassis_t* My_Chassis
 * @retval None
 */
static void Chassis_HeartBeat(Chassis_t *My_Chassis)
{
    My_Chassis->Wheel->group_heartbeat(My_Chassis->Wheel);
    My_Chassis->Sd->group_heartbeat(My_Chassis->Sd);

    Chassis_Motor_Group_Offline_Check(My_Chassis);

    if (My_Chassis->state->sd_state == DEV_ONLINE && My_Chassis->state->wheel_state == DEV_ONLINE)
    {
        Balance.Flag->Chassis_Online_Flag = true;
    }
    else
    {
        Balance.Flag->Chassis_Online_Flag = false;
    }
}

/**
 * @brief  底盘运行
 * @param  Chassis_t* My_Chassis
 * @retval None
 */
static void Chassis_Work(Chassis_t *My_Chassis)
{
    My_Chassis->status_react(My_Chassis);
    My_Chassis->data_update(My_Chassis);
    My_Chassis->ctrl(My_Chassis);
}

/**
 * @brief  底盘数据更新
 * @param  Chassis_t* My_Chassis
 * @retval None
 */
static void Chassis_Data_Update(Chassis_t *My_Chassis)
{
    /*更新原始数据*/
    My_Chassis->Posture->data_update(My_Chassis->Posture);
    /*更新五连杆测量数据*/
    float phi1 = My_Phi1_Transform(R_Leg, My_Chassis->Sd->motor[R_F_Sd_M]);
    float phi4 = My_Phi4_Transform(R_Leg, My_Chassis->Sd->motor[R_B_Sd_M]);
    float phi1_d1 = My_Chassis->Sd->motor[R_F_Sd_M]->rx_info->speed;
    float phi4_d1 = My_Chassis->Sd->motor[R_B_Sd_M]->rx_info->speed;
    float torque_phi1_mea = My_Chassis->Sd->motor[R_F_Sd_M]->rx_info->torque;
    float torque_phi4_mea = My_Chassis->Sd->motor[R_B_Sd_M]->rx_info->torque;
    My_Chassis->Leg_Unit[R_Leg]->Link->mea_data_update(My_Chassis->Leg_Unit[R_Leg]->Link, phi1, phi1_d1, phi4, phi4_d1, torque_phi1_mea, torque_phi4_mea);

    phi1 = My_Phi1_Transform(L_Leg, My_Chassis->Sd->motor[L_F_Sd_M]);
    phi4 = My_Phi4_Transform(L_Leg, My_Chassis->Sd->motor[L_B_Sd_M]);
    phi1_d1 = -My_Chassis->Sd->motor[L_F_Sd_M]->rx_info->speed;
    phi4_d1 = -My_Chassis->Sd->motor[L_B_Sd_M]->rx_info->speed;
    torque_phi1_mea = -My_Chassis->Sd->motor[L_F_Sd_M]->rx_info->torque;
    torque_phi4_mea = -My_Chassis->Sd->motor[L_B_Sd_M]->rx_info->torque;
    My_Chassis->Leg_Unit[L_Leg]->Link->mea_data_update(My_Chassis->Leg_Unit[L_Leg]->Link, phi1, phi1_d1, phi4, phi4_d1, torque_phi1_mea, torque_phi4_mea);

    /*更新五连杆数据*/
    My_Chassis->Leg_Unit[R_Leg]->Link->link_update(My_Chassis->Leg_Unit[R_Leg]->Link);
    My_Chassis->Leg_Unit[L_Leg]->Link->link_update(My_Chassis->Leg_Unit[L_Leg]->Link);

    /*更新氮气弹簧数据*/
    My_Spring_Former_Input_Cal(My_Chassis);

    /*VMC逆解算,并与串*/
    //	My_Chassis->Leg_Unit[R_Leg]->Link->Fb1_Tp_cal(My_Chassis->Leg_Unit[R_Leg]->Link);
    //	My_Chassis->Leg_Unit[L_Leg]->Link->Fb1_Tp_cal(My_Chassis->Leg_Unit[L_Leg]->Link);

    My_Link_reverse_cal(My_Chassis->Leg_Unit[L_Leg]->Link, -(Sd_Group.motor[L_F_Sd_M]->rx_info->torque + My_Chassis->Leg_Unit[L_Leg]->force->T_Spring_Compensation_Front), -(Sd_Group.motor[L_B_Sd_M]->rx_info->torque - My_Chassis->Leg_Unit[L_Leg]->force->T_Spring_Compensation_Back));
    My_Link_reverse_cal(My_Chassis->Leg_Unit[R_Leg]->Link, Sd_Group.motor[R_F_Sd_M]->rx_info->torque - My_Chassis->Leg_Unit[R_Leg]->force->T_Spring_Compensation_Front, Sd_Group.motor[R_B_Sd_M]->rx_info->torque + My_Chassis->Leg_Unit[R_Leg]->force->T_Spring_Compensation_Back);
    // My_Link_reverse_cal(My_Chassis->Leg_Unit[L_Leg]->Link, -(Sd_Group.motor[L_F_Sd_M]->rx_info->torque), -(Sd_Group.motor[L_B_Sd_M]->rx_info->torque));
    // My_Link_reverse_cal(My_Chassis->Leg_Unit[R_Leg]->Link, Sd_Group.motor[R_F_Sd_M]->rx_info->torque, Sd_Group.motor[R_B_Sd_M]->rx_info->torque);
    /*输入到直腿模型*/
    My_Chassis->Leg_Unit[R_Leg]->Straight->ex_data_update(My_Chassis->Leg_Unit[R_Leg]->Straight, My_Chassis->Leg_Unit[R_Leg]->Link->info->length->l0);
    My_Chassis->Leg_Unit[L_Leg]->Straight->ex_data_update(My_Chassis->Leg_Unit[L_Leg]->Straight, My_Chassis->Leg_Unit[L_Leg]->Link->info->length->l0);

/*K矩阵更新*/
#ifndef NO_K_Fitting
    My_Chassis->Leg_Unit[R_Leg]->Straight->K_fitting(My_Chassis->Leg_Unit[R_Leg]->Straight);
    My_Chassis->Leg_Unit[L_Leg]->Straight->K_fitting(My_Chassis->Leg_Unit[L_Leg]->Straight);
#endif

    /*状态量更新，直接对Straight结构体操作*/
    Chassis_State_Var_Update(My_Chassis);

    /*目标值更新*/
    Chassis_Target_Update(My_Chassis); // 放在Chassis_State_Var_Update后面，才好处理离地情况和加速时额外的s位移
}

/**
 * @brief  底盘thetab目标值更新
 * @param  None
 * @retval None
 */
static void Chassis_Thetab_Target_Update(Chassis_t *My_Chassis)
{
    if (My_Chassis->mode == C_Cycle && My_Chassis->last_mode == C_Cycle)
    {
        My_Chassis->target->thetab = My_Chassis->etc_config->cycle_thetab_target;
    }
    else
    {
        My_Chassis->target->thetab = 0.f;
    }
}
/**
 * @brief  根据两腿平均腿长线性映射更新左右腿thetal目标值
 * @param  Chassis_t* My_Chassis
 * @retval None
 */

static void Chassis_Thetal_Target_By_Leg_Length_Update(Chassis_t *My_Chassis)
{
    // 计算两腿平均目标腿长，并限制在机械定义范围内
    float leg_length_avg = 0.5f * (My_Chassis->target->leg_length_l + My_Chassis->target->leg_length_r);
    leg_length_avg = constrain(leg_length_avg, MIN_LEG_LENGTH, MAX_LEG_LENGTH);

    // 线性映射: thetal = k * leg_length_avg + b
    My_Chassis->target->thetal_by_length = My_Chassis->etc_config->thetal_length_linear_k * leg_length_avg + My_Chassis->etc_config->thetal_length_linear_b;
}

/**
 * @brief  左右腿thetal目标值更新
 * @param  Chassis_t* My_Chassis
 * @retval None
 */

static void Chassis_Thetal_Target_Update(Chassis_t *My_Chassis)
{
    // 此为默认情况，特殊情况时后续可以覆盖此处的目标值
    My_Chassis->target->thetal_r = My_Chassis->target->thetal_by_length;
    My_Chassis->target->thetal_l = My_Chassis->target->thetal_by_length;

    // C_RTS模式：根据云台方向叠加前摆或后摆角度
    if (My_Chassis->mode == C_RTS && My_Chassis->rts_info->var.rts_step == RTS_Front_Swing)
    {
        float gimbal_yaw = gimbal.base_info.yaw_motor_angle;
        RTS_Config_t *config = &My_Chassis->rts_info->config;

        // 底盘和云台方向一致（gimbal yaw_angle < PI/2.f）时，使用前摆角度叠加（减）
        // 底盘和云台方向相反（gimbal yaw_angle > PI/2.f）时，使用后摆角度叠加（正）
        if (fabsf(gimbal_yaw) < PI / 2.f)
        {
            // 前摆：角度叠加（减）
            My_Chassis->target->thetal_l -= angle2rad(config->front_angle);
            My_Chassis->target->thetal_r -= angle2rad(config->front_angle);
        }
        else
        {
            My_Chassis->target->thetal_l += angle2rad(config->behind_angle);
            My_Chassis->target->thetal_r += angle2rad(config->behind_angle);
        }
    }
    else if (My_Chassis->mode == C_Cycle)
    {
        My_Chassis->target->thetal_r = My_Chassis->etc_config->cycle_thetal_l_target;
        My_Chassis->target->thetal_l = My_Chassis->etc_config->cycle_thetal_r_target;
    }
}

static void Chassis_Target_Update(Chassis_t *My_Chassis)
{
    if (Balance.ctrl != KEY_CTRL)
    {
        Chassis_Rc_Input_Update(My_Chassis); // 遥控器输入值步进限幅滤波
    }
    else
    {
        Chassis_Key_Input_Update(My_Chassis);
    }
    Chassis_sd1_Target_Update(My_Chassis);                  // 控sd1
    Chassis_Yaw_Target_Process_All(My_Chassis);             // 输出My_Chassis->target->yaw
    Chassis_Leg_Length_Target_Process(My_Chassis);          // 腿长目标值控制
    Chassis_Thetal_Target_By_Leg_Length_Update(My_Chassis); // 基于平均腿长更新thetal_r/thetal_l
    Chassis_Thetal_Target_Update(My_Chassis);               // thetal目标值更新
    Chassis_Thetab_Target_Update(My_Chassis);               // thetab目标值更新
    Chassis_Speed_Limit(My_Chassis);                        // sd1目标值限制
    Chassis_Model_Power_Limit_Update(My_Chassis);           // 模型功率限制更新,仅获取数据更新，不对发送力矩或电流进行处理

    /*离地、小陀螺时s和pitch的目标值=状态量，使这两项输出为0*/
    if (My_Chassis->Leg_Unit[R_Leg]->off_ground == true)
    {
        My_Chassis->Leg_Unit[R_Leg]->Straight->target_state_update(My_Chassis->Leg_Unit[R_Leg]->Straight,
                                                                   My_Chassis->target->thetal_r, My_Chassis->target->thetald1,
                                                                   My_Chassis->Leg_Unit[R_Leg]->Straight->info->s, My_Chassis->Leg_Unit[R_Leg]->Straight->info->sd1,
                                                                   My_Chassis->Leg_Unit[R_Leg]->Straight->info->thetab, My_Chassis->Leg_Unit[R_Leg]->Straight->info->thetabd1);
    }
    else
    {
        My_Chassis->Leg_Unit[R_Leg]->Straight->target_state_update(My_Chassis->Leg_Unit[R_Leg]->Straight,
                                                                   My_Chassis->target->thetal_r, My_Chassis->target->thetald1,
                                                                   My_Chassis->target->s, My_Chassis->target->sd1,
                                                                   My_Chassis->target->thetab, My_Chassis->target->thetabd1);
    }
    if (My_Chassis->Leg_Unit[L_Leg]->off_ground == true)
    {
        My_Chassis->Leg_Unit[L_Leg]->Straight->target_state_update(My_Chassis->Leg_Unit[L_Leg]->Straight,
                                                                   My_Chassis->target->thetal_l, My_Chassis->target->thetald1,
                                                                   My_Chassis->Leg_Unit[L_Leg]->Straight->info->s, My_Chassis->Leg_Unit[L_Leg]->Straight->info->sd1,
                                                                   My_Chassis->Leg_Unit[L_Leg]->Straight->info->thetab, My_Chassis->Leg_Unit[L_Leg]->Straight->info->thetabd1);
    }
    else
    {
        My_Chassis->Leg_Unit[L_Leg]->Straight->target_state_update(My_Chassis->Leg_Unit[L_Leg]->Straight,
                                                                   My_Chassis->target->thetal_l, My_Chassis->target->thetald1,
                                                                   My_Chassis->target->s, My_Chassis->target->sd1,
                                                                   My_Chassis->target->thetab, My_Chassis->target->thetabd1);
    }
}

/**
 * @brief  状态变量更新
 * @param  Chassis_t* My_Chassis
 * @retval None
 */
static void Chassis_State_Var_Update(Chassis_t *My_Chassis) // 角度均用弧度制
{
    /*结构体输入*/
    State_info_t *R_Leg_State_Var = My_Chassis->Leg_Unit[R_Leg]->Straight->info;
    State_info_t *L_Leg_State_Var = My_Chassis->Leg_Unit[L_Leg]->Straight->info;
    Link_t *R_Leg_Link = My_Chassis->Leg_Unit[R_Leg]->Link;
    Link_t *L_Leg_Link = My_Chassis->Leg_Unit[L_Leg]->Link;
    Chassis_Posture_info_t *My_Posture = My_Chassis->Posture->info;
    Motor_RM_t L_Wheel = *My_Chassis->Wheel->motor[L_WHEEL_M];
    Motor_RM_t R_Wheel = *My_Chassis->Wheel->motor[R_WHEEL_M];

    /*杆倾斜角度、机体角度 begin*/
    // 与上交模型不同，thetal在VMC中逆时针为正，在LQR建模中顺时针为正
    R_Leg_State_Var->thetal_last = R_Leg_State_Var->thetal;
    R_Leg_State_Var->thetal = (R_VIR_PHI0_ORDER_CORRECT * R_Leg_Link->info->angle->vir_phi0 - My_Posture->pitch);
    R_Leg_State_Var->thetal_degree = R_Leg_State_Var->thetal / Degree_to_rad;
    R_Leg_State_Var->thetald1 = R_VIR_PHI0_ORDER_CORRECT * R_Leg_Link->info->angle->vir_phi0_d1 - My_Posture->pitch_v;
    R_Leg_State_Var->thetald1_l_now = R_Leg_State_Var->thetald1;
    R_Leg_State_Var->thetald2 = R_Leg_State_Var->thetald1_l_now - R_Leg_State_Var->thetald1_l_last;
    R_Leg_State_Var->thetald1_l_last = R_Leg_State_Var->thetald1_l_now;
    R_Leg_State_Var->thetab = My_Posture->pitch;
    R_Leg_State_Var->thetabd1 = My_Posture->pitch_v;

    L_Leg_State_Var->thetal_last = L_Leg_State_Var->thetal;
    L_Leg_State_Var->thetal = (L_VIR_PHI0_ORDER_CORRECT * L_Leg_Link->info->angle->vir_phi0 - My_Posture->pitch);
    L_Leg_State_Var->thetal_degree = L_Leg_State_Var->thetal / Degree_to_rad;
    L_Leg_State_Var->thetald1 = L_VIR_PHI0_ORDER_CORRECT * L_Leg_Link->info->angle->vir_phi0_d1 - My_Posture->pitch_v;
    L_Leg_State_Var->thetald1_l_now = L_Leg_State_Var->thetald1;
    L_Leg_State_Var->thetald2 = L_Leg_State_Var->thetald1_l_now - L_Leg_State_Var->thetald1_l_last;
    L_Leg_State_Var->thetald1_l_last = L_Leg_State_Var->thetald1_l_now;
    L_Leg_State_Var->thetab = My_Posture->pitch;
    L_Leg_State_Var->thetabd1 = My_Posture->pitch_v;
    /*杆倾斜角度、机体角度 end*/

    /*消除杆动-->电机定子动-->编码器变化带来的影响*/
    Stator_Correction_Cal(My_Chassis);

    /*俯仰角与俯仰角速度 begin*/
    R_Leg_State_Var->thetab = My_Posture->pitch;
    L_Leg_State_Var->thetab = My_Posture->pitch;
    R_Leg_State_Var->thetabd1 = My_Posture->pitch_v;
    L_Leg_State_Var->thetabd1 = My_Posture->pitch_v;
    /*俯仰角与俯仰角速度 end*/

    /*卡尔曼滤波更新*/
    /*路程与速度 begin*/
    float My_Wheel_Sb;
    float My_Imu_Sb;
    float My_filter_Sb;
    float My_filter_S;
    float My_filter_S_d1;

    float L_Wheel_speed_Transformed = L_W_SPEED_ORDER_CORRECT * L_Wheel.rx_info->speed;
    float R_Wheel_speed_Transformed = R_W_SPEED_ORDER_CORRECT * R_Wheel.rx_info->speed;
    float L_Wheel_anglesum_Transformed = L_Wheel.rx_info->motor_angle_sum; // 已在chassis_motor矫正方向
    float R_Wheel_anglesum_Transformed = R_Wheel.rx_info->motor_angle_sum;
    // 轮速+腿速
    My_Wheel_Sb = (WHEEL_RADIUS * (L_Wheel_speed_Transformed + R_Wheel_speed_Transformed) + (R_Leg_Link->info->length->l0 * arm_cos_f32(R_Leg_State_Var->thetal) * R_Leg_State_Var->thetald1) + (L_Leg_Link->info->length->l0 * arm_cos_f32(L_Leg_State_Var->thetal) * L_Leg_State_Var->thetald1)) / 2.f;
    My_Imu_Sb = My_Chassis->Posture->info->x_world; // 机体加速度
    xvEstimateKF_Update(&vaEstimateKF, My_Imu_Sb, My_Wheel_Sb);
    My_filter_Sb = vaEstimateKF.FilteredValue[0]; // 轮速+腿速

    // 轮子位移（已考虑定子转动）
    float s = 0.5f * WHEEL_RADIUS * ((float)L_Wheel_anglesum_Transformed - L_Leg_Link->info->stator_correction->stator_bias + (float)R_Wheel_anglesum_Transformed - R_Leg_Link->info->stator_correction->stator_bias);

    // 轮速
    float sd1 = My_filter_Sb -
                (((L_Leg_Link->info->length->l0 * arm_cos_f32(L_Leg_State_Var->thetal) * L_Leg_State_Var->thetald1) + (R_Leg_Link->info->length->l0 * arm_cos_f32(R_Leg_State_Var->thetal) * R_Leg_State_Var->thetald1)) / 2.f);

    XEstimateKF_Update(&XEstimateKF, sd1, s);
    My_filter_S = XEstimateKF.FilteredValue[0];    // 位移
    My_filter_S_d1 = XEstimateKF.FilteredValue[1]; // 速度

    R_Leg_State_Var->s = My_filter_S;
    L_Leg_State_Var->s = My_filter_S;
    R_Leg_State_Var->sd1 = My_filter_S_d1;
    L_Leg_State_Var->sd1 = My_filter_S_d1;

    /*路程与速度 end*/
}
static void Test_Straight_Ctrl(Chassis_t *My_Chassis)
{
    Link_t *My_L_Link = My_Chassis->Leg_Unit[L_Leg]->Link;
    Link_t *My_R_Link = My_Chassis->Leg_Unit[R_Leg]->Link;
    Straight_Leg_t *R_Straight = My_Chassis->Leg_Unit[R_Leg]->Straight;
    Straight_Leg_t *L_Straight = My_Chassis->Leg_Unit[L_Leg]->Straight;
    /*直腿模型计算，得到驱动轮输出力矩和虚拟关节力矩*/
    /*-----------求Tp_target begin--------*/

    Chassis_Leg_Sync_Cal(My_Chassis);

    R_Straight->LQR_cal(R_Straight); // 在目标值处做离地处理
    L_Straight->LQR_cal(L_Straight);
    My_Chassis->Leg_Unit[R_Leg]->force->Tp_LQR = R_Straight->get_Tp(R_Straight);
    My_Chassis->Leg_Unit[L_Leg]->force->Tp_LQR = L_Straight->get_Tp(L_Straight);

    if (My_Chassis->Leg_Unit[R_Leg]->off_ground == true ||
        My_Chassis->Leg_Unit[L_Leg]->off_ground == true) // 离地处理
    {
        My_Chassis->Leg_Unit[R_Leg]->force->Tp_target = My_Chassis->Leg_Unit[R_Leg]->force->Tp_sync;
        My_Chassis->Leg_Unit[L_Leg]->force->Tp_target = My_Chassis->Leg_Unit[L_Leg]->force->Tp_sync;
    }
    else
    {
        My_Chassis->Leg_Unit[R_Leg]->force->Tp_target = R_TP_LQR_ORDER_CORRECT * My_Chassis->Leg_Unit[R_Leg]->force->Tp_LQR + My_Chassis->Leg_Unit[R_Leg]->force->Tp_sync;
        My_Chassis->Leg_Unit[L_Leg]->force->Tp_target = L_TP_LQR_ORDER_CORRECT * My_Chassis->Leg_Unit[L_Leg]->force->Tp_LQR + My_Chassis->Leg_Unit[R_Leg]->force->Tp_sync;
    }

    /*-----------求Tp_target end-----------*/

    /*-----------求Fb1_target begin--------*/
    /*roll控制力计算*/
    Chassis_Roll_Control(My_Chassis);

    /*腿长控制力计算*/
    Chassis_Leg_Length_Strength_Cal(My_Chassis);

    /*前馈计算,内部含离地处理*/
    Chassis_Link_Feedforward_Cal(My_Chassis);

    /*汇总得到Fbl_target*/
    static float t = 0;
    if ((my_abs(Chassis.Leg_Unit[L_Leg]->Link->info->angle->vir_phi0) <= 0.2f && my_abs(Chassis.Leg_Unit[R_Leg]->Link->info->angle->vir_phi0) <= 0.2f) || t == 1)
    {
        t = 1;
        My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target = My_Chassis->Leg_Unit[R_Leg]->force->F + My_Chassis->Leg_Unit[R_Leg]->force->F_gravity
                                                          //+ My_Chassis->Leg_Unit[R_Leg]->force->F_roll
                                                          + My_Chassis->Leg_Unit[R_Leg]->force->F_inertial;
        My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target = My_Chassis->Leg_Unit[L_Leg]->force->F + My_Chassis->Leg_Unit[L_Leg]->force->F_gravity
                                                          //+ My_Chassis->Leg_Unit[L_Leg]->force->F_roll
                                                          + My_Chassis->Leg_Unit[L_Leg]->force->F_inertial;
    }
    else
    {
        My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target = My_Chassis->Leg_Unit[R_Leg]->force->F;
        //															+ My_Chassis->Leg_Unit[R_Leg]->force->F_gravity;
        //														+ My_Chassis->Leg_Unit[R_Leg]->force->F_roll
        //														+ My_Chassis->Leg_Unit[R_Leg]->force->F_inertial;
        My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target = My_Chassis->Leg_Unit[L_Leg]->force->F;
        //															+ My_Chassis->Leg_Unit[L_Leg]->force->F_gravity;
        //														+ My_Chassis->Leg_Unit[L_Leg]->force->F_roll
        //														+ My_Chassis->Leg_Unit[L_Leg]->force->F_inertial;
    }

    /*-----------求Fb1_target end--------*/

    /*-----------求Tw_target begin--------*/
    /*驱动轮转向环Tw_turn*/
    Chassis_Wheel_Turn_Cal(My_Chassis);
    My_Chassis->Leg_Unit[R_Leg]->force->Tw_LQR = R_Straight->get_Tw(R_Straight);
    My_Chassis->Leg_Unit[L_Leg]->force->Tw_LQR = L_Straight->get_Tw(L_Straight);
    /* 驱动轮电机最终输出 */
    if (My_Chassis->Leg_Unit[R_Leg]->off_ground == true) // 离地处理
    {
        My_Chassis->Leg_Unit[R_Leg]->force->Tw_target = 0;
    }
    else
    {
        My_Chassis->Leg_Unit[R_Leg]->force->Tw_target = My_Chassis->Leg_Unit[R_Leg]->force->Tw_LQR + My_Chassis->Leg_Unit[R_Leg]->force->Tw_turn;
    }
    if (My_Chassis->Leg_Unit[L_Leg]->off_ground == true) // 离地处理
    {
        My_Chassis->Leg_Unit[L_Leg]->force->Tw_target = 0;
    }
    else
    {
        My_Chassis->Leg_Unit[L_Leg]->force->Tw_target = My_Chassis->Leg_Unit[L_Leg]->force->Tw_LQR + My_Chassis->Leg_Unit[L_Leg]->force->Tw_turn;
    }
    /*-----------求Tw_target end--------*/

    My_R_Link->tar_data_update(My_R_Link, My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target, My_Chassis->Leg_Unit[R_Leg]->force->Tp_target);
    My_L_Link->tar_data_update(My_L_Link, My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target, My_Chassis->Leg_Unit[L_Leg]->force->Tp_target);

    /*转换为关节力矩,输出到Link结构体的F_Sd_Output_Torque，B_Sd_Output_Torque*/
    My_L_Link->torque_cal(My_L_Link);
    My_R_Link->torque_cal(My_R_Link);

    /* 关节电机最终输出 */
    My_Chassis->Leg_Unit[R_Leg]->force->Sd_F_Torque = My_R_Link->info->F_Sd_Output_Torque + My_Chassis->Leg_Unit[R_Leg]->force->Sd_F_Limit_Tor_Fix;
    My_Chassis->Leg_Unit[R_Leg]->force->Sd_B_Torque = My_R_Link->info->B_Sd_Output_Torque + My_Chassis->Leg_Unit[R_Leg]->force->Sd_B_Limit_Tor_Fix;
    My_Chassis->Leg_Unit[L_Leg]->force->Sd_F_Torque = My_L_Link->info->F_Sd_Output_Torque + My_Chassis->Leg_Unit[L_Leg]->force->Sd_F_Limit_Tor_Fix;
    My_Chassis->Leg_Unit[L_Leg]->force->Sd_B_Torque = My_L_Link->info->B_Sd_Output_Torque + My_Chassis->Leg_Unit[L_Leg]->force->Sd_B_Limit_Tor_Fix;
}

/**
 * @brief  集成了一些功能测试(宏定义这一块)
 * @param  Chassis_t* My_Chassis
 * @retval None
 */
static void Test_Basic_Control(Chassis_t *My_Chassis)
{
    // 默认只测试右腿，开启此宏定义则左腿也一起测试
#define TEST_LEFT_LEG

    // 以下最多只能同时开启一个测试功能，开启前注释掉其他测试功能
    // 请按顺序测试以下功能
#define Test_spring_feedforward // 氮气弹簧前馈
    // #define Test_Leg_Gravity_feedforward // 腿部竖直重力、扭矩前馈
    //  #define Test_vir_phi0 // 测试控vir_phi0摆角
    // #define Test_l0       // 控腿长

    Link_t *My_L_Link = My_Chassis->Leg_Unit[L_Leg]->Link;
    Link_t *My_R_Link = My_Chassis->Leg_Unit[R_Leg]->Link;
#ifdef Test_spring_feedforward
    // 氮气弹簧补偿力已经在data_update处计算
    // My_Spring_Former_Input_Cal(My_Chassis);
    My_Chassis->Leg_Unit[R_Leg]->force->Sd_F_Torque = FRONT_SPRING_COMPENSATION_ORDER_CORRECT * My_Chassis->Leg_Unit[R_Leg]->force->T_Spring_Compensation_Front;
    My_Chassis->Leg_Unit[R_Leg]->force->Sd_B_Torque = BACK_SPRING_COMPENSATION_ORDER_CORRECT * My_Chassis->Leg_Unit[R_Leg]->force->T_Spring_Compensation_Back;
#ifdef TEST_LEFT_LEG
    My_Chassis->Leg_Unit[L_Leg]->force->Sd_F_Torque = FRONT_SPRING_COMPENSATION_ORDER_CORRECT * My_Chassis->Leg_Unit[L_Leg]->force->T_Spring_Compensation_Front;
    My_Chassis->Leg_Unit[L_Leg]->force->Sd_B_Torque = BACK_SPRING_COMPENSATION_ORDER_CORRECT * My_Chassis->Leg_Unit[L_Leg]->force->T_Spring_Compensation_Back;
#endif
#endif

#ifdef Test_Leg_Gravity_feedforward
    // 腿部重力扭矩补偿
    Cal_Leg_Gravity_Torque(My_Chassis);
    My_Chassis->Leg_Unit[R_Leg]->force->Tp_target = LEG_GRAVITY_COMPENSATION_ORDER_CORRECT * My_Chassis->Leg_Unit[R_Leg]->force->Leg_Gravity_Torque;
    /*腿部竖直力前馈*/
    // My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target = -(My_R_Link->info->centroid->centriod_coefficient * m_l) * g * cos(My_R_Link->info->angle->vir_phi0);
#ifdef TEST_LEFT_LEG
    My_Chassis->Leg_Unit[L_Leg]->force->Tp_target = LEG_GRAVITY_COMPENSATION_ORDER_CORRECT * My_Chassis->Leg_Unit[L_Leg]->force->Leg_Gravity_Torque;
    // My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target = -(My_L_Link->info->centroid->centriod_coefficient * m_l) * g * cos(My_L_Link->info->angle->vir_phi0);
#endif
#endif

#ifdef Test_vir_phi0
    My_Chassis->target->vir_phi0_r += My_Chassis->rc_input->ch2_now / 660.f * TIME_STEP;
    My_Chassis->target->vir_phi0_r = constrain(My_Chassis->target->vir_phi0_r, angle2rad(-60), angle2rad(60));
#ifdef TEST_LEFT_LEG
    My_Chassis->target->vir_phi0_l += My_Chassis->rc_input->ch2_now / 660.f * TIME_STEP;
    My_Chassis->target->vir_phi0_l = constrain(My_Chassis->target->vir_phi0_l, angle2rad(-60), angle2rad(60));
#endif

    Chassis_Leg_vir_phi0_Cal(My_Chassis); // 内部Tp_vir_phi0赋值给chassis

    My_Chassis->Leg_Unit[R_Leg]->force->Tp_target = My_Chassis->Leg_Unit[R_Leg]->force->Tp_vir_phi0;
#ifdef TEST_LEFT_LEG
    My_Chassis->Leg_Unit[L_Leg]->force->Tp_target = My_Chassis->Leg_Unit[L_Leg]->force->Tp_vir_phi0;
#endif
#endif

#ifdef Test_l0
    My_Chassis->target->leg_length_r += My_Chassis->rc_input->ch1_now / 660.f * TIME_STEP * 0.1;
#ifdef TEST_LEFT_LEG
    My_Chassis->target->leg_length_l += My_Chassis->rc_input->ch1_now / 660.f * TIME_STEP * 0.1;
#endif
    /*腿长控制力计算*/
    Chassis_Leg_Length_Strength_Cal(My_Chassis);
    My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target = My_Chassis->Leg_Unit[R_Leg]->force->F - (My_R_Link->info->centroid->centriod_coefficient * m_l) * g * cos(My_R_Link->info->angle->vir_phi0);
#ifdef TEST_LEFT_LEG
    My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target = My_Chassis->Leg_Unit[L_Leg]->force->F - (My_L_Link->info->centroid->centriod_coefficient * m_l) * g * cos(My_L_Link->info->angle->vir_phi0);
#endif
#endif

    /*建模的Tp方向是顺时针，VMC是逆时针，所以从建模--->VMC要加个负号*/
    My_R_Link->tar_data_update(My_R_Link, My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target, My_Chassis->Leg_Unit[R_Leg]->force->Tp_target);
    /*VMC转换为关节力矩,输出到Link结构体的F_Sd_Output_Torque，B_Sd_Output_Torque*/
    My_R_Link->torque_cal(My_R_Link);
#ifdef TEST_LEFT_LEG
    My_L_Link->tar_data_update(My_L_Link, My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target, My_Chassis->Leg_Unit[L_Leg]->force->Tp_target);
    My_L_Link->torque_cal(My_L_Link);
#endif

/*赋值到chassis目标力矩结构体*/
#ifndef Test_spring_feedforward
    My_Chassis->Leg_Unit[R_Leg]->force->Sd_F_Torque = My_R_Link->info->F_Sd_Output_Torque + FRONT_SPRING_COMPENSATION_ORDER_CORRECT * My_Chassis->Leg_Unit[R_Leg]->force->T_Spring_Compensation_Front;
    My_Chassis->Leg_Unit[R_Leg]->force->Sd_B_Torque = My_R_Link->info->B_Sd_Output_Torque + BACK_SPRING_COMPENSATION_ORDER_CORRECT * My_Chassis->Leg_Unit[R_Leg]->force->T_Spring_Compensation_Back;
#ifdef TEST_LEFT_LEG
    My_Chassis->Leg_Unit[L_Leg]->force->Sd_F_Torque = My_L_Link->info->F_Sd_Output_Torque + FRONT_SPRING_COMPENSATION_ORDER_CORRECT * My_Chassis->Leg_Unit[L_Leg]->force->T_Spring_Compensation_Front;
    My_Chassis->Leg_Unit[L_Leg]->force->Sd_B_Torque = My_L_Link->info->B_Sd_Output_Torque + BACK_SPRING_COMPENSATION_ORDER_CORRECT * My_Chassis->Leg_Unit[L_Leg]->force->T_Spring_Compensation_Back;
#endif
#endif
}
/**
 * @brief  底盘工作模式更新
 * @param  Chassis_t* My_Chassis
 * @retval None
 */
static void Chassis_Status_React(Chassis_t *My_Chassis)
{
    switch (Balance.mode)
    {
    case Sleep_Mode:
        My_Chassis->mode = C_Sleep;
        break;

    case Imu_Mode:
        My_Chassis->mode = C_Follow;
        // 有特殊动作标志位就进特殊模式，否则还是C_Follow
        if (Balance.Flag->KNEE_STRIKE_Flag == 1)
        {
            My_Chassis->mode = C_KNEE_STRIKE;
        }
        else if (Balance.Flag->JUMP_AND_MID_Flag == 1)
        {
            My_Chassis->mode = C_JUMP_AND_MID;
        }
        else if (Balance.Flag->RTS_Flag == 1)
        {
            My_Chassis->mode = C_RTS;
        }
        else if (Balance.Flag->Jumping_Flag == 1)
        {
            My_Chassis->mode = C_Jump;
        }
        break;

    case Mec_Mode:
        My_Chassis->mode = C_Boss;
        if (Balance.Flag->KNEE_STRIKE_Flag == 1)
        {
            My_Chassis->mode = C_KNEE_STRIKE;
        }
        break;
    case Cycle_Mode:
        My_Chassis->mode = C_Cycle;
        break;

    case LEG_TEST_Mode:
        My_Chassis->mode = C_Test;
        break;

    case Rescue_Mode:
    case Init_Mode:
        My_Chassis->mode = C_Rescue;
        break;

    case Manual_Rescue_Mode:
        My_Chassis->mode = C_Manual_Rescue;
        break;

    case SitDown_Mode:
        My_Chassis->mode = C_SitDown;
        break;

    default:
        My_Chassis->mode = C_Sleep;
        break;
    }

    /*-------- GIMBAL_180_Flag上升沿切C_Boss，下降沿恢复C_Follow -------*/
    {
        bool GIMBAL_180_current_flag = Balance.Flag->GIMBAL_180_Flag;
        if (GIMBAL_180_current_flag == true)
        {
            switch (My_Chassis->mode)
            {
            case C_Cycle:
                My_Chassis->mode = C_Cycle;
                break;

            case C_Follow:
            case C_Boss:
                My_Chassis->mode = C_Boss;
                break;

            default:
                break;
            }
        }
    }

    /*---- 超功率模拟底盘断电(感觉比赛也可以开，自己睡觉比底盘断电好) ---*/
    {
#ifdef TEST_POWER_LIMIT
        static uint16_t power_limit_sleep_cnt;
        static uint8_t power_limit_sleep_flag;           // 标志位：锁定sleep状态
        const uint16_t power_limit_sleep_max_cnt = 5000; // 模拟底盘断电时间

        // 缓冲能量低于20且尚未锁定sleep时，启动sleep计时
        if (My_Judge.info->chassis_power_buffer <= 10 && power_limit_sleep_flag == 0)
        {
            power_limit_sleep_flag = 1;
            power_limit_sleep_cnt = 0;
        }

        // sleep锁定状态下，持续sleep直到计时结束
        if (power_limit_sleep_flag == 1)
        {
            My_Chassis->mode = C_Sleep;
            if (power_limit_sleep_cnt < power_limit_sleep_max_cnt)
            {
                power_limit_sleep_cnt++;
            }
            else
            {
                // 5s结束，恢复正常
                power_limit_sleep_flag = 0;
            }
        }
#endif
    }

    /*-------- 坐下测试 -------*/
#ifdef TEST_SITDOWN_MODE
    My_Chassis->mode = C_SitDown;
#endif

    /*-------- 离线保护 -------*/
    {
        if (My_Chassis->state->sd_state == DEV_OFFLINE ||
            My_Chassis->state->wheel_state == DEV_OFFLINE)
        {
            My_Chassis->mode = C_Sleep;
        }
    }
}

/**
 * @brief  底盘阻尼卸力,不含can发送
 * @param  Chassis_t* My_Chassis
 * @retval None
 */
static void Chassis_Damping_Sleep(Chassis_t *My_Chassis)
{
    if (Balance.Flag->Chassis_Online_Flag == false)
    {
        My_Chassis->Sd->group_sleep(My_Chassis->Sd);
        My_Chassis->Wheel->group_sleep(My_Chassis->Wheel);
    }
    else
    {
        if (My_Chassis->damping_delay_cnt < DAMPING_DELAY_MAX_CNT)
        {
            Chassis_Stop_Damping(My_Chassis);

            My_Chassis->damping_delay_cnt++;
        }
        else
        {
            My_Chassis->Sd->group_sleep(My_Chassis->Sd);
            My_Chassis->Wheel->group_sleep(My_Chassis->Wheel);
        }
    }
}

/**
 * @brief  底盘总控制
 * @param  None
 * @retval None
 */
static void Chassis_Ctrl(Chassis_t *My_Chassis)
{
    Chassis_Clean_Process(My_Chassis);
    switch (My_Chassis->mode)
    {
    case C_Sleep:
        Chassis_Takeoff_Detect(My_Chassis);
        Chassis_Damping_Sleep(My_Chassis);
        /*离线处理*/
        Chassis_Offline_Process(My_Chassis);
        break;

    case C_Rescue:
        Rescue_State_Process(My_Chassis);
        Chassis_Torque_Cal(My_Chassis);
        Chassis_Set_Torque(My_Chassis);
        break;

    case C_Manual_Rescue:
        Chassis_Manual_Rescue_Process(My_Chassis);
        Chassis_Torque_Cal(My_Chassis);
        Chassis_Set_Torque(My_Chassis);
        break;

    case C_Follow:
        Chassis_Takeoff_Detect(My_Chassis);
        Chassis_Torque_Cal(My_Chassis);
        Chassis_Set_Torque(My_Chassis);
        break;

    case C_Boss:
        Chassis_Takeoff_Detect(My_Chassis);
        Chassis_Torque_Cal(My_Chassis);
        Chassis_Set_Torque(My_Chassis);
        break;

    case C_Cycle:
        Cycle_Target_Process(My_Chassis);
        Chassis_Torque_Cal(My_Chassis);
        Chassis_Set_Torque(My_Chassis);
        break;

    case C_Test:
        Chassis_Takeoff_Detect(My_Chassis);
        Test_Basic_Control(My_Chassis);
        // Balance.Flag->Leg_length_ctrl_Flag = true;
        // Chassis_Torque_Cal(My_Chassis);
        Chassis_Set_Torque(My_Chassis);
        break;

    case C_SitDown:
        My_Chassis->Leg_Unit[R_Leg]->force->Sd_F_Torque = 0;
        My_Chassis->Leg_Unit[R_Leg]->force->Sd_B_Torque = 0;
        My_Chassis->Leg_Unit[L_Leg]->force->Sd_F_Torque = 0;
        My_Chassis->Leg_Unit[L_Leg]->force->Sd_B_Torque = 0;
        My_Chassis->Leg_Unit[R_Leg]->force->Tw_target = 0;
        My_Chassis->Leg_Unit[L_Leg]->force->Tw_target = 0;
        Chassis_Set_Torque(My_Chassis);
        break;

    case C_Jump:
        Chassis_Takeoff_Detect(My_Chassis);
        Jump_Target_Process(My_Chassis);
        Chassis_Torque_Cal(My_Chassis);
        Chassis_Set_Torque(My_Chassis);
        break;

    case C_KNEE_STRIKE:
        Chassis_Takeoff_Detect(My_Chassis);
        KNEE_STRIKE_Target_Process(My_Chassis);
        Chassis_Torque_Cal(My_Chassis);
        Chassis_Set_Torque(My_Chassis);
        break;

    case C_JUMP_THEN_KNEE_STRIKE:
        Chassis_Takeoff_Detect(My_Chassis);
        JUMP_THEN_KNEE_STRIKE_Target_Process(My_Chassis);
        Chassis_Torque_Cal(My_Chassis);
        Chassis_Set_Torque(My_Chassis);
        break;

    case C_JUMP_AND_MID:
        Chassis_Takeoff_Detect(My_Chassis);
        JUMP_AND_MID_Target_Process(My_Chassis);
        Chassis_Torque_Cal(My_Chassis);
        Chassis_Set_Torque(My_Chassis);
        break;

    case C_RTS:
        Retract_down_Two_Step_Process(My_Chassis);
        Chassis_Torque_Cal(My_Chassis);
        Chassis_Set_Torque(My_Chassis);
        break;

    default:
        My_Chassis->Sd->group_sleep(My_Chassis->Sd); // 含发送
        My_Chassis->Wheel->group_sleep(My_Chassis->Wheel);
        Chassis_Offline_Process(My_Chassis);
        break;
    }
}
/**
 * @brief  清标志位等处理
 * @param  Chassis_t* My_Chassis, 底盘
 * @retval None
 */
static void Chassis_Clean_Process(Chassis_t *My_Chassis)
{
    if (My_Chassis->mode != C_KNEE_STRIKE)
    {
        My_Chassis->knee_strike_info->KNEE_STRIKE_step = Knee_IDLE;
        My_Chassis->knee_strike_info->Idle_tick = 0;
        My_Chassis->knee_strike_info->Stand_High_tick = 0;
        My_Chassis->knee_strike_info->RETRACT_tick = 0;
        My_Chassis->knee_strike_info->knee_strike_exit_tick = 0;
        Balance.Flag->KNEE_STRIKE_Flag = false;

        // 恢复被可能修改的腿长pid参数上限 (如果IDLE_length_kp等非空则可以考虑恢复，但最好只恢复固定值或默认值)
        // 保证在中断等状态下出问题能恢复原先kp
        if (My_Chassis->last_mode == C_KNEE_STRIKE)
        {
            My_Chassis->chassis_PID->length_cal[R_Leg]->kp = My_Chassis->pid_init_parament->l0_length_kp;
            My_Chassis->chassis_PID->length_cal[L_Leg]->kp = My_Chassis->pid_init_parament->l0_length_kp;
            My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max = My_Chassis->pid_init_parament->l0_length_speed_outmax;
            My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out_max = My_Chassis->pid_init_parament->l0_length_speed_outmax;
        }
    }

    // 进入Follow/Boss/Cycle模式时，如果上个模式不是这三个，也复位腿长PID参数
    if ((My_Chassis->mode == C_Follow || My_Chassis->mode == C_Boss || My_Chassis->mode == C_Cycle) &&
        (My_Chassis->last_mode != C_Follow && My_Chassis->last_mode != C_Boss && My_Chassis->last_mode != C_Cycle))
    {
        My_Chassis->chassis_PID->length_cal[R_Leg]->kp = My_Chassis->pid_init_parament->l0_length_kp;
        My_Chassis->chassis_PID->length_cal[L_Leg]->kp = My_Chassis->pid_init_parament->l0_length_kp;
        My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max = My_Chassis->pid_init_parament->l0_length_speed_outmax;
        My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out_max = My_Chassis->pid_init_parament->l0_length_speed_outmax;
    }

    if (My_Chassis->mode != C_Rescue)
    {
        My_Chassis->rescue_info->rescue_state_mac = Reset;

        Balance.Flag->Rescue_Flag = false;
        // 如果自救被打断，恢复默认摆角pid参数
        My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg] = &My_Link_vir_phi0_d1_Pid[R_Leg];
        My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg] = &My_Link_vir_phi0_d1_Pid[L_Leg];
    }
    if (My_Chassis->mode != C_Manual_Rescue &&
        My_Chassis->mode != C_Rescue)
    {
        My_Chassis->Leg_Unit[R_Leg]->force->Tp_rescue = 0;
        My_Chassis->Leg_Unit[L_Leg]->force->Tp_rescue = 0;
    }

    // 退出手动自救模式时清零F_Manual_Rescue
    if (My_Chassis->mode != C_Manual_Rescue)
    {
        My_Chassis->Leg_Unit[R_Leg]->force->F_Manual_Rescue = 0;
        My_Chassis->Leg_Unit[L_Leg]->force->F_Manual_Rescue = 0;
    }

    if (My_Chassis->mode != C_JUMP_THEN_KNEE_STRIKE)
    {
        My_Chassis->knee_strike_info->JUMP_THEN_KNEE_STRIKE_Step = JK_IDLE;
        My_Chassis->jump_info->jump_step = J_IDLE;
    }

    if (My_Chassis->last_mode == C_Jump && My_Chassis->mode != C_Jump)
    {
        My_Chassis->jump_info->jump_step = J_IDLE;
        Balance.Flag->Jumping_Flag = false;
    }

    if (My_Chassis->mode != C_JUMP_AND_MID)
    {
        My_Chassis->jump_and_mid_info->JUMP_AND_MID_step = JAM_IDLE;
        My_Chassis->jump_and_mid_info->JUMP_AND_MID_tick = 0;
        Balance.Flag->JUMP_AND_MID_Flag = false;
        // Middle_Flag不由Clean_Process清除，仅由以下场景清除：
        // 1. Reset_to_Normal（Balance.c中）
        // 2. SW_MID_LENGTH Toggle为关闭状态
        // 3. C_JUMP_AND_MID状态机内部超时/完成时（在状态机逻辑中处理）
    }

    if (My_Chassis->last_mode == C_RTS && My_Chassis->mode != C_RTS)
    {
        My_Chassis->rts_info->var.rts_step = RTS_IDLE;
        My_Chassis->rts_info->var.rts_tick = 0;
        My_Chassis->rts_info->var.front_swing_tick = 0;
        My_Chassis->rts_info->var.landing_tick = 0;
        My_Chassis->target->leg_length_l = TAR_LEG_LENGTH_INITIAL;
        My_Chassis->target->leg_length_r = TAR_LEG_LENGTH_INITIAL;
        Balance.Flag->RTS_Flag = false;
    }

    if (My_Chassis->mode != C_Sleep)
    {
        My_Chassis->damping_delay_cnt = 0;
    }
}

/**
 * @brief 跳+中腿长下台阶处理
 * 流程: 跳跃跳上二级台阶 → 落地后切换中腿长下台阶
 */
static void JUMP_AND_MID_Target_Process(Chassis_t *My_Chassis)
{
    Chassis_Jump_And_Mid_t *jump_and_mid_info = My_Chassis->jump_and_mid_info;
    Link_t *R_Link = My_Chassis->Leg_Unit[R_Leg]->Link;
    Link_t *L_Link = My_Chassis->Leg_Unit[L_Leg]->Link;
    jump_and_mid_info->r_offground = My_Chassis->Leg_Unit[R_Leg]->off_ground;
    jump_and_mid_info->l_offground = My_Chassis->Leg_Unit[L_Leg]->off_ground;
    jump_and_mid_info->l0_average = 0.5f * (R_Link->info->length->l0 + L_Link->info->length->l0);

    jump_and_mid_info->JUMP_AND_MID_tick++; // 整体计时，10s超时保护

    // 10s超时保护
    if (jump_and_mid_info->JUMP_AND_MID_tick >= jump_and_mid_info->Max_JUMP_AND_MID_tick)
    {
        jump_and_mid_info->JUMP_AND_MID_step = JAM_IDLE;
        My_Chassis->target->leg_length_l = TAR_LEG_LENGTH_INITIAL;
        My_Chassis->target->leg_length_r = TAR_LEG_LENGTH_INITIAL;
        My_Chassis->chassis_PID->length_cal[R_Leg]->kp = jump_and_mid_info->IDLE_length_kp;
        My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp = jump_and_mid_info->IDLE_length_speed_kp;
        My_Chassis->chassis_PID->length_cal[L_Leg]->kp = jump_and_mid_info->IDLE_length_kp;
        My_Chassis->chassis_PID->length_speed_cal[L_Leg]->kp = jump_and_mid_info->IDLE_length_speed_kp;
        My_Chassis->chassis_PID->length_cal[R_Leg]->out_max = jump_and_mid_info->IDLE_length_outmax;
        My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max = jump_and_mid_info->IDLE_length_speed_outmax;
        My_Chassis->chassis_PID->length_cal[L_Leg]->out_max = jump_and_mid_info->IDLE_length_outmax;
        My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out_max = jump_and_mid_info->IDLE_length_speed_outmax;
        Balance.Flag->JUMP_AND_MID_Flag = false;
        jump_and_mid_info->JUMP_AND_MID_tick = 0;
        My_Chassis->mode = C_Follow; // 超时强制退出到C_Follow
        return;
    }

    switch (jump_and_mid_info->JUMP_AND_MID_step)
    {
    case JAM_IDLE:
        // 保存当前PID参数（独立存入JUMP_AND_MID信息体）
        jump_and_mid_info->IDLE_length_kp = My_Chassis->chassis_PID->length_cal[R_Leg]->kp;
        jump_and_mid_info->IDLE_length_speed_kp = My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp;
        jump_and_mid_info->IDLE_length_outmax = My_Chassis->chassis_PID->length_cal[R_Leg]->out_max;
        jump_and_mid_info->IDLE_length_speed_outmax = My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max;

        // 跳+中腿长动作期使用固定输出限幅
        My_Chassis->chassis_PID->length_cal[R_Leg]->out_max = 100.f;
        My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max = 500.f;
        My_Chassis->chassis_PID->length_cal[L_Leg]->out_max = 100.f;
        My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out_max = 500.f;

        // 进入流程前清零所有分段计时
        jump_and_mid_info->COMPRESS_tick = 0;
        jump_and_mid_info->EXTEND_tick = 0;
        jump_and_mid_info->RETRACT_tick = 0;
        jump_and_mid_info->PRE_LANDING_tick = 0;
        jump_and_mid_info->LANDING_tick = 0;
        jump_and_mid_info->JUMP_AND_MID_tick = 0; // 重新计时
        jump_and_mid_info->JUMP_AND_MID_step = JAM_COMPRESS;
        break;

    case JAM_COMPRESS: // 压缩腿
        // 压缩阶段：腿长收至压缩目标，并以容差或超时进入伸腿
        jump_and_mid_info->COMPRESS_tick++;
        My_Chassis->target->leg_length_l = jump_and_mid_info->JAM_COMPRESS_target_length;
        My_Chassis->target->leg_length_r = jump_and_mid_info->JAM_COMPRESS_target_length;
        My_Chassis->chassis_PID->length_cal[R_Leg]->kp = jump_and_mid_info->COMPRESS_length_kp;
        My_Chassis->chassis_PID->length_cal[L_Leg]->kp = jump_and_mid_info->COMPRESS_length_kp;

        if ((jump_and_mid_info->l0_average <=
             jump_and_mid_info->JAM_COMPRESS_target_length + jump_and_mid_info->Minimum_l0_range) ||
            (jump_and_mid_info->COMPRESS_tick >= jump_and_mid_info->Max_COMPRESS_tick))
        {
            jump_and_mid_info->JUMP_AND_MID_step = JAM_EXTEND;
        }
        break;

    case JAM_EXTEND: // 跳
        // 伸腿阶段：快速伸腿，达到目标附近或超时后进入收腿滞空
        jump_and_mid_info->EXTEND_tick++;
        My_Chassis->target->leg_length_l = jump_and_mid_info->JAM_EXTEND_target_length;
        My_Chassis->target->leg_length_r = jump_and_mid_info->JAM_EXTEND_target_length;
        My_Chassis->chassis_PID->length_cal[R_Leg]->kp = jump_and_mid_info->EXTEND_length_kp;
        My_Chassis->chassis_PID->length_cal[L_Leg]->kp = jump_and_mid_info->EXTEND_length_kp;

        if ((jump_and_mid_info->l0_average >=
             jump_and_mid_info->JAM_EXTEND_target_length - jump_and_mid_info->Max_l0_range) ||
            (jump_and_mid_info->EXTEND_tick >= jump_and_mid_info->Max_EXTEND_tick))
        {
            jump_and_mid_info->JUMP_AND_MID_step = JAM_RETRACT;
        }
        break;

    case JAM_RETRACT: // 收腿滞空
        // 收腿滞空：收回腿长到最小附近，按时间转入预落地
        jump_and_mid_info->RETRACT_tick++;
        My_Chassis->target->leg_length_l = jump_and_mid_info->JAM_RETRACT_target_length;
        My_Chassis->target->leg_length_r = jump_and_mid_info->JAM_RETRACT_target_length;
        My_Chassis->chassis_PID->length_cal[R_Leg]->kp = jump_and_mid_info->RETRACT_length_kp;
        My_Chassis->chassis_PID->length_cal[L_Leg]->kp = jump_and_mid_info->RETRACT_length_kp;

        if (jump_and_mid_info->RETRACT_tick >= jump_and_mid_info->Max_RETRACT_tick)
        {
            jump_and_mid_info->JUMP_AND_MID_step = JAM_PRE_LANDING;
        }
        break;

    case JAM_PRE_LANDING: // 伸腿准备缓冲
        // 预落地阶段：将腿长伸到预落地目标，检测触地或超时进入缓冲
        jump_and_mid_info->PRE_LANDING_tick++;
        My_Chassis->target->leg_length_l = jump_and_mid_info->JAM_PRE_LANDING_target_length;
        My_Chassis->target->leg_length_r = jump_and_mid_info->JAM_PRE_LANDING_target_length;
        My_Chassis->chassis_PID->length_cal[R_Leg]->kp = jump_and_mid_info->PRE_LANDING_length_kp;
        My_Chassis->chassis_PID->length_cal[L_Leg]->kp = jump_and_mid_info->PRE_LANDING_length_kp;

        // 两腿同时在地面(不离地)计时
        static uint16_t both_onground_tick = 0;
        if (jump_and_mid_info->r_offground == false && jump_and_mid_info->l_offground == false)
        {
            both_onground_tick++;
        }
        else
        {
            both_onground_tick = 0;
        }

        if (both_onground_tick >= jump_and_mid_info->Max_Both_Onground_tick ||
            jump_and_mid_info->PRE_LANDING_tick >= jump_and_mid_info->Max_PRE_LANDING_tick)
        {
            jump_and_mid_info->JUMP_AND_MID_step = JAM_LANDING;
        }
        break;

    case JAM_LANDING: // 缓冲
        // 缓冲阶段：落地后回中腿长，达到容差或超时即退出特殊动作
        jump_and_mid_info->LANDING_tick++;
        My_Chassis->target->leg_length_l = jump_and_mid_info->JAM_LANDING_target_length;
        My_Chassis->target->leg_length_r = jump_and_mid_info->JAM_LANDING_target_length;
        My_Chassis->chassis_PID->length_cal[R_Leg]->kp = jump_and_mid_info->LANDING_length_kp;
        My_Chassis->chassis_PID->length_cal[L_Leg]->kp = jump_and_mid_info->LANDING_length_kp;

        if (jump_and_mid_info->LANDING_tick >= jump_and_mid_info->Max_LANDING_tick ||
            (my_abs(jump_and_mid_info->l0_average - jump_and_mid_info->JAM_LANDING_target_length) <= jump_and_mid_info->Landing_l0_range))
        {
            // 落地完成 → 恢复PID参数、退出C_Jump进入C_Follow、触发Middle_Flag
            My_Chassis->chassis_PID->length_cal[R_Leg]->kp = jump_and_mid_info->IDLE_length_kp;
            My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp = jump_and_mid_info->IDLE_length_speed_kp;
            My_Chassis->chassis_PID->length_cal[L_Leg]->kp = jump_and_mid_info->IDLE_length_kp;
            My_Chassis->chassis_PID->length_speed_cal[L_Leg]->kp = jump_and_mid_info->IDLE_length_speed_kp;
            My_Chassis->chassis_PID->length_cal[R_Leg]->out_max = jump_and_mid_info->IDLE_length_outmax;
            My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max = jump_and_mid_info->IDLE_length_speed_outmax;
            My_Chassis->chassis_PID->length_cal[L_Leg]->out_max = jump_and_mid_info->IDLE_length_outmax;
            My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out_max = jump_and_mid_info->IDLE_length_speed_outmax;

            // 落地后立即触发中腿长模式（使用默认PID参数）
            Balance.Flag->Middle_Flag = true;
            // 退出C_Jump进入C_Follow
            Balance.Flag->Jumping_Flag = false;
            jump_and_mid_info->JUMP_AND_MID_step = JAM_IDLE;
            My_Chassis->mode = C_Follow;
            jump_and_mid_info->JUMP_AND_MID_tick = 0;
            Balance.Flag->JUMP_AND_MID_Flag = false;
        }
        break;

    default:
        break;
    }
}

/**
 * @brief  收腿下二级台阶处理
 * @param  Chassis_t* My_Chassis, 底盘
 * @retval None
 */
static void Retract_down_Two_Step_Process(Chassis_t *My_Chassis)
{
    Chassis_Retract_to_Second_Step_Info_t *rts_info = My_Chassis->rts_info;
    RTS_Config_t *config = &rts_info->config;
    RTS_Var_t *var = &rts_info->var;
    Link_t *R_Link = My_Chassis->Leg_Unit[R_Leg]->Link;
    Link_t *L_Link = My_Chassis->Leg_Unit[L_Leg]->Link;

    float encoder_speed_l = my_abs(My_Chassis->Wheel->motor[R_Leg]->rx_info->encoder_speed);
    float encoder_speed_r = my_abs(My_Chassis->Wheel->motor[L_Leg]->rx_info->encoder_speed);
    float theta_average = 0.5f * (My_Chassis->Leg_Unit[R_Leg]->Straight->info->thetal + My_Chassis->Leg_Unit[L_Leg]->Straight->info->thetal);

    var->rts_tick++;

    // ========== 超时保护 ==========
    if (var->rts_tick >= config->front_swing_timeout && var->rts_step == RTS_Front_Swing)
    {
        My_Chassis->mode = C_Follow;
        Balance.Flag->RTS_Flag = false;
        var->rts_step = RTS_IDLE;
        var->rts_tick = 0;
        My_Chassis->target->leg_length_l = var->initial_leg_length;
        My_Chassis->target->leg_length_r = var->initial_leg_length;
        return;
    }

    switch (var->rts_step)
    {
    case RTS_IDLE: {
        var->initial_theta_target = My_Chassis->target->thetal_l;
        var->initial_leg_length = My_Chassis->target->leg_length_l;
        var->front_swing_tick = 0;
        var->landing_tick = 0;
        var->rising_tick = 0;
        var->rts_tick = 0;
        var->rts_step = RTS_Rising; // 改为抬腿递增阶段
        break;
    }

    case RTS_Rising: {
        var->rising_tick++;
        // 抬腿阶段改为递增式抬腿，避免目标值突变
        My_Chassis->target->leg_length_l += 0.001f * 0.25f;
        My_Chassis->target->leg_length_r += 0.001f * 0.25f;
        My_Chassis->target->leg_length_l = constrain(My_Chassis->target->leg_length_l, MIN_LEG_LENGTH, MAX_LEG_LENGTH);
        My_Chassis->target->leg_length_r = constrain(My_Chassis->target->leg_length_r, MIN_LEG_LENGTH, MAX_LEG_LENGTH);

        // 事件：腿长达到前摆目标后进入前摆阶段
        if (My_Chassis->target->leg_length_l >= config->front_swing_leg_length)
        {
            var->rts_step = RTS_Front_Swing;
        }
        break;
    }

    case RTS_Front_Swing: {
        var->front_swing_tick++;
        // float theta_target = var->initial_theta_target - config->theta_offset;
        // My_Chassis->target->thetal_l = theta_target;
        // My_Chassis->target->thetal_r = theta_target;

        float theta_threshold;
        bool condition1 = (encoder_speed_l < config->encoder_speed_threshold) &&
                          (encoder_speed_r < config->encoder_speed_threshold);
        bool condition2;
        float gimbal_yaw = gimbal.base_info.yaw_motor_angle;
        if (fabsf(gimbal_yaw) < PI / 2.f)
        {
            theta_threshold = var->initial_theta_target - config->theta_threshold_offset / Degree_to_rad;
            condition2 = (theta_average > theta_threshold);
        }
        else
        {
            theta_threshold = var->initial_theta_target + config->theta_threshold_offset / Degree_to_rad;
            condition2 = (theta_average < theta_threshold);
        }
        bool condition3 = (var->front_swing_tick >= config->min_front_swing_time);
        if (condition1 && condition2 && condition3)
        {
            My_Chassis->target->thetal_l = var->initial_theta_target;
            My_Chassis->target->thetal_r = var->initial_theta_target;
            My_Chassis->target->leg_length_l = config->landing_leg_length;
            My_Chassis->target->leg_length_r = config->landing_leg_length;
            var->landing_tick = 0;
            var->front_swing_tick = 0;
            var->rts_step = RTS_Landing;
        }
        break;
    }

    case RTS_Landing: {
        var->landing_tick++;
        if (var->landing_tick >= config->landing_reset_time)
        {
            My_Chassis->target->leg_length_l = var->initial_leg_length;
            My_Chassis->target->leg_length_r = var->initial_leg_length;
            Balance.Flag->RTS_Flag = false;
            var->rts_step = RTS_IDLE;
            var->rts_tick = 0;
            My_Chassis->mode = C_Follow;
        }
        break;
    }

    default:
        break;
    }
}

/**
 * @brief  跳跃过程处理
 * @param  Chassis_t* My_Chassis, 底盘
 * @retval None
 */
static void Jump_Target_Process(Chassis_t *My_Chassis)
{
    // #define TIME_ONLY
    Chassis_Jump_t *jump_info = My_Chassis->jump_info;
    Link_t *R_Link = My_Chassis->Leg_Unit[R_Leg]->Link;
    Link_t *L_Link = My_Chassis->Leg_Unit[L_Leg]->Link;
    jump_info->r_offground = My_Chassis->Leg_Unit[R_Leg]->off_ground;
    jump_info->l_offground = My_Chassis->Leg_Unit[L_Leg]->off_ground;

    jump_info->l0_average = 0.5f * (R_Link->info->length->l0 + L_Link->info->length->l0);

    switch (jump_info->jump_step)
    {
    case J_IDLE:
        // 动作
        jump_info->IDLE_length_kp = My_Chassis->chassis_PID->length_cal[R_Leg]->kp;
        jump_info->IDLE_length_speed_kp = My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp;
        jump_info->IDLE_length_outmax = My_Chassis->chassis_PID->length_cal[R_Leg]->out_max;
        jump_info->IDLE_length_speed_outmax = My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max;

        jump_info->COMPRESS_tick = 0;
        jump_info->EXTEND_tick = 0;
        jump_info->RETRACT_tick = 0;
        jump_info->PRE_LANDING_tick = 0;
        jump_info->LANDING_tick = 0;
        // 事件
        jump_info->jump_step = J_COMPRESS;
        break;

    case J_COMPRESS: // 压缩腿
        // 动作
        jump_info->COMPRESS_tick++;
        My_Chassis->target->leg_length_l = MIN_LEG_LENGTH;
        My_Chassis->target->leg_length_r = MIN_LEG_LENGTH;
        My_Chassis->chassis_PID->length_cal[R_Leg]->kp = jump_info->COMPRESS_length_kp;
        My_Chassis->chassis_PID->length_cal[L_Leg]->kp = jump_info->COMPRESS_length_kp;

        if ((jump_info->l0_average - (jump_info->J_COMPRESS_length_target + jump_info->Minimum_l0_range) < 0) ||
            jump_info->COMPRESS_tick >= jump_info->Max_COMPRESS_tick)
        {
            jump_info->jump_step = J_EXTEND;
        }

        break;

    case J_EXTEND: // 跳
        // 动作
        jump_info->EXTEND_tick++;
        My_Chassis->target->leg_length_l = MAX_LEG_LENGTH - jump_info->Max_l0_range;
        My_Chassis->target->leg_length_r = MAX_LEG_LENGTH - jump_info->Max_l0_range;
        My_Chassis->chassis_PID->length_cal[R_Leg]->kp = jump_info->EXTEND_length_kp;
        My_Chassis->chassis_PID->length_cal[L_Leg]->kp = jump_info->EXTEND_length_kp;
        // 事件

        if (((jump_info->r_offground == true) && (jump_info->l_offground == true)) || jump_info->EXTEND_tick >= jump_info->Max_EXTEND_tick)

            jump_info->jump_step = J_RETRACT;
        break;

    case J_RETRACT: // 收腿滞空
        // 动作
        jump_info->RETRACT_tick++;
        My_Chassis->target->leg_length_l = MIN_LEG_LENGTH + jump_info->Minimum_l0_range;
        My_Chassis->target->leg_length_r = MIN_LEG_LENGTH + jump_info->Minimum_l0_range;
        My_Chassis->chassis_PID->length_cal[R_Leg]->kp = jump_info->RETRACT_length_kp;
        My_Chassis->chassis_PID->length_cal[L_Leg]->kp = jump_info->RETRACT_length_kp;

        if ((jump_info->l0_average - (jump_info->J_RETRACT_length_target + jump_info->Minimum_l0_range) < 0) ||
            jump_info->RETRACT_tick >= jump_info->Max_RETRACT_tick)
        {
            jump_info->jump_step = J_PRE_LANDING;
        }

        break;

    case J_PRE_LANDING: // 伸腿准备缓冲
        // 动作
        jump_info->PRE_LANDING_tick++;
        My_Chassis->target->leg_length_l = MAX_LEG_LENGTH - jump_info->Max_l0_range;
        My_Chassis->target->leg_length_r = MAX_LEG_LENGTH - jump_info->Max_l0_range;
        My_Chassis->chassis_PID->length_cal[R_Leg]->kp = jump_info->PRE_LANDING_length_kp;
        My_Chassis->chassis_PID->length_cal[L_Leg]->kp = jump_info->PRE_LANDING_length_kp;
        // 事件
        if (((jump_info->r_offground == false) && (jump_info->l_offground == false)) ||
            jump_info->PRE_LANDING_tick >= jump_info->Max_PRE_LANDING_tick)
        {
            jump_info->jump_step = J_LANDING;
        }
        break;

    case J_LANDING: // 缓冲
        // 动作
        jump_info->LANDING_tick++;
        My_Chassis->target->leg_length_l = TAR_LEG_LENGTH_INITIAL;
        My_Chassis->target->leg_length_r = TAR_LEG_LENGTH_INITIAL;
        My_Chassis->chassis_PID->length_cal[R_Leg]->kp = jump_info->LANDING_length_kp;
        My_Chassis->chassis_PID->length_cal[L_Leg]->kp = jump_info->LANDING_length_kp;
        My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp = jump_info->LANDING_length_speed_kp;
        My_Chassis->chassis_PID->length_speed_cal[L_Leg]->kp = jump_info->LANDING_length_speed_kp;
        // 事件
        if (((jump_info->r_offground == true) && (jump_info->l_offground == true)) || jump_info->LANDING_tick >= jump_info->Max_LANDING_tick)
        {
            Balance.Flag->Jumping_Flag = false;
            jump_info->jump_step = J_IDLE;
            // My_Chassis->target->leg_length_r = TAR_LEG_LENGTH_INITIAL;
            // My_Chassis->target->leg_length_l = TAR_LEG_LENGTH_INITIAL;
            My_Chassis->chassis_PID->length_cal[R_Leg]->kp = jump_info->IDLE_length_kp;
            My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp = jump_info->IDLE_length_speed_kp;
            My_Chassis->chassis_PID->length_cal[L_Leg]->kp = jump_info->IDLE_length_kp;
            My_Chassis->chassis_PID->length_speed_cal[L_Leg]->kp = jump_info->IDLE_length_speed_kp;
            My_Chassis->chassis_PID->length_cal[R_Leg]->out_max = jump_info->IDLE_length_outmax;
            My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max = jump_info->IDLE_length_speed_outmax;
            My_Chassis->chassis_PID->length_cal[L_Leg]->out_max = jump_info->IDLE_length_outmax;
            My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out_max = jump_info->IDLE_length_speed_outmax;
        }
        break;

    default:
        break;
    }
}

/**
 * @brief  撞膝上台阶
 * @param  Chassis_t* My_Chassis, 底盘
 * @author 2026 RobotPilots LYQ
 * @retval None
 */
static void KNEE_STRIKE_Target_Process(Chassis_t *My_Chassis)
{

    Chassis_Knee_Strike_t *knee_strike_info = My_Chassis->knee_strike_info;
    Link_t *R_Link = My_Chassis->Leg_Unit[R_Leg]->Link;
    Link_t *L_Link = My_Chassis->Leg_Unit[L_Leg]->Link;
    knee_strike_info->thetal_average = Rad2Angle * fabsf(0.5f * (My_Chassis->Leg_Unit[R_Leg]->Straight->info->thetal + My_Chassis->Leg_Unit[L_Leg]->Straight->info->thetal));

    knee_strike_info->l0_average = 0.5f * (R_Link->info->length->l0 + L_Link->info->length->l0);

    switch (knee_strike_info->KNEE_STRIKE_step)
    {
    case Knee_IDLE:

        // 动作
        knee_strike_info->IDLE_length_kp = My_Chassis->chassis_PID->length_cal[R_Leg]->kp; // 保存
        knee_strike_info->Stand_High_tick = 0;
        // 事件
        knee_strike_info->KNEE_STRIKE_step = Knee_Stand_High;

        break;

    case Knee_Stand_High: // 立着
        // 动作
        knee_strike_info->Stand_High_tick++;
        // 立高阶段改为递增式抬腿，避免目标值突变
        My_Chassis->target->leg_length_l += 0.001f * 0.25f;
        My_Chassis->target->leg_length_r += 0.001f * 0.25f;
        My_Chassis->target->leg_length_l = constrain(My_Chassis->target->leg_length_l, MIN_LEG_LENGTH, MAX_LEG_LENGTH);
        My_Chassis->target->leg_length_r = constrain(My_Chassis->target->leg_length_r, MIN_LEG_LENGTH, MAX_LEG_LENGTH);

        // 事件
        if (knee_strike_info->thetal_average >= knee_strike_info->thetal_threshold)
        {
            // 两腿thetal差值超过阈值时不触发收腿，避免踩弹丸时腿部剧烈摆动导致收腿误判
            float thetal_diff = fabsf(My_Chassis->Leg_Unit[R_Leg]->Straight->info->thetal - My_Chassis->Leg_Unit[L_Leg]->Straight->info->thetal) * Rad2Angle;
            if (thetal_diff <= knee_strike_info->thetal_diff_max)
            {
                knee_strike_info->KNEE_STRIKE_step = Knee_RETRACT;
                knee_strike_info->RETRACT_tick = 0;
            }
        }
        if (knee_strike_info->Stand_High_tick >= knee_strike_info->Max_Stand_High_tick)
        {
            knee_strike_info->KNEE_STRIKE_step = Knee_IDLE;
            // 外部会检测KNEE_STRIKE_Flag下降沿来自动缓慢降腿长，所以这里不直接设置目标腿长为初始值
            //  My_Chassis->target->leg_length_l = TAR_LEG_LENGTH_INITIAL;
            //  My_Chassis->target->leg_length_r = TAR_LEG_LENGTH_INITIAL;
            My_Chassis->chassis_PID->length_cal[R_Leg]->kp = knee_strike_info->IDLE_length_kp;
            My_Chassis->chassis_PID->length_cal[L_Leg]->kp = knee_strike_info->IDLE_length_kp;
            Balance.Flag->KNEE_STRIKE_Flag = false;
        }

        break;

    case Knee_RETRACT: // 收腿
        // 动作
        knee_strike_info->RETRACT_tick++;
        My_Chassis->target->leg_length_l = MIN_LEG_LENGTH;
        My_Chassis->target->leg_length_r = MIN_LEG_LENGTH;
        My_Chassis->chassis_PID->length_cal[R_Leg]->kp = knee_strike_info->RETRACT_length_kp;
        My_Chassis->chassis_PID->length_cal[L_Leg]->kp = knee_strike_info->RETRACT_length_kp;
        My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max = 500.f;
        My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out_max = 500.f;
        My_Chassis->target->s = My_Chassis->Leg_Unit[R_Leg]->Straight->info->s;
        // 事件
        if (knee_strike_info->RETRACT_tick >= knee_strike_info->Max_RETRACT_tick || my_abs(knee_strike_info->l0_average - My_Chassis->target->leg_length_l) <= knee_strike_info->Minimum_l0_range)
        {
            knee_strike_info->RETRACT_tick = 0;
            knee_strike_info->Idle_tick = 0;
            My_Chassis->chassis_PID->length_cal[R_Leg]->kp = knee_strike_info->IDLE_length_kp;
            My_Chassis->chassis_PID->length_cal[L_Leg]->kp = knee_strike_info->IDLE_length_kp;
            My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max = 200.f;
            My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out_max = 200.f;
            My_Chassis->target->leg_length_l = TAR_LEG_LENGTH_INITIAL;
            My_Chassis->target->leg_length_r = TAR_LEG_LENGTH_INITIAL;
            knee_strike_info->KNEE_STRIKE_step = Knee_IDLE;
            Balance.Flag->KNEE_STRIKE_Flag = false;
        }
        break;

    default:
        break;
    }
}

/**
 * @brief  跳跃撞膝上台阶
 * @param  Chassis_t* My_Chassis, 底盘
 * @retval None
 */
static void JUMP_THEN_KNEE_STRIKE_Target_Process(Chassis_t *My_Chassis)
{
    Chassis_Knee_Strike_t *knee_strike_info = My_Chassis->knee_strike_info;
    knee_strike_info->thetal_average = Rad2Angle * fabsf(0.5f * (My_Chassis->Leg_Unit[R_Leg]->Straight->info->thetal + My_Chassis->Leg_Unit[L_Leg]->Straight->info->thetal));
    Chassis_Jump_t *jump_info = My_Chassis->jump_info;
    Link_t *R_Link = My_Chassis->Leg_Unit[R_Leg]->Link;
    Link_t *L_Link = My_Chassis->Leg_Unit[L_Leg]->Link;
    jump_info->r_offground = My_Chassis->Leg_Unit[R_Leg]->off_ground;
    jump_info->l_offground = My_Chassis->Leg_Unit[L_Leg]->off_ground;

    jump_info->l0_average = 0.5f * (R_Link->info->length->l0 + L_Link->info->length->l0);
    knee_strike_info->l0_average = 0.5f * (R_Link->info->length->l0 + L_Link->info->length->l0);

    switch (knee_strike_info->JUMP_THEN_KNEE_STRIKE_Step)
    {
    case JK_IDLE:
        // 动作
        //			knee_strike_info->IDLE_length_kp=My_Chassis->chassis_PID->length_cal[R_Leg]->kp;//保存
        //			knee_strike_info->Stand_High_tick=0;
        // 事件
        //			knee_strike_info->JUMP_THEN_KNEE_STRIKE_Step=JK_Stand_High;
        //			break;
        //		case J_IDLE:
        // 动作
        jump_info->IDLE_length_kp = My_Chassis->chassis_PID->length_cal[R_Leg]->kp;
        jump_info->IDLE_length_speed_kp = My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp;
        jump_info->IDLE_length_outmax = My_Chassis->chassis_PID->length_cal[R_Leg]->out_max;
        jump_info->IDLE_length_speed_outmax = My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max;

        My_Chassis->chassis_PID->length_cal[R_Leg]->out_max = 1000.f;
        My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max = 1000.f;
        My_Chassis->chassis_PID->length_cal[L_Leg]->out_max = 1000.f;
        My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out_max = 1000.f;

        jump_info->COMPRESS_tick = 0;
        jump_info->EXTEND_tick = 0;
        jump_info->RETRACT_tick = 0;
        jump_info->PRE_LANDING_tick = 0;
        jump_info->LANDING_tick = 0;
        knee_strike_info->Stand_High_tick = 0;
        // 事件
        knee_strike_info->JUMP_THEN_KNEE_STRIKE_Step = JK_COMPRESS;
        break;

    case JK_COMPRESS: // 压缩腿
        // 动作
        jump_info->COMPRESS_tick++;
        My_Chassis->target->leg_length_l = MIN_LEG_LENGTH + jump_info->Minimum_l0_range;
        My_Chassis->target->leg_length_r = MIN_LEG_LENGTH + jump_info->Minimum_l0_range;
        My_Chassis->chassis_PID->length_cal[R_Leg]->kp = jump_info->COMPRESS_length_kp;
        My_Chassis->chassis_PID->length_cal[L_Leg]->kp = jump_info->COMPRESS_length_kp;
// 事件
#ifdef TIME_ONLY
        if (jump_info->COMPRESS_tick >= jump_info->Max_COMPRESS_tick)
            jump_info->jump_step = J_EXTEND;
#else
        if ((jump_info->l0_average <= MIN_LEG_LENGTH + jump_info->Minimum_l0_range) || jump_info->COMPRESS_tick >= jump_info->Max_COMPRESS_tick)
            knee_strike_info->JUMP_THEN_KNEE_STRIKE_Step = JK_EXTEND;
#endif

        break;

    case JK_EXTEND: // 跳
        // 动作
        jump_info->EXTEND_tick++;
        My_Chassis->target->leg_length_l = MAX_LEG_LENGTH - jump_info->Max_l0_range;
        My_Chassis->target->leg_length_r = MAX_LEG_LENGTH - jump_info->Max_l0_range;
        My_Chassis->chassis_PID->length_cal[R_Leg]->kp = jump_info->EXTEND_length_kp;
        My_Chassis->chassis_PID->length_cal[L_Leg]->kp = jump_info->EXTEND_length_kp;
// 事件
#ifdef TIME_ONLY
        if (jump_info->EXTEND_tick >= jump_info->Max_EXTEND_tick)
#else
        if ((jump_info->l0_average >= MAX_LEG_LENGTH - jump_info->Max_l0_range) || jump_info->EXTEND_tick >= jump_info->Max_EXTEND_tick)
#endif

            knee_strike_info->JUMP_THEN_KNEE_STRIKE_Step = JK_RETRACT_1;
        break;

    case JK_RETRACT_1: // 收腿滞空
        // 动作
        jump_info->RETRACT_tick++;
        My_Chassis->target->leg_length_l = MIN_LEG_LENGTH + jump_info->Minimum_l0_range;
        My_Chassis->target->leg_length_r = MIN_LEG_LENGTH + jump_info->Minimum_l0_range;
        My_Chassis->chassis_PID->length_cal[R_Leg]->kp = jump_info->RETRACT_length_kp;
        My_Chassis->chassis_PID->length_cal[L_Leg]->kp = jump_info->RETRACT_length_kp;
        // 事件
        //			if(jump_info->r_offground==false&&jump_info->l_offground==false)
        //			{
        //				Balance.Flag->Jumping_Flag = false;
        //				jump_info->jump_step=J_IDLE;
        //				My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->IDLE_length_kp;
        //				My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp=jump_info->IDLE_length_speed_kp;
        //				My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->IDLE_length_kp;
        //				My_Chassis->chassis_PID->length_speed_cal[L_Leg]->kp=jump_info->IDLE_length_speed_kp;
        //			}

#ifdef NO_PRE_LANDING
#ifdef TIME_ONLY
        if (jump_info->RETRACT_tick >= jump_info->Max_RETRACT_tick)
#else
        if (jump_info->l0_average - (MIN_LEG_LENGTH + jump_info->Minimum_l0_range) <= 0.f || jump_info->RETRACT_tick >= jump_info->Max_RETRACT_tick)
#endif

        {
            Balance.Flag->Jumping_Flag = false;
            knee_strike_info->JUMP_THEN_KNEE_STRIKE_Step = JK_IDLE;
            My_Chassis->chassis_PID->length_cal[R_Leg]->kp = jump_info->IDLE_length_kp;
            My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp = jump_info->IDLE_length_speed_kp;
            My_Chassis->chassis_PID->length_cal[L_Leg]->kp = jump_info->IDLE_length_kp;
            My_Chassis->chassis_PID->length_speed_cal[L_Leg]->kp = jump_info->IDLE_length_speed_kp;
            My_Chassis->chassis_PID->length_cal[R_Leg]->out_max = jump_info->IDLE_length_outmax;
            My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max = jump_info->IDLE_length_speed_outmax;
            My_Chassis->chassis_PID->length_cal[L_Leg]->out_max = jump_info->IDLE_length_outmax;
            My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out_max = jump_info->IDLE_length_speed_outmax;
        }

#else
        if (jump_info->RETRACT_tick >= jump_info->Max_RETRACT_tick)
        {
            knee_strike_info->JUMP_THEN_KNEE_STRIKE_Step = JK_PRE_LANDING;
        }
#endif
        break;

    case JK_PRE_LANDING: // 伸腿准备缓冲
        // 动作
        jump_info->PRE_LANDING_tick++;
        My_Chassis->target->leg_length_l = MAX_LEG_LENGTH - jump_info->Max_l0_range;
        My_Chassis->target->leg_length_r = MAX_LEG_LENGTH - jump_info->Max_l0_range;
        My_Chassis->chassis_PID->length_cal[R_Leg]->kp = jump_info->PRE_LANDING_length_kp;
        My_Chassis->chassis_PID->length_cal[L_Leg]->kp = jump_info->PRE_LANDING_length_kp;
        // 事件
        if (jump_info->r_offground == false || jump_info->l_offground == false)
        {
            knee_strike_info->JUMP_THEN_KNEE_STRIKE_Step = JK_LANDING;
        }
        if (jump_info->PRE_LANDING_tick >= jump_info->Max_PRE_LANDING_tick)
        {
            knee_strike_info->JUMP_THEN_KNEE_STRIKE_Step = JK_LANDING;
        }
        break;

    case JK_LANDING: // 缓冲
        // 动作
        jump_info->LANDING_tick++;
        My_Chassis->target->leg_length_l = MAX_LEG_LENGTH - 5.0f * jump_info->Max_l0_range;
        My_Chassis->target->leg_length_r = MAX_LEG_LENGTH - 5.0f * jump_info->Max_l0_range;
        My_Chassis->chassis_PID->length_cal[R_Leg]->kp = jump_info->LANDING_length_kp;
        My_Chassis->chassis_PID->length_cal[L_Leg]->kp = jump_info->LANDING_length_kp;
        My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp = jump_info->LANDING_length_speed_kp;
        My_Chassis->chassis_PID->length_speed_cal[L_Leg]->kp = jump_info->LANDING_length_speed_kp;
        // 事件
        if (jump_info->LANDING_tick >= jump_info->Max_LANDING_tick ||
            (my_abs(jump_info->l0_average - TAR_LEG_LENGTH_INITIAL) <= jump_info->Landing_l0_range))
        {
            knee_strike_info->JUMP_THEN_KNEE_STRIKE_Step = JK_Stand_High;
            ////				My_Chassis->target->leg_length_r = TAR_LEG_LENGTH_INITIAL;
            ////				My_Chassis->target->leg_length_l = TAR_LEG_LENGTH_INITIAL;
            //				My_Chassis->chassis_PID->length_cal[R_Leg]->kp=jump_info->IDLE_length_kp;
            //				My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp=jump_info->IDLE_length_speed_kp;
            //				My_Chassis->chassis_PID->length_cal[L_Leg]->kp=jump_info->IDLE_length_kp;
            //				My_Chassis->chassis_PID->length_speed_cal[L_Leg]->kp=jump_info->IDLE_length_speed_kp;
            //				My_Chassis->chassis_PID->length_cal[R_Leg]->out_max=jump_info->IDLE_length_outmax;
            //				My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max=jump_info->IDLE_length_speed_outmax;
            //				My_Chassis->chassis_PID->length_cal[L_Leg]->out_max=jump_info->IDLE_length_outmax;
            //				My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out_max=jump_info->IDLE_length_speed_outmax;
        }
        break;

    case JK_Stand_High: // 立着
        // 动作
        knee_strike_info->Stand_High_tick++;
        My_Chassis->target->leg_length_l = MAX_LEG_LENGTH - knee_strike_info->Max_l0_range;
        My_Chassis->target->leg_length_r = MAX_LEG_LENGTH - knee_strike_info->Max_l0_range;

        // 事件
        if (knee_strike_info->thetal_average >= knee_strike_info->thetal_threshold)
        {
            knee_strike_info->JUMP_THEN_KNEE_STRIKE_Step = JK_RETRACT_2;
        }
        if (knee_strike_info->Stand_High_tick >= knee_strike_info->Max_Stand_High_tick)
        {
            knee_strike_info->JUMP_THEN_KNEE_STRIKE_Step = JK_IDLE;
            My_Chassis->target->leg_length_l = TAR_LEG_LENGTH_INITIAL;
            My_Chassis->target->leg_length_r = TAR_LEG_LENGTH_INITIAL;
            My_Chassis->chassis_PID->length_cal[R_Leg]->kp = jump_info->IDLE_length_kp;
            My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp = jump_info->IDLE_length_speed_kp;
            My_Chassis->chassis_PID->length_cal[L_Leg]->kp = jump_info->IDLE_length_kp;
            My_Chassis->chassis_PID->length_speed_cal[L_Leg]->kp = jump_info->IDLE_length_speed_kp;
            My_Chassis->chassis_PID->length_cal[R_Leg]->out_max = jump_info->IDLE_length_outmax;
            My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max = jump_info->IDLE_length_speed_outmax;
            My_Chassis->chassis_PID->length_cal[L_Leg]->out_max = jump_info->IDLE_length_outmax;
            My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out_max = jump_info->IDLE_length_speed_outmax;
            Balance.Flag->JUMP_THEN_KNEE_STRIKE_Flag = false;
        }
        break;

    case JK_RETRACT_2: // 收腿
        // 动作
        knee_strike_info->RETRACT_tick++;
        My_Chassis->target->leg_length_l = MIN_LEG_LENGTH + knee_strike_info->Minimum_l0_range;
        My_Chassis->target->leg_length_r = MIN_LEG_LENGTH + knee_strike_info->Minimum_l0_range;
        My_Chassis->chassis_PID->length_cal[R_Leg]->kp = knee_strike_info->RETRACT_length_kp;
        My_Chassis->chassis_PID->length_cal[L_Leg]->kp = knee_strike_info->RETRACT_length_kp;
        // 事件
        if (knee_strike_info->RETRACT_tick >= knee_strike_info->Max_RETRACT_tick || knee_strike_info->l0_average < MIN_LEG_LENGTH + knee_strike_info->Minimum_l0_range)
        {
            knee_strike_info->JUMP_THEN_KNEE_STRIKE_Step = JK_IDLE;
            // My_Chassis->target->leg_length_l=TAR_LEG_LENGTH_INITIAL;
            // My_Chassis->target->leg_length_r=TAR_LEG_LENGTH_INITIAL;
            My_Chassis->target->leg_length_l = TAR_LEG_LENGTH_INITIAL;
            My_Chassis->target->leg_length_r = TAR_LEG_LENGTH_INITIAL;
            My_Chassis->chassis_PID->length_cal[R_Leg]->kp = jump_info->IDLE_length_kp;
            My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp = jump_info->IDLE_length_speed_kp;
            My_Chassis->chassis_PID->length_cal[L_Leg]->kp = jump_info->IDLE_length_kp;
            My_Chassis->chassis_PID->length_speed_cal[L_Leg]->kp = jump_info->IDLE_length_speed_kp;
            My_Chassis->chassis_PID->length_cal[R_Leg]->out_max = jump_info->IDLE_length_outmax;
            My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out_max = jump_info->IDLE_length_speed_outmax;
            My_Chassis->chassis_PID->length_cal[L_Leg]->out_max = jump_info->IDLE_length_outmax;
            My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out_max = jump_info->IDLE_length_speed_outmax;
            Balance.Flag->JUMP_THEN_KNEE_STRIKE_Flag = false;
        }

        break;

    default:
        break;
    }
}

/**
 * @brief  消去因小腿连接定子转动导致车体位移测量值不准确的误差(比如在腿进行竖直伸缩的时候，会出现位移的偏差）
 * @param  Link_Var_t* Link_Var, float thetal
 * @retval None
 */
static void Stator_Correction_Cal(Chassis_t *My_Chassis)
{
    // 右腿
    float thetal = My_Chassis->Leg_Unit[R_Leg]->Straight->info->thetal;
    float phi0 = My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->phi0;
    float phi3 = My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->phi3;
    Link_t *Link_Var = My_Chassis->Leg_Unit[R_Leg]->Link;

    // 初中知识，看五连杆的图就可以算出来，需要注意方向
    // stator_angle_now为l3杆与竖直方向的夹角
    Link_Var->info->stator_correction->stator_angle_now = phi3 - phi0 - thetal;

    if (Link_Var->info->stator_correction->stator_angle_last == 0)
    {
        Link_Var->info->stator_correction->stator_angular_speed = 0;
    }
    else
    {
        Link_Var->info->stator_correction->stator_angular_speed =
            (Link_Var->info->stator_correction->stator_angle_now - Link_Var->info->stator_correction->stator_angle_last) / TIME_STEP;
    }

    Link_Var->info->stator_correction->stator_angle_last = Link_Var->info->stator_correction->stator_angle_now;
    // 伸腿时相当于轮子后退（逆时针），此处stator_angle变小，输出负的stator_bias，则s需要加上一个正的值，故Chassis_State_Var_Update里是减去
    Link_Var->info->stator_correction->stator_bias = R_STATOR_ORDER_CORRECT * Link_Var->info->stator_correction->stator_angular_speed * TIME_STEP + Link_Var->info->stator_correction->stator_bias;

    // 左腿
    thetal = My_Chassis->Leg_Unit[L_Leg]->Straight->info->thetal;
    phi0 = My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->phi0;
    phi3 = My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->phi3;
    Link_Var = My_Chassis->Leg_Unit[L_Leg]->Link;

    Link_Var->info->stator_correction->stator_angle_now = phi3 - phi0 - thetal;

    if (Link_Var->info->stator_correction->stator_angle_last == 0)
    {
        Link_Var->info->stator_correction->stator_angular_speed = 0;
    }
    else
    {
        Link_Var->info->stator_correction->stator_angular_speed =
            (Link_Var->info->stator_correction->stator_angle_now - Link_Var->info->stator_correction->stator_angle_last) / TIME_STEP;
    }

    Link_Var->info->stator_correction->stator_angle_last = Link_Var->info->stator_correction->stator_angle_now;

    Link_Var->info->stator_correction->stator_bias = L_STATOR_ORDER_CORRECT * Link_Var->info->stator_correction->stator_angular_speed * TIME_STEP + Link_Var->info->stator_correction->stator_bias;
}
/**
 * @brief  底盘卸力前加阻尼保护机械结构,记得限制最大输出，或者速度大于一定值不处理
 * @param  None
 * @retval None
 */
static void Chassis_Stop_Damping(Chassis_t *My_Chassis)
{
    float torque_test = 0.f;

    torque_test = -My_Chassis->Wheel->motor[0]->rx_info->speed * Wheel_Damping_Coefficient;

    My_Chassis->Wheel->motor[0]->tx_info->torque = torque_test;

    torque_test = -My_Chassis->Wheel->motor[1]->rx_info->speed * Wheel_Damping_Coefficient;

    My_Chassis->Wheel->motor[1]->tx_info->torque = torque_test;

    torque_test = -My_Chassis->Sd->motor[0]->rx_info->speed * Sd_Damping_Coefficient;

    My_Chassis->Sd->motor[0]->tx_info->torque = torque_test;
    ;

    torque_test = -My_Chassis->Sd->motor[1]->rx_info->speed * Sd_Damping_Coefficient;

    My_Chassis->Sd->motor[1]->tx_info->torque = torque_test;
    ;

    torque_test = -My_Chassis->Sd->motor[2]->rx_info->speed * Sd_Damping_Coefficient;

    My_Chassis->Sd->motor[2]->tx_info->torque = torque_test;
    ;

    torque_test = -My_Chassis->Sd->motor[3]->rx_info->speed * Sd_Damping_Coefficient;

    My_Chassis->Sd->motor[3]->tx_info->torque = torque_test; //- 3;
}

/**
 * @brief  底盘离线数据处理
 * @param  Chassis_t* chassis, 底盘
 * @retval None
 */
static void Chassis_Offline_Process(Chassis_t *My_Chassis)
{
    /*驱动轮测量值，目标值归零*/
    My_Chassis->Wheel->motor[0]->rx_info->motor_angle_last = 0;
    My_Chassis->Wheel->motor[0]->rx_info->motor_angle_sum = 0;

    My_Chassis->Wheel->motor[1]->rx_info->motor_angle_last = 0;
    My_Chassis->Wheel->motor[1]->rx_info->motor_angle_sum = 0;

    My_Chassis->target->leg_length_l = TAR_LEG_LENGTH_INITIAL;              // 腿长目标值改为初始值
    My_Chassis->target->leg_length_r = TAR_LEG_LENGTH_INITIAL;              // 腿长目标值改为初始值
    My_Chassis->target->yaw_degree = My_Chassis->Posture->info->yaw_degree; // 偏航角目标值等于测量值
    //	My_Chassis->chassis_PID->length_cal[R_Leg]->kp=My_Chassis->pid_init_parament->l0_length_kp;//防止命令执行过程中进入sleep模式导致pid参数不恢复
    //	My_Chassis->chassis_PID->length_cal[L_Leg]->kp=My_Chassis->pid_init_parament->l0_length_kp;
    //	My_Chassis->chassis_PID->length_speed_cal[R_Leg]->kp=My_Chassis->pid_init_parament->l0_length_speed_kp;
    //	My_Chassis->chassis_PID->length_speed_cal[L_Leg]->kp=My_Chassis->pid_init_parament->l0_length_speed_kp;
    My_Chassis->target->roll = 0;

    My_Chassis->target->s = 0;

    My_Chassis->target->sd1 = 0;

    My_Chassis->target->yaw_v_degree = 0;

    XEstimateKF_Clear(&XEstimateKF);
    /*连杆信息清零*/
    My_Chassis->Leg_Unit[R_Leg]->Link->info->stator_correction->stator_bias = 0;
    My_Chassis->Leg_Unit[R_Leg]->Link->info->stator_correction->stator_angle_last = 0;
    My_Chassis->Leg_Unit[L_Leg]->Link->info->stator_correction->stator_bias = 0;
    My_Chassis->Leg_Unit[L_Leg]->Link->info->stator_correction->stator_angle_last = 0;

    /*pid控制器清零*/
    pid_clear(My_Chassis->chassis_PID->roll_cal[R_Leg]);
    pid_clear(My_Chassis->chassis_PID->yaw_imu_speed_cal[R_Leg]);
    pid_clear(My_Chassis->chassis_PID->length_cal[R_Leg]);
    pid_clear(My_Chassis->chassis_PID->sync_cal[R_Leg]);
    pid_clear(My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg]);
    pid_clear(My_Chassis->chassis_PID->vir_phi0_cal[R_Leg]);

    pid_clear(My_Chassis->chassis_PID->roll_cal[L_Leg]);
    pid_clear(My_Chassis->chassis_PID->yaw_imu_speed_cal[L_Leg]);
    pid_clear(My_Chassis->chassis_PID->length_cal[L_Leg]);
    pid_clear(My_Chassis->chassis_PID->sync_cal[L_Leg]);
    pid_clear(My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg]);
    pid_clear(My_Chassis->chassis_PID->vir_phi0_cal[L_Leg]);

    My_Chassis->Leg_Unit[L_Leg]->off_ground_cnt = 0;
    My_Chassis->Leg_Unit[R_Leg]->off_ground_cnt = 0;

    /*命令状态机清零*/
    Balance.Flag->Jumping_Flag = false;
    Balance.Flag->Chassis_Alignment_Flag = false;
    Balance.Flag->KNEE_STRIKE_Flag = false;
    Balance.Flag->JUMP_THEN_KNEE_STRIKE_Flag = false;
    Balance.Shoot_Flag_struct.Shoot_Ctrl_Flag = false;
    Balance.Flag->Rescue_Flag = false;
    Balance.Flag->Middle_Flag = false;
    Balance.Flag->JUMP_AND_MID_Flag = false;
    My_Chassis->jump_info->jump_step = J_IDLE;
    My_Chassis->knee_strike_info->KNEE_STRIKE_step = Knee_IDLE;
    My_Chassis->jump_and_mid_info->JUMP_AND_MID_step = JAM_IDLE;
    My_Chassis->jump_and_mid_info->JUMP_AND_MID_tick = 0;
    My_Chassis->knee_strike_info->JUMP_THEN_KNEE_STRIKE_Step = JK_IDLE;
}

/**
 * @brief  腿部角度协调计算
 * @param  Chassis_t* My_Chassis
 * @retval 力Tp
 */
static void Chassis_Leg_Sync_Cal(Chassis_t *My_Chassis)
{
    My_Chassis->chassis_PID->sync_cal[R_Leg]->target = 0;
    My_Chassis->chassis_PID->sync_cal[R_Leg]->measure = R_SYNC_ORDER_CORRECT * (My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->vir_phi0 - My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->vir_phi0);

    My_Chassis->chassis_PID->sync_cal[L_Leg]->target = 0;
    My_Chassis->chassis_PID->sync_cal[L_Leg]->measure = L_SYNC_ORDER_CORRECT * (My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->vir_phi0 - My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->vir_phi0);

    pid_err_cal(My_Chassis->chassis_PID->sync_cal[L_Leg]);
    pid_err_cal(My_Chassis->chassis_PID->sync_cal[R_Leg]);
    single_pid_ctrl(My_Chassis->chassis_PID->sync_cal[R_Leg]);
    single_pid_ctrl(My_Chassis->chassis_PID->sync_cal[L_Leg]);
    My_Chassis->Leg_Unit[R_Leg]->force->Tp_sync = My_Chassis->chassis_PID->sync_cal[R_Leg]->out;
    My_Chassis->Leg_Unit[L_Leg]->force->Tp_sync = My_Chassis->chassis_PID->sync_cal[L_Leg]->out;
}

/**
 * @brief  双腿vir_phi0PID计算
 * @param  Chassis_t* My_Chassis
 * @retval 力Tp
 */
static void Chassis_Leg_vir_phi0_Cal(Chassis_t *My_Chassis)
{

    /*外环*/
    My_Chassis->chassis_PID->vir_phi0_cal[R_Leg]->measure = My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->vir_phi0;
    My_Chassis->chassis_PID->vir_phi0_cal[R_Leg]->target = My_Chassis->target->vir_phi0_r;

    My_Chassis->chassis_PID->vir_phi0_cal[L_Leg]->measure = My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->vir_phi0;
    My_Chassis->chassis_PID->vir_phi0_cal[L_Leg]->target = My_Chassis->target->vir_phi0_l;

    pid_err_cal(My_Chassis->chassis_PID->vir_phi0_cal[L_Leg]);
    pid_err_cal(My_Chassis->chassis_PID->vir_phi0_cal[R_Leg]);
    single_pid_ctrl(My_Chassis->chassis_PID->vir_phi0_cal[R_Leg]);
    single_pid_ctrl(My_Chassis->chassis_PID->vir_phi0_cal[L_Leg]);

    /*内环*/
    My_Chassis->chassis_PID->vir_phi0_speed_cal[R_Leg]->target = My_Chassis->chassis_PID->vir_phi0_cal[R_Leg]->out;
    My_Chassis->chassis_PID->vir_phi0_speed_cal[L_Leg]->target = My_Chassis->chassis_PID->vir_phi0_cal[L_Leg]->out;

    My_Chassis->chassis_PID->vir_phi0_speed_cal[R_Leg]->measure = My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->vir_phi0_d1;
    My_Chassis->chassis_PID->vir_phi0_speed_cal[L_Leg]->measure = My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->vir_phi0_d1;

    pid_err_cal(My_Chassis->chassis_PID->vir_phi0_speed_cal[L_Leg]);
    pid_err_cal(My_Chassis->chassis_PID->vir_phi0_speed_cal[R_Leg]);
    single_pid_ctrl(My_Chassis->chassis_PID->vir_phi0_speed_cal[R_Leg]);
    single_pid_ctrl(My_Chassis->chassis_PID->vir_phi0_speed_cal[L_Leg]);

    My_Chassis->Leg_Unit[R_Leg]->force->Tp_vir_phi0 = My_Chassis->chassis_PID->vir_phi0_speed_cal[R_Leg]->out;
    My_Chassis->Leg_Unit[L_Leg]->force->Tp_vir_phi0 = My_Chassis->chassis_PID->vir_phi0_speed_cal[L_Leg]->out;
}
/**
 * @brief  双腿vir_phi0d1PID计算，°为单位
 * @param  Chassis_t* My_Chassis
 * @retval 力Tp
 */
static void Chassis_Leg_vir_phi0_d1_Cal(Chassis_t *My_Chassis)
{
    My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg]->measure = My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->vir_phi0_d1_degree;
    My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg]->target = My_Chassis->target->vir_phi0d1_r_degree;

    My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg]->measure = My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->vir_phi0_d1_degree;
    My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg]->target = My_Chassis->target->vir_phi0d1_l_degree;

    pid_err_cal(My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg]);
    pid_err_cal(My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg]);
    single_pid_ctrl(My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg]);
    single_pid_ctrl(My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg]);
}

/**
 * @brief  驱动轮转向力矩PID计算
 * @param  Chassis_t* My_Chassis
 * @retval 力Tp
 */
static void Chassis_Wheel_Turn_Cal(Chassis_t *My_Chassis)
{
    switch (My_Chassis->mode)
    {
    case C_Boss:
        My_Chassis->chassis_PID->yaw_imu_cal[R_Leg]->measure = My_Chassis->Posture->info->yaw_degree;
        My_Chassis->chassis_PID->yaw_imu_cal[R_Leg]->target = My_Chassis->target->yaw_degree;
        My_Chassis->chassis_PID->yaw_imu_cal[L_Leg]->measure = My_Chassis->Posture->info->yaw_degree;
        My_Chassis->chassis_PID->yaw_imu_cal[L_Leg]->target = My_Chassis->target->yaw_degree;
        pid_err_cal(My_Chassis->chassis_PID->yaw_imu_cal[R_Leg]);
        pid_err_cal(My_Chassis->chassis_PID->yaw_imu_cal[L_Leg]);
        My_Chassis->chassis_PID->yaw_imu_cal[R_Leg]->err = half_cycle(My_Chassis->chassis_PID->yaw_imu_cal[R_Leg]->err, 360.f);
        My_Chassis->chassis_PID->yaw_imu_cal[L_Leg]->err = half_cycle(My_Chassis->chassis_PID->yaw_imu_cal[L_Leg]->err, 360.f);
        single_pid_ctrl(My_Chassis->chassis_PID->yaw_imu_cal[R_Leg]);
        single_pid_ctrl(My_Chassis->chassis_PID->yaw_imu_cal[L_Leg]);
        My_Chassis->chassis_PID->yaw_imu_speed_cal[R_Leg]->target = My_Chassis->chassis_PID->yaw_imu_cal[R_Leg]->out;
        My_Chassis->chassis_PID->yaw_imu_speed_cal[L_Leg]->target = My_Chassis->chassis_PID->yaw_imu_cal[L_Leg]->out;
        My_Chassis->chassis_PID->yaw_imu_speed_cal[R_Leg]->measure = My_Chassis->Posture->info->yaw_v_degree;
        My_Chassis->chassis_PID->yaw_imu_speed_cal[L_Leg]->measure = My_Chassis->Posture->info->yaw_v_degree;
        pid_err_cal(My_Chassis->chassis_PID->yaw_imu_speed_cal[R_Leg]);
        pid_err_cal(My_Chassis->chassis_PID->yaw_imu_speed_cal[L_Leg]);
        single_pid_ctrl(My_Chassis->chassis_PID->yaw_imu_speed_cal[R_Leg]);
        single_pid_ctrl(My_Chassis->chassis_PID->yaw_imu_speed_cal[L_Leg]);
        My_Chassis->Leg_Unit[R_Leg]->force->Tw_turn = R_TURN_ORDER_CORRECT * My_Chassis->chassis_PID->yaw_imu_speed_cal[R_Leg]->out;
        My_Chassis->Leg_Unit[L_Leg]->force->Tw_turn = L_TURN_ORDER_CORRECT * My_Chassis->chassis_PID->yaw_imu_speed_cal[L_Leg]->out;
        // My_Chassis->Leg_Unit[R_Leg]->force->Tw_turn = 0;
        // My_Chassis->Leg_Unit[L_Leg]->force->Tw_turn = 0;

        break;

    case C_KNEE_STRIKE:
    case C_JUMP_AND_MID:
    case C_Follow:
    case C_RTS:

        My_Chassis->chassis_PID->yaw_mec_cal[R_Leg]->measure = gimbal.base_info.yaw_mec_360_angle;
        My_Chassis->chassis_PID->yaw_mec_cal[R_Leg]->target = My_Chassis->target->yaw_degree;
        My_Chassis->chassis_PID->yaw_mec_cal[L_Leg]->measure = gimbal.base_info.yaw_mec_360_angle;
        My_Chassis->chassis_PID->yaw_mec_cal[L_Leg]->target = My_Chassis->target->yaw_degree;
        pid_err_cal(My_Chassis->chassis_PID->yaw_mec_cal[R_Leg]);
        pid_err_cal(My_Chassis->chassis_PID->yaw_mec_cal[L_Leg]);
        My_Chassis->chassis_PID->yaw_mec_cal[R_Leg]->err = half_cycle(My_Chassis->chassis_PID->yaw_mec_cal[R_Leg]->err, 360.f);
        My_Chassis->chassis_PID->yaw_mec_cal[L_Leg]->err = half_cycle(My_Chassis->chassis_PID->yaw_mec_cal[L_Leg]->err, 360.f);
        single_pid_ctrl(My_Chassis->chassis_PID->yaw_mec_cal[R_Leg]);
        single_pid_ctrl(My_Chassis->chassis_PID->yaw_mec_cal[L_Leg]);
        My_Chassis->chassis_PID->yaw_mec_speed_cal[R_Leg]->target = My_Chassis->chassis_PID->yaw_mec_cal[R_Leg]->out;
        My_Chassis->chassis_PID->yaw_mec_speed_cal[L_Leg]->target = My_Chassis->chassis_PID->yaw_mec_cal[L_Leg]->out;
        My_Chassis->chassis_PID->yaw_mec_speed_cal[R_Leg]->measure = My_Chassis->Posture->info->yaw_v_degree;
        My_Chassis->chassis_PID->yaw_mec_speed_cal[L_Leg]->measure = My_Chassis->Posture->info->yaw_v_degree;
        pid_err_cal(My_Chassis->chassis_PID->yaw_mec_speed_cal[R_Leg]);
        pid_err_cal(My_Chassis->chassis_PID->yaw_mec_speed_cal[L_Leg]);
        single_pid_ctrl(My_Chassis->chassis_PID->yaw_mec_speed_cal[R_Leg]);
        single_pid_ctrl(My_Chassis->chassis_PID->yaw_mec_speed_cal[L_Leg]);
        My_Chassis->Leg_Unit[R_Leg]->force->Tw_turn = R_TURN_ORDER_CORRECT * My_Chassis->chassis_PID->yaw_mec_speed_cal[R_Leg]->out;
        My_Chassis->Leg_Unit[L_Leg]->force->Tw_turn = L_TURN_ORDER_CORRECT * My_Chassis->chassis_PID->yaw_mec_speed_cal[L_Leg]->out;
        break;

    case C_Cycle:
        /* ---------------------- 小陀螺yaw_imu_pid begin --------------------------- */
        // target = cycle_info->target_yaw_v_degree (由Cycle_Target_Process计算)
        // measure = posture->yaw_v_degree (IMU角速度反馈)
        My_Chassis->chassis_PID->cycle_yaw_imu_speed_cal[R_Leg]->target = My_Chassis->cycle_info->target_yaw_v_degree;
        My_Chassis->chassis_PID->cycle_yaw_imu_speed_cal[R_Leg]->measure = My_Chassis->Posture->info->yaw_v_degree;
        My_Chassis->chassis_PID->cycle_yaw_imu_speed_cal[L_Leg]->target = My_Chassis->cycle_info->target_yaw_v_degree;
        My_Chassis->chassis_PID->cycle_yaw_imu_speed_cal[L_Leg]->measure = My_Chassis->Posture->info->yaw_v_degree;

        // PID误差计算
        pid_err_cal(My_Chassis->chassis_PID->cycle_yaw_imu_speed_cal[R_Leg]);
        pid_err_cal(My_Chassis->chassis_PID->cycle_yaw_imu_speed_cal[L_Leg]);

        // PID输出计算
        single_pid_ctrl(My_Chassis->chassis_PID->cycle_yaw_imu_speed_cal[R_Leg]);
        single_pid_ctrl(My_Chassis->chassis_PID->cycle_yaw_imu_speed_cal[L_Leg]);
        /* ---------------------- 小陀螺yaw_imu_pid end --------------------------- */

        /* ---------------------- 防偏心小陀螺，轮子转速差pid（没啥用） begin --------------------------- */
        // target = cycle_info->target_yaw_v_degree (由Cycle_Target_Process计算)
        // measure = posture->yaw_v_degree (IMU角速度反馈)
        My_Chassis->chassis_PID->cycle_wheel_speed_err_Pid[R_Leg]->target = 0;
        My_Chassis->chassis_PID->cycle_wheel_speed_err_Pid[R_Leg]->measure = My_Chassis->Wheel->motor[R_Leg]->rx_info->encoder_speed - My_Chassis->Wheel->motor[L_Leg]->rx_info->encoder_speed;
        My_Chassis->chassis_PID->cycle_wheel_speed_err_Pid[L_Leg]->target = 0;
        My_Chassis->chassis_PID->cycle_wheel_speed_err_Pid[L_Leg]->measure = -(My_Chassis->Wheel->motor[R_Leg]->rx_info->encoder_speed - My_Chassis->Wheel->motor[L_Leg]->rx_info->encoder_speed);

        // PID误差计算
        pid_err_cal(My_Chassis->chassis_PID->cycle_wheel_speed_err_Pid[R_Leg]);
        pid_err_cal(My_Chassis->chassis_PID->cycle_wheel_speed_err_Pid[L_Leg]);

        // PID输出计算
        single_pid_ctrl(My_Chassis->chassis_PID->cycle_wheel_speed_err_Pid[R_Leg]);
        single_pid_ctrl(My_Chassis->chassis_PID->cycle_wheel_speed_err_Pid[L_Leg]);
        /* ---------------------- 防偏心小陀螺，轮子转速差pid end --------------------------- */

        // 输出到Tw_turn
        My_Chassis->Leg_Unit[R_Leg]->force->Tw_turn = R_TURN_ORDER_CORRECT * (My_Chassis->chassis_PID->cycle_yaw_imu_speed_cal[R_Leg]->out + My_Chassis->chassis_PID->cycle_wheel_speed_err_Pid[R_Leg]->out);
        My_Chassis->Leg_Unit[L_Leg]->force->Tw_turn = L_TURN_ORDER_CORRECT * (My_Chassis->chassis_PID->cycle_yaw_imu_speed_cal[L_Leg]->out + My_Chassis->chassis_PID->cycle_wheel_speed_err_Pid[L_Leg]->out);
        break;

    default:
        My_Chassis->Leg_Unit[R_Leg]->force->Tw_turn = 0;
        My_Chassis->Leg_Unit[L_Leg]->force->Tw_turn = 0;
        break;
    }
}
/**
 * @brief  离地检测，在Chassis_Target_Update处理离地
 * @param  Chassis_t* My_Chassis, 底盘
 * @retval None
 */
static void Chassis_Takeoff_Detect(Chassis_t *My_Chassis)
{
    Link_info_t *R_Link_info = My_Chassis->Leg_Unit[R_Leg]->Link->info;
    Link_info_t *L_Link_info = My_Chassis->Leg_Unit[L_Leg]->Link->info;
    State_info_t *R_Straight_info = My_Chassis->Leg_Unit[R_Leg]->Straight->info;
    State_info_t *L_Straight_info = My_Chassis->Leg_Unit[L_Leg]->Straight->info;

    /*计算左右驱动轮的支持力*/
    // VMC力+腿重加速度力
    My_Chassis->Leg_Unit[L_Leg]->force->F_support = L_Link_info->force->F_bl_mea * cos(L_Straight_info->thetal) + m_l * (My_Chassis->Posture->info->z_world - (1.f - L_Link_info->centroid->centriod_coefficient) * L_Link_info->length->l0_dot2 * cos(L_Straight_info->thetal));
    My_Chassis->Leg_Unit[R_Leg]->force->F_support = R_Link_info->force->F_bl_mea * cos(R_Straight_info->thetal) + m_l * (My_Chassis->Posture->info->z_world - (1.f - R_Link_info->centroid->centriod_coefficient) * R_Link_info->length->l0_dot2 * cos(R_Straight_info->thetal));
    //	L_Link_info->force->F_support = L_Link_info->force->F_bl_mea*arm_cos_f32(Leg_info->thetal_l) + \
    //                                  L_Link_info->force->Tp_mea*arm_sin_f32(Leg_info->thetal_l)/L_Link_info->length->l0 + \
    //	                                mw * (My_Chassis->Posture->info->z_world - L_Link_info->length->l0_dot2*arm_cos_f32(Leg_info->thetal_l) +
    //	                                2*L_Link_info->length->l0_dot*Leg_info->thetald1_l*arm_sin_f32(Leg_info->thetal_l) + \
    //	                                L_Link_info->length->l0*Leg_info->thetald2_l*arm_sin_f32(Leg_info->thetal_l) + \
    //	                                L_Link_info->length->l0*powf(Leg_info->thetald1_l, 2)*arm_cos_f32(Leg_info->thetal_l));
    //
    //	R_Link_info->force->F_support = R_Link_info->force->F_bl_mea*arm_cos_f32(Leg_info->thetal_l) + \
    //                                  R_Link_info->force->Tp_mea*arm_sin_f32(Leg_info->thetal_l)/R_Link_info->length->l0 + \
    //	                                mw * (My_Chassis->Posture->info->z_world - R_Link_info->length->l0_dot2*arm_cos_f32(Leg_info->thetal_l) +
    //	                                2*R_Link_info->length->l0_dot*Leg_info->thetald1_l*arm_sin_f32(Leg_info->thetal_l) + \
    //	                                R_Link_info->length->l0*Leg_info->thetald2_l*arm_sin_f32(Leg_info->thetal_l) + \
    //	                                R_Link_info->length->l0*powf(Leg_info->thetald1_l, 2)*arm_cos_f32(Leg_info->thetal_l));

    // 右腿
    if (My_Chassis->Leg_Unit[R_Leg]->force->F_support <= OFF_GROUND_SUPPORT)
    {
        My_Chassis->Leg_Unit[R_Leg]->off_ground_cnt++;
    }
    else
    {
        My_Chassis->Leg_Unit[R_Leg]->off_ground_cnt = 0;
    }
    if (My_Chassis->Leg_Unit[R_Leg]->off_ground_cnt >= OFF_GROUND_TIME_THRESHOLD)
    {
        My_Chassis->Leg_Unit[R_Leg]->off_ground = true;

        My_Chassis->Leg_Unit[R_Leg]->off_ground_cnt = OFF_GROUND_TIME_THRESHOLD;
    }
    else
    {
        My_Chassis->Leg_Unit[R_Leg]->off_ground = false;
    }

    // 左腿
    if (My_Chassis->Leg_Unit[L_Leg]->force->F_support <= OFF_GROUND_SUPPORT)
    {
        My_Chassis->Leg_Unit[L_Leg]->off_ground_cnt++;
    }
    else
    {
        My_Chassis->Leg_Unit[L_Leg]->off_ground_cnt = 0;
    }
    if (My_Chassis->Leg_Unit[L_Leg]->off_ground_cnt >= OFF_GROUND_TIME_THRESHOLD)
    {
        My_Chassis->Leg_Unit[L_Leg]->off_ground = true;

        My_Chassis->Leg_Unit[L_Leg]->off_ground_cnt = OFF_GROUND_TIME_THRESHOLD;
    }
    else
    {
        My_Chassis->Leg_Unit[L_Leg]->off_ground = false;
    }
}

/**
 * @brief  用于计算补偿自救时腿重力引起的力矩
 * @param  Chassis_t* My_Chassis
 * @retval None
 */
static void Cal_Leg_Gravity_Torque(Chassis_t *My_Chassis)
{
    // 补偿系数
    float k_r = My_Chassis->Leg_Unit[R_Leg]->K_Leg_Gravity_Torque;
    float k_l = My_Chassis->Leg_Unit[L_Leg]->K_Leg_Gravity_Torque;
    // 质心系数、腿长、腿与竖直方向夹角
    float l_centriod_coefficient = My_Chassis->Leg_Unit[L_Leg]->Link->info->centroid->centriod_coefficient;
    float r_centriod_coefficient = My_Chassis->Leg_Unit[R_Leg]->Link->info->centroid->centriod_coefficient;
    float l_length = My_Chassis->Leg_Unit[L_Leg]->Link->info->length->l0;
    float r_length = My_Chassis->Leg_Unit[R_Leg]->Link->info->length->l0;
    float l_thetal = My_Chassis->Leg_Unit[L_Leg]->Straight->info->thetal;
    float r_thetal = My_Chassis->Leg_Unit[R_Leg]->Straight->info->thetal;
    float G_leg = (m_l3 + m_l4 + mw) * g;
    // 实际补偿时应该添加“-”号，因为腿重力产生的力矩方向与补偿力矩方向相反
    My_Chassis->Leg_Unit[R_Leg]->force->Leg_Gravity_Torque = k_r * r_length * G_leg * arm_sin_f32(r_thetal);
    My_Chassis->Leg_Unit[L_Leg]->force->Leg_Gravity_Torque = k_l * l_length * G_leg * arm_sin_f32(l_thetal);
}

/**
 * @brief  腿长控制， 计算不同腿长下所需沿杆方向力F
 * @param  Chassis_t* My_Chassis
 */
static void Chassis_Leg_Length_Strength_Cal(Chassis_t *My_Chassis)
{
    Link_t *R_Link_Var = My_Chassis->Leg_Unit[R_Leg]->Link;
    Link_t *L_Link_Var = My_Chassis->Leg_Unit[L_Leg]->Link;

    /*位置环*/
    My_Chassis->chassis_PID->length_cal[R_Leg]->measure = R_Link_Var->info->length->l0;
    My_Chassis->chassis_PID->length_cal[L_Leg]->measure = L_Link_Var->info->length->l0;

    /*赋予处理后的目标值 begin*/

    My_Chassis->chassis_PID->length_cal[R_Leg]->target = My_Chassis->target->leg_length_r + My_Chassis->Leg_Unit[R_Leg]->force->F_roll;
    My_Chassis->chassis_PID->length_cal[L_Leg]->target = My_Chassis->target->leg_length_l - My_Chassis->Leg_Unit[R_Leg]->force->F_roll;
    // 目标腿长限幅
    My_Chassis->chassis_PID->length_cal[R_Leg]->target = constrain(My_Chassis->chassis_PID->length_cal[R_Leg]->target, MIN_LEG_LENGTH, MAX_LEG_LENGTH);
    My_Chassis->chassis_PID->length_cal[L_Leg]->target = constrain(My_Chassis->chassis_PID->length_cal[L_Leg]->target, MIN_LEG_LENGTH, MAX_LEG_LENGTH);

    pid_err_cal(My_Chassis->chassis_PID->length_cal[R_Leg]);
    pid_err_cal(My_Chassis->chassis_PID->length_cal[L_Leg]);
    /*积分分离*/
    if (fabsf(My_Chassis->chassis_PID->length_cal[R_Leg]->err) > 0.05f)
    {
        My_Chassis->chassis_PID->length_cal[R_Leg]->integral_max = 0;
    }
    else
    {
        My_Chassis->chassis_PID->length_cal[R_Leg]->integral_max = 2;
    }
    single_pid_ctrl(My_Chassis->chassis_PID->length_cal[R_Leg]);

    /*积分分离*/
    if (fabsf(My_Chassis->chassis_PID->length_cal[L_Leg]->err) > 0.05f)
    {
        My_Chassis->chassis_PID->length_cal[L_Leg]->integral_max = 0;
    }
    else
    {
        My_Chassis->chassis_PID->length_cal[L_Leg]->integral_max = 2;
    }
    single_pid_ctrl(My_Chassis->chassis_PID->length_cal[L_Leg]);

    My_Chassis->chassis_PID->length_speed_cal[R_Leg]->measure = R_Link_Var->info->length->l0_dot; // R_Link_Var->info->length->l0;
    My_Chassis->chassis_PID->length_speed_cal[L_Leg]->measure = L_Link_Var->info->length->l0_dot; // L_Link_Var->info->length->l0;

    My_Chassis->chassis_PID->length_speed_cal[R_Leg]->target = My_Chassis->chassis_PID->length_cal[R_Leg]->out; // R_Link_Var->info->length->l0;
    My_Chassis->chassis_PID->length_speed_cal[L_Leg]->target = My_Chassis->chassis_PID->length_cal[L_Leg]->out; // L_Link_Var->info->length->l0;

    pid_err_cal(My_Chassis->chassis_PID->length_speed_cal[R_Leg]);
    pid_err_cal(My_Chassis->chassis_PID->length_speed_cal[L_Leg]);

    single_pid_ctrl(My_Chassis->chassis_PID->length_speed_cal[R_Leg]);
    single_pid_ctrl(My_Chassis->chassis_PID->length_speed_cal[L_Leg]);

    My_Chassis->Leg_Unit[R_Leg]->force->F = My_Chassis->chassis_PID->length_speed_cal[R_Leg]->out;
    My_Chassis->Leg_Unit[L_Leg]->force->F = My_Chassis->chassis_PID->length_speed_cal[L_Leg]->out;
}
/**
 * @brief  求Tp_target，含部分模式特殊处理
 * @param  None
 * @retval None
 */
static void Chassis_Tp_target_Cal(Chassis_t *My_Chassis)
{
    // 自救vir_phi0d1的Tp的PID计算
    // Chassis_Leg_vir_phi0_Cal(My_Chassis); //状态机里面再计算吧，平时就不计算了
    // 自救里给Tp_rescue赋值了
    //    My_Chassis->Leg_Unit[R_Leg]->force->Tp_rescue = My_Chassis->chassis_PID->vir_phi0_cal[R_Leg]->out;
    //    My_Chassis->Leg_Unit[L_Leg]->force->Tp_rescue = My_Chassis->chassis_PID->vir_phi0_cal[L_Leg]->out;
    Link_t *My_L_Link = My_Chassis->Leg_Unit[L_Leg]->Link;
    Link_t *My_R_Link = My_Chassis->Leg_Unit[R_Leg]->Link;
    Straight_Leg_t *R_Straight = My_Chassis->Leg_Unit[R_Leg]->Straight;
    Straight_Leg_t *L_Straight = My_Chassis->Leg_Unit[L_Leg]->Straight;
    // 双腿协调Tp力
    Chassis_Leg_Sync_Cal(My_Chassis);
    // 计算腿重力引起的力矩，让自救摆腿更丝滑
    Cal_Leg_Gravity_Torque(My_Chassis);

    My_Chassis->Leg_Unit[R_Leg]->force->Tp_LQR = R_Straight->get_Tp(R_Straight);
    My_Chassis->Leg_Unit[L_Leg]->force->Tp_LQR = L_Straight->get_Tp(L_Straight);

    if (My_Chassis->mode == C_Rescue) // 离地时关节力接口为Tp_rescue
    {
        if (My_Chassis->rescue_info->rescue_state_mac == RetractLegs) // 收腿过程不控腿角
        {
            My_Chassis->Leg_Unit[R_Leg]->force->Tp_target = 0;
            My_Chassis->Leg_Unit[L_Leg]->force->Tp_target = 0;
        }
        else if (My_Chassis->rescue_info->rescue_state_mac == PRNormalBackwardLeg) // 后摆这一步才加上腿部重力前馈
        {
            My_Chassis->Leg_Unit[R_Leg]->force->Tp_target = My_Chassis->Leg_Unit[R_Leg]->force->Tp_rescue +
                                                            LEG_GRAVITY_COMPENSATION_ORDER_CORRECT * My_Chassis->Leg_Unit[R_Leg]->force->Leg_Gravity_Torque;
            My_Chassis->Leg_Unit[L_Leg]->force->Tp_target = My_Chassis->Leg_Unit[L_Leg]->force->Tp_rescue +
                                                            LEG_GRAVITY_COMPENSATION_ORDER_CORRECT * My_Chassis->Leg_Unit[L_Leg]->force->Leg_Gravity_Torque;
        }
        else // 用力自救时就直接用控速的Tp_rescue
        {
            My_Chassis->Leg_Unit[R_Leg]->force->Tp_target = My_Chassis->Leg_Unit[R_Leg]->force->Tp_rescue;
            My_Chassis->Leg_Unit[L_Leg]->force->Tp_target = My_Chassis->Leg_Unit[L_Leg]->force->Tp_rescue;
        }
    }
    else if (My_Chassis->mode == C_Manual_Rescue) // 手动自救模式：直接使用Tp_rescue
    {
        My_Chassis->Leg_Unit[R_Leg]->force->Tp_target = My_Chassis->Leg_Unit[R_Leg]->force->Tp_rescue;
        My_Chassis->Leg_Unit[L_Leg]->force->Tp_target = My_Chassis->Leg_Unit[L_Leg]->force->Tp_rescue;
    }
    else if (Balance.Flag->Jumping_Flag == false ||
             (My_Chassis->Leg_Unit[R_Leg]->off_ground == false && My_Chassis->Leg_Unit[L_Leg]->off_ground == false)) // 在地面上时保持LQR和双腿协调的Tp
    {
        My_Chassis->Leg_Unit[R_Leg]->force->Tp_target = R_TP_LQR_ORDER_CORRECT * My_Chassis->Leg_Unit[R_Leg]->force->Tp_LQR + My_Chassis->Leg_Unit[R_Leg]->force->Tp_sync;
        My_Chassis->Leg_Unit[L_Leg]->force->Tp_target = L_TP_LQR_ORDER_CORRECT * My_Chassis->Leg_Unit[L_Leg]->force->Tp_LQR + My_Chassis->Leg_Unit[L_Leg]->force->Tp_sync;
        //   My_Chassis->Leg_Unit[R_Leg]->force->Tp_target = R_TP_LQR_ORDER_CORRECT * My_Chassis->Leg_Unit[R_Leg]->force->Tp_LQR;
        //   My_Chassis->Leg_Unit[L_Leg]->force->Tp_target = L_TP_LQR_ORDER_CORRECT * My_Chassis->Leg_Unit[L_Leg]->force->Tp_LQR ;
    }
    else // 离地只保持双腿协调Tp
    {
        My_Chassis->Leg_Unit[R_Leg]->force->Tp_target = My_Chassis->Leg_Unit[R_Leg]->force->Tp_sync;
        My_Chassis->Leg_Unit[L_Leg]->force->Tp_target = My_Chassis->Leg_Unit[L_Leg]->force->Tp_sync;
    }
}
/**
 * @brief  求Fbl_target，含部分模式特殊处理
 * @param  None
 * @retval None
 */
static void Chassis_Fbl_target_Cal(Chassis_t *My_Chassis)
{
    /*roll控制力计算*/
    Chassis_Roll_Control(My_Chassis); // 含离地处理

    /*腿长控制力计算*/
    Chassis_Leg_Length_Strength_Cal(My_Chassis);

    /*前馈计算*/
    Chassis_Link_Feedforward_Cal(My_Chassis); // 含离地处理

    Chassis_Leg_Fbl_Cal(My_Chassis); // 含跳跃和离地处理
}

/**
 * @brief  求Fbl_target，含部分模式特殊处理
 * @param  None
 * @retval None
 */
static void Chassis_Tw_target_Cal(Chassis_t *My_Chassis)
{
    Link_t *My_L_Link = My_Chassis->Leg_Unit[L_Leg]->Link;
    Link_t *My_R_Link = My_Chassis->Leg_Unit[R_Leg]->Link;
    Straight_Leg_t *R_Straight = My_Chassis->Leg_Unit[R_Leg]->Straight;
    Straight_Leg_t *L_Straight = My_Chassis->Leg_Unit[L_Leg]->Straight;
    /*驱动轮转向环Tw_turn*/
    Chassis_Wheel_Turn_Cal(My_Chassis);
    My_Chassis->Leg_Unit[R_Leg]->force->Tw_LQR = R_Straight->get_Tw(R_Straight);
    My_Chassis->Leg_Unit[L_Leg]->force->Tw_LQR = L_Straight->get_Tw(L_Straight);
    /* 驱动轮电机最终输出 */
    // 特殊模式两轮都置零
    if (My_Chassis->mode == C_Rescue || My_Chassis->mode == C_Manual_Rescue ||
        (My_Chassis->mode == C_KNEE_STRIKE && My_Chassis->knee_strike_info->KNEE_STRIKE_step == Knee_RETRACT))
    {
        My_Chassis->Leg_Unit[R_Leg]->force->Tw_target = 0;
        My_Chassis->Leg_Unit[L_Leg]->force->Tw_target = 0;
    }
    else // 左右腿离地分别轮子力矩置0
    {
#ifdef Power_limit
        // My_Chassis->Leg_Unit[R_Leg]->force->Tw_turn *= My_Chassis->Power_Limit_info->k_turn_power_limit;
        // My_Chassis->Leg_Unit[L_Leg]->force->Tw_turn *= My_Chassis->Power_Limit_info->k_turn_power_limit;
#endif
        // 右腿离地处理
        if (My_Chassis->Leg_Unit[R_Leg]->off_ground == true)
        {
            My_Chassis->Leg_Unit[R_Leg]->force->Tw_target = 0;
        }
        else // 正常情况正常赋值
        {
            My_Chassis->Leg_Unit[R_Leg]->force->Tw_target = My_Chassis->Leg_Unit[R_Leg]->force->Tw_LQR + My_Chassis->Leg_Unit[R_Leg]->force->Tw_turn;
        }

        // 左腿离地处理
        if (My_Chassis->Leg_Unit[L_Leg]->off_ground == true)
        {
            My_Chassis->Leg_Unit[L_Leg]->force->Tw_target = 0;
        }
        else // 正常情况正常赋值
        {
            My_Chassis->Leg_Unit[L_Leg]->force->Tw_target = My_Chassis->Leg_Unit[L_Leg]->force->Tw_LQR + My_Chassis->Leg_Unit[L_Leg]->force->Tw_turn;
        }
    }
    /*---------- 磕膝上台阶防超功率限幅处理 begin ---------*/
    static uint32_t knee_strike_exit_timer = 500;
    if (My_Chassis->mode == C_KNEE_STRIKE)
    {
        knee_strike_exit_timer = 0;
    }
    else if (knee_strike_exit_timer < 500)
    {
        knee_strike_exit_timer++;
    }

    // 轮毂力矩特殊模式限幅 (退出C_KNEE_STRIKE后500ms内才执行)
    if (knee_strike_exit_timer < 500)
    {
        My_Chassis->Leg_Unit[R_Leg]->force->Tw_target = constrain(My_Chassis->Leg_Unit[R_Leg]->force->Tw_target, -3.5f, 3.5f);
        My_Chassis->Leg_Unit[L_Leg]->force->Tw_target = constrain(My_Chassis->Leg_Unit[L_Leg]->force->Tw_target, -3.5f, 3.5f);
    }
    /*---------- 磕膝上台阶防超功率限幅处理 end ---------*/

#ifdef Power_limit
    My_Chassis->Leg_Unit[R_Leg]->force->Tw_target *= My_Chassis->Power_Limit_info->k_turn_power_limit;
    My_Chassis->Leg_Unit[L_Leg]->force->Tw_target *= My_Chassis->Power_Limit_info->k_turn_power_limit;
#endif
}
/**
 * @brief  在所有数据更新后，计算关节、驱动轮所需力矩，含部分标志位、特殊模式处理
 * @param  None
 * @retval None
 */
static void Chassis_Torque_Cal(Chassis_t *My_Chassis)
{

    Link_t *My_L_Link = My_Chassis->Leg_Unit[L_Leg]->Link;
    Link_t *My_R_Link = My_Chassis->Leg_Unit[R_Leg]->Link;
    Straight_Leg_t *R_Straight = My_Chassis->Leg_Unit[R_Leg]->Straight;
    Straight_Leg_t *L_Straight = My_Chassis->Leg_Unit[L_Leg]->Straight;

    /*直腿模型计算，得到驱动轮输出力矩和虚拟关节力矩*/
    R_Straight->LQR_cal(R_Straight);
    L_Straight->LQR_cal(L_Straight);

    Chassis_Tp_target_Cal(My_Chassis);  // 求虚拟关节力
    Chassis_Fbl_target_Cal(My_Chassis); // 求竖直力
    Chassis_Tw_target_Cal(My_Chassis);  // 求轮子目标力矩

    // 正VMC力输入
    My_R_Link->tar_data_update(My_R_Link, My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target, My_Chassis->Leg_Unit[R_Leg]->force->Tp_target);
    My_L_Link->tar_data_update(My_L_Link, My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target, My_Chassis->Leg_Unit[L_Leg]->force->Tp_target);

    // VMC求关节力矩
    My_L_Link->torque_cal(My_L_Link);
    My_R_Link->torque_cal(My_R_Link);
    /*-----------------------------------关节电机最终输出 begin--------------------------------*/

    // 自救过程中除了收腿那一步其余都不用加氮气弹簧前馈
    if (My_Chassis->mode == C_Rescue && (My_Chassis->rescue_info->rescue_state_mac != RetractLegs ||
                                         My_Chassis->rescue_info->rescue_state_mac != RESET))
    {
        My_Chassis->Leg_Unit[R_Leg]->force->Sd_F_Torque = My_R_Link->info->F_Sd_Output_Torque;
        My_Chassis->Leg_Unit[R_Leg]->force->Sd_B_Torque = My_R_Link->info->B_Sd_Output_Torque;
        My_Chassis->Leg_Unit[L_Leg]->force->Sd_F_Torque = My_L_Link->info->F_Sd_Output_Torque;
        My_Chassis->Leg_Unit[L_Leg]->force->Sd_B_Torque = My_L_Link->info->B_Sd_Output_Torque;
    }
    else // 其他情况都加上氮气弹簧前馈
    {
        My_Chassis->Leg_Unit[R_Leg]->force->Sd_F_Torque = My_R_Link->info->F_Sd_Output_Torque + FRONT_SPRING_COMPENSATION_ORDER_CORRECT * My_Chassis->Leg_Unit[R_Leg]->force->T_Spring_Compensation_Front;
        My_Chassis->Leg_Unit[R_Leg]->force->Sd_B_Torque = My_R_Link->info->B_Sd_Output_Torque + BACK_SPRING_COMPENSATION_ORDER_CORRECT * My_Chassis->Leg_Unit[R_Leg]->force->T_Spring_Compensation_Back;
        My_Chassis->Leg_Unit[L_Leg]->force->Sd_F_Torque = My_L_Link->info->F_Sd_Output_Torque + FRONT_SPRING_COMPENSATION_ORDER_CORRECT * My_Chassis->Leg_Unit[L_Leg]->force->T_Spring_Compensation_Front;
        My_Chassis->Leg_Unit[L_Leg]->force->Sd_B_Torque = My_L_Link->info->B_Sd_Output_Torque + BACK_SPRING_COMPENSATION_ORDER_CORRECT * My_Chassis->Leg_Unit[L_Leg]->force->T_Spring_Compensation_Back;
    }
    /*-----------------------------------关节电机最终输出 end--------------------------------*/
}

/**
 * @brief  沿腿方向力前馈补偿计算
 * @param  Link_Var_t* Link_Var
 * @retval None
 */
float k_inertial = 1.5f;
static void Chassis_Link_Feedforward_Cal(Chassis_t *My_Chassis)
{
    Link_t *R_Link_Var = My_Chassis->Leg_Unit[R_Leg]->Link;
    Link_t *L_Link_Var = My_Chassis->Leg_Unit[L_Leg]->Link;
    State_info_t *R_Straight_info = My_Chassis->Leg_Unit[R_Leg]->Straight->info;

    /*重力前馈——右腿*/
    // if (My_Chassis->Leg_Unit[R_Leg]->off_ground == true ||
    //     My_Chassis->mode == C_Rescue) // 离地了取消机体重力前馈
    bool both_offground_flag = (My_Chassis->Leg_Unit[R_Leg]->off_ground == true && My_Chassis->Leg_Unit[L_Leg]->off_ground == true);

    if (My_Chassis->mode == C_Rescue ||
        My_Chassis->Leg_Unit[R_Leg]->off_ground == true && both_offground_flag == false) // 单腿离地取消那边重力前馈
    {
        My_Chassis->Leg_Unit[R_Leg]->force->F_gravity = (R_Link_Var->info->centroid->centriod_coefficient * m_l) * g * cos(R_Link_Var->info->angle->vir_phi0);
    }
    else // 正常运动
    {
        My_Chassis->Leg_Unit[R_Leg]->force->F_gravity = (0.5f * mb + R_Link_Var->info->centroid->centriod_coefficient * m_l) * g * cos(R_Link_Var->info->angle->vir_phi0);
    }

    /*重力前馈——左腿*/
    // if (My_Chassis->Leg_Unit[L_Leg]->off_ground == true ||
    //     My_Chassis->mode == C_Rescue) // 离地了取消机体重力前馈
    if (My_Chassis->mode == C_Rescue ||
        My_Chassis->Leg_Unit[L_Leg]->off_ground == true && both_offground_flag == false) // 单腿离地取消那边重力前馈
    {
        My_Chassis->Leg_Unit[L_Leg]->force->F_gravity = (L_Link_Var->info->centroid->centriod_coefficient * m_l) * g * cos(L_Link_Var->info->angle->vir_phi0);
    }
    else // 正常运动
    {
        My_Chassis->Leg_Unit[L_Leg]->force->F_gravity = (0.5f * mb + L_Link_Var->info->centroid->centriod_coefficient * m_l) * g * cos(L_Link_Var->info->angle->vir_phi0);
    }

    // 双腿离地加重力前馈让腿伸长
    // 退出KNEE_STRIKE后1.5s内不触发
    // 跳跃过程不触发
    if (both_offground_flag == true &&
        My_Chassis->mode != C_KNEE_STRIKE &&
        My_Chassis->mode != C_Jump)
    {
        if (My_Chassis->knee_strike_info->knee_strike_exit_tick < 1500)
        {
            My_Chassis->knee_strike_info->knee_strike_exit_tick++;
        }
        else
        {
            My_Chassis->Leg_Unit[R_Leg]->force->F_gravity = (0.5f * mb + R_Link_Var->info->centroid->centriod_coefficient * m_l) * g * cos(R_Link_Var->info->angle->vir_phi0);
            My_Chassis->Leg_Unit[L_Leg]->force->F_gravity = (0.5f * mb + L_Link_Var->info->centroid->centriod_coefficient * m_l) * g * cos(L_Link_Var->info->angle->vir_phi0);
        }
    }

    /*侧向力前馈*/
    My_Chassis->Leg_Unit[R_Leg]->force->F_inertial = R_F_INERTIAL_ORDER_CORRECT * ((0.5f * mb + R_Link_Var->info->centroid->centriod_coefficient * m_l) * (R_Link_Var->info->length->l0 / (2.f * Rl)) * My_Chassis->Posture->info->yaw_v * R_Straight_info->sd1) * k_inertial;
    My_Chassis->Leg_Unit[L_Leg]->force->F_inertial = L_F_INERTIAL_ORDER_CORRECT * ((0.5f * mb + L_Link_Var->info->centroid->centriod_coefficient * m_l) * (L_Link_Var->info->length->l0 / (2.f * Rl)) * My_Chassis->Posture->info->yaw_v * R_Straight_info->sd1) * k_inertial;
}

/**
 * @brief  计算腿部竖直方向力（期望输出）
 * @param  None
 * @retval None
 */
static void Chassis_Leg_Fbl_Cal(Chassis_t *My_Chassis)
{
    static float t = 0;
    /*在自救中，除了RetractLegs和RESET状态外都不控腿长*/
    if (My_Chassis->mode == C_Rescue &&
        My_Chassis->rescue_info->rescue_state_mac != RetractLegs &&
        My_Chassis->rescue_info->rescue_state_mac != RESET)
    {
        My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target = 0;
        My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target = 0;
    }
    /* 手动自救模式：F_bl_target直接等于F_Manual_Rescue */
    else if (My_Chassis->mode == C_Manual_Rescue)
    {
        My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target = My_Chassis->Leg_Unit[R_Leg]->force->F_Manual_Rescue;
        My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target = My_Chassis->Leg_Unit[L_Leg]->force->F_Manual_Rescue;
    }

    /* 正常运动 */
    else if (Balance.Flag->Jumping_Flag == false &&
             (My_Chassis->Leg_Unit[R_Leg]->off_ground == false || My_Chassis->Leg_Unit[L_Leg]->off_ground == false))
    {
        t = 1;
        My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target = My_Chassis->Leg_Unit[R_Leg]->force->F
                                                          //+ My_Chassis->Leg_Unit[R_Leg]->force->F_roll //roll控制器直接输出到腿长目标值
                                                          + My_Chassis->Leg_Unit[R_Leg]->force->F_gravity + My_Chassis->Leg_Unit[R_Leg]->force->F_inertial;
        My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target = My_Chassis->Leg_Unit[L_Leg]->force->F
                                                          //+ My_Chassis->Leg_Unit[L_Leg]->force->F_roll //roll控制器直接输出到腿长目标值
                                                          + My_Chassis->Leg_Unit[L_Leg]->force->F_gravity + My_Chassis->Leg_Unit[L_Leg]->force->F_inertial;
    }
    /*离地时不加侧向补偿力*/
    else if (Balance.Flag->Jumping_Flag == true || (My_Chassis->Leg_Unit[R_Leg]->off_ground == true &&
                                                    My_Chassis->Leg_Unit[L_Leg]->off_ground == true))
    {
        My_Chassis->Leg_Unit[R_Leg]->force->F_bl_target = My_Chassis->Leg_Unit[R_Leg]->force->F + My_Chassis->Leg_Unit[R_Leg]->force->F_gravity;
        My_Chassis->Leg_Unit[L_Leg]->force->F_bl_target = My_Chassis->Leg_Unit[L_Leg]->force->F + My_Chassis->Leg_Unit[L_Leg]->force->F_gravity;
    }
}

/**
 * @brief  roll控制
 * @param  Link_Var_t* Link_Var
 * @retval 力F
 * @note  右腿减左腿加
 */
static void Chassis_Roll_Control(Chassis_t *My_Chassis)
{

    My_Chassis->chassis_PID->roll_cal[R_Leg]->measure = My_Chassis->Posture->info->roll;
    My_Chassis->chassis_PID->roll_cal[L_Leg]->measure = My_Chassis->Posture->info->roll;
    My_Chassis->chassis_PID->roll_cal[R_Leg]->target = My_Chassis->target->roll;
    My_Chassis->chassis_PID->roll_cal[L_Leg]->target = My_Chassis->target->roll;

    pid_err_cal(My_Chassis->chassis_PID->roll_cal[R_Leg]);
    single_pid_ctrl(My_Chassis->chassis_PID->roll_cal[R_Leg]);

    pid_err_cal(My_Chassis->chassis_PID->roll_cal[L_Leg]);
    single_pid_ctrl(My_Chassis->chassis_PID->roll_cal[L_Leg]);
    if (My_Chassis->Leg_Unit[R_Leg]->off_ground != true)
    {
        My_Chassis->Leg_Unit[R_Leg]->force->F_roll = R_TP_Roll_ORDER_CORRECT * My_Chassis->chassis_PID->roll_cal[R_Leg]->out;
    }
    else
    {
        My_Chassis->Leg_Unit[R_Leg]->force->F_roll = 0;
    }

    if (My_Chassis->Leg_Unit[L_Leg]->off_ground != true)
    {
        My_Chassis->Leg_Unit[L_Leg]->force->F_roll = L_TP_Roll_ORDER_CORRECT * My_Chassis->chassis_PID->roll_cal[L_Leg]->out;
    }
    else
    {
        My_Chassis->Leg_Unit[L_Leg]->force->F_roll = 0;
    }
}

/**
 * @brief  Phi1由前关节电机编码器值转换得到
 * @param  Link_e My_Link_e 杆的左右标志位
 * @retval 转换后的角度phi1
 */
static float My_Phi1_Transform(Leg_e My_Leg_e, Motor_DM_t *my_motor)
{
    float phi1 = 0.f;

    if (My_Leg_e == R_Leg)
    {
        phi1 = R_F_TIME * my_motor->rx_info->motor_angle + R_F_HORIZON_ANGLE_ORDER_CORRECT * R_F_HORIZON_ANGLE;
    }
    else if (My_Leg_e == L_Leg)
    {
        phi1 = L_F_TIME * my_motor->rx_info->motor_angle + L_F_HORIZON_ANGLE_ORDER_CORRECT * L_F_HORIZON_ANGLE;
    }

    if (phi1 > PI)
    {
        phi1 -= 2 * PI;
    }
    if (phi1 < -PI)
    {
        phi1 += 2 * PI;
    }

    return phi1;
}

/**
 * @brief  Phi4由后关节电机编码器值转换得到
 * @param  Link_e My_Link_e 杆的左右标志位
 * @retval 转换后的角度phi4
 */
static float My_Phi4_Transform(Leg_e My_Leg_e, Motor_DM_t *my_motor)
{
    float phi4 = 0.f;

    if (My_Leg_e == R_Leg)
    {
        phi4 = R_B_TIME * my_motor->rx_info->motor_angle + R_B_HORIZON_ANGLE_ORDER_CORRECT * R_B_HORIZON_ANGLE;
    }
    else if (My_Leg_e == L_Leg)
    {
        phi4 = L_B_TIME * my_motor->rx_info->motor_angle + L_B_HORIZON_ANGLE_ORDER_CORRECT * L_B_HORIZON_ANGLE;
    }

    if (phi4 > PI)
    {
        phi4 -= 2 * PI;
    }
    if (phi4 < -PI)
    {
        phi4 += 2 * PI;
    }

    return phi4;
}

/**
 * @brief  底盘电机组离线检测
 * @param  Chassis_t* My_Chassis, 底盘
 * @retval None
 */
static void Chassis_Motor_Group_Offline_Check(Chassis_t *My_Chassis)
{
    if (My_Chassis->Sd->motor[0]->state->status == DEV_OFFLINE ||
        My_Chassis->Sd->motor[2]->state->status == DEV_OFFLINE ||
        My_Chassis->Sd->motor[3]->state->status == DEV_OFFLINE ||
        My_Chassis->Sd->motor[1]->state->status == DEV_OFFLINE)
    {
        My_Chassis->state->sd_state = DEV_OFFLINE;
    }
    else
    {
        My_Chassis->state->sd_state = DEV_ONLINE;
    }
    if (My_Chassis->Wheel->motor[0]->state->status == DEV_OFFLINE ||
        My_Chassis->Wheel->motor[1]->state->status == DEV_OFFLINE)
    {
        My_Chassis->state->wheel_state = DEV_OFFLINE;
    }
    else
    {
        My_Chassis->state->wheel_state = DEV_ONLINE;
    }
}

/**
 * @brief  底盘转向目标值控制，角速度积分成角度
 * @param  Chassis_t* My_Chassis
 */
static void Chassis_Yaw_Target_Process_All(Chassis_t *My_Chassis)
{
    // 切C_Boss时同步yaw_degree为当前IMU角度，避免遗留值导致乱转
    if (My_Chassis->last_mode != My_Chassis->mode &&
        My_Chassis->mode == C_Boss)
    {
        My_Chassis->target->yaw_degree = My_Chassis->Posture->info->yaw_degree;
    }
    // 其他模式切机械模式时锁yaw方向用
    My_Chassis->last_mode = My_Chassis->mode;

    switch (My_Chassis->mode)
    {
    case C_Sleep:
        My_Chassis->target->yaw_v_degree = 0;
        break;

    case C_Follow:
    case C_KNEE_STRIKE:
    case C_JUMP_THEN_KNEE_STRIKE:
    case C_JUMP_AND_MID:
    case C_RTS: {
        static uint32_t Chassis_Alignment_tick;
        static bool last_chassis_alignment_flag = false;

        // 上升沿检测：Chassis_Alignment_Flag从false变为true时，重置tick
        if (Balance.Flag->Chassis_Alignment_Flag == true && last_chassis_alignment_flag == false)
        {
            Chassis_Alignment_tick = 0;
        }
        last_chassis_alignment_flag = Balance.Flag->Chassis_Alignment_Flag;

        if (Balance.Flag->Chassis_Alignment_Flag == true)
        {
            Chassis_Alignment_tick++;
            My_Chassis->target->yaw_degree = 0;
            if (my_abs(My_Chassis->Posture->info->yaw_degree - My_Chassis->target->yaw_degree) <= 0.1f ||
                Chassis_Alignment_tick >= 1000)
            {
                Balance.Flag->Chassis_Alignment_Flag = false;
            }
        }
        // 改成Chassis_Follow_Mode_e case
        else
        {
            if (my_abs(gimbal.base_info.yaw_motor_angle) >= PI / 2.f)
            {
                My_Chassis->target->yaw_degree = sgn(gimbal.base_info.yaw_motor_angle) * 180.f;
            }
            else
            {
                My_Chassis->target->yaw_degree = 0;
            }
        }
        break;
    }

    case C_Boss:
        // 首次进来保证顺滑，主要转头时候用
        if (My_Chassis->last_mode != C_Boss)
        {
            My_Chassis->target->yaw_degree = My_Chassis->Posture->info->yaw_degree;
        }

        if (Balance.ctrl != KEY_CTRL)
        {
            My_Chassis->target->yaw_v_degree = -((float)My_Chassis->rc_input->ch0_now / 660.f) * MAX_SPIN_SPEED;
        }
        else
        {
            My_Chassis->target->yaw_v_degree = -((float)My_Chassis->key_input->keyboard_lateral_speed / 660.f) * MAX_SPIN_SPEED;
        }

        break;

    case C_Test:
        My_Chassis->target->yaw_v_degree = -((float)My_Chassis->rc_input->ch0_now / 660.f) * MAX_SPIN_SPEED;
        break;

    default:
        break;
    }

    if (fabsf(My_Chassis->target->yaw_v_degree) >= MAX_SPIN_SPEED / 660.f / 10.f)
    {
        My_Chassis->target->yaw_degree += My_Chassis->target->yaw_v_degree * TIME_STEP;
        My_Chassis->target->yaw_degree = half_cycle(My_Chassis->target->yaw_degree, 360.f);
    }
}
/**
 * @brief  主要限幅、恢复腿长用，腿长目标值在特殊命令/模式内部处理
 * @param  Chassis_t* My_Chassis
 * @retval None
 */
static void Chassis_Leg_Length_Target_Process(Chassis_t *My_Chassis)
{
#define MID_LEG_LOWING_SPEED 0.13f            // 中腿长模式下降沿时的腿长递减速度，单位m/s
#define KNEE_STRIKE_EXIT_DECREMENT_SPEED 0.3f // 磕膝退出时的腿长递减速度，单位m/s
#define MID_LEG_RISING_SPEED 0.25f            // 中腿长模式上升沿时的腿长递增速度，单位m/s
    static bool last_knee_strike_flag = false;
    static bool knee_strike_exit_decrement_active = false;

    // 特殊模式切回正常模式时恢复正常腿长
    if ((My_Chassis->mode == C_Sleep ||
         ((My_Chassis->mode == C_Follow || My_Chassis->mode == C_Boss) &&
          My_Chassis->last_mode != C_Follow && My_Chassis->last_mode != C_Boss)) &&
        knee_strike_exit_decrement_active == false)
    {
        My_Chassis->target->leg_length_l = TAR_LEG_LENGTH_INITIAL;
        My_Chassis->target->leg_length_r = TAR_LEG_LENGTH_INITIAL;
    }
    if (My_Chassis->mode == C_Cycle && My_Chassis->last_mode != C_Cycle)
    {
        My_Chassis->target->leg_length_l = My_Chassis->etc_config->cycle_length_l_target;
        My_Chassis->target->leg_length_r = My_Chassis->etc_config->cycle_length_r_target;
    }

    // 遥控器可控腿长标志位
    if (Balance.Flag->Leg_length_ctrl_Flag == true)
    {
        My_Chassis->target->leg_length_l += ((float)My_Chassis->rc_input->ch1_now / 660.f) * MAX_LIFT_SPEED * TIME_STEP;
        My_Chassis->target->leg_length_r += ((float)My_Chassis->rc_input->ch1_now / 660.f) * MAX_LIFT_SPEED * TIME_STEP;
    }

    /* -------------------- 退出磕膝上台阶模式时相关处理  begin-------------------- */

    bool current_knee_strike_flag = Balance.Flag->KNEE_STRIKE_Flag;

    // KNEE_STRIKE_Flag下降沿触发递减模式
    if (last_knee_strike_flag == true && current_knee_strike_flag == false)
    {
        knee_strike_exit_decrement_active = true;
    }
    // 磕膝退出后的递减处理：逐步回到初始腿长
    if (knee_strike_exit_decrement_active == true)
    {
        My_Chassis->target->leg_length_r -= 0.001f * KNEE_STRIKE_EXIT_DECREMENT_SPEED;
        My_Chassis->target->leg_length_l -= 0.001f * KNEE_STRIKE_EXIT_DECREMENT_SPEED;

        if (My_Chassis->target->leg_length_r <= TAR_LEG_LENGTH_INITIAL)
        {
            My_Chassis->target->leg_length_r = TAR_LEG_LENGTH_INITIAL;
            My_Chassis->target->leg_length_l = TAR_LEG_LENGTH_INITIAL;
            knee_strike_exit_decrement_active = false;
        }
    }
    /* -------------------- 退出磕膝上台阶模式时相关处理  end-------------------- */

    /* -------------------- 中腿长相关处理  begin-------------------- */
    /* Middle_Flag处理: 中腿长模式 (上升沿递增缓冲，下降沿递减缓冲) */
    {
        static uint32_t middle_flag_enter_tick = 0;      // 记录进入Middle_Flag时的时间戳
        static bool off_ground_increment_active = false; // 递增模式活跃标志
        static bool off_ground_decrement_active = false; // 递减模式活跃标志
        static bool last_both_off_ground = false;        // 上一次两腿离地状态
        static bool last_middle_flag = false;            // 上一次Middle_Flag状态，用于边缘检测

        bool current_middle_flag = Balance.Flag->Middle_Flag;
        bool current_both_off_ground = (My_Chassis->Leg_Unit[L_Leg]->off_ground == true &&
                                        My_Chassis->Leg_Unit[R_Leg]->off_ground == true);

        /* 上升沿: Middle_Flag从0变1时，触发递增缓冲并开始计时 */
        if (current_middle_flag == true && last_middle_flag == false)
        {
            middle_flag_enter_tick = HAL_GetTick();
            off_ground_increment_active = true;
            off_ground_decrement_active = false;
        }

        /* 下降沿: Middle_Flag从1变0时，触发递减缓冲，不直接跳回初始腿长 */
        if (current_middle_flag == false && last_middle_flag == true)
        {
            off_ground_increment_active = false;
            off_ground_decrement_active = true;
            middle_flag_enter_tick = 0;
        }

        /* 超时15s: 强制退出中腿长模式 */
        if (current_middle_flag == true && (HAL_GetTick() - middle_flag_enter_tick) > 15000)
        {
            My_Chassis->target->leg_length_r = TAR_LEG_LENGTH_INITIAL;
            My_Chassis->target->leg_length_l = TAR_LEG_LENGTH_INITIAL;
            off_ground_increment_active = false;
            off_ground_decrement_active = false;
            Balance.Flag->Middle_Flag = false;
            middle_flag_enter_tick = 0;
        }

        /* Middle_Flag == true时: 检测双腿下降沿(都离地→都触地) → 触发递减缓冲 */
        if (current_middle_flag == true)
        {
            /* 下降沿: 两腿从离地变触地 → 触发递减缓冲，Middle_Flag置false */
            if (current_both_off_ground == false && last_both_off_ground == true &&
                off_ground_decrement_active == false)
            {
                Balance.Flag->Middle_Flag = false;
                off_ground_increment_active = false;
                off_ground_decrement_active = true;
            }
        }

        /* 递增处理: 仅在off_ground_increment_active为true时递增 */
        if (off_ground_increment_active == true)
        {
            My_Chassis->target->leg_length_r += 0.001f * MID_LEG_RISING_SPEED;
            My_Chassis->target->leg_length_l += 0.001f * MID_LEG_RISING_SPEED;
        }

        /* 腿长已到中腿长 → 退出递增模式 */
        if (off_ground_increment_active == true && My_Chassis->target->leg_length_r >= MID_LEG_LENGTH)
        {
            My_Chassis->target->leg_length_r = MID_LEG_LENGTH;
            My_Chassis->target->leg_length_l = MID_LEG_LENGTH;
            off_ground_increment_active = false;
        }

        /* 递减处理: 仅在off_ground_decrement_active为true时递减 */
        if (off_ground_decrement_active == true && current_both_off_ground == false)
        {
            My_Chassis->target->leg_length_r -= 0.001f * MID_LEG_LOWING_SPEED;
            My_Chassis->target->leg_length_l -= 0.001f * MID_LEG_LOWING_SPEED;
        }

        /* 腿长已恢复到初始值 → 退出递减模式 */
        if (off_ground_decrement_active == true && My_Chassis->target->leg_length_r <= TAR_LEG_LENGTH_INITIAL)
        {
            My_Chassis->target->leg_length_r = TAR_LEG_LENGTH_INITIAL;
            My_Chassis->target->leg_length_l = TAR_LEG_LENGTH_INITIAL;
            off_ground_decrement_active = false;
        }
        /* -------------------- 中腿长相关处理  end-------------------- */
        /* 更新上一次状态 */
        last_both_off_ground = current_both_off_ground;
        last_middle_flag = current_middle_flag;
    }

    /*限制腿长极限值*/
    if (My_Chassis->target->leg_length_l > MAX_LEG_LENGTH)
    {
        My_Chassis->target->leg_length_l = MAX_LEG_LENGTH;
    }
    else if (My_Chassis->target->leg_length_l < MIN_LEG_LENGTH)
    {

        My_Chassis->target->leg_length_l = MIN_LEG_LENGTH;
    }

    if (My_Chassis->target->leg_length_r > MAX_LEG_LENGTH)
    {
        My_Chassis->target->leg_length_r = MAX_LEG_LENGTH;
    }
    else if (My_Chassis->target->leg_length_r < MIN_LEG_LENGTH)
    {

        My_Chassis->target->leg_length_r = MIN_LEG_LENGTH;
    }

    // 更新上一拍KNEE_STRIKE标志位
    last_knee_strike_flag = current_knee_strike_flag;
}

/**
 * @brief  由控制层到电机层，将计算出来的扭矩赋给相应电机
 * @param  None
 * @retval None
 */
static void Chassis_Set_Torque(Chassis_t *My_Chassis)
{
    My_Chassis->Sd->motor[R_F_Sd_M]->tx_info->torque = My_Chassis->Leg_Unit[R_Leg]->force->Sd_F_Torque * R_F_ORDER_CORRECT;
    My_Chassis->Sd->motor[R_B_Sd_M]->tx_info->torque = My_Chassis->Leg_Unit[R_Leg]->force->Sd_B_Torque * R_B_ORDER_CORRECT;
    My_Chassis->Sd->motor[L_F_Sd_M]->tx_info->torque = My_Chassis->Leg_Unit[L_Leg]->force->Sd_F_Torque * L_F_ORDER_CORRECT;
    My_Chassis->Sd->motor[L_B_Sd_M]->tx_info->torque = My_Chassis->Leg_Unit[L_Leg]->force->Sd_B_Torque * L_B_ORDER_CORRECT;

    My_Chassis->Wheel->motor[R_WHEEL_M]->tx_info->torque = My_Chassis->Leg_Unit[R_Leg]->force->Tw_target * R_W_ORDER_CORRECT;
    My_Chassis->Wheel->motor[L_WHEEL_M]->tx_info->torque = My_Chassis->Leg_Unit[L_Leg]->force->Tw_target * L_W_ORDER_CORRECT;
}

/**
 * @brief  电机层目标力矩赋0，不发送
 */
static void Chassis_Motor_Set_Sleep(Chassis_t *My_Chassis)
{
    My_Chassis->Sd->motor[R_F_Sd_M]->tx_info->torque = 0;
    My_Chassis->Sd->motor[R_B_Sd_M]->tx_info->torque = 0;
    My_Chassis->Sd->motor[L_F_Sd_M]->tx_info->torque = 0;
    My_Chassis->Sd->motor[L_B_Sd_M]->tx_info->torque = 0;

    My_Chassis->Wheel->motor[R_WHEEL_M]->tx_info->torque = 0;
    My_Chassis->Wheel->motor[L_WHEEL_M]->tx_info->torque = 0;
}

/**
 * @brief  读取并处理遥控器（RC）输入通道值
 * @param  Chassis_t* My_Chassis
 * @retval None
 * @note  从rc_sensor读取四个通道的原始值（ch0~ch3），对关键通道做步进限幅滤波后存入rc_input结构体。
 */
static void Chassis_Rc_Input_Update(Chassis_t *My_Chassis)
{
    Chassis_Rc_Input_t *rc_input = My_Chassis->rc_input;

    /* 通道0：偏航（左右转向） */
    // 读取原始值后经过步进限幅，限制每周期最大变化量为2，
    // 避免转向指令突变导致车身侧翻
    rc_input->ch0_now = rc_sensor.info->ch0;
    rc_input->ch0_now = step_limit_filter(rc_input->ch0_now, rc_input->ch0_last, 2);
    rc_input->ch0_last = rc_input->ch0_now;

    rc_input->ch1_now = rc_sensor.info->ch1;

    /* 通道2：腿长控制 */
    rc_input->ch2_now = rc_sensor.info->ch2;

    /* 通道3：前后直行速度 */
    rc_input->ch3_now = rc_sensor.info->ch3;
    rc_input->ch3_now = step_limit_filter(rc_input->ch3_now, rc_input->ch3_last, 2);
    rc_input->ch3_last = rc_input->ch3_now;
}

/**
 * @brief  读取并处理键盘（WASD）输入
 * @param  Chassis_t* My_Chassis
 * @retval None
 * @note  W/S控制前后速度（>0前进），A/D控制横向速度（>0向左）。
 *        前后向有步进限幅滤波防止启动跳变，横向无滤波响应更直接。
 */
static void Chassis_Key_Input_Update(Chassis_t *My_Chassis)
{
    Chassis_Key_Input_t *key_input = My_Chassis->key_input;

    /* 前后向速度：W-S差值，步进限幅防止启动突变 */
    key_input->keyboard_forward_speed = (rc_sensor.info->W.cnt - rc_sensor.info->S.cnt);
    if (abs(key_input->keyboard_forward_speed - key_input->keyboard_forward_speed_last) > 1)
    {
        key_input->keyboard_forward_speed = 2.0 * sgn(key_input->keyboard_forward_speed - key_input->keyboard_forward_speed_last) + key_input->keyboard_forward_speed_last;
        key_input->keyboard_forward_speed_last = key_input->keyboard_forward_speed;
    }
    else
    {
        key_input->keyboard_forward_speed_last = key_input->keyboard_forward_speed;
    }

    /* 横向速度：A-D差值，无限幅（>0表示向左） */
    key_input->keyboard_lateral_speed = (rc_sensor.info->A.cnt - rc_sensor.info->D.cnt);
}

/**
 * @brief  sd1目标值处理，含各种特殊模式下的处理以及部分s处理
 * @param  Chassis_t* My_Chassis, 底盘
 */
static void Chassis_sd1_Target_Update(Chassis_t *My_Chassis)
{
#define MAX_CYCLE_MOVE_SPEED 1.3f         // 小陀螺模式下的最大斜向平移速度（m/s）
    static bool r_offground_prev = false; // 上一次右腿离地状态
    static bool l_offground_prev = false; // 上一次左腿离地状态
    if (My_Chassis->mode == C_Cycle)
    {
        /* -------------------- 小陀螺模式下的斜向平移速度计算 -------------------- */
        // 小陀螺模式时，底盘在自旋的同时需要平移运动。
        // 此时将操作手的横向和纵向输入，按当前云台朝向角进行矢量分解，
        // 得到车身斜向的目标速度，再将x/y分量合并为差速轮需要的sd1。
        float target_x, target_y;         // 车身坐标系下x(前向)和y(横向)的目标速度分量
        float move_speed_x, move_speed_y; // 经过角度分解后的速度分量
        float move_angle, angle_err;      // 云台相对车身的偏角（用于矢量分解）

        // 取云台yaw电机相对底盘的角度作为运动分解角
        angle_err = gimbal.base_info.yaw_motor_angle;
        // 半圈处理
        if (fabs(angle_err) >= PI)
        {
            angle_err -= sgn(angle_err) * 2 * PI;
        }
        move_angle = angle_err;

        // 根据控制方式（遥控器/键盘）读取操作手输入，并映射为目标速度
        if (Balance.ctrl == RC_CTRL)
        {
            // 遥控器：ch3是前后通道，ch2是左右通道
            target_x = -(My_Chassis->rc_input->ch2_now / 660.f) * MAX_CYCLE_MOVE_SPEED;
            target_y = -(My_Chassis->rc_input->ch3_now / 660.f) * MAX_CYCLE_MOVE_SPEED;
        }
        else if (Balance.ctrl == KEY_CTRL)
        {
            // 键盘：W前进、S后退、A左移、D右移
            target_x = (My_Chassis->key_input->keyboard_lateral_speed / KEY_A_CNT_MAX) * MAX_CYCLE_MOVE_SPEED;
            target_y = -(My_Chassis->key_input->keyboard_forward_speed / KEY_W_CNT_MAX) * MAX_CYCLE_MOVE_SPEED;
        }
#ifndef IS_VARY_CYCLE
        target_x = -target_x; // 根据实际测试，键盘控制下前后左右的正负方向与遥控器相反，这里做一次修正
        target_y = -target_y;
#endif

        move_speed_x = target_x * arm_cos_f32(move_angle);
        move_speed_y = target_y * arm_sin_f32(move_angle);

        My_Chassis->target->sd1 = move_speed_x + move_speed_y;
    }
    /* -------------------- 普通模式下的直行速度 -------------------- */
    else
    {

        // 普通模式：直接根据操作手的前后输入和云台朝向计算直行速度目标
        // 注意：此处只读取了"前后"通道ch3（或键盘W/S），不含横向通道，
        // 因为直行模式不允许斜向移动，横向输入由云台转向来体现

        if (Balance.ctrl != KEY_CTRL) // 遥控器控制
        {
            // 通道归一化后映射到最大直行速度，带方向修正系数
            My_Chassis->target->sd1 = RC_INPUT_SD1_ORDER_CORRECT * ((float)My_Chassis->rc_input->ch3_now / 660.f) * MAX_STRAIGHT_SPEED;
        }
        else // 键盘控制
        {
            My_Chassis->target->sd1 = RC_INPUT_SD1_ORDER_CORRECT * ((float)My_Chassis->key_input->keyboard_forward_speed / KEY_W_CNT_MAX) * MAX_STRAIGHT_SPEED;
        }

        /* -------------------- 换头处理 -------------------- */
        // 例如：云台朝后，此时操作手"前进"实际上应该是车身后退
        if (fabsf(gimbal.base_info.yaw_motor_angle) >= PI / 2)
        {
            My_Chassis->target->sd1 *= -1;
        }
    }
    /* -------------------- 直行下的目标速度限制 begin -------------------- */

/* -------------------- 功率限制 -------------------- */
#ifdef Power_limit
    Chassis_Power_Limit();
    // 飞坡时不进行功率限制
    if (Balance.Flag->Middle_Flag != true)
    {
        My_Chassis->target->sd1 = My_Chassis->target->sd1 * My_Chassis->target->limit_v / MAX_STRAIGHT_SPEED;
    }
#endif

    /* ------------ 中腿长 飞坡/下台阶 最大目标速度限制(偏小) ------------ */
    if (Balance.Flag->Middle_Flag == true)
    {
        My_Chassis->target->sd1 = constrain(My_Chassis->target->sd1,
                                            -My_Chassis->etc_config->Mid_Leg_Max_Speed,
                                            My_Chassis->etc_config->Mid_Leg_Max_Speed);
    }

    /* ------------ 磕膝上台阶 最大目标速度限制(偏小) ------------ */
    if (Balance.Flag->KNEE_STRIKE_Flag == true)
    {
        My_Chassis->target->sd1 = constrain(My_Chassis->target->sd1,
                                            -1.9f,
                                            1.9f);
    }

    /* ------------ 飞坡或下台阶时sd1给0防止过冲,落地后限制目标速度 ------------ */
    static bool mid_offground_low_speed_flag;
    static uint16_t mid_offground_low_speed_tick;
    if (Balance.Flag->Middle_Flag == true &&
        My_Chassis->Leg_Unit[R_Leg]->off_ground == true &&
        My_Chassis->Leg_Unit[L_Leg]->off_ground == true)
    {
        My_Chassis->target->sd1 = 0;
        mid_offground_low_speed_flag = true;
    }
    if (mid_offground_low_speed_flag == true)
    {
        mid_offground_low_speed_tick++;
        if (mid_offground_low_speed_tick >= 700)
        {
            mid_offground_low_speed_flag = false;
            mid_offground_low_speed_tick = 0;
        }
        My_Chassis->target->sd1 = constrain(My_Chassis->target->sd1, 0.f, 1.2f);
    }

    // 最大速度限幅处理(冗余)
    My_Chassis->target->sd1 = constrain(My_Chassis->target->sd1, -MAX_STRAIGHT_SPEED, MAX_STRAIGHT_SPEED);

    /* -------------------- 直行下的目标速度限制 end -------------------- */

    /* ------------------------------ 重置位移 begin ------------------------------ */
    {
        static uint32_t sd1_zero_start_tick = 0;   // 记录sd1变为0的时刻
        static bool sd1_zero_s_reset_done = false; // 标志位：sd1为0时重置位移是否已完成
        static float last_sd1 = 0.0f;

        // sd1从非0变为0时，记录时刻并重置标志位
        if (My_Chassis->target->sd1 == 0.0f && last_sd1 != 0.0f)
        {
            sd1_zero_start_tick = HAL_GetTick();
            sd1_zero_s_reset_done = false;
        }

        // sd1为0超过一定时间时，设置目标位移为当前位移（仅触发一次）
        if (My_Chassis->target->sd1 == 0.0f &&
            sd1_zero_s_reset_done == false &&
            (HAL_GetTick() - sd1_zero_start_tick) > 600)
        {
            My_Chassis->target->s = My_Chassis->Leg_Unit[R_Leg]->Straight->info->s;
            sd1_zero_s_reset_done = true;
        }

        last_sd1 = My_Chassis->target->sd1;
    }

    // 当存在较大幅度的前后移动指令时，重置位移
    {
        if ((float)fabsf(My_Chassis->rc_input->ch3_now / 660.f) >= 0.2f ||
            (float)fabsf(My_Chassis->key_input->keyboard_forward_speed / 660.f) >= 0.2f ||
            // 任意一只腿从离地变为触地时（off_ground下降沿），重置目标位移
            (r_offground_prev == true && My_Chassis->Leg_Unit[R_Leg]->off_ground == false) ||
            (l_offground_prev == true && My_Chassis->Leg_Unit[L_Leg]->off_ground == false) ||
            // 小陀螺模式下重置位移
            My_Chassis->mode == C_Cycle)
        {
            // 重置目标位移
            My_Chassis->target->s = My_Chassis->Leg_Unit[L_Leg]->Straight->info->s;
        }
    }

    // 更新离地状态记录（用于下次检测下降沿）
    r_offground_prev = My_Chassis->Leg_Unit[R_Leg]->off_ground;
    l_offground_prev = My_Chassis->Leg_Unit[L_Leg]->off_ground;
    /* ------------------------------ 重置位移 end ------------------------------ */
}

static int binarySearchClosestLess(float arr[], int size, float target, float mean);

/**
 * @brief  底盘速度限幅，防止侧向惯性力过大导致翻车
 * @param  My_Chassis: 底盘结构体指针
 * @retval None
 * @note  通过限制轮腿合成加速度来防止惯性力超过地面提供的最大摩擦力
 *        F_acc: 前进方向加速度产生的惯性力
 *        Fw:    转向时车轮切向加速度产生的惯性力
 *        Ff_max: 地面最大静摩擦力(由支撑力F_support决定)
 *        当Ff_cal(合成惯性力) > Ff_max时,说明侧向惯性力可能超过地面附着力,需要限速
 */
static void Chassis_Speed_Limit(Chassis_t *My_Chassis)
{

    float F_acc, Fw, Ff_cal, Ff_max;
    static float limit_coe_arr[10] = {0.f, 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 0.9f}; // 速度限制系数查表
    float limit_coe = 1.f;

    /**
     * 分支1: yaw_v_degree * sd1 > 0
     * 说明: 前进方向与小陀螺自旋方向同向(车体整体向斜前方/斜后方运动)
     * 此时主要用左腿的支撑力来计算最大摩擦力
     */
    if (My_Chassis->target->yaw_v_degree * My_Chassis->target->sd1 > 0)
    {
        /* sd1>0时用较大的摩擦系数0.2, sd1<0时用较小的0.06 */
        if (My_Chassis->target->sd1 > 0)
            Ff_max = My_Chassis->Leg_Unit[L_Leg]->force->F_support * 0.2f;
        else
            Ff_max = My_Chassis->Leg_Unit[L_Leg]->force->F_support * 0.06f;
        /* 计算前进方向惯性力: 质量 * 前进加速度 */
        F_acc = My_Chassis->Posture->info->x_world * mw;
        /* 计算转向惯性力: 质量 * 轮速 * 转向角速度 */
        Fw = mw * My_Chassis->Leg_Unit[L_Leg]->Straight->info->sd1 * My_Chassis->Posture->info->yaw_v;
        /* 计算合成惯性力大小 */
        arm_sqrt_f32((F_acc * F_acc + Fw * Fw), &Ff_cal);
        if (Ff_cal > Ff_max)
        {
            /* 查找最接近的限制系数,对sd1进行限幅 */
            limit_coe = limit_coe_arr[binarySearchClosestLess(limit_coe_arr, 10, Ff_max, Ff_cal)];
            My_Chassis->target->sd1 = My_Chassis->Leg_Unit[L_Leg]->Straight->info->sd1 * limit_coe;
        }
    }
    /**
     * 分支2: yaw_v_degree * sd1 < 0
     * 说明: 前进方向与小陀螺自旋方向相反(车体整体向斜前方/斜后方运动,但自旋方向相反)
     * 此时主要用右腿的支撑力来计算最大摩擦力
     */
    else if (My_Chassis->target->yaw_v_degree * My_Chassis->target->sd1 < 0)
    {
        /* sd1>0时用较大的摩擦系数0.2, sd1<0时用较小的0.06 */
        if (My_Chassis->target->sd1 > 0)
            Ff_max = My_Chassis->Leg_Unit[L_Leg]->force->F_support * 0.2f;
        else
            Ff_max = My_Chassis->Leg_Unit[L_Leg]->force->F_support * 0.06f;
        /* 计算前进方向惯性力 */
        F_acc = My_Chassis->Posture->info->x_world * mw;
        /* 计算转向惯性力(使用右腿轮速) */
        Fw = mw * My_Chassis->Leg_Unit[R_Leg]->Straight->info->sd1 * My_Chassis->Posture->info->yaw_v;
        /* 计算合成惯性力大小 */
        arm_sqrt_f32((F_acc * F_acc + Fw * Fw), &Ff_cal);
        if (Ff_cal > Ff_max)
        {
            /* 查找最接近的限制系数,对sd1进行限幅 */
            limit_coe = limit_coe_arr[binarySearchClosestLess(limit_coe_arr, 10, Ff_max, Ff_cal)];
            //			My_Chassis->target->yaw_v_degree *= limit_coe;
            My_Chassis->target->sd1 = My_Chassis->Leg_Unit[R_Leg]->Straight->info->sd1 * limit_coe;
        }
    }
}

static int binarySearchClosestLess(float arr[], int size, float target, float mean)
{
    int low = 0;
    int high = size - 1;
    int result = 0; // 用于记录满足条件的最后一个索引

    while (low <= high)
    {
        int mid = low + (high - low) / 2; // 防止溢出

        // 如果当前元素小于 target，则记录下索引，并向右边继续查找更大的数
        if ((arr[mid] * mean) < target)
        {
            result = mid;
            low = mid + 1;
        }
        // 如果 arr[mid] 大于等于 target，则需要向左侧寻找
        else
        {
            high = mid - 1;
        }
    }
    return result;
}

/*小陀螺/变速小陀螺*/
// 宏定义：切换匀速/变速小陀螺模式
static void Cycle_Target_Process(Chassis_t *My_Chassis)
{
    Chassis_cycle_info_t *cycle = My_Chassis->cycle_info;

    if (My_Chassis->mode == C_Cycle)
    {

#ifdef IS_VARY_CYCLE
        /* ==================== 变速小陀螺模式 ==================== */
        // 周期总时长 (ms)
        uint16_t total_cycle_duration = cycle->vary_accel_duration + cycle->vary_const_duration + cycle->vary_decel_duration;

        // 时间在周期中的位置
        uint16_t time_in_cycle = cycle->vary_cycle_tick % total_cycle_duration;

        // 根据时间区间计算目标速度
        if (time_in_cycle < cycle->vary_accel_duration)
        {
            // 0 ~ 加速阶段：线性加速从min_speed到max_speed
            float accel_ratio = (float)time_in_cycle / cycle->vary_accel_duration; // 范围0~1
            cycle->target_yaw_v_degree = cycle->vary_min_speed + (cycle->vary_max_speed - cycle->vary_min_speed) * accel_ratio;
        }
        else if (time_in_cycle < cycle->vary_accel_duration + cycle->vary_const_duration)
        {
            // 加速阶段结束 ~ 恒速阶段结束：恒定最大速度
            cycle->target_yaw_v_degree = cycle->vary_max_speed;
        }
        else
        {
            // 恒速阶段结束 ~ 周期结束：线性减速从max_speed到min_speed
            float decel_time = time_in_cycle - cycle->vary_accel_duration - cycle->vary_const_duration;
            float decel_ratio = decel_time / cycle->vary_decel_duration; // 范围0~1
            cycle->target_yaw_v_degree = cycle->vary_max_speed - (cycle->vary_max_speed - cycle->vary_min_speed) * decel_ratio;
        }

        // 周期计时递增 (TIME_STEP=1ms)
        cycle->vary_cycle_tick += 1;

#else
        /* ==================== 匀速小陀螺模式 ==================== */
        // 负号决定旋转方向
        if (cap.info.cap_u >= cycle->high_cap_threshold)
        {
            cycle->target_yaw_v_degree = -cycle->high_cap_constant_speed;
        }
        else
        {
            cycle->target_yaw_v_degree = -cycle->low_cap_constant_speed;
        }

#endif
    }
}

/*氮气弹簧动态前馈*/
// 推导过程：https://robotpilots.feishu.cn/wiki/POwzwqmCniP2aJkYUyScjoflnSe
void My_Spring_Former_Input_Cal(Chassis_t *My_Chassis)
{
    Spring_compensation_info_t *info = My_Chassis->spring_c_info;
    float angle_sigma; // 动态量：膝关节腿与虚拟小腿的夹角（单位：弧度） = PI-phi1 + phi2
    float angle_alpha; // 动态量：大腿、膝关节腿夹角的一半 = (phi1-phi4)/2
    float angle_theta; // 动态量：起点在轮毂、平行于大腿的线与虚拟小腿的夹角（单位：弧度） = PI-α-σ
    float angle_sigma_R = PI -
                          My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->phi1 +
                          My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->phi2;
    float angle_alpha_R = (My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->phi1 -
                           My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->phi4) /
                          2.f;
    float angle_beta_R = PI - angle_alpha_R - angle_sigma_R;
    float angle_theta_R = angle_alpha_R - angle_beta_R;
    // 腿长越小，系数越大，范围0~1
    float leg_length_ratio_R = constrain((MAX_LEG_LENGTH - My_Chassis->Leg_Unit[R_Leg]->Link->info->length->l0) /
                                             (MAX_LEG_LENGTH - MIN_LEG_LENGTH),
                                         0.0f, 1.0f);

    float angle_sigma_L = PI -
                          My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->phi1 +
                          My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->phi2;
    float angle_alpha_L = (My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->phi1 -
                           My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->phi4) /
                          2.f;
    float angle_beta_L = PI - angle_alpha_L - angle_sigma_L;
    float angle_theta_L = angle_alpha_L - angle_beta_L;
    // 腿长越小，系数越大，范围0~1
    float leg_length_ratio_L = constrain((MAX_LEG_LENGTH - My_Chassis->Leg_Unit[L_Leg]->Link->info->length->l0) /
                                             (MAX_LEG_LENGTH - MIN_LEG_LENGTH),
                                         0.0f, 1.0f);

    My_Chassis->Leg_Unit[R_Leg]->force->T_Spring_Compensation_Front = info->k_compensation_knee_joint_leg * leg_length_ratio_R +
                                                                      info->springForce * info->Knee_joint_leg_length *
                                                                          (info->m_length / info->n_length) *
                                                                          (info->j_length / info->k_length) *
                                                                          arm_sin_f32(angle_sigma_R) * arm_cos_f32(angle_theta_R);
    My_Chassis->Leg_Unit[R_Leg]->force->T_Spring_Compensation_Back = info->d_BigLeg * info->springForce * arm_sin_f32(info->angle_cauchy);

    My_Chassis->Leg_Unit[L_Leg]->force->T_Spring_Compensation_Front = info->k_compensation_knee_joint_leg * leg_length_ratio_L +
                                                                      info->springForce * info->Knee_joint_leg_length *
                                                                          (info->m_length / info->n_length) *
                                                                          (info->j_length / info->k_length) *
                                                                          arm_sin_f32(angle_sigma_L) * arm_cos_f32(angle_theta_L);
    My_Chassis->Leg_Unit[L_Leg]->force->T_Spring_Compensation_Back = info->d_BigLeg * info->springForce * arm_sin_f32(info->angle_cauchy);
}

/*功率限制*/
float k_power = 14.f; // 功率上限系数，越大相当于允许的功率越大
static void Chassis_Power_Limit(void)
{
    //	static float a = 0,Nf = 0,kk;
    //	static float power_limit = 60.f;
    /**----------------------前后运动功率限制 begin------------------------- */
    Link_t *Link;
    if (fabsf(Chassis.Leg_Unit[L_Leg]->Link->info->force->Tp_target) >= fabsf(Chassis.Leg_Unit[R_Leg]->Link->info->force->Tp_target))
    {
        Power_Limit.Tp_Big = fabsf(Chassis.Leg_Unit[L_Leg]->Link->info->force->Tp_target);
        Link = Chassis.Leg_Unit[L_Leg]->Link;
    }
    else
    {
        Power_Limit.Tp_Big = fabsf(Chassis.Leg_Unit[R_Leg]->Link->info->force->Tp_target);
        Link = Chassis.Leg_Unit[R_Leg]->Link;
    }

    Power_Limit.Nf = (k_power * My_Judge.info->chassis_power_limit / 24.f * _3508_SELFMADE_TORQUE_CONSTANT / WHEEL_RADIUS) -
                     ((2 * Power_Limit.Tp_Big / Chassis.target->leg_length_l) * fabsf(arm_cos_f32(Link->info->angle->vir_phi0))) -
                     (float)(mb * g * (Chassis.target->leg_length_l) * fabsf(arm_sin_f32(Link->info->angle->vir_phi0))) - (float)(mb * g * fabsf(arm_cos_f32(Link->info->angle->vir_phi0)) * fabsf(arm_sin_f32(Link->info->angle->vir_phi0)));

    Power_Limit.kk = arm_sin_f32(Link->info->angle->vir_phi0);

    Power_Limit.a = (float)Power_Limit.Nf / mb; // 质量得换整车的

    Chassis.target->limit_v = Chassis.target->limit_v + Power_Limit.a * TIME_STEP;

    if (Chassis.target->limit_v >= MAX_STRAIGHT_SPEED)
    {
        Chassis.target->limit_v = MAX_STRAIGHT_SPEED;
    }

    if (Chassis.target->limit_v <= 0.4) // 阈值得调
    {
        Chassis.target->limit_v = 0.4;
    }
    /**----------------------前后运动功率限制 end------------------------- */

    /**----------------------转向功率限制系数计算 begin------------------------- */
    float k_buffer;
    float k_cap;
    float chassis_power_buffer = My_Judge.info->chassis_power_buffer;
    // 根据裁判系统是否在线来决定是否根据缓冲能量限制
    if (My_Judge.status->status == DEV_ONLINE)
    {
        k_buffer = constrain(My_Judge.info->chassis_power_buffer / 60.f, 0.f, 1.f);
    }
    else
    {
        k_buffer = 1;
    }
    // 超电剩余容量比例
    if (cap.state == DEV_ONLINE)
    {
        k_cap = constrain(cap.info.cap_u / 24.6f, 0.f, 1.f);
    }
    else
    {
        k_cap = 1.f;
    }

    if (chassis_power_buffer > Power_Limit.turn_limit_high_buffer_threshold)
    {
        Power_Limit.k_buffer_limit = k_buffer;
    }
    else if (chassis_power_buffer > Power_Limit.turn_limit_mid_buffer_threshold)
    {
        Power_Limit.k_buffer_limit = k_buffer;
    }
    else if (chassis_power_buffer > Power_Limit.turn_limit_low_buffer_threshold)
    {
        Power_Limit.k_buffer_limit = k_buffer * k_buffer * k_buffer;
    }
    Power_Limit.k_cap_limit = constrain(k_cap * Power_Limit.k_cap_capacity_scale_limit, 0.5f, 1.f);
    Power_Limit.k_turn_power_limit = Power_Limit.k_cap_limit * Power_Limit.k_buffer_limit; // 总限制比例=电池容量限制比例*缓冲能量限制比例

    /**----------------------转向功率限制系数计算 end------------------------- */
}

/**
 * @brief  自救状态机处理，主要对Tp_rescue赋值
 * @author LYQ
 */
static const char *Rescue_State_Name(int state);

static void Rescue_State_Process(Chassis_t *My_Chassis)
{
    // 减少变量命名长度，方便阅读
    float theta_R_degree = My_Chassis->Leg_Unit[R_Leg]->Straight->info->thetal / Degree_to_rad;
    float theta_L_degree = My_Chassis->Leg_Unit[L_Leg]->Straight->info->thetal / Degree_to_rad;
    Chassis_Rescue_info_t *rescue_info = My_Chassis->rescue_info;
    Chassis_Posture_info_t *posture_info = My_Chassis->Posture->info;
    Chassis_Rescue_config_t *rescue_config = &rescue_info->rescue_config;
    Link_t *R_Link = My_Chassis->Leg_Unit[R_Leg]->Link;
    Link_t *L_Link = My_Chassis->Leg_Unit[L_Leg]->Link;
    float R_Leg_length = R_Link->info->length->l0;
    float L_Leg_length = L_Link->info->length->l0;
    float pitch_degree = posture_info->pitch_degree;
    float roll_degree = posture_info->roll_degree;

    static int last_state = -1;
    int current_state = rescue_info->rescue_state_mac;
    if (last_state != current_state)
    {
        RP_LOG_INFO("Rescue Enter: %s, pitch=%.2f, roll=%.2f, thetaR=%.2f, thetaL=%.2f",
                    Rescue_State_Name(current_state), pitch_degree, roll_degree, theta_R_degree, theta_L_degree);
        last_state = current_state;
    }

    switch (rescue_info->rescue_state_mac)
    {
    case ForwardFlip_and_L_Rollover:

        // 换pid参数，换之前清积分
        pid_clear(My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg]);
        pid_clear(My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg]);
        My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg] = &ForwardFlip_vir_phi0_d1_Pid[R_Leg];
        My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg] = &ForwardFlip_vir_phi0_d1_Pid[L_Leg];
        My_Chassis->target->vir_phi0d1_r_degree = rescue_config->phi0ForwardFlipSpeed;
        My_Chassis->target->vir_phi0d1_l_degree = 0;
        uint32_t elapsed_ms = HAL_GetTick() - rescue_info->stateMacineTimelineTick;
        uint8_t by_timeout = (elapsed_ms >= rescue_config->RolloverTimeoutPeriod);
        if (
            // 超时
            by_timeout ||
            // roll到位了,比一个负数大说明没那么负，也就是比较正
            (roll_degree > rescue_config->rollLeftRolloverThreshold))
        {

            RP_LOG_INFO("Rescue Transition: %s -> %s, reason=%s, elapsed=%lu, roll=%.2f, threshold=%.2f",
                        Rescue_State_Name(rescue_info->rescue_state_mac), Rescue_State_Name(ForwardFlip),
                        by_timeout ? "timeout" : "roll_threshold",
                        (unsigned long)elapsed_ms, roll_degree, rescue_config->rollLeftRolloverThreshold);

            rescue_info->stateMacineTimelineTick = HAL_GetTick();
            rescue_info->rescue_state_mac = ForwardFlip;

            if (by_timeout)
            {
                RP_LOG_WARN("L_Rollover Timeout");
            }
            else
            {
                RP_LOG_INFO("L_Rollover rescue succeed");
            }
        }
        break;

    case ForwardFlip_and_R_Rollover:

        // 换pid参数，换之前清积分
        pid_clear(My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg]);
        pid_clear(My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg]);
        My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg] = &ForwardFlip_vir_phi0_d1_Pid[R_Leg];
        My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg] = &ForwardFlip_vir_phi0_d1_Pid[L_Leg];
        My_Chassis->target->vir_phi0d1_r_degree = 0;
        My_Chassis->target->vir_phi0d1_l_degree = rescue_config->phi0ForwardFlipSpeed;
        elapsed_ms = HAL_GetTick() - rescue_info->stateMacineTimelineTick;
        by_timeout = (elapsed_ms >= rescue_config->RolloverTimeoutPeriod);
        if (
            // 超时
            by_timeout ||
            // roll到位了,比一个正数小说明没那么正，也就是比较小
            (roll_degree < rescue_config->rollRightRolloverThreshold))
        {

            RP_LOG_INFO("Rescue Transition: %s -> %s, reason=%s, elapsed=%lu, roll=%.2f, threshold=%.2f",
                        Rescue_State_Name(rescue_info->rescue_state_mac), Rescue_State_Name(ForwardFlip),
                        by_timeout ? "timeout" : "roll_threshold",
                        (unsigned long)elapsed_ms, roll_degree, rescue_config->rollRightRolloverThreshold);

            rescue_info->stateMacineTimelineTick = HAL_GetTick();
            rescue_info->rescue_state_mac = ForwardFlip;

            if (by_timeout)
            {
                RP_LOG_WARN("R_Rollover Timeout");
            }
            else
            {
                RP_LOG_INFO("R_Rollover rescue succeed");
            }
        }
        break;

    case BackwardFlip_and_L_Rollover:
    case L_Rollover:
        // 后翻且左侧翻时，左腿后摆
        pid_clear(My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg]);
        pid_clear(My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg]);
        My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg] = &PRNormalBackwardLeg_vir_phi0_d1_Pid[R_Leg];
        My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg] = &PRNormalBackwardLeg_vir_phi0_d1_Pid[L_Leg];
        My_Chassis->target->vir_phi0d1_r_degree = rescue_config->phi0BackwardFlipSpeed;
        My_Chassis->target->vir_phi0d1_l_degree = 0;
        elapsed_ms = HAL_GetTick() - rescue_info->stateMacineTimelineTick;
        by_timeout = (elapsed_ms >= rescue_config->RolloverTimeoutPeriod);
        if (
            // 超时
            by_timeout ||
            // roll到位了, 比一个负数大说明没那么负，也就是比较正
            (roll_degree > rescue_config->rollLeftRolloverThreshold))
        {

            RP_LOG_INFO("Rescue Transition: %s -> %s, reason=%s, elapsed=%lu, roll=%.2f, threshold=%.2f",
                        Rescue_State_Name(rescue_info->rescue_state_mac), Rescue_State_Name(BackwardFlip),
                        by_timeout ? "timeout" : "roll_threshold",
                        (unsigned long)elapsed_ms, roll_degree, rescue_config->rollLeftRolloverThreshold);

            rescue_info->stateMacineTimelineTick = HAL_GetTick();
            rescue_info->rescue_state_mac = BackwardFlip;

            if (by_timeout)
            {
                RP_LOG_WARN("BackwardFlip_and_L_Rollover Timeout");
            }
            else
            {
                RP_LOG_INFO("BackwardFlip_and_L_Rollover rescue succeed");
            }
        }
        break;

    case BackwardFlip_and_R_Rollover:
    case R_Rollover:
        // 后翻且右侧翻时，右腿后摆
        pid_clear(My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg]);
        pid_clear(My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg]);
        My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg] = &PRNormalBackwardLeg_vir_phi0_d1_Pid[R_Leg];
        My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg] = &PRNormalBackwardLeg_vir_phi0_d1_Pid[L_Leg];
        My_Chassis->target->vir_phi0d1_r_degree = 0;
        My_Chassis->target->vir_phi0d1_l_degree = rescue_config->phi0BackwardFlipSpeed;
        elapsed_ms = HAL_GetTick() - rescue_info->stateMacineTimelineTick;
        by_timeout = (elapsed_ms >= rescue_config->RolloverTimeoutPeriod);
        if (
            // 超时
            by_timeout ||
            // roll到位了,比一个正数小说明正了
            (roll_degree < rescue_config->rollRightRolloverThreshold))
        {

            RP_LOG_INFO("Rescue Transition: %s -> %s, reason=%s, elapsed=%lu, roll=%.2f, threshold=%.2f",
                        Rescue_State_Name(rescue_info->rescue_state_mac), Rescue_State_Name(BackwardFlip),
                        by_timeout ? "timeout" : "roll_threshold",
                        (unsigned long)elapsed_ms, roll_degree, rescue_config->rollRightRolloverThreshold);

            rescue_info->stateMacineTimelineTick = HAL_GetTick();
            rescue_info->rescue_state_mac = BackwardFlip;

            if (by_timeout)
            {
                RP_LOG_WARN("BackwardFlip_and_R_Rollover Timeout");
            }
            else
            {
                RP_LOG_INFO("BackwardFlip_and_R_Rollover rescue succeed");
            }
        }
        break;

    case ForwardFlip:
        // 换pid参数，换之前清积分
        pid_clear(My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg]);
        pid_clear(My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg]);
        My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg] = &ForwardFlip_vir_phi0_d1_Pid[R_Leg];
        My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg] = &ForwardFlip_vir_phi0_d1_Pid[L_Leg];
        if (roll_degree > rescue_config->rollRightRolloverThreshold)
        {

            My_Chassis->target->vir_phi0d1_r_degree = 0;
        }
        else
        {
            My_Chassis->target->vir_phi0d1_r_degree = rescue_config->phi0ForwardFlipSpeed;
        }
        if (roll_degree < rescue_config->rollLeftRolloverThreshold)
        {
            My_Chassis->target->vir_phi0d1_l_degree = 0;
        }
        else
        {
            My_Chassis->target->vir_phi0d1_l_degree = rescue_config->phi0ForwardFlipSpeed;
        }

        elapsed_ms = HAL_GetTick() - rescue_info->stateMacineTimelineTick;
        by_timeout = (elapsed_ms >= rescue_config->ForwardFlipTimeoutPeriod);
        if (
            // 超时
            by_timeout ||
            // abs(pitch_degree)在一个范围内，说明pitch正常了
            (fabsf(pitch_degree) < rescue_config->pitchNormalThreshold))
        {

            RP_LOG_INFO("Rescue Transition: %s -> %s, reason=%s, elapsed=%lu, pitch=%.2f, threshold=%.2f",
                        Rescue_State_Name(rescue_info->rescue_state_mac), Rescue_State_Name(PRNormalBackwardLeg),
                        by_timeout ? "timeout" : "pitch_threshold",
                        (unsigned long)elapsed_ms, pitch_degree, rescue_config->pitchNormalThreshold);

            rescue_info->stateMacineTimelineTick = HAL_GetTick();
            rescue_info->rescue_state_mac = PRNormalBackwardLeg;

            if (by_timeout)
            {
                RP_LOG_WARN("ForwardFlip Timeout");
            }
            else
            {
                RP_LOG_INFO("ForwardFlip rescue succeed");
            }
        }
        break;
    case BackwardFlip:
        // 后翻时，双腿后摆直到机身回正
        pid_clear(My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg]);
        pid_clear(My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg]);
        My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg] = &BackwardFlip_vir_phi0_d1_Pid[R_Leg];
        My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg] = &BackwardFlip_vir_phi0_d1_Pid[L_Leg];
        if (roll_degree > rescue_config->rollRightRolloverThreshold)
        {

            My_Chassis->target->vir_phi0d1_r_degree = 0;
        }
        else
        {
            My_Chassis->target->vir_phi0d1_r_degree = rescue_config->phi0BackwardFlipSpeed;
        }
        if (roll_degree < rescue_config->rollLeftRolloverThreshold)
        {
            My_Chassis->target->vir_phi0d1_l_degree = 0;
        }
        else
        {
            My_Chassis->target->vir_phi0d1_l_degree = rescue_config->phi0BackwardFlipSpeed;
        }
        // My_Chassis->target->vir_phi0d1_r_degree = rescue_config->phi0BackwardFlipSpeed;
        // My_Chassis->target->vir_phi0d1_l_degree = rescue_config->phi0BackwardFlipSpeed;
        elapsed_ms = HAL_GetTick() - rescue_info->stateMacineTimelineTick;
        by_timeout = (elapsed_ms >= rescue_config->BackwardFlipTimeoutPeriod);
        if (
            // 超时
            by_timeout ||
            // abs(pitch_degree)在一个范围内，说明pitch正常了
            (fabsf(pitch_degree) < rescue_config->pitchNormalThreshold))
        {

            RP_LOG_INFO("Rescue Transition: %s -> %s, reason=%s, elapsed=%lu, pitch=%.2f, threshold=%.2f",
                        Rescue_State_Name(rescue_info->rescue_state_mac), Rescue_State_Name(PRNormalBackwardLeg),
                        by_timeout ? "timeout" : "pitch_threshold",
                        (unsigned long)elapsed_ms, pitch_degree, rescue_config->pitchNormalThreshold);

            rescue_info->stateMacineTimelineTick = HAL_GetTick();
            rescue_info->rescue_state_mac = PRNormalBackwardLeg;

            if (by_timeout)
            {
                RP_LOG_WARN("BackwardFlip Timeout");
            }
            else
            {
                RP_LOG_INFO("BackwardFlip rescue succeed");
            }
        }
        break;

    case PRNormalBackwardLeg:
        // Rescue_Check里已经设置时间戳
        // 动作
        // 设置目标速度
        // gimbal.mode=MEC;
        // 换pid参数，换之前清积分
        pid_clear(My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg]);
        pid_clear(My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg]);

        My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg] = &PRNormalBackwardLeg_vir_phi0_d1_Pid[R_Leg];
        My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg] = &PRNormalBackwardLeg_vir_phi0_d1_Pid[L_Leg];
        My_Chassis->target->vir_phi0d1_r_degree = rescue_config->phi0PRNormalBackwardSpeed;
        My_Chassis->target->vir_phi0d1_l_degree = rescue_config->phi0PRNormalBackwardSpeed;
        // 任意一边腿到位了就把这条腿的speed target设0，另一边继续转
        if (theta_R_degree > 0 && theta_R_degree < rescue_config->thetaBackwardRetractLegThreshold)
        {
            My_Chassis->target->vir_phi0d1_r_degree = 0;
        }
        else
        {
            My_Chassis->target->vir_phi0d1_r_degree = rescue_config->phi0PRNormalBackwardSpeed;
        }
        if (theta_L_degree > 0 && theta_L_degree < rescue_config->thetaBackwardRetractLegThreshold)
        {
            My_Chassis->target->vir_phi0d1_l_degree = 0;
        }
        else
        {
            My_Chassis->target->vir_phi0d1_l_degree = rescue_config->phi0PRNormalBackwardSpeed;
        }
        elapsed_ms = HAL_GetTick() - rescue_info->stateMacineTimelineTick;
        by_timeout = (elapsed_ms >= rescue_config->PRNormalBackwardLegTimeoutPeriod);
        // 事件：两条腿都到位了或超时才进入下一状态
        if (by_timeout ||
            (theta_R_degree > 0 && theta_R_degree < rescue_config->thetaBackwardRetractLegThreshold &&
             theta_L_degree > 0 && theta_L_degree < rescue_config->thetaBackwardRetractLegThreshold))
        {
            My_Chassis->target->vir_phi0d1_r_degree = 0;
            My_Chassis->target->vir_phi0d1_l_degree = 0;
            My_Chassis->target->leg_length_l = MIN_LEG_LENGTH;
            My_Chassis->target->leg_length_r = MIN_LEG_LENGTH;

            RP_LOG_INFO("Rescue Transition: %s -> %s, reason=%s, elapsed=%lu, thetaR=%.2f, thetaL=%.2f, threshold=%.2f",
                        Rescue_State_Name(rescue_info->rescue_state_mac), Rescue_State_Name(CorrectGimbalDirection),
                        by_timeout ? "timeout" : "theta_threshold",
                        (unsigned long)elapsed_ms, theta_R_degree, theta_L_degree, rescue_config->thetaBackwardRetractLegThreshold);

            rescue_info->stateMacineTimelineTick = HAL_GetTick();
            rescue_info->rescue_state_mac = CorrectGimbalDirection;

            if (by_timeout)
            {
                RP_LOG_WARN("PRNormalBackwardLeg Timeout");
            }
            else
            {
                RP_LOG_INFO("PRNormalBackwardLeg Succeed");
            }
        }
        break;

    case CorrectGimbalDirection: {
        uint32_t elapsed_ms = HAL_GetTick() - rescue_info->stateMacineTimelineTick;
        bool by_timeout = (elapsed_ms >= rescue_config->CorrectGimbalDirectionTimeoutPeriod);

        if (by_timeout || gimbal.gimbal_reset_state == DEV_RESET_OK)
        {
            RP_LOG_INFO("Rescue Transition: %s -> %s, reason=%s, elapsed=%lu",
                        Rescue_State_Name(rescue_info->rescue_state_mac),
                        Rescue_State_Name(RetractLegs),
                        by_timeout ? "timeout" : "gimbal_reset_ok",
                        (unsigned long)elapsed_ms);

            rescue_info->stateMacineTimelineTick = HAL_GetTick();
            rescue_info->rescue_state_mac = RetractLegs;

            if (by_timeout)
            {
                RP_LOG_WARN("CorrectGimbalDirection Timeout");
            }
            else
            {
                RP_LOG_INFO("CorrectGimbalDirection Succeed");
            }
        }
    }
    break;

    case RetractLegs:
        My_Chassis->target->leg_length_l = MIN_LEG_LENGTH;
        My_Chassis->target->leg_length_r = MIN_LEG_LENGTH;
        // 换回默认摆角pid参数，虽然也用不到
        pid_clear(My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg]);
        pid_clear(My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg]);
        My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg] = &My_Link_vir_phi0_d1_Pid[R_Leg];
        My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg] = &My_Link_vir_phi0_d1_Pid[L_Leg];
        // gimbal.mode=MEC; //云台会判断是否在自救、自救步骤来控制归正
        elapsed_ms = HAL_GetTick() - rescue_info->stateMacineTimelineTick;
        by_timeout = (elapsed_ms >= rescue_config->RetractLegsTimeoutPeriod);
        if (
            // 超时
            by_timeout ||
            // 腿收的差不多了
            (fabsf(R_Leg_length - My_Chassis->target->leg_length_r) < rescue_config->retractLegLengthTolerance &&
             fabsf(L_Leg_length - My_Chassis->target->leg_length_l) < rescue_config->retractLegLengthTolerance))
        {
            My_Chassis->target->leg_length_l = TAR_LEG_LENGTH_INITIAL;
            My_Chassis->target->leg_length_r = TAR_LEG_LENGTH_INITIAL;

            RP_LOG_INFO("Rescue Transition: %s -> %s, reason=%s, elapsed=%lu, dR=%.4f, dL=%.4f, tol=%.4f",
                        Rescue_State_Name(rescue_info->rescue_state_mac), Rescue_State_Name(Reset),
                        by_timeout ? "timeout" : "leg_length_threshold",
                        (unsigned long)elapsed_ms,
                        fabsf(R_Leg_length - My_Chassis->target->leg_length_r),
                        fabsf(L_Leg_length - My_Chassis->target->leg_length_l),
                        rescue_config->retractLegLengthTolerance);

            rescue_info->stateMacineTimelineTick = HAL_GetTick();
            rescue_info->rescue_state_mac = Reset;
            if (by_timeout)
            {
                RP_LOG_WARN("Leg Retraction Timeout");
            }
            else
            {
                RP_LOG_INFO("Leg Retraction Successful");
            }
        }
        break;

    case Reset:
        pid_clear(My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg]);
        pid_clear(My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg]);
        My_Chassis->target->leg_length_l = TAR_LEG_LENGTH_INITIAL;
        My_Chassis->target->leg_length_r = TAR_LEG_LENGTH_INITIAL;
        My_Chassis->Leg_Unit[R_Leg]->force->Tp_rescue = 0;
        My_Chassis->Leg_Unit[L_Leg]->force->Tp_rescue = 0;
        rescue_info->stateMacineTimelineTick = HAL_GetTick();
        Balance.Flag->Rescue_Flag = false;
        RP_LOG_INFO("Rescue Exit: state=%s, reset_state=%d, Rescue_Flag=%d, leg_l=%.3f, leg_r=%.3f, TpR=%.3f, TpL=%.3f",
                    Rescue_State_Name(rescue_info->rescue_state_mac),
                    Balance.reset_state,
                    Balance.Flag->Rescue_Flag,
                    My_Chassis->target->leg_length_l,
                    My_Chassis->target->leg_length_r,
                    My_Chassis->Leg_Unit[R_Leg]->force->Tp_rescue,
                    My_Chassis->Leg_Unit[L_Leg]->force->Tp_rescue);
        break;
    default:
        RP_LOG_WARN("Rescue Unknown State: %d", rescue_info->rescue_state_mac);
        break;
    }
    // 自救过程对腿长力Fbl_target的处理在Chassis_Leg_Fbl_Cal函数
    //  计算Tp输出
    if (rescue_info->rescue_state_mac != RetractLegs && rescue_info->rescue_state_mac != Reset)
    {
        Chassis_Leg_vir_phi0_d1_Cal(My_Chassis);
        My_Chassis->Leg_Unit[R_Leg]->force->Tp_rescue = My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg]->out;
        My_Chassis->Leg_Unit[L_Leg]->force->Tp_rescue = My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg]->out;
    }
    else
    {
        My_Chassis->Leg_Unit[R_Leg]->force->Tp_rescue = 0;
        My_Chassis->Leg_Unit[L_Leg]->force->Tp_rescue = 0;
    }
}

static const char *Rescue_State_Name(int state)
{
    switch (state)
    {
    case ForwardFlip_and_L_Rollover:
        return "ForwardFlip_and_L_Rollover";
    case L_Rollover:
        return "L_Rollover";
    case ForwardFlip_and_R_Rollover:
        return "ForwardFlip_and_R_Rollover";
    case R_Rollover:
        return "R_Rollover";
    case BackwardFlip_and_L_Rollover:
        return "BackwardFlip_and_L_Rollover";
    case BackwardFlip_and_R_Rollover:
        return "BackwardFlip_and_R_Rollover";
    case ForwardFlip:
        return "ForwardFlip";
    case BackwardFlip:
        return "BackwardFlip";
    case PRNormalBackwardLeg:
        return "PRNormalBackwardLeg";
    case CorrectGimbalDirection:
        return "CorrectGimbalDirection";
    case RetractLegs:
        return "RetractLegs";
    case Reset:
        return "Reset";
    default:
        return "Unknown";
    }
}

/**
 * @brief 手动自救模式处理
 * @param My_Chassis 底盘对象指针
 * @note
 *   - 使用Manual_Rescue_vir_phi0_d1_Pid进行腿摆角速度PID计算
 *   - 键盘控制：
 *     F键: 左腿顺时针摆动(速度为负)
 *     D键: 左腿逆时针摆动(速度为正)
 *     S键: 右腿逆时针摆动(速度为正)
 *     A键: 右腿顺时针摆动(速度为负)
 *     Q键: 伸腿(增加目标腿部竖直力)
 *     E键: 收腿(减小目标腿部竖直力)
 */
static void Chassis_Manual_Rescue_Process(Chassis_t *My_Chassis)
{
    Manual_Rescue_Config_t *config = &My_Chassis->manual_rescue_info->config;
    Manual_Rescue_Var_t *var = &My_Chassis->manual_rescue_info->var;

    // 1. 设置PID计算器
    pid_clear(My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg]);
    pid_clear(My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg]);
    My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg] = &Manual_Rescue_vir_phi0_d1_Pid[R_Leg];
    My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg] = &Manual_Rescue_vir_phi0_d1_Pid[L_Leg];

    // 2. 键盘读取目标摆角速度
    float target_phi0d1_r = 0; // 右腿目标速度
    float target_phi0d1_l = 0; // 左腿目标速度

    // F: 左腿顺时针(负), D: 左腿逆时针(正)
    if (rc_sensor.info->F.value == true)
    {
        target_phi0d1_r = -config->phi0d1_speed_abs;
    }
    else if (rc_sensor.info->D.value == true)
    {
        target_phi0d1_r = config->phi0d1_speed_abs;
    }

    // A: 右腿顺时针(负), S: 右腿逆时针(正)
    if (rc_sensor.info->A.value == true)
    {
        target_phi0d1_l = -config->phi0d1_speed_abs;
    }
    else if (rc_sensor.info->S.value == true)
    {
        target_phi0d1_l = config->phi0d1_speed_abs;
    }

    // 设置目标速度
    My_Chassis->target->vir_phi0d1_r_degree = target_phi0d1_r;
    My_Chassis->target->vir_phi0d1_l_degree = target_phi0d1_l;

    // 3. PID计算（使用与Rescue_State_Process相同的计算方式）
    My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg]->target = target_phi0d1_r;
    My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg]->measure = My_Chassis->Leg_Unit[R_Leg]->Link->info->angle->vir_phi0_d1_degree;

    My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg]->target = target_phi0d1_l;
    My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg]->measure = My_Chassis->Leg_Unit[L_Leg]->Link->info->angle->vir_phi0_d1_degree;

    pid_err_cal(My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg]);
    pid_err_cal(My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg]);
    single_pid_ctrl(My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg]);
    single_pid_ctrl(My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg]);

    // 4. 输出摆角力到Tp_rescue
    My_Chassis->Leg_Unit[R_Leg]->force->Tp_rescue = My_Chassis->chassis_PID->vir_phi0d1_cal[R_Leg]->out;
    My_Chassis->Leg_Unit[L_Leg]->force->Tp_rescue = My_Chassis->chassis_PID->vir_phi0d1_cal[L_Leg]->out;

    // 5. 键盘控制腿部竖直力
    // Q/W: 左腿伸/收腿(增加/减小目标力), R/E: 右腿伸/收腿(增加/减小目标力)
    // 左腿
    if (rc_sensor.info->Q.value == true)
    {
        var->F_target_l += config->F_change_speed * 0.001f;
    }
    else if (rc_sensor.info->W.value == true)
    {
        var->F_target_l -= config->F_change_speed * 0.001f;
    }
    // 右腿
    if (rc_sensor.info->R.value == true)
    {
        var->F_target_r += config->F_change_speed * 0.001f;
    }
    else if (rc_sensor.info->E.value == true)
    {
        var->F_target_r -= config->F_change_speed * 0.001f;
    }

    // 限幅
    var->F_target_l = constrain(var->F_target_l, config->F_min, config->F_max);
    var->F_target_r = constrain(var->F_target_r, config->F_min, config->F_max);

    // 6. 输出竖直力到F_Manual_Rescue
    My_Chassis->Leg_Unit[R_Leg]->force->F_Manual_Rescue = var->F_target_r;
    My_Chassis->Leg_Unit[L_Leg]->force->F_Manual_Rescue = var->F_target_l;
}

/**
 * @brief 底盘功率限制预测
 */
static void Chassis_Power_Predict(Chassis_t *My_Chassis)
{
    for (int leg = 0; leg < Leg_Num; leg++)
    {
        // float w = My_Chassis->Wheel->motor[leg]->rx_info->encoder_speed;
        // float i = My_Chassis->Wheel->motor[leg]->tx_info->torque_current_raw;
        float w = My_Chassis->Model_PL[leg]->var.input_var.measure_w;
        float i = My_Chassis->Model_PL[leg]->var.input_var.tx_i;
        Model_Power_Limit_config_t *config = &My_Chassis->Model_PL[leg]->config;
        Model_Power_Limit_var_t *var = &My_Chassis->Model_PL[leg]->var;

        float i2 = i * i;
        float iw = i * w;
        float w2 = w * w;

        var->mid_var.term_i2 = config->k_i2 * i2;
        var->mid_var.term_iw = config->k_iw * iw;
        var->mid_var.term_w2 = config->k_w2 * w2;

        var->output_var.predicted_power = config->a + var->mid_var.term_i2 + var->mid_var.term_iw + var->mid_var.term_w2;
    }
}

/**
 * @brief 根据最大功率计算最大可用轮子力矩
 */
static void Chassis_Wheel_Torque_Limit_Calc(Chassis_t *My_Chassis)
{
    for (int leg = R_Leg; leg < Leg_Num; leg++)
    {
        Model_Power_Limit_config_t *config = &My_Chassis->Model_PL[leg]->config;
        Model_Power_Limit_var_t *var = &My_Chassis->Model_PL[leg]->var;

        float target_power = var->input_var.max_power;
        float w = var->input_var.measure_w;
        float raw_current = var->input_var.tx_i;

        if (target_power < 0)
        {
            var->output_var.limited_i = (int16_t)raw_current;
            var->output_var.limited_torque = raw_current / config->k_torque_to_LSB;
            continue;
        }

        // 求解 i 的二次方程: k_i2*i^2 + k_iw*w*i + (k_w2*w^2 + a - P) = 0
        float A = config->k_i2;
        float B = config->k_iw * w;
        float C = config->k_w2 * w * w + config->a - target_power;
        float discriminant = B * B - 4 * A * C;

        // 吴姐
        if (discriminant < 0)
        {
            var->output_var.limited_i = (int16_t)raw_current;
            var->output_var.limited_torque = raw_current / config->k_torque_to_LSB;
            continue;
        }

        float sqrt_disc = sqrt(discriminant);
        float i_1 = (-B + sqrt_disc) / (2 * A);
        float i_2 = (-B - sqrt_disc) / (2 * A);

        float limited_i;
        if (raw_current > 0)
        {
            limited_i = i_1;
        }
        else if (raw_current < 0)
        {
            limited_i = i_2;
        }
        else
        {
            limited_i = 0;
        }

        var->output_var.limited_i = (int16_t)limited_i;
        var->output_var.limited_torque = limited_i / config->k_torque_to_LSB;
    }
}
/**
 * @brief 功率模型功率限制更新函数
 * @note 在Chassis_Data_Update调用此函数，Chassis_Torque_Cal里会对轮子力矩进行限制
 */
static void Chassis_Model_Power_Limit_Update(Chassis_t *My_Chassis)
{
    float power_predict_sum; // 两轮预测功率和
    for (int leg = 0; leg < Leg_Num; leg++)
    {
        My_Chassis->Model_PL[leg]->var.input_var.measure_i = My_Chassis->Wheel->motor[leg]->rx_info->torque_current_raw;
        My_Chassis->Model_PL[leg]->var.input_var.measure_w = My_Chassis->Wheel->motor[leg]->rx_info->encoder_speed;
        My_Chassis->Model_PL[leg]->var.input_var.tx_i = My_Chassis->Wheel->motor[leg]->tx_info->torque_current_raw;
    }
    Chassis_Power_Predict(My_Chassis); // 计算预测功率

    // 计算两轮预测功率和
    for (int leg = 0; leg < Leg_Num; leg++)
    {
        power_predict_sum += My_Chassis->Model_PL[leg]->var.output_var.predicted_power;
    }

    float power_rate; // 功率限制折算率=底盘功率上限/预测功率和
    if (power_predict_sum > My_Judge.info->chassis_power_limit)
    {
        power_rate = My_Judge.info->chassis_power_limit / power_predict_sum;
    }
    else
    {
        power_rate = 1.0f;
    }

    // 计算各轮最大可用功率
    for (int leg = 0; leg < Leg_Num; leg++)
    {
        My_Chassis->Model_PL[leg]->var.input_var.max_power = power_rate * My_Chassis->Model_PL[leg]->var.output_var.predicted_power;
    }

    Chassis_Wheel_Torque_Limit_Calc(My_Chassis);
}