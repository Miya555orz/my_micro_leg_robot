#ifndef __CHASSIS_H
#define __CHASSIS_H

/* Includes ------------------------------------------------------------------*/
#include "arm_math.h"
#include "rp_config.h"
#include "chassis_motor.h"
#include "gimbal_motor.h"
#include "Chassis_Posture.h"
#include "Link_Instance.h"
#include "Straight_Instance.h"
#include "PID_Instance.h"
#include "PID.h"
#include "Balance.h"
#include "gimbal.h"
#include "RP_Log.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/*底盘模式*/
typedef enum {
    C_Sleep, // 卸力

    C_Follow,        // 跟随
    C_Boss,          // 控制底盘
    C_Rescue,        // 自救
    C_Manual_Rescue, // 手动自救
    C_Cycle,         // 小陀螺
    C_Jump,
    C_KNEE_STRIKE,           // 磕膝上台阶
    C_JUMP_THEN_KNEE_STRIKE, // 跳跃上一级、磕膝上二级
    C_JUMP_AND_MID,          // 跳+中腿长下台阶
    C_Test,                  // 测试模式
    C_SitDown,               // 底盘卸力
    C_RTS                    // 收腿下二级台阶
} Chassis_Mode_e;

typedef enum {

    C_Follow_Straight, // 跟随模式下的普通跟随
    C_Follow_Right,    // 右侧身
    C_Follow_Left,     // 左侧身

} Chassis_Follow_Mode_e;

typedef enum {
    Front,
    Behind,
    L_Front,
    R_Front,
} Chassis_Init_State_e;

typedef enum {
    Chassis_reset_NO,
    Chassis_reset_OK,
} Chassis_reset_state_e;

/*底盘设备状态*/
typedef struct Chassis_state_struct_t {
    dev_work_state_t sd_state;

    dev_work_state_t wheel_state;
} Chassis_state_t;

/*底盘目标值*/
typedef struct Chassis_Target_struct_t {

    float thetal_l;
    float thetal_r;
    float thetal_by_length; // 平均腿长到thetal目标值的线性映射结果

    float vir_phi0_l;          // 弧度制
    float vir_phi0_r;          // 弧度制
    float vir_phi0d1_l;        // 弧度制
    float vir_phi0d1_r;        // 弧度制
    float vir_phi0_l_degree;   // 度数制
    float vir_phi0_r_degree;   // 度数制
    float vir_phi0d1_l_degree; // 度数制
    float vir_phi0d1_r_degree; // 度数制

    float thetald1;

    float s;

    float sd1;

    float thetab;

    float thetabd1;

    float velocity_y; // 小陀螺时使用

    float limit_v;

    float yaw_v_degree;

    float yaw_degree; // 参考量可能是imu角度，也可能是机械角度

    float roll;

    float leg_length_r;
    float leg_length_l;

} Chassis_Target_t;

/*遥控器输入*/
typedef struct Chassis_Rc_Input_struct_t {
    int16_t ch3_now; // 前后

    int16_t ch3_last;

    int16_t ch2_now; // 腿长

    int16_t ch2_last;

    int16_t ch1_now; // 头pitch

    int16_t ch1_last;

    int16_t ch0_now; // 偏航

    int16_t ch0_last;

    float w_now;

    float w_last;

    float a_now;

    float a_last;

    float s_now;

    float s_last;

    float d_now;

    float d_last;

} Chassis_Rc_Input_t;

typedef struct Chassis_Key_Input_struct_t {
    int16_t w_now;

    int16_t w_last;

    int16_t s_now;

    int16_t s_last;

    int16_t keyboard_forward_speed; // 键盘控制时的前进/后退基础移速目标值 (基于W键与S键的差值算得)

    int16_t keyboard_forward_speed_last; // 上一时刻的键盘前进移速，用于步进限幅滤波（减缓速度突变）

    int16_t keyboard_lateral_speed; // 键盘控制时的左/右侧横移基础速度目标值 (基于A键与D键的差值算得)

    int16_t keyboard_lateral_speed_last; // 上一时刻的键盘侧向移速
} Chassis_Key_Input_t;

typedef struct
{
    /*竖直力begin*/
    float F_gravity;       // 腿部、机体重力补偿力
    float F_inertial;      // 侧向惯性补偿力
    float F_roll;          // roll轴补偿力
    float F;               // 保持腿长力,pid,伸腿为正
    float F_support;       // 支持力
    float F_Manual_Rescue; // 手动自救竖直力
    float F_bl_target;     // 合力、最终输出接口,F+F_roll+F_inertial+F_gravity
    /*竖直力end*/

    /*关节力begin*/
    float Tp_sync; // 双腿协调
    float Tp_LQR;
    float Tp_vir_phi0;        // 测试或自救用
    float Tp_vir_phi0_d1;     // 测试或自救用
    float Leg_Gravity_Torque; // 自救时腿部重力力矩补偿
    float Tp_rescue;          // 自救虚拟关节力
    float Tp_target;
    /*关节力end*/

    /*关节限位补偿力，防止撞限位*/
    float Sd_F_Limit_Tor_Fix;
    float Sd_B_Limit_Tor_Fix;

    /*氮气弹簧前馈*/
    float T_Spring_Compensation_Front;
    float T_Spring_Compensation_Back;

    /*关节电机实际输出力矩*/
    float Sd_F_Torque;
    float Sd_B_Torque;

    /*驱动轮力矩begin*/
    float Tw_turn;
    float Tw_LQR;
    float Tw_target;
    /*驱动轮力矩end*/

} Leg_force_t;

typedef struct
{
    float springForce;                   // 氮气弹簧力
    float d_BigLeg;                      // 中轴 到 大腿与氮气弹簧连接点 的距离
    float angle_cauchy;                  // 中轴 到 大腿与氮气弹簧连接线 与 氮气弹簧力方向 的夹角（单位°）
    float m_length;                      // 大腿中间的那根传导力到小腿的杆的上段长度
    float n_length;                      // 大腿中间的那根传导力到小腿的杆的下段长度
    float k_length;                      // 大腿与小腿连接点 与 氮气弹簧连接到小腿的点的距离
    float j_length;                      // 大腿与小腿连接点 与 顺着小腿方向往上那根连杆的终点的距离
    float Knee_joint_leg_length;         // 膝关节腿长度、小加工件长度(单位:m)
    float k_compensation_knee_joint_leg; // 用于补偿氮气弹簧增压比

} Spring_compensation_info_t; // 氮气弹簧补偿

/*==================== Manual Rescue Config Begin ====================*/
// 手动自救模式配置参数
typedef struct {
    float phi0d1_speed_abs; // 两腿摆角目标速度绝对值，默认150°/s
    float F_change_speed;   // 目标腿竖直力增加速度，默认50N/s
    float F_max;            // 腿部竖直力限幅最大值，默认300
    float F_min;            // 腿部竖直力限幅最小值，默认-300
} Manual_Rescue_Config_t;

// 手动自救模式中间变量
typedef struct {
    float F_target_r;          // 右腿目标腿部竖直力
    float F_target_l;          // 左腿目标腿部竖直力
    uint8_t z_key_count;       // Z键连续按下计数
    uint32_t z_key_last_time;  // 上次Z键按下时间
    uint8_t ctrl_pressed_last; // 上次Ctrl键状态（用于检测跳变）
} Manual_Rescue_Var_t;

// 手动自救模式完整结构体
typedef struct {
    Manual_Rescue_Config_t config;
    Manual_Rescue_Var_t var;
} Chassis_manual_rescue_info_t;
/*==================== Manual Rescue Config End ====================*/

typedef struct
{
    Link_t *Link;             // 五连杆解算、VMC
    Straight_Leg_t *Straight; // 直腿模型
    Leg_force_t *force;
    uint16_t off_ground_cnt;
    uint8_t off_ground;

    float K_Leg_Gravity_Torque; // 自救时腿部重力力矩补偿系数

} Leg_Unit_t;

typedef enum {
    J_IDLE,
    J_COMPRESS,    // 压缩
    J_EXTEND,      // 伸腿
    J_RETRACT,     // 收腿
    J_PRE_LANDING, // 伸腿准备落地
    J_LANDING,     // 缓冲
    Jump_Step_Num,

} Jump_Step_e;

typedef struct
{
    bool r_offground;
    bool l_offground;
    float l0_average;
    float Minimum_l0_range; //  收腿阶段完成判断容差阈值
    float Max_l0_range;     //  伸腿阶段完成判断容差阈值
    float Landing_l0_range; //
    uint16_t jump_tick;

    uint16_t COMPRESS_tick;

    uint16_t EXTEND_tick;

    uint16_t RETRACT_tick;

    uint16_t PRE_LANDING_tick;

    uint16_t LANDING_tick;

    float J_COMPRESS_length_target; // TAR_LEG_LENGTH_INITIAL 到目标腿长（使用收腿容差判断）或超时进入下一阶段
    uint16_t Max_COMPRESS_tick;

    float J_EXTEND_length_target; // TAR_LEG_LENGTH_INITIAL+5 到目标腿长（使用伸腿容差判断）、离地、超时进入下一阶段
    uint16_t Max_EXTEND_tick;

    float J_RETRACT_length_target; // MIN_LEG_LENGTH 到目标腿长（使用收腿容差判断）、超时
    uint16_t Max_RETRACT_tick;

    float J_PRE_LANDING_length_target; // TAR_LEG_LENGTH_INITIAL+5 到目标腿长（使用伸腿容差判断）、双腿触地、超时
    uint16_t Max_PRE_LANDING_tick;

    float J_LANDING_length_target; // TAR_LEG_LENGTH_INITIAL  到目标腿长（使用收腿容差判断）、超时
    uint16_t Max_LANDING_tick;

    float IDLE_length_kp;
    float IDLE_length_speed_kp;
    float IDLE_length_outmax;
    float IDLE_length_speed_outmax;

    float COMPRESS_length_kp;
    float EXTEND_length_kp;
    float RETRACT_length_kp;
    float PRE_LANDING_length_kp;
    float LANDING_length_kp;
    float LANDING_length_speed_kp;

    Jump_Step_e jump_step;
} Chassis_Jump_t;

typedef enum {
    Knee_IDLE,
    Knee_Stand_High,
    Knee_RETRACT,
    Knee_Strike_Num,
} KNEE_STRIKE_Step_e;

typedef enum {
    JK_IDLE,
    JK_COMPRESS,    // 压缩
    JK_EXTEND,      // 伸腿
    JK_RETRACT_1,   // 收腿
    JK_PRE_LANDING, // 伸腿准备落地
    JK_LANDING,     // 缓冲

    JK_Stand_High,
    JK_RETRACT_2,
    JK_Strike_Num,
} JUMP_THEN_KNEE_STRIKE_Step_e;

typedef enum {
    JAM_IDLE,
    JAM_COMPRESS,    // 压缩
    JAM_EXTEND,      // 伸腿（跳）
    JAM_RETRACT,     // 收腿滞空
    JAM_PRE_LANDING, // 伸腿准备落地
    JAM_LANDING,     // 缓冲
    JAM_Step_Num,
} JUMP_AND_MID_Step_e;

// RTS_Step_e 枚举（新增）
typedef enum {
    RTS_IDLE,
    RTS_Rising, // 抬腿递增阶段
    RTS_Front_Swing,
    RTS_Landing,
    RTS_Step_Num
} RTS_Step_e;

typedef struct
{
    bool r_offground;
    bool l_offground;
    float l0_average;
    float Minimum_l0_range; // 收腿完成判断容差阈值
    float Max_l0_range;     // 伸腿完成判断容差阈值
    float Landing_l0_range; // 落地完成判断容差阈值

    float JAM_COMPRESS_target_length;    // 压缩腿阶段目标腿长
    float JAM_EXTEND_target_length;      // 伸腿阶段目标腿长
    float JAM_RETRACT_target_length;     // 收腿滞空阶段目标腿长
    float JAM_PRE_LANDING_target_length; // 伸腿准备落地阶段目标腿长
    float JAM_LANDING_target_length;     // 落地阶段目标腿长

    uint16_t jump_tick;
    uint16_t COMPRESS_tick;
    uint16_t EXTEND_tick;
    uint16_t RETRACT_tick;
    uint16_t PRE_LANDING_tick;
    uint16_t LANDING_tick;

    uint16_t Max_COMPRESS_tick;
    uint16_t Max_EXTEND_tick;
    uint16_t Max_RETRACT_tick;
    uint16_t Max_PRE_LANDING_tick;
    uint16_t Max_LANDING_tick;
    uint16_t Max_Both_Onground_tick; // 两腿同时在地面(不离地)的最长容忍时间，超过则进入下一阶段
    float IDLE_length_kp;
    float IDLE_length_speed_kp;
    float IDLE_length_outmax;
    float IDLE_length_speed_outmax;

    float COMPRESS_length_kp;
    float EXTEND_length_kp;
    float RETRACT_length_kp;
    float PRE_LANDING_length_kp;
    float LANDING_length_kp;

    JUMP_AND_MID_Step_e JUMP_AND_MID_step; // 跳+中腿长状态机步
    uint16_t JUMP_AND_MID_tick;            // 跳+中腿长总计时
    uint16_t Max_JUMP_AND_MID_tick;        // 最大计时（10s=10000）
} Chassis_Jump_And_Mid_t;

// RTS_Config_t 配置结构体（新增）
typedef struct {
    float front_swing_leg_length;  // 前摆状态目标腿长
    float landing_leg_length;      // 着陆状态目标腿长
    float front_angle;             // 前摆角度（云台朝前时使用）
    float behind_angle;            // 后摆角度（云台朝后时使用）
    float front_threshold;         // 前摆检测阈值
    float behind_threshold;        // 后摆检测阈值
    float sd1_speed_reduced;       // sd1目标速度折扣系数
    float theta_offset;            // 前摆状态theta目标偏移量
    uint16_t front_swing_timeout;  // 前摆状态超时时间（ms）
    uint16_t min_front_swing_time; // 前摆状态最短持续时间，让腿有时间前摆
    uint16_t landing_reset_time;   // 着陆状态复位时间（ms）
    float encoder_speed_threshold; // 两轮encoder速度绝对值判断阈值
    float theta_threshold_offset;  // theta角度超出初始值的偏移阈值
} RTS_Config_t;

// RTS_Var_t 中间变量结构体（新增）
typedef struct {
    RTS_Step_e rts_step;        // 当前RTS状态机步
    uint32_t rts_tick;          // RTS总计时
    uint16_t front_swing_tick;  // 前摆状态计时
    uint16_t landing_tick;      // 着陆状态计时
    uint16_t rising_tick;       // 抬腿递增计时
    float initial_theta_target; // 初始目标摆杆角度
    float initial_leg_length;   // 初始腿长
} RTS_Var_t;

// Chassis_Retract_to_Second_Step_Info_t 完整结构体（新增）
typedef struct {
    RTS_Config_t config;
    RTS_Var_t var;
} Chassis_Retract_to_Second_Step_Info_t;

/**------------- 模型功率限制 begin ------------- */
typedef struct {
    float k_i2;
    float k_iw;
    float k_w2;
    float a;               // 常数
    float k_torque_to_LSB; // Nm转化为电流命令数值单位的系数
} Model_Power_Limit_config_t;

typedef struct {
    float measure_i;
    float measure_w;
    float tx_i;      // 要发送的原始电流
    float max_power; // 最大允许功率
} Model_Power_Limit_input_var_t;

typedef struct {
    float limited_i; // 最大功率限制后的电流
    float limited_torque;
    float predicted_power; // 预测功率
} Model_Power_Limit_output_var_t;

typedef struct {
    float term_i2; // i2项输出
    float term_iw;
    float term_w2;

} Model_Power_Limit_Mid_var_t;

typedef struct {
    Model_Power_Limit_input_var_t input_var;
    Model_Power_Limit_output_var_t output_var;
    Model_Power_Limit_Mid_var_t mid_var;
} Model_Power_Limit_var_t;
typedef struct {
    Model_Power_Limit_config_t config;
    Model_Power_Limit_var_t var;
} Chassis_Model_Power_Limit_Info_t;
/**------------- 模型功率限制 end ------------- */

typedef struct
{
    float Minimum_l0_range;
    float Max_l0_range;
    float Idle_tick;
    float Max_Stand_High_tick;
    float Max_RETRACT_tick;

    float l0_average;
    float thetal_average;
    float Stand_High_tick;
    float RETRACT_tick;
    float thetal_threshold;
    float thetal_diff_max; // 两腿thetal差值上限，超过则不触发收腿
    float IDLE_length_kp;
    float RETRACT_length_kp;
    float knee_strike_exit_tick; // 退出KNEE_STRIKE后的计时器

    KNEE_STRIKE_Step_e KNEE_STRIKE_step;
    JUMP_THEN_KNEE_STRIKE_Step_e JUMP_THEN_KNEE_STRIKE_Step;
} Chassis_Knee_Strike_t;

typedef struct
{
    float l0_length_kp;
    float l0_length_speed_kp;
    float l0_length_outmax;
    float l0_length_speed_outmax;
} Chassis_pid_init_parament_t; // 存储最开始的pid参数

typedef struct
{
    float a;
    float Nf;
    float kk;
    float power_limit;
    float Tw_Enable;
    float Tp_Big;
    float turn_limit_high_buffer_threshold; // 转向力矩比例限制缓冲能量阈值
    float turn_limit_mid_buffer_threshold;  // 转向力矩平方限制缓冲能量阈值
    float turn_limit_low_buffer_threshold;  // 转向力矩立方限制缓冲能量阈值
    float k_buffer_limit;                   // 根据缓冲能量算出来的系数
    float k_cap_capacity_scale_limit;       // 此系数乘以超电容量比例=k_cap_limit
    float k_cap_limit;                      // 缓冲能量算出的系数
    float k_turn_power_limit;               // 根据剩余缓冲能量、超电容量比例算出来的转向力矩限制系数
} Chassis_Power_Limit_t;

typedef enum {
    Reset = 0,                   // 收腿后、调LQR、清标志位、屏蔽一会自救
    ForwardFlip_and_L_Rollover,  // 前翻，左翻，仅左腿前摆，右腿不动，机体Roll回正后进`ForwardFlip`
    ForwardFlip_and_R_Rollover,  // 前翻，右翻，仅右腿前摆，左腿不动，机体Roll回正后进`ForwardFlip`
    BackwardFlip_and_L_Rollover, // 后翻，左翻，仅左腿后摆，右腿不动，机体Roll回正后进`BackwardFlip`
    BackwardFlip_and_R_Rollover, // 后翻，右翻，仅右腿后摆，左腿不动，机体Roll回正后进`BackwardFlip`
    L_Rollover,                  // Pitch正常，左翻，仅左腿后摆，右腿不动，机体Roll回正后进`PRNormalBackwardLeg`
    R_Rollover,                  // Pitch正常，右翻，仅右腿后摆，左腿不动，机体Roll回正后进`PRNormalBackwardLeg`
    ForwardFlip,                 // 前翻，腿前摆 ，机体回正后进`PRNormalBackwardLeg`
    BackwardFlip,                // 后翻，腿后摆
    PRNormalBackwardLeg,         // 机体Pitch、Roll正常，腿后摆头归正
    CorrectGimbalDirection,      // 等待云台yaw归正到0°
    RetractLegs,                 // 平躺、腿后摆偏上一点位置、收腿

} Rescue_state_machine_e; // 自救步骤

typedef struct
{
    float pitchForwardFlipThreshold;              // 前翻pitch判断阈值
    float pitchBackwardFlipThreshold;             // 后翻pitch判断阈值
    float pitchNormalThreshold;                   // pitch正常判断阈值，主要用于前翻时从前摆腿切换到后摆腿 以及头归位
    float rollRightRolloverThreshold;             // 右侧翻roll判断阈值，左腿腾空
    float rollLeftRolloverThreshold;              // 左侧翻roll判断阈值，右腿腾空
    float thetalForwardThreshold;                 // 腿前伸theta角度自救判断阈值
    float thetalBackwardThreshold;                // 腿后伸theta角度自救判断阈值
    float thetaBackwardRetractLegThreshold;       // 腿后摆 收腿theta角度判断阈值
    float phi0PRNormalBackwardSpeed;              // 自救最后一步腿phi0后摆速度，单位：°/s
    float phi0ForwardFlipSpeed;                   // 前翻自救腿phi0前摆速度，单位：°/s
    float phi0BackwardFlipSpeed;                  // 后翻自救腿phi0后摆速度，单位：°/s
    float retractLegLengthTolerance;              // 收腿成功判断容忍度
    uint16_t RetractLegsTimeoutPeriod;            // *** 平躺收腿超时时间
    uint16_t PRNormalBackwardLegTimeoutPeriod;    // *** Pitch正常，腿后摆，头归正超时时间
    uint16_t CorrectGimbalDirectionTimeoutPeriod; // *** 等待云台yaw归正超时时间
    uint16_t RolloverTimeoutPeriod;               // roll侧翻超时时间
    uint16_t ForwardFlipTimeoutPeriod;            // 前翻到pitch正常超时时间
    uint16_t BackwardFlipTimeoutPeriod;           // 后翻到pitch正常超时时间

} Chassis_Rescue_config_t; // 自救动作阈值配置结构体

typedef struct
{
    uint32_t unknown_posture_cnt;     // 进入未知姿态计数
    uint32_t stateMacineTimelineTick; // 状态机时间戳，用于超时退出
    Rescue_state_machine_e rescue_state_mac;
    Chassis_Rescue_config_t rescue_config;
} Chassis_Rescue_info_t; // 自救动作信息结构体

typedef struct
{
    uint16_t time_stop_sd1_after_landing; // 中腿长离地触地后禁止给速度时间配置，防止前冲
    float thetal_length_linear_k;         // 平均腿长到thetal目标值的线性斜率
    float thetal_length_linear_b;         // 平均腿长到thetal目标值的线性截距
    float cycle_thetal_l_target;          // 左腿小陀螺摆杆角度目标值
    float cycle_thetal_r_target;          // 右腿小陀螺摆杆角度目标值
    float cycle_length_l_target;          // 左腿小陀螺腿长目标值
    float cycle_length_r_target;          // 右腿小陀螺腿长目标值
    float cycle_thetab_target;            // 小陀螺thetab目标值
    float Mid_Leg_Max_Speed;              // 中腿长时最大直行速度
    float max_thetab_err;                 // thetab状态误差限幅处理

} Chassis_etc_config_t; // 杂项配置结构体，只有一两项的配置

/*小陀螺动作信息结构体*/
typedef struct
{
    /* ====== 通用参数 ====== */
    float target_yaw_v_degree; // 小陀螺目标角速度 (deg/s)，由Cycle_Target_Process计算

    /* ====== 匀速小陀螺配置 ====== */
    float high_cap_constant_speed; // 匀速小陀螺目标速度 (deg/s)
    float low_cap_constant_speed;

    float high_cap_threshold;

    /* ====== 变速小陀螺配置 ====== */
    // 速度参数
    float vary_min_speed; // 变速小陀螺最低速度 (deg/s)，默认200°/s
    float vary_max_speed; // 变速小陀螺最高速度 (deg/s)，默认400°/s
    // 时间参数 (单位: ms)
    uint16_t vary_accel_duration; // 加速段时间，默认600ms
    uint16_t vary_const_duration; // 恒速段时间，默认400ms
    uint16_t vary_decel_duration; // 减速段时间，默认300ms

    // 运行状态变量
    float current_speed;      // 当前运行的自旋角速度 (deg/s)
    int8_t vary_flag;         // 变速标志位：1为正在加速，-1为正在减速
    uint16_t vary_cycle_tick; // 变速小陀螺周期计时 (ms)
} Chassis_cycle_info_t;

/*底盘结构体*/
typedef struct Chassis_struct_t {
    Chassis_Mode_e mode;
    Leg_Unit_t *Leg_Unit[Leg_Num];
    Chassis_Pid_t *chassis_PID;
    Chassis_Posture_t *Posture;
    Chassis_etc_config_t *etc_config;
    Chassis_state_t *state;
    Motor_RM_Group_t *Wheel;

    Motor_DM_Group_t *Sd;
    Chassis_Target_t *target;
    Chassis_Model_Power_Limit_Info_t *Model_PL[Leg_Num];
    Spring_compensation_info_t *spring_c_info;

    Chassis_Rescue_info_t *rescue_info;

    Chassis_Rc_Input_t *rc_input;

    Chassis_Key_Input_t *key_input;

    uint16_t damping_delay_cnt;

    Chassis_Mode_e last_mode;
    Chassis_Jump_t *jump_info;

    Chassis_Jump_And_Mid_t *jump_and_mid_info;

    Chassis_Knee_Strike_t *knee_strike_info;

    Chassis_Power_Limit_t *Power_Limit_info;

    Chassis_cycle_info_t *cycle_info;

    Chassis_manual_rescue_info_t *manual_rescue_info;

    Chassis_Retract_to_Second_Step_Info_t *rts_info;

    void (*Init)(struct Chassis_struct_t *My_Chassis); // 初始化函数

    void (*heartbeat)(struct Chassis_struct_t *My_Chassis);

    void (*data_update)(struct Chassis_struct_t *My_Chassis);

    void (*status_react)(struct Chassis_struct_t *My_Chassis);

    void (*ctrl)(struct Chassis_struct_t *My_Chassis);

    void (*work)(struct Chassis_struct_t *My_Chassis);
    Chassis_pid_init_parament_t *pid_init_parament;

} Chassis_t;

/* Exported functions --------------------------------------------------------*/
extern Chassis_t Chassis;
extern float My_Wheel_Sb;
extern float My_Imu_Sb;
extern float My_filter_Sb;
extern float My_filter_Sb2;
void My_Theta_Save_Cal(Chassis_t *My_Chassis);
void My_Spring_Former_Input_Cal(Chassis_t *My_Chassis);
/* Servo functions */

#endif
