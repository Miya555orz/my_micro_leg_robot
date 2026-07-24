#include "gimbal.h"

#define M_PI 3.14159265358979323846f
// 更新云台模式状态
static void Gimbal_Status_Update(gimbal_t *gimbal);
// 更新陀螺仪模式目标
static void Gimbal_Gyro_Update(gimbal_t *gimbal, uint8_t ctrl_mode);
// 更新机械模式目标
static void Gimbal_Mec_Update(gimbal_t *gimbal);
// 更新机械主动移动模式
static void Gimbal_Mec_Move_Update(gimbal_t *gimbal);
// 云台归中初始化模式
static void Gimbal_Init_Update(gimbal_t *gimbal);
// 更新云台外部反馈信息
static void Gimbal_Extern_Update(gimbal_t *gimbal);
// 约束yaw目标角度范围
static void Gimbal_Yaw_Angle_Limit(gimbal_t *gimbal);
// 约束pitch机械目标角度范围
static void Gimbal_Pitch_Mec_Angle_Limit(gimbal_t *gimbal);
// 约束pitch陀螺仪目标角度范围
static void Gimbal_Pitch_Gyro_Angle_Limit(gimbal_t *gimbal);
// 计算PID输出
static void Gimbal_Pid_Cal(gimbal_t *gimbal);
// 云台初始化流程
static void Gimbal_Check_init(gimbal_t *gimbal);
// 云台总工作流程入口
void Gimbal_Work(gimbal_t *gimbal);

gimbal_offset_info_t offset_info =
    {
        .vision_yaw_offset = 0, // 视觉偏置
};

static gimbal_180_state_t gimbal_180_state = {
    .is_rotating = false,
    .target_angle = 0.f,
    .angle_tolerance = 3.0f, // 度
    .speed_tolerance = 0.5f, // rad/s
};

gimbal_t gimbal =
    {
        .gimbal_y = &Yaw_Motor,
        .offset_info = &offset_info,
        .all_pid_calc = &all_pid_calc,
        .pid_info = &gimbal_pid,
        .work = Gimbal_Work,
        .gimbal_last_mode = GIMB_SLEEP,
        .gimbal_reset_state = DEV_RESET_NO,
        .initInfo.init_time = 0,
        .initInfo.init_time_max = 3000.f,
        .initInfo.pitchInitAngleTolerance = 0.3f,
        .initInfo.yawInitAngleTolerance = 0.3f,
        .initInfo.yawInitSpeedTolerance = 0.2f,
        .initInfo.pitchInitSpeedTolerance = 30.f,
        .base_info.pitch_imu_angle_target = 0,
        .base_info.pitch_mec_angle_target = 0,
};

/*云台状态更新*/
static void Gimbal_Status_Update(gimbal_t *gimbal)
{
    switch (Balance.mode)
    {
    case Sleep_Mode:
        gimbal->mode = GIMB_SLEEP;
        gimbal->gimbal_reset_state = DEV_RESET_NO;
        break;

    case Mec_Mode:
        gimbal->mode = G_MEC;
        break;

    case Init_Mode:
        gimbal->mode = G_INIT;
        break;

    case Manual_Rescue_Mode:
        gimbal->mode = GIMB_SLEEP;
        gimbal->gimbal_reset_state = DEV_RESET_NO;
        break;

    case Rescue_Mode:
        /**
         * PRNormalBackwardLeg就近归位
         * CorrectGimbalDirection\RetractLegs\Reset则归云台零点
         * 具体看Gimbal_Mec_Update此函数
         */
        if (
            Balance.mode == Rescue_Mode &&
            (Chassis.rescue_info->rescue_state_mac == PRNormalBackwardLeg ||
             Chassis.rescue_info->rescue_state_mac == CorrectGimbalDirection ||
             Chassis.rescue_info->rescue_state_mac == RetractLegs ||
             Chassis.rescue_info->rescue_state_mac == Reset))
        {
            gimbal->mode = G_MEC;
        }
        else
        {
            gimbal->mode = GIMB_SLEEP;
        }

        break;

    case Imu_Mode:
    case Cycle_Mode:
        // balance在自救完就进imu模式了，没有判断云台是否复位完成，所以在此处判断
        if (gimbal->gimbal_reset_state == DEV_RESET_OK)
        {
            gimbal->mode = G_GYRO;
        }
        else
        {
            gimbal->mode = G_MEC;
        }
        break;

    case SitDown_Mode:
        if (Balance.Vision.Auto_Catch_Flag == true)
        {
            gimbal->mode = G_GYRO;
        }
        else
        {
            gimbal->mode = G_MEC_MOVE;
        }
        break;

    default:
        gimbal->mode = GIMB_SLEEP;
        gimbal->gimbal_reset_state = DEV_RESET_NO;
        break;
    }

    // C_RTS模式使用IMU模式进行云台控制
    if (Balance.Flag->RTS_Flag == true)
    {
        if (gimbal->gimbal_reset_state == DEV_RESET_OK)
        {
            gimbal->mode = G_GYRO;
        }
        else
        {
            gimbal->mode = G_MEC;
        }
    }
}

/*云台陀螺仪模式*/
static bool last_GIMBAL_180_Flag = false; // 上一周期标志位状态
static float k_yaw_imu_RC_speed = 200;
static float k_pitch_imu_RC_speed = 30;
static float k_yaw_imu_Key_speed = 3;
static float k_pitch_imu_Key_speed = 1;
static void Gimbal_Gyro_Update(gimbal_t *gimbal, uint8_t ctrl_mode)
{
    /*------------------------- 一键换头命令 begin ----------------------------*/
    bool current_GIMBAL_180_Flag = Balance.Flag->GIMBAL_180_Flag;
    bool rising_edge = (current_GIMBAL_180_Flag == true) && (last_GIMBAL_180_Flag == false);
    float angle_diff = my_abs(half_cycle(
        (gimbal->base_info.yaw_imu_angle_target - gimbal->base_info.yaw_imu_angle), 360.f));
    float speed = my_abs(gimbal->base_info.yaw_imu_speed);

    // 检测到上升沿且正在旋转未完成时，设置目标角度+=180
    if (rising_edge)
    {
        gimbal->base_info.yaw_imu_angle_target += 180.f;
        gimbal->base_info.yaw_imu_angle_target = half_cycle(gimbal->base_info.yaw_imu_angle_target, 360.f);
    }

    // 检查旋转是否完成
    if (Balance.Flag->GIMBAL_180_Flag == true)
    {

        if (angle_diff <= 2.f &&
            speed <= 0.3f)
        {
            // 旋转完成，清除标志位
            Balance.Flag->GIMBAL_180_Flag = false;
        }
    }
    // 保存当前标志位状态用于下一周期上升沿检测
    last_GIMBAL_180_Flag = current_GIMBAL_180_Flag;
    /*-------------------------- 一键换头命令 end ----------------------------*/

    // 正常遥杆/键盘控制（仅在非180旋转时生效）
    if (current_GIMBAL_180_Flag != true)
    {
        if ((Balance.Vision.Auto_Catch_Flag == true ||
             Balance.Vision.Auto_Catch_Engi_Flag == true) &&
            Board_Rx_Info.flag.is_vision_online == true &&
            Board_Rx_Info.flag.is_find_target == true)
        {
            gimbal->base_info.yaw_imu_angle_target = Board_Rx_Info.vision_target_yaw;
            gimbal->base_info.pitch_imu_angle_target = Board_Rx_Info.vision_target_pitch;
        }
        else
        {
            if (ctrl_mode == RC_CTRL)
            {
                gimbal->base_info.yaw_imu_angle_target -= rc_sensor.info->ch0 / 660.f * 0.001f * k_yaw_imu_RC_speed;
                gimbal->base_info.pitch_imu_angle_target += rc_sensor.info->ch1 / 660.f * 0.001f * k_pitch_imu_RC_speed;
            }
            else
            {
                gimbal->base_info.yaw_imu_angle_target -= rc_sensor.info->mouse_x * 0.001f;
                gimbal->base_info.pitch_imu_angle_target += rc_sensor.info->mouse_y * 0.001f;
            }
        }
        gimbal->base_info.yaw_imu_angle_target = half_cycle(gimbal->base_info.yaw_imu_angle_target, 360.f);
    }

    // 从其他模式首次切入G_GYRO时，强制将pitch机械目标清零
    if (gimbal->gimbal_last_mode != G_GYRO)
    {
        gimbal->base_info.pitch_imu_angle = 0;
    }

    gimbal->base_info.yaw_mec_angle_target = gimbal->base_info.yaw_motor_angle;
}

/*云台机械模式*/
static void Gimbal_Mec_Update(gimbal_t *gimbal)
{

    if ((my_abs(gimbal->base_info.yaw_motor_angle) > PI / 2.f) &&
        Chassis.rescue_info->rescue_state_mac != RetractLegs &&
        Chassis.rescue_info->rescue_state_mac != CorrectGimbalDirection &&
        Chassis.rescue_info->rescue_state_mac != Reset)
    {
        gimbal->base_info.yaw_mec_angle_target = sgn(gimbal->base_info.yaw_motor_angle) * PI;
    }
    else
    {
        gimbal->base_info.yaw_mec_angle_target = 0;
    }
    gimbal->base_info.pitch_mec_angle_target += rc_sensor.info->ch1 * 0.001f * 0.003f;
    gimbal->base_info.pitch_mec_angle_target = half_cycle(gimbal->base_info.pitch_mec_angle_target, 2 * M_PI);

    gimbal->base_info.yaw_imu_angle_target = gimbal->base_info.yaw_imu_angle;
    gimbal->base_info.pitch_imu_angle_target = gimbal->base_info.pitch_imu_angle;
}

/*云台机械头主动模式(调试用)*/
static void Gimbal_Mec_Move_Update(gimbal_t *gimbal)
{
    gimbal->base_info.yaw_mec_angle_target += rc_sensor.info->ch0 * 0.001f * 0.007f;
    gimbal->base_info.yaw_mec_angle_target = half_cycle(gimbal->base_info.yaw_mec_angle_target, 2 * M_PI);
    gimbal->base_info.pitch_mec_angle_target += rc_sensor.info->ch1 * 0.001f * 0.003f;
    gimbal->base_info.pitch_mec_angle_target = half_cycle(gimbal->base_info.pitch_mec_angle_target, 2 * M_PI);

    gimbal->base_info.yaw_imu_angle_target = gimbal->base_info.yaw_imu_angle;
    gimbal->base_info.pitch_imu_angle_target = gimbal->base_info.pitch_imu_angle;
}

/*云台归中初始化模式，等待Balance完成Init后进入Imu_Mode*/
static void Gimbal_Init_Update(gimbal_t *gimbal)
{
    gimbal->base_info.yaw_mec_angle_target = 0; // yaw归中
    // gimbal->base_info.pitch_mec_angle_target = 0; // pitch归中
    gimbal->base_info.yaw_imu_angle_target = gimbal->base_info.yaw_imu_angle;
    gimbal->base_info.pitch_imu_angle_target = gimbal->base_info.pitch_imu_angle;
}

/*云台信息更新*/

static void Gimbal_Extern_Update(gimbal_t *gimbal)
{

    gimbal->base_info.pitch_motor_angle = PITCH_MOTOR_ENCODER_MIDDLE - Board_Rx_Info.pitch_mec_angle;
    gimbal->base_info.pitch_motor_angle = half_cycle(gimbal->base_info.pitch_motor_angle, 2 * PI);
    gimbal->base_info.pitch_imu_angle = Board_Rx_Info.pitch_imu_angle;
    gimbal->base_info.pitch_imu_speed = Board_Rx_Info.pitch_imu_speed; // 上板原始的pitch的imu角度变化和速度变化就是反的，奇怪
    // gimbal->base_info.pitch_motor_speed = Board_Rx_Info.pitch_mec_speed;

    gimbal->base_info.yaw_imu_angle = Board_Rx_Info.yaw_imu_angle;
    gimbal->base_info.yaw_imu_speed = Board_Rx_Info.yaw_imu_speed;
    gimbal->base_info.yaw_imu_angle = half_cycle(gimbal->base_info.yaw_imu_angle, 360.f);

    /*yaw轴电机角度更新*/
    gimbal->base_info.yaw_motor_angle = YAW_MOTOR_ANGLE_MIDDLE - (float)gimbal->gimbal_y->rx_info->motor_angle;
    gimbal->base_info.yaw_motor_angle = half_cycle(gimbal->base_info.yaw_motor_angle, 2 * PI);
    gimbal->base_info.yaw_motor_speed = (float)gimbal->gimbal_y->rx_info->speed;

    /*360度标准化角度*/
    gimbal->base_info.yaw_mec_360_angle = gimbal->base_info.yaw_motor_angle / (2 * PI) * 360.f;
    gimbal->base_info.pitch_mec_360_angle = gimbal->base_info.pitch_motor_angle / (2 * PI) * 360.f;
}

/*云台yaw轴角度检查*/
static void Gimbal_Yaw_Angle_Limit(gimbal_t *gimbal)
{
    float angle = gimbal->base_info.yaw_imu_angle_target; //-180°~180°

    if (my_abs(angle) > 180.f) // 用while包卡死
    {
        angle -= 360.f * sgn(angle);
    }
    gimbal->base_info.yaw_imu_angle_target = angle;
}

/*云台pitch轴陀螺仪角度限位*/
static void Gimbal_Pitch_Gyro_Angle_Limit(gimbal_t *gimbal)
{
    float angle = gimbal->base_info.pitch_imu_angle_target;
    if (angle > GIMBAL_MAX_GYRO_ANGEL)
    {
        angle = GIMBAL_MAX_GYRO_ANGEL;
    }
    if (angle < GIMBAL_MIN_GYRO_ANGEL)
    {
        angle = GIMBAL_MIN_GYRO_ANGEL;
    }
    gimbal->base_info.pitch_imu_angle_target = angle;
}

/*云台pitch轴机械角度限位*/
static void Gimbal_Pitch_Mec_Angle_Limit(gimbal_t *gimbal)
{
    float angle = gimbal->base_info.pitch_mec_angle_target;
    if (angle > GIMBAL_MAX_MEC_ANGEL)
    {
        angle = GIMBAL_MAX_MEC_ANGEL;
    }
    if (angle < GIMBAL_MIN_MEC_ANGEL)
    {
        angle = GIMBAL_MIN_MEC_ANGEL;
    }
    gimbal->base_info.pitch_mec_angle_target = angle;
}
/*云台yaw轴PID计算*/
static void Gimbal_Pid_Cal(gimbal_t *gimbal)
{
    float gyro_meas_in, gyro_meas_out, gyro_target, mec_meas_in, mec_meas_out, mec_target;

    switch (gimbal->mode)
    {
    case G_GYRO:
        // Yaw pid计算
        gyro_meas_out = gimbal->base_info.yaw_imu_angle;      // 外环
        gyro_meas_in = gimbal->base_info.yaw_imu_speed;       // 内环
        gyro_target = gimbal->base_info.yaw_imu_angle_target; // 目标值
        gimbal->base_info.output_gimbal_y = gimbal->all_pid_calc(gimbal->pid_info->yaw_gyro_outer, gimbal->pid_info->yaw_gyro_inner, gyro_target, gyro_meas_out, gyro_meas_in, -1, 3);
        // pitch pid计算
        gyro_meas_out = gimbal->base_info.pitch_imu_angle;      // 外环
        gyro_meas_in = gimbal->base_info.pitch_imu_speed;       // 内环
        gyro_target = gimbal->base_info.pitch_imu_angle_target; // 目标值
        gimbal->base_info.output_gimbal_p = -gimbal->all_pid_calc(gimbal->pid_info->pitch_gyro_outer, gimbal->pid_info->pitch_gyro_inner, gyro_target, gyro_meas_out, gyro_meas_in, -1, 3);
        break;

    case G_MEC:
    case G_INIT:
    case G_MEC_MOVE:
        // yaw pid计算
        mec_meas_out = (float)gimbal->base_info.yaw_mec_360_angle; // 外环 转为角度
        mec_meas_in = gimbal->base_info.yaw_imu_speed;             // 内环
        mec_target = gimbal->base_info.yaw_mec_angle_target / PI * 180.f;
        gimbal->base_info.output_gimbal_y = -gimbal->all_pid_calc(gimbal->pid_info->yaw_mec_outer, gimbal->pid_info->yaw_mec_inner, mec_target, mec_meas_out, mec_meas_in, 1, 3);

        // pitch pid计算
        mec_meas_out = (float)gimbal->base_info.pitch_mec_360_angle; // 外环 转为角度
        mec_meas_in = gimbal->base_info.pitch_imu_speed;             // 内环
        mec_target = gimbal->base_info.pitch_mec_angle_target / PI * 180.f;
        gimbal->base_info.output_gimbal_p = -gimbal->all_pid_calc(gimbal->pid_info->pitch_mec_outer, gimbal->pid_info->pitch_mec_inner, mec_target, mec_meas_out, mec_meas_in, -1, 3);
        break;

    case GIMB_SLEEP:
        gimbal->base_info.output_gimbal_y = 0.f;
        gimbal->base_info.output_gimbal_p = 0.f;
        break;

    default:
        gimbal->base_info.output_gimbal_y = 0.f;
        break;
    }
}

/*云台检查初始化是否完成*/
static void Gimbal_Check_init(gimbal_t *gimbal)
{
    // 初始化计时: 仅在G_INIT模式、或G_MEC模式且处于RetractLegs/CorrectGimbalDirection阶段时计时
    // PRNormalBackwardLeg阶段不计时，等进入CorrectGimbalDirection再开始计时
    if (gimbal->gimbal_reset_state == DEV_RESET_OK)
    {
        return;
    }
    if (gimbal->mode == G_INIT ||
        (gimbal->mode == G_MEC && (Chassis.rescue_info->rescue_state_mac == RetractLegs ||
                                   Chassis.rescue_info->rescue_state_mac == CorrectGimbalDirection)))
    {
        if (gimbal->gimbal_reset_state == DEV_RESET_NO)
        {
            gimbal->initInfo.init_time++;
        }
    }

    // 到位判断
    if (gimbal->base_info.yaw_mec_angle_target == 0 &&
        (my_abs(half_cycle((gimbal->base_info.yaw_motor_angle - gimbal->base_info.yaw_mec_angle_target), 2 * M_PI)) <= gimbal->initInfo.yawInitAngleTolerance) &&
        (my_abs(half_cycle((gimbal->base_info.pitch_motor_angle - gimbal->base_info.pitch_mec_angle_target), 2 * M_PI)) <= gimbal->initInfo.pitchInitAngleTolerance) &&
        (my_abs(gimbal->base_info.yaw_motor_speed) <= gimbal->initInfo.yawInitSpeedTolerance))
    {
        gimbal->base_info.yaw_imu_angle_target = gimbal->base_info.yaw_imu_angle; // 方便丝滑转陀螺仪控
        gimbal->base_info.pitch_imu_angle_target = 0;
        gimbal->gimbal_reset_state = DEV_RESET_OK;
        gimbal->initInfo.init_time = 0;
    }
    // 超时退出
    if (gimbal->initInfo.init_time >= gimbal->initInfo.init_time_max)
    {
        gimbal->base_info.yaw_imu_angle_target = gimbal->base_info.yaw_imu_angle; // 方便丝滑转陀螺仪控
        gimbal->base_info.pitch_imu_angle_target = 0;
        gimbal->gimbal_reset_state = DEV_RESET_OK;
        gimbal->initInfo.init_time = 0;
    }
}

void Gimbal_Work(gimbal_t *gimbal)
{
    Gimbal_Status_Update(gimbal); // 根据车状态更新云台状态
    Gimbal_Extern_Update(gimbal); // 外部数据更新，导入陀螺仪等速度

    switch (gimbal->mode)
    {
    case GIMB_SLEEP:
        // 复位输出
        gimbal->initInfo.init_time = 0;
        gimbal->base_info.output_gimbal_y = 0.f;
        gimbal->base_info.output_gimbal_p = 0.f;

        // 清标志位
        last_GIMBAL_180_Flag = false;

        // 设置好初始目标值，随时准备好启动
        gimbal->base_info.yaw_imu_angle_target = gimbal->base_info.yaw_imu_angle;
        gimbal->base_info.pitch_imu_angle_target = gimbal->base_info.pitch_imu_angle;
        if (my_abs(gimbal->base_info.yaw_motor_angle) > PI / 2.f)
        {
            gimbal->base_info.yaw_mec_angle_target = sgn(gimbal->base_info.yaw_motor_angle) * PI;
            gimbal->base_info.pitch_mec_angle_target = 30;
        }
        else
        {
            gimbal->base_info.yaw_mec_angle_target = 0;
            gimbal->base_info.pitch_mec_angle_target = 0;
        }

        break;

    case G_GYRO:
        Gimbal_Gyro_Update(gimbal, Balance.ctrl);
        break;

    case G_MEC:
        Gimbal_Mec_Update(gimbal);
        break;

    case G_MEC_MOVE:
        Gimbal_Mec_Move_Update(gimbal);
        break;

    case G_INIT:
        Gimbal_Init_Update(gimbal);
        break;

    default:
        break;
    }
    Gimbal_Check_init(gimbal);      // 非Sleep模式且未DEV_RESET_OK时检查是否初始化完成
    Gimbal_Yaw_Angle_Limit(gimbal); // Yaw角度检查
    Gimbal_Pitch_Gyro_Angle_Limit(gimbal);
    Gimbal_Pitch_Mec_Angle_Limit(gimbal);

    if (RC_ONLINE) // 开控
    {
        Gimbal_Pid_Cal(gimbal);
        // 填充到CAN发送入口变量
        gimbal->gimbal_y->tx_info->torque = gimbal->base_info.output_gimbal_y;
        // gimbal->base_info.output_gimbal_p = 0;
        //   Board_Tx_Info.pitch_output = gimbal->base_info.output_gimbal_p;
    }
    else if (RC_OFFLINE) // 关控
    {
        gimbal->gimbal_reset_state = DEV_RESET_NO;
        gimbal->base_info.pitch_mec_angle_target = 0;
        // 填充到CAN发送入口变量
        Board_Tx_Info.pitch_output = 0;
        gimbal->gimbal_y->tx_info->torque = 0;
    }

    // 记录当前模式，供下一周期检测模式跳变
    gimbal->gimbal_last_mode = gimbal->mode;
}
