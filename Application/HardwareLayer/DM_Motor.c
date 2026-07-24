/**

 * @file        DM_Motor.c

 * @author      2025_YZJ

 * @Version     V1.0

 * @date        8-Febraruary-2025

 * @brief       达秒mit控制电机包

 */

/* Includes ------------------------------------------------------------------*/

#include "DM_Motor.h"

static uint8_t Motor_Command[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};

static void Motor_Send_Data(Motor_DM_t *motor, uint8_t *buf);

static void Motor_SetControlPara(Motor_DM_t *motor);

static uint16_t float_to_uint_(float x, float x_min, float x_max, uint8_t bits);

static float uint_to_float_(uint16_t x_int, float x_min, float x_max, uint8_t bits);

static void Motor_Send_Command(Motor_DM_t *motor, Motor_MIT_Command_e Command);

static void Angle_Sum_Cal(Motor_DM_t *motor);

static void Motor_ERR_Check(Motor_DM_t *motor, uint8_t err_word);

static void Group_Motor_Heartbeat(Motor_DM_Group_t *group);

/*..........................................单电机..........................................*/

/**

 * @brief          单电机卸力

 * @param[in]      Motor_DM_t *motor     电机本体

 * @retval         none

 */

void DM_Single_Motor_Sleep(Motor_DM_t *motor)

{

    if (motor != NULL)

    {

        motor->tx_info->torque = 0;

        motor->tx_info->Kd = 0;

        motor->tx_info->Kp = 0;
    }
}

/**

 * @brief          单电机置零编码器

 * @param[in]      Motor_DM_t *motor     电机本体

 * @retval         none

 */

void DM_Single_Motor_ZeroPosSensor(Motor_DM_t *motor)

{

    if (motor != NULL)

    {

        motor->single_sleep(motor); // 先对电机卸力

        Motor_Send_Command(motor, Zero_Position_Sensor);
    }
}

/**

 * @brief          单电机控制输出转矩,含有CAN发送操作

 * @param[in]      Motor_DM_t *motor     电机本体

 * @retval         none

 */

void DM_Single_Motor_Set_Torque(Motor_DM_t *motor)

{

    if (motor != NULL)

    {

        if (motor->state->motor_state == Motor_Unenable)

        {

            motor->state->motor_state = Motor_Enable;
            // Motor_Send_Command(motor, Clear_Err);
            Motor_Send_Command(motor, Enter_Motor_Mode);
        }

        else

        {

            Motor_DM_Tx_Info_t *motor_tx_info = motor->tx_info;

            motor_tx_info->Kp = 0;

            motor_tx_info->Kd = 0;

            Motor_SetControlPara(motor);

            motor->tx_info->torque = 0;
        }
    }
}

/**

 * @brief          单电机控制速度,需要自行设置kd\target_speed\torque,含有CAN发送操作

 * @param[in]      Motor_DM_t *motor     电机本体

 * @retval         none

 */

void DM_Single_Motor_Set_Speed(Motor_DM_t *motor)

{

    if (motor != NULL)

    {

        if (motor->state->motor_state == Motor_Unenable)

        {

            motor->state->motor_state = Motor_Enable;

            Motor_Send_Command(motor, Enter_Motor_Mode);
        }

        else

        {

            Motor_DM_Tx_Info_t *motor_tx_info = motor->tx_info;

            motor_tx_info->Kp = 0;

            Motor_SetControlPara(motor);
        }
    }
}

/**

 * @brief          单电机控制角度,需要自行设置kp\target_angle\kd\target_speed\torque,含有CAN发送操作

 * @param[in]      Motor_DM_t *motor     电机本体

 * @retval         none

 */

void DM_Single_Motor_Set_Angle(Motor_DM_t *motor)

{

    if (motor != NULL)

    {

        if (motor->state->motor_state == Motor_Unenable)

        {

            motor->state->motor_state = Motor_Enable;

            Motor_Send_Command(motor, Enter_Motor_Mode);

            Motor_Send_Command(motor, Enter_Motor_Mode);
        }

        else

        {

            motor->tx_info->target_speed = 0;

            Motor_SetControlPara(motor);
        }
    }
}

void Motor_Set_Angle_Position_DM(Motor_DM_t *motor)

{

    pid_ctrl_t *my_angle_ctrl = motor->ctrl->position_out;

    pid_ctrl_t *my_speed_ctrl = motor->ctrl->position_inn;

    /*外环计算*/

    if (motor->ctrl->Angle_Input_Flag == false)

    {

        my_angle_ctrl->measure = motor->rx_info->motor_angle_sum;
    }

    my_angle_ctrl->err = my_angle_ctrl->target - my_angle_ctrl->measure;

    //	if(motor->ctrl->Nearest_Return == true)

    //	{

    //		if(motor->ctrl->Angle_Input_Flag == false)

    //		{

    if (my_angle_ctrl->err < P_MAX_8009)

    {

        my_angle_ctrl->err += 2.0f * PI;
    }

    if (my_angle_ctrl->err > P_MIN_8009)

    {

        my_angle_ctrl->err -= 2.0f * PI;
    }

    //		}

    //		else

    //		{

    //			if(my_angle_ctrl->err < -180.f)

    //			{

    //				my_angle_ctrl->err += 360.f;

    //			}

    //			if(my_angle_ctrl->err > 180.f)

    //			{

    //				my_angle_ctrl->err -= 360.f;

    //			}

    //		}

    //	}

    single_pid_ctrl(my_angle_ctrl);

    my_speed_ctrl->target = my_angle_ctrl->out;

    if (motor->ctrl->Speed_Input_Flag == false)

        my_speed_ctrl->measure = motor->rx_info->speed;

    my_speed_ctrl->err = my_speed_ctrl->target - my_speed_ctrl->measure;

    single_pid_ctrl(my_speed_ctrl);

    motor->tx_info->torque = my_speed_ctrl->out;
}

void Motor_Set_Speed_Position_DM(Motor_DM_t *motor)

{

    pid_ctrl_t *my_speed_ctrl = motor->ctrl->speed_ctrl;

    my_speed_ctrl->measure = motor->rx_info->speed;

    my_speed_ctrl->err = my_speed_ctrl->target - my_speed_ctrl->measure;

    single_pid_ctrl(my_speed_ctrl);

    motor->tx_info->torque = my_speed_ctrl->out;
}

void DM_Single_Motor_Set(Motor_DM_t *motor)

{

    if (motor != NULL)

    {

        if (motor->state->motor_state == Motor_Unenable)

        {

            motor->state->motor_state = Motor_Enable;

            Motor_Send_Command(motor, Enter_Motor_Mode);

            Motor_Send_Command(motor, Enter_Motor_Mode);
        }
    }
}

/**

 * @brief          电机CAN中断接收数据处理

 * @param[in]      Motor_DM_t *motor      电机本体

 * @param[in]      uint8_t *rxBuf						CAN接收数据包

 * @retval         none

 */

static void Motor_ReceiveData(Motor_DM_t *motor, uint8_t *rxBuf)

{

    static float pmin, pmax, vmax, vmin, tmax, tmin;

    if (motor->type == dm_6006)

    {

        pmax = P_MAX_6006;

        pmin = P_MIN_6006;

        vmax = V_MAX_6006;

        vmin = V_MIN_6006;

        tmax = T_MAX_6006;

        tmin = T_MIN_6006;
    }

    else if (motor->type == dm_4310)

    {

        pmax = P_MAX_4310;

        pmin = P_MIN_4310;

        vmax = V_MAX_4310;

        vmin = V_MIN_4310;

        tmax = T_MAX_4310;

        tmin = T_MIN_4310;
    }

    else /*(motor->type == leg_8009)*/

    {

        pmax = P_MAX_8009;

        pmin = P_MIN_8009;

        vmax = V_MAX_8009;

        vmin = V_MIN_8009;

        tmax = T_MAX_8009;

        tmin = T_MIN_8009;
    }

    Motor_DM_Rx_Info_t *motor_rx_info = motor->rx_info;

    Motor_ERR_Check(motor, rxBuf[0] >> 4);

    motor_rx_info->motor_angle = uint_to_float((uint16_t)((rxBuf[1] << 8) | rxBuf[2]), pmin, pmax, 16);

    motor_rx_info->speed = uint_to_float((uint16_t)((rxBuf[3] << 4) | (rxBuf[4] >> 4)), vmin, vmax, 12);

    if (my_abs(motor_rx_info->speed) == 0.0109901428f)

    {

        motor_rx_info->speed = 0;
    }

    motor_rx_info->torque = uint_to_float((uint16_t)(((rxBuf[4] & 0x0F) << 8) | rxBuf[5]), tmin, tmax, 12);

    Angle_Sum_Cal(motor);

    motor->state->offline_cnt = 0;

    motor_rx_info->ERR = (rxBuf[0] >> 4) & 0x0F;

    motor_rx_info->T_MOS = rxBuf[6];

    motor_rx_info->T_Rotor = rxBuf[7];
}

/**

 * @brief          单电机心跳包

 * @param[in]      Motor_HT_t *motor    电机本体

 * @retval         none

 */

static void DM_Motor_Hearbeat(Motor_DM_t *motor)

{

    motor->state->offline_cnt++;

    if (motor->state->offline_cnt > motor->state->offline_cnt_max)

    {

        motor->state->offline_cnt = motor->state->offline_cnt_max;

        motor->state->status = DEV_OFFLINE;

        motor->state->motor_state = Motor_Unenable;
    }

    else

    {

        if (motor->state->status == DEV_OFFLINE)

            motor->state->status = DEV_ONLINE;
    }
}

/**

 * @brief          单电机初始化

 * @param[in]      Motor_DM_t *motor     电机本体

 * @retval         none

 */

void DM_Single_Motor_Init(Motor_DM_t *motor)

{

    motor->single_sleep = DM_Single_Motor_Sleep;

    motor->single_set_torque = DM_Single_Motor_Set_Torque;

    motor->single_set_speed = DM_Single_Motor_Set_Speed;

    motor->single_set_angle = DM_Single_Motor_Set_Angle;

    motor->rx = Motor_ReceiveData;

    motor->single_heart_beat = DM_Motor_Hearbeat;

    /*开启电机控制*/

    motor->state->motor_state = Motor_Unenable;

    motor->state->last_motor_state = Motor_Unenable;

    motor->state->offline_cnt_max = 100;

    motor->rx_info->motor_angle_sum = 0;
}

/*..........................................电机组..........................................*/

/**

 * @brief          多电机（1~4个）电机控制输出转矩,含发送

 * @param[in]      Motor_DM_Group_t *group     电机组

 * @retval         none

 */

static void Group_Motor_Set_Torque(Motor_DM_Group_t *group)

{

    static uint8_t rx_num = 0;

    if (group->motor[rx_num] != NULL)

    {

        group->motor[rx_num]->single_set_torque(group->motor[rx_num]);

        rx_num++;
    }

    if (rx_num >= group->motor_num)

    {

        rx_num = 0;
    }
}

/**

 * @brief          电机组卸力

 * @param[in]      Motor_DM_Group_t *group     电机组

 * @retval         none

 */

static void Group_Motor_Sleep(Motor_DM_Group_t *group)

{

    if (group->motor[0] != NULL)

    {

        group->motor[0]->single_sleep(group->motor[0]);
    }

    if (group->motor[1] != NULL)

    {

        group->motor[1]->single_sleep(group->motor[1]);
    }

    if (group->motor[2] != NULL)

    {

        group->motor[2]->single_sleep(group->motor[2]);
    }

    if (group->motor[3] != NULL)

    {

        group->motor[3]->single_sleep(group->motor[3]);
    }
}

/**

 * @brief          电机组心跳包

 * @param[in]      Motor_DM_Group_t *group     电机组

 * @retval         none

 */

static void Group_Motor_Heartbeat(Motor_DM_Group_t *group)

{

    if (group->motor[0] != NULL)

    {

        group->motor[0]->single_heart_beat(group->motor[0]);
    }

    if (group->motor[1] != NULL)

    {

        group->motor[1]->single_heart_beat(group->motor[1]);
    }

    if (group->motor[2] != NULL)

    {

        group->motor[2]->single_heart_beat(group->motor[2]);
    }

    if (group->motor[3] != NULL)

    {

        group->motor[3]->single_heart_beat(group->motor[3]);
    }
}

/**

 * @brief          电机组初始化

 * @param[in]      Motor_DM_Group_t *group     电机组

 * @retval         none

 */

void Group_Motor_Init(Motor_DM_Group_t *group)

{

    uint8_t num_init = 0;

    if (group->motor[0] != NULL)

    {

        group->motor[0]->single_init = DM_Single_Motor_Init;

        group->motor[0]->single_init(group->motor[0]);

        num_init++;
    }

    if (group->motor[1] != NULL)

    {

        group->motor[1]->single_init = DM_Single_Motor_Init;

        group->motor[1]->single_init(group->motor[1]);

        num_init++;
    }

    if (group->motor[2] != NULL)

    {

        group->motor[2]->single_init = DM_Single_Motor_Init;

        group->motor[2]->single_init(group->motor[2]);

        num_init++;
    }

    if (group->motor[3] != NULL)

    {

        group->motor[3]->single_init = DM_Single_Motor_Init;

        group->motor[3]->single_init(group->motor[3]);

        num_init++;
    }

    group->motor_num = num_init;

    group->group_set_torque = Group_Motor_Set_Torque;

    group->group_heartbeat = Group_Motor_Heartbeat;

    group->group_sleep = Group_Motor_Sleep;
}

/*..........................................工具函数..........................................*/

/**

 * @brief          整合并发送电机命令报文

 * @param          Motor_DM_t *motor

 * @param[in]      Motor_DM_Command_e Command

 * @retval         none

 */

static void Motor_Send_Command(Motor_DM_t *motor, Motor_MIT_Command_e Command)

{

    switch (Command)

    {

    case Enter_Motor_Mode:

        Motor_Command[7] = 0xFC;

        break;

    case Exit_Motor_Mode:

        Motor_Command[7] = 0xFD;

        break;

    case Zero_Position_Sensor:

        Motor_Command[7] = 0xFE;

    case Clear_Err:

        Motor_Command[7] = 0xFB;

        break;

    default:

        break;
    }

    Motor_Send_Data(motor, Motor_Command);
}

/**

 * @brief          根据结构体信息发送报文

 * @param          Motor_DM_t *motor

 * @param          uint8_t* buf 要发送的报文信息

 * @retval         none

 */

static void Motor_Send_Data(Motor_DM_t *motor, uint8_t *buf)

{

    Motor_DM_Born_Info_t *motor_born_info = motor->born_info;

    CAN_SendData(motor_born_info->hcan, motor_born_info->txId, buf);
}

/**

 * @brief          根据发送的报文信息设置报文并发送

 * @param          Motor_DM_t *motor

 * @retval         none

 */

static void Motor_SetControlPara(Motor_DM_t *motor)

{

    Motor_DM_Tx_Info_t *motor_tx_info = motor->tx_info;

    static float pmin, pmax, vmax, vmin, cmax, cmin, tmax, tmin;

    if (motor->type == dm_6006)

    {

        pmax = P_MAX_6006;

        pmin = P_MIN_6006;

        vmax = V_MAX_6006;

        vmin = V_MIN_6006;

        cmax = C_MAX_6006;

        cmin = C_MIN_6006;

        tmax = T_MAX_6006;

        tmin = T_MIN_6006;
    }

    else if (motor->type == dm_4310)

    {

        pmax = P_MAX_4310;

        pmin = P_MIN_4310;

        vmax = V_MAX_4310;

        vmin = V_MIN_4310;

        cmax = C_MAX_4310;

        cmin = C_MIN_4310;

        tmax = T_MAX_4310;

        tmin = T_MIN_4310;
    }

    else /*(motor->type == leg_8009)*/

    {

        pmax = P_MAX_8009;

        pmin = P_MIN_8009;

        vmax = V_MAX_8009;

        vmin = V_MIN_8009;

        cmax = C_MAX_8009;

        cmin = C_MIN_8009;

        tmax = T_MAX_8009;

        tmin = T_MIN_8009;
    }

    uint16_t p, v, kp, kd, t;

    uint8_t *buf = motor_tx_info->single_tx_buff;

    /* 限制输入的参数在定义的范围内 */

    motor_tx_info->target_angle = constrain(motor_tx_info->target_angle, pmin, pmax);

    motor_tx_info->target_speed = constrain(motor_tx_info->target_speed, vmin, vmax);

    motor_tx_info->Kp = constrain(motor_tx_info->Kp, KP_MIN, KP_MAX);

    motor_tx_info->Kd = constrain(motor_tx_info->Kd, KD_MIN, KD_MAX);

    motor_tx_info->torque = constrain(motor_tx_info->torque, tmin, tmax);

    /* 根据协议，对float参数进行转换 */

    p = float_to_uint(motor_tx_info->target_angle, pmin, pmax, 16);

    v = float_to_uint(motor_tx_info->target_speed, vmin, vmax, 12);

    kp = float_to_uint(motor_tx_info->Kp, KP_MIN, KP_MAX, 12);

    kd = float_to_uint(motor_tx_info->Kd, KD_MIN, KD_MAX, 12);

    t = float_to_uint(motor_tx_info->torque, tmin, tmax, 12);

    /* 根据传输协议，把数据转换为CAN命令数据字段 */

    buf[0] = p >> 8;

    buf[1] = p & 0xFF;

    buf[2] = v >> 4;

    buf[3] = ((v & 0xF) << 4) | (kp >> 8);

    buf[4] = kp & 0xFF;

    buf[5] = kd >> 4;

    buf[6] = ((kd & 0xF) << 4) | (t >> 8);

    buf[7] = t & 0xff;

    Motor_Send_Data(motor, buf);
}

/**

 * @brief  将float转为uint，并对正负做处理,与通信协议保持一致

 * @param

 * @retval

 */

static uint16_t float_to_uint_(float x, float x_min, float x_max, uint8_t bits)

{

    float span = x_max - x_min;

    float offset = x_min;

    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/**

 * @brief  将uint转为float，并对正负做处理

 * @param

 * @retval

 */

static float uint_to_float_(uint16_t x_int, float x_min, float x_max, uint8_t bits)

{

    float span = x_max - x_min;

    float offset = x_min;

    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**

 * @brief          计算电机旋转角度和

 * @param[in]      Motor_DM_t *motor     电机本体

 * @retval         none

 */

static void Angle_Sum_Cal(Motor_DM_t *motor)

{

    float err = 0.f;

    float order_correction = 0.f;

    if (motor->born_info->order_correction == 1 || motor->born_info->order_correction == -1)

    {

        order_correction = (float)motor->born_info->order_correction;
    }

    else

    {

        order_correction = 1.f;
    }

    if (!motor->rx_info->motor_angle_last && !motor->rx_info->motor_angle_sum) // 上一角度值为0且角度和为零时（电机启动），不计算误差

    {

        err = 0.f;
    }

    else

    {

        err = motor->rx_info->motor_angle - motor->rx_info->motor_angle_last;
    }

    if (my_abs(err) > (float)PI) // 过零点

    {

        if (err > 0.f)

        {

            motor->rx_info->motor_angle_sum += (-(float)PI * 2.f + err) * order_correction;
        }

        else

        {

            motor->rx_info->motor_angle_sum += ((float)PI * 2.f + err) * order_correction;
        }
    }

    else

    {

        motor->rx_info->motor_angle_sum += err * order_correction;
    }

    motor->rx_info->motor_angle_last = motor->rx_info->motor_angle;
}

/**

 * @brief          判断电机错误码

 * @param[in]      Motor_DM_t *motor     电机本体

 * @retval         none

 */

static void Motor_ERR_Check(Motor_DM_t *motor, uint8_t err_word)

{

    Motor_DM_State_t *my_state = motor->state;

    //	static Motor_DM_Work_state_e temp_state = Motor_Unenable;

    switch (err_word)

    {

    case 0:

        my_state->motor_state = Motor_Unenable;

        break;

    case 1:

        my_state->motor_state = Motor_Enable;

        break;

    case 8:

        my_state->motor_state = Over_Voltage;

        break;

    case 9:

        my_state->motor_state = Lack_Voltage;

        break;

    case 10:

        my_state->motor_state = Over_Current;

        break;

    case 11:

        my_state->motor_state = MOS_OverTemp;

        break;

    case 12:

        my_state->motor_state = Motor_OverTemp;

        break;

    case 13:

        my_state->motor_state = Commun_Loss;

        break;

    default:

        my_state->motor_state = Unknow_Err;

        break;
    };

    /*获取上一次不同于当前的电机状态*/

    //	if(temp_state != my_state->motor_state)

    //	{

    //		if(temp_state != my_state->last_motor_state)

    //		{

    //			my_state->last_motor_state = temp_state;

    //		}

    //	}

    //	temp_state = my_state->motor_state;

    if (my_state->motor_state != Motor_Enable &&

        my_state->motor_state != Motor_Unenable)

    {

        my_state->last_motor_state = my_state->motor_state;
    }
}

/*示例代码*/

/*-------------电机变量创建-------------*/

/*

Motor_DM_Born_Info_t Yaw_Born_Info =

{

    .txId = 0x001,//电机控制报文ID



    .hcan = &hfdcan2,//使用的Can总线



};



Motor_DM_Rx_Info_t Yaw_Rx_Info_t;



Motor_DM_Tx_Info_t Yaw_Tx_Info_t;



Motor_DM_State_t Yaw_State_t;



Motor_DM_t Yaw_Motor =

{

    .born_info = &Yaw_Born_Info,



    .rx_info = &Yaw_Rx_Info_t,



    .tx_info = &Yaw_Tx_Info_t,



    .state = &Yaw_State_t,



    .single_init = &DM_Single_Motor_Init,

};

*/

/*-------------初始化-------------*/

/*

Yaw_Motor.single_init(&Yaw_Motor);

*/

/*-------------接收函数-------------*/

/*

void CAN2_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)

{

    switch (rxId)

    {

        case 0x000://接收ID

        Yaw_Motor.rx(&Yaw_Motor, rxBuf);

        break;

        default:

            break;

    }

}

*/

/*-------------任务执行-------------*/

/*

  * @file    monitor_task.c

  * @brief   监控任务

  *          1. 各模块心跳失联检测

  *          2. 监控遥控器状态，软件复位

void StartMonitorTask(void const * argument)//

{



    for(;;)

    {

        Yaw_Motor.heartbeat(&Yaw_Motor);



        osDelay(1);

    }

}



  * @file    monitor_task.c

  * @brief   电机控制任务

  *          1. 给电机发送控制报文

  *          2. 对状态标志位进行响应

void StartMonitorTask(void const * argument)//

{



    for(;;)

    {

        //发送控制报文，控制电机输出扭矩为0.5N*m

        Yaw_Motor.tx_info->torque = 0.5f;

        Yaw_Motor.single_set_torque(&Yaw_Motor);



        osDelay(1);

    }

}



*/
