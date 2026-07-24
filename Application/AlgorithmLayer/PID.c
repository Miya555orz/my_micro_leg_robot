#include "pid.h"

#include "rp_math.h"

#define M_PI 3.1415926

/**

 *  @name   single_pid_ctrl

 *	@brief  新的pid计算，对微分进行低通滤波，积分进行变速

 *  @author HERMIT_PURPLE

 */

void single_pid_ctrl(pid_ctrl_t *pid)

{

    // 保存误差值(需要在外面自行计算误差)

    // pid->err = pid->target-pid->measure;



    pid->integral += pid->err;



    pid->integral = constrain(pid->integral, -pid->integral_max, +pid->integral_max);

    // p i d 输出项计算

    pid->pout = pid->kp * pid->err;

    pid->iout = pid->ki * pid->integral;

    // 对微分项低通滤波

    pid->dout = Lowpass(pid->last_dout, pid->kd * (pid->err - pid->last_err), pid->a);

    pid->last_dout = pid->dout;

    // 累加pid输出值

    pid->out = pid->pout + pid->iout + pid->dout;

    pid->out = constrain(pid->out, -pid->out_max, pid->out_max);

    // 记录上次误差值

    pid->last_err = pid->err;

}



/*离线状态下对积分清零*/

void pid_clear(pid_ctrl_t *pid)

{

    pid->integral = 0;
    pid->last_err = 0;
    pid->last_dout = 0;
    pid->pout = 0;
    pid->iout = 0;
    pid->dout = 0;
    pid->out = 0;

}



void pid_err_cal(pid_ctrl_t *pid)

{

    pid->err = pid->target - pid->measure;

}

/**

 *	@brief	pid总控制 参数：外环 内环  外环或内环目标值 外环观测值 内环观测值  内环观测值kp，一般填负的 err处理方式

 *          err_cal_mode：err处理方式 半圈还是四分之一圈 0，1，2 速度环使用0 yaw轴使用1

                        陀螺仪角度环 3

 *         内环不能为NULL

 *	@note   使用示例：

            pid_ctrl_t *out	  = ;

            pid_ctrl_t *inn	  = ;

            float target   	  = ;

            float mea_out       = ;

            float mea_in        = ;

            float inner_kp      = ;

            uint8_t err_cal_mode= ;

            =all_pid_calc (out,inn,target,mea_out,mea_in,inner_kp,err_cal_mode);

 *  @author HERMIT_PURPLE

 *

 *  @return 返回计算结果

 */



float all_pid_calc(pid_ctrl_t *out, pid_ctrl_t *inn, float target, float mea_out, float mea_in, float inner_kp, uint8_t err_cal_mode)

{

    if (inn == NULL)

        return 0; // 没有内环，为0



    else if (out == NULL && inn != NULL) // 只有速度环

    {

        inn->target = target;

        inn->measure = mea_in;

        inn->err = inn->target - mea_in;

        single_pid_ctrl(inn);

        return inn->out;

    }



    else if (out != NULL && inn != NULL) // 双环PID

    {



        out->target = target;

        out->measure = mea_out;

        out->err = out->target - out->measure; // 计算角度环误差，后面再进行误差处理

        switch (err_cal_mode)

        {



        case 0:

            break;



        case 1:

            out->err = half_cycle(out->err, 8191);

            break;



        case 2:

            out->err = half_cycle(out->err, 8191);

            out->err = half_cycle(out->err, 4095);

            break;



        case 3:

            out->err = half_cycle(out->err, 360);

            break;



        case 4:

            out->err = half_cycle(out->err, 65535);

            break;



        case 5:

            out->err = half_cycle(out->err, 2 * M_PI);

            break;



        default:

            break;

        }



        single_pid_ctrl(out);                  // 计算出处理过误差的角度环的值

        inn->target = out->out;                // 角度环输出作为速度环目标值

        inn->measure = mea_in * inner_kp;      // 内环输入kp，可以调整正负和大小

        inn->err = inn->target + inn->measure; // 速度环误差计算

        single_pid_ctrl(inn);

        return inn->out; // 输出内环计算值

    }

    else // 只有角度环

    {

        return 0;

    }

}

