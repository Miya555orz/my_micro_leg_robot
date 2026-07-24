#ifndef __PID_H
#define __PID_H
#include "stdlib.h"
#include "stdint.h"

typedef struct feedforward {
	float ka;
	float kb;  //二阶差分系数太大容易急停
	
	float val_now;
	float val_last;
	float val_pre;
	
	float const_val;//前馈常数
	float feedforward_outmax;
	float feed_val;

	//	int16_t		(*cal)(struct Feedforward_Struct_t*	Feedforward);
} feedforward_t;


typedef struct pid_ctrl {
	float	target;
	float	measure;
	float 	err;
	float 	last_err;
	float	kp;
	float 	ki;
	float 	kd;
	float 	pout;
	float 	iout;
	float 	dout;
	float 	out;
	float  	last_dout;
	float	integral;
	float 	integral_max;
	float 	out_max;
	float 	a;//低通滤波系数
	feedforward_t 	   feedforward;
} pid_ctrl_t;

void integral_to_zero(pid_ctrl_t *pid);
void pid_clear(pid_ctrl_t *pid);
void pid_err_cal(pid_ctrl_t *pid);
void single_pid_ctrl(pid_ctrl_t *pid);
float  all_pid_calc (pid_ctrl_t *out,pid_ctrl_t *inn,float target,float mea_out,float mea_in,float inner_kp,uint8_t err_cal_mode);
float feedforward_pid_calc(pid_ctrl_t *out,pid_ctrl_t *inn,float target,float mea_out,float mea_in,float inner_kp,uint8_t err_cal_mode);
#endif


