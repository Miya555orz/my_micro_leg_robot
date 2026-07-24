
/* Includes ------------------------------------------------------------------*/
#include "motor_def.h"

/**
 * @brief  电机PID单个PID结构体初始化
 */
void motor_pid_init(motor_pid_t *motor_pid,motor_pid_t extern_motor_pid)
{ 
	if (motor_pid == NULL) return;
	*motor_pid=extern_motor_pid;
}

 
