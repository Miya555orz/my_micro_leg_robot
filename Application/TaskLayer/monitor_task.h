#ifndef __MONITOR_TASK
#define __MONITOR_TASK

#include "main.h"
#include "cmsis_os.h"
#include "iwdg.h"
#include "command.h"
#include "communicate.h"
#include "DM_Motor.h"
#include "Chassis.h"
void StartMonitorTask(void const * argument);

#endif
