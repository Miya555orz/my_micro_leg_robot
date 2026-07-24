#ifndef __CONTROL_TASK
#define __CONTROL_TASK

#include "cmsis_os.h"
#include "main.h"
#include "Chassis.h"
#include "command.h"
#include "RP_Log.h"
void StartCtrlTask(void const * argument);

#endif
