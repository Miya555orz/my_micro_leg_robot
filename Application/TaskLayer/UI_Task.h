#ifndef __UI_TASK
#define __UI_TASK

#include "main.h"
#include "cmsis_os.h"
#include "drv_uart.h"
#include "chassis.h"
#include "Balance.h"
#include "ui.h"
#include "ui_priority.h"
void StartRCTask(void const * argument);

#endif
