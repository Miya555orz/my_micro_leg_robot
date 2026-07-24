#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "main.h"
#include "task.h"

osThreadId_t CtrlTaskHandle;
osThreadId_t CommandTaskHandle;
osThreadId_t MonitorTaskHandle;

static const osThreadAttr_t CtrlTask_attributes = {
    .name = "CtrlTask",
    .stack_size = 512U * 4U,
    .priority = (osPriority_t)osPriorityRealtime,
};

static const osThreadAttr_t CommandTask_attributes = {
    .name = "CommandTask",
    .stack_size = 384U * 4U,
    .priority = (osPriority_t)osPriorityAboveNormal,
};

static const osThreadAttr_t MonitorTask_attributes = {
    .name = "MonitorTask",
    .stack_size = 384U * 4U,
    .priority = (osPriority_t)osPriorityNormal,
};

void StartCtrlTask(void *argument);
void StartCommandTask(void *argument);
void StartMonitorTask(void *argument);

void MX_FREERTOS_Init(void)
{
    CtrlTaskHandle = osThreadNew(StartCtrlTask, NULL, &CtrlTask_attributes);
    CommandTaskHandle = osThreadNew(StartCommandTask, NULL, &CommandTask_attributes);
    MonitorTaskHandle = osThreadNew(StartMonitorTask, NULL, &MonitorTask_attributes);
}
