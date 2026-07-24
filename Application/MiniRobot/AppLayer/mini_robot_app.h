#ifndef MINI_ROBOT_APP_H
#define MINI_ROBOT_APP_H

#include <stdint.h>

void MiniRobot_Init(void);
void MiniRobot_ControlStep(void);
void MiniRobot_TelemetryStep(void);
void MiniRobot_CommandStep(void);

#endif
