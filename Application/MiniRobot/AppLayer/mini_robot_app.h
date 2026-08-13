/**
 ******************************************************************************
 * @file    mini_robot_app.h
 * @brief   Task-level entry of the mini wheel-leg controller.
 * @author  Miya Zheng
 * @date    2026-08-02
 ******************************************************************************
 */
#ifndef MINI_ROBOT_APP_H
#define MINI_ROBOT_APP_H

void MiniRobot_Init(void);
void MiniRobot_ControlStep(void);
void MiniRobot_CommandStep(void);
void MiniRobot_TelemetryStep(void);

#endif
