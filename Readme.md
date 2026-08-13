# Mini Wheel-Leg Robot Controller

STM32H723VGT6 firmware for a compact wheel-leg robot controller. The board brings up two hub motors through CAN, two Feetech STS serial servos through UART, an MPU6050 IMU, and a UART7 debug link for VOFA or a serial terminal.

This repository is currently used for hardware bring-up and single-module control tests. Balance control parameters are intentionally left in configuration files so they can be tuned on the real robot.

## Hardware

| Module | Interface | Notes |
| --- | --- | --- |
| Main MCU | STM32H723VGT6 | Keil MDK project generated from CubeMX |
| PC debug | UART7, 115200 8N1 | Text commands and telemetry |
| Left servo | USART10, 1 Mbps | Feetech STS protocol |
| Right servo | USART1, 1 Mbps | Feetech STS protocol |
| Left hub motor | CAN1, 1 Mbps | MiyaFOC board |
| Right hub motor | CAN2, 1 Mbps | MiyaFOC board |
| IMU | I2C2 | MPU6050 module |
| Status LEDs | GPIO | Heartbeat and communication activity |

## Power-On State

The firmware starts in a safe state:

- wheel motor commands are zero;
- servo torque is not enabled automatically;
- telemetry is off by default;
- UART7 prints a short ready message.

Send `stand` only after the servos, CAN feedback, and IMU angle are confirmed correct on the bench.

## UART7 Commands

Servo bring-up:

```text
servo ping l
servo ping r
servo read l
servo read r
servo torque all 1
servo torque all 0
servo pos l 2048
servo pos r 2048
servo both 2048
servo pair 1800 2300
```

FOC bring-up:

```text
foc stop 1
foc stop 2
foc speed 1 3
foc speed 2 3
foc torque 1 0.05
foc torque 2 0.05
```

Telemetry:

```text
telemetry 1
telemetry 0
```

## Important Parameters

Most project parameters are in:

```text
Application/MiniRobot/Config/mini_robot_config.h
```

STS servo defaults:

- baudrate: 1 Mbps;
- ID: left = 1, right = 1;
- center position: 2048;
- raw range: 0..4095;
- default move time: 300 ms;
- default move speed: 300;
- default acceleration: 30.

## Bring-Up Checklist

1. Power the controller only and check the heartbeat LED.
2. Connect UART7 and confirm the ready message.
3. Test one servo at a time: `servo ping`, `servo read`, `servo torque`, then a small `servo pos` step.
4. Test CAN1 and CAN2 separately with one FOC board per bus.
5. Enable telemetry only after the communication links are stable.
6. Tune balance parameters with the robot mechanically constrained.

## Repository Notes

Commit source files, the CubeMX `.ioc`, Keil project files, and documentation. Keep generated folders such as `build/` and `MDK-ARM/MiniWheelLegRobot/` out of Git.

Author: Miya Zheng
