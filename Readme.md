# Mini Wheel-Leg Robot Controller / 小串腿机器人主控固件

STM32H723VGT6 firmware for a compact wheel-leg robot controller. It brings up two Feetech STS serial servos, two CAN FOC hub-motor driver links, an MPU6050 IMU, and a UART7 debug console.

这是一个基于 STM32H723VGT6 的小型轮腿机器人主控固件，用于调试两路飞特 STS 串行舵机、两路 CAN FOC 轮毂电机驱动、MPU6050 IMU，以及 UART7 串口调试链路。

The repository is currently focused on hardware bring-up, safety interlocks, and single-module tests. Balance-control parameters are intentionally kept in configuration files for bench tuning.

当前仓库主要用于硬件联调、安全保护和单模块测试。平衡控制参数保留在配置文件中，方便后续在实物台架上逐步整定。

## Hardware / 硬件接口

| Module | Interface | Notes |
| --- | --- | --- |
| Main MCU / 主控 MCU | STM32H723VGT6 | Keil MDK project generated from CubeMX |
| PC debug / 上位机调试 | UART7, 115200 8N1 | Text commands and VOFA telemetry |
| Left servo / 左舵机 | USART10, 1 Mbps | Feetech STS protocol, ID 1 |
| Right servo / 右舵机 | USART1, 1 Mbps | Feetech STS protocol, ID 1 |
| Left hub motor / 左轮毂驱动 | CAN1, 1 Mbps | MiyaFOC feedback ID `0x291` |
| Right hub motor / 右轮毂驱动 | CAN2, 1 Mbps | MiyaFOC feedback ID `0x291` on CAN2 |
| IMU / 姿态传感器 | I2C2 | MPU6050, address `0x68` |
| Status LEDs / 状态灯 | GPIO | Heartbeat and communication activity |

USART10 uses separate alternate functions on STM32H723: `PE2 / USART10_RX` is configured as `GPIO_AF4_USART10`, while `PE3 / USART10_TX` remains `GPIO_AF11_USART10`.

STM32H723 上 USART10 的两个引脚复用档位不同：`PE2 / USART10_RX` 配置为 `GPIO_AF4_USART10`，`PE3 / USART10_TX` 保持 `GPIO_AF11_USART10`。如果把 `PE2` 也配置成 AF11，左舵机链路会出现能发但收不到回包的现象。

## Power-On Safety / 上电安全状态

On boot, the firmware stays locked until the critical links are checked:

固件上电后默认保持锁定，直到关键链路检查通过：

- wheel motor commands are zero / 轮毂电机命令为 0；
- balance control is disabled / 平衡控制不开启；
- telemetry is off by default / 默认不发送连续遥测；
- servo motion is blocked when communication or position range checks fail / 舵机通信或位置范围检查失败时禁止运动；
- IMU initialization status is printed at boot / 上电会打印 IMU 初始化状态。

Do not enable balance control until servo positions, IMU raw data, and CAN feedback have been verified on the bench.

在确认舵机位置、IMU 原始数据和 CAN 反馈之前，不要开启平衡控制。

## UART7 Commands / UART7 串口命令

Basic help and status:

基础帮助和状态查询：

```text
help
servo status
imu status
imu raw
imu angle
motor status
control status
```

Servo bring-up:

舵机调试：

```text
servo ping left
servo ping right
servo left pos
servo right pos
servo left set <pos>
servo right set <pos>
servo pair <left_pos> <right_pos>
servo safe
```

Motor and control bring-up:

电机与控制调试：

```text
motor status
motor left <speed>
motor right <speed>
control enable
control disable
stop
```

Compatibility commands from earlier bring-up code are still parsed for low-level tests, including `foc speed`, `foc torque`, `can restart`, `telemetry`, `pid`, and `lqr` commands.

早期联调用的兼容命令仍然保留解析，包括 `foc speed`、`foc torque`、`can restart`、`telemetry`、`pid` 和 `lqr` 相关命令。

## Bring-Up Checklist / 联调检查顺序

1. Power the controller only and check the UART7 ready message.
2. Query `imu status` and `imu raw`. A healthy MPU6050 should report `init=1`, `online=1`, `who=0x68`, and one acceleration axis near gravity.
3. Query both servos with `servo ping left`, `servo ping right`, `servo left pos`, and `servo right pos`.
4. Confirm servo raw positions are inside the configured safe ranges before sending any movement command.
5. Query `motor status`. `online=1` means the controller has received a matching CAN feedback frame; it does not prove the motor phase wires are connected.
6. Test CAN1 and CAN2 separately with one FOC driver board per bus.
7. Enable telemetry and closed-loop control only after all sensor and actuator channels are plausible.

中文步骤：

1. 只给主控上电，先确认 UART7 ready 信息正常。
2. 查询 `imu status` 和 `imu raw`。正常 MPU6050 应显示 `init=1`、`online=1`、`who=0x68`，且至少一个加速度轴接近重力加速度。
3. 用 `servo ping left`、`servo ping right`、`servo left pos`、`servo right pos` 分别检查两路舵机。
4. 只有舵机原始位置落在配置的安全范围内，才允许发送运动命令。
5. 查询 `motor status`。`online=1` 只表示主控收到了匹配的 CAN 反馈帧，不等于电机相线一定已经连接。
6. CAN1 和 CAN2 分开测试，每次只接一路 FOC 驱动板。
7. 所有传感器和执行器数据可信之后，再开启遥测和闭环控制。

## Key Parameters / 关键参数

Most project parameters are in:

主要配置集中在：

```text
Application/MiniRobot/Config/mini_robot_config.h
```

Current important defaults:

当前重要默认值：

| Parameter | Value |
| --- | --- |
| STS baudrate / 舵机波特率 | 1 Mbps |
| Left servo ID / 左舵机 ID | 1 |
| Right servo ID / 右舵机 ID | 1 |
| Left safe position / 左舵机安全目标 | 2148 |
| Right safe position / 右舵机安全目标 | 1948 |
| Left safe range / 左舵机安全范围 | 2110..2510 |
| Right safe range / 右舵机安全范围 | 1586..1986 |
| Default servo speed / 默认舵机速度 | 200 |
| Default servo acceleration / 默认舵机加速度 | 8 |
| CAN command period / CAN 命令周期 | 2 ms |
| Command timeout / 命令超时 | 300 ms |
| MPU6050 I2C address / IMU 地址 | `0x68` |

## Diagnostics / 当前诊断输出

The UART status output is intentionally verbose during bring-up:

联调阶段串口状态输出会故意保留较多诊断信息：

- `servo safety fault=0x...` shows communication and range-lock reasons.
- `imu status` prints `init`, `online`, `who`, update timestamp, failure count, raw acceleration, raw gyro, and fused pitch.
- `motor status` prints motor online age plus the latest CAN RX ID and payload on CAN1/CAN2.

含义如下：

- `servo safety fault=0x...` 用于判断舵机通信和位置范围锁定原因。
- `imu status` 会打印 `init`、`online`、`who`、更新时间、失败计数、原始加速度、原始陀螺仪和融合 pitch。
- `motor status` 会打印电机 online 年龄，以及 CAN1/CAN2 最近收到的 ID 和数据。

## Build / 编译

The Keil MDK project is:

Keil MDK 工程位于：

```text
MDK-ARM/MiniWheelLegRobot.uvprojx
```

Command-line rebuild example:

命令行全量编译示例：

```text
D:\keil5\UV4\UV4.exe -r MDK-ARM\MiniWheelLegRobot.uvprojx -t MiniWheelLegRobot
```

The latest local rebuild after the UART, IMU, CAN, and safety-diagnostic changes passed with `0 Error(s), 0 Warning(s)`.

本次 USART、IMU、CAN 和安全诊断相关修改后，已在本地完成全量编译，结果为 `0 Error(s), 0 Warning(s)`。

## Repository Notes / 仓库说明

Commit source files, the CubeMX `.ioc`, Keil project files, and documentation. Generated outputs such as `build/`, `MDK-ARM/MiniWheelLegRobot/`, `.o`, `.axf`, `.hex`, `.map`, `.htm`, and local `.uvguix.*` files should stay out of normal Git commits.

建议提交源码、CubeMX `.ioc`、Keil 工程文件和文档。`build/`、`MDK-ARM/MiniWheelLegRobot/`、`.o`、`.axf`、`.hex`、`.map`、`.htm`、本机 `.uvguix.*` 等生成文件不建议作为常规提交内容。

Author / 作者: Miya Zheng
