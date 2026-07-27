# 小串腿机器人主控固件

本工程是小串腿机器人主控板固件，基于 STM32H723VGT6。当前版本适配自研主控板 V3，用于连接两块 MiyaFOC 轮毂电机驱动板、两路 STS3032 串行舵机、MPU6050 姿态模块、nRF24L01/无线透传模块以及 PC/VOFA+ 调试上位机。

当前策略是安全上电：主控启动后不自动使能轮毂电机，不自动发送舵机位置，所有执行机构都需要通过 PC 调试串口或后续无线链路明确下发命令。

## 工程入口

- Keil 工程：`MDK-ARM/MiniWheelLegRobot.uvprojx`
- CubeMX 配置：`MiniWheelLegRobot.ioc`
- 板级驱动：`Core/Src`
- 驱动适配层：`Application/DriverLayer`
- 机器人代码：`Application/MiniRobot`
- 参数总表：`Application/MiniRobot/Config/mini_robot_config.h`

## 硬件接口

| 功能 | MCU 引脚 | 配置 |
| --- | --- | --- |
| PC/VOFA+ | UART7 RX=PE7，TX=PE8 | 115200，8N1，RX-to-idle DMA |
| 左 STS3032 | USART1 RX=PA10，TX=PA9 | 1 Mbps，8N1 |
| 右 STS3032 | USART10 RX=PE2，TX=PE3 | 1 Mbps，8N1 |
| 左 FOC | FDCAN1 RX=PD0，TX=PD1 | Classic CAN，1 Mbps |
| 右 FOC | FDCAN2 RX=PB5，TX=PB6 | Classic CAN，1 Mbps |
| MPU6050 | I2C2 SCL=PB10，SDA=PB11 | 100 kHz，板载 4.7 kOhm 上拉 |
| nRF24L01 | SPI1 SCK=PA5，MISO=PA6，MOSI=PA7 | Mode 0 |
| nRF24L01 控制 | CE=PC4，CSN=PC5 | CE 下拉，CSN 上拉 |
| SWD | SWDIO=PA13，SWCLK=PA14 | 下载与调试 |

USART1/USART10 是 MCU 的全双工串口。STS3032 的三线单总线信号需要外部半双工/全双工转换电路，不能把 TX 和 RX 直接硬短接。舵机电源地必须与主控地共地。默认左舵机 ID=1，右舵机 ID=2。

## 指示灯

通信灯和心跳灯均为 GPIO 高电平点亮。

| 引脚 | 含义 |
| --- | --- |
| PE5 | USART10 右舵机收发活动 |
| PC7 | USART1 左舵机收发活动 |
| PD3 | CAN1 收发活动 |
| PD7 | CAN2 收发活动 |
| PD5 | 程序心跳，约 1 Hz 完整闪烁周期 |

通信灯采用短脉冲限频显示，表示 MCU 确实执行了对应接口的收发动作。舵机写命令没有应答时只能说明发包成功，需用 `servo ping` 或 `servo read` 判断舵机是否真实回包。

## 编译和下载

1. 用 Keil 打开 `MDK-ARM/MiniWheelLegRobot.uvprojx`。
2. 选择 `MiniWheelLegRobot` target，执行 Build。
3. 使用 ST-Link 连接 PA13、PA14、3V3、GND 和 NRST 下载。
4. UART7 接 USB-TTL：板 TX 接模块 RX，板 RX 接模块 TX，并共地。
5. VOFA+ 串口设置为 115200、8N1、无流控。

启动后 UART7 应输出类似信息：

```text
mini controller v2 init
pc: uart7 pe7/pe8 115200 8N1
servos: usart1 pa9/pa10, usart10 pe3/pe2, 1Mbps
telemetry default off, send telemetry 1 for JustFloat
mpu6050 i2c2=1, nrf24 spi1=1
safe boot: motors stopped, no servo command sent
```

HAL 状态 `0` 表示初始化成功。MPU6050 或 nRF24L01 未安装时会报告非零，但不会阻止 UART7、CAN 和舵机测试。

## PC 调试命令

文本命令以换行结束。建议优先使用带 `foc` 前缀的命令，避免与旧命令混淆。

```text
help
telemetry 0
telemetry 1
can stat
stop

foc speed 1 3
foc speed 1 10
foc speed 1 0
foc pos 1 30
foc torque 1 0.01
foc stop 1

vel 0.2 0.0
enable 1
pos 6.28 6.28

servo ping 1
servo read 1
servo torque 1 1
servo pos 1 2048
servo pair 1800 2296
servo shutdown

pid speed 0 0.20 0.02 0.00
pid pos 0 4.0 0.0 0.0
lqr enable 1
lqr k 0 0 -1.0
lqr target 0 0 0 0 0 0
lqr limit 4.0
```

## MiyaFOC CAN 通信

当前主控与 MiyaFOC 已完成实测联调。主控通过 CAN1/CAN2 分别连接左右 FOC 板，每条总线一块 FOC，因此两块 FOC 均可保持节点 ID `1`。

- CAN 波特率：1 Mbps
- 主控发送命令标准帧：`0x211`
- FOC 反馈标准帧：`0x291`
- 反馈周期：10 ms
- 命令值单位与 MiyaFOC 串口命令保持一致：
  - `foc speed 1 x` 等价于 FOC 串口 `SPEED:x`
  - `foc pos 1 x` 等价于 FOC 串口 `POSITION:x`
  - `foc torque 1 x` 等价于 FOC 串口 `TORQUE:x`
  - `foc stop 1` 等价于 FOC 串口 `STOP`

注意：`foc pos 1 0` 是转到绝对零位，不是停机。需要停机时使用 `foc stop 1` 或全局 `stop`。

## VOFA+ 遥测

默认关闭 JustFloat 遥测，避免刷屏影响调试。发送：

```text
telemetry 1
```

即可打开 VOFA+ 曲线输出。发送：

```text
telemetry 0
```

关闭遥测。建议调 CAN、舵机和命令解析时先关闭遥测，只看文本日志；调速度环、位置环和姿态时再打开曲线。

## MPU6050、PID 和 LQR

MPU6050 用于获取车体俯仰角和角速度。安装方向不一致时调整：

```c
MINI_MPU6050_PITCH_SIGN
MINI_MPU6050_PITCH_OFFSET_RAD
```

PID、LQR 初值和遥控映射系数位于：

```text
Application/MiniRobot/Config/mini_robot_config.h
```

LQR 的 A、B、K 默认作为可调参数保留，需要根据实车模型、轮距、质心高度、舵机腿长和轮毂响应重新填写或通过 VOFA+ 在线下发。

## 调试建议

1. 主控单板上电，确认 PD5 心跳、UART7 启动信息、MPU6050 姿态数据。
2. 分别测试 CAN1/CAN2 到 MiyaFOC，先空载低速 `foc speed 1 3`，再测试 `foc stop 1`。
3. 半双工转换板完成后，分别测试左右 STS3032 的 `ping/read/torque/pos`。
4. 机械结构装好前，只做架空测试，不做落地闭环。
5. 机械装好后，先锁舵机高度，再低速测试轮毂正反转方向。
6. 最后再进入站立平衡、LQR 和跳跃相关状态机调试。

初次调试时必须架空车体、设置电流/速度上限，并准备物理断电开关。
