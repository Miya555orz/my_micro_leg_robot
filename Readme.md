# 小串腿机器人主控固件

适配主控 V2（STM32H723VGT6）。工程使用 HAL、FreeRTOS、Keil MDK-ARM，
包含双轮毂 FOC、双 STS3032、MPU6050、nRF24L01 和 VOFA+ 调试接口。

## 工程入口

- Keil 工程：`MDK-ARM/MiniWheelLegRobot.uvprojx`
- CubeMX 配置：`MiniWheelLegRobot.ioc`
- 板级驱动：`Core/Src`
- 驱动适配层：`Application/DriverLayer`
- 机器人代码：`Application/MiniRobot`
- 参数总表：`Application/MiniRobot/Config/mini_robot_config.h`

上电采用安全策略：轮毂保持停止，不自动使能舵机，也不自动发送舵机位置。

## 最新引脚

| 功能 | MCU 引脚 | 配置 |
|---|---|---|
| PC/VOFA+ | UART7 RX=PE7，TX=PE8 | 115200，8N1，RX-to-idle DMA |
| 左 STS3032 | USART1 RX=PA10，TX=PA9 | 1 Mbps，8N1 |
| 右 STS3032 | USART10 RX=PE2，TX=PE3 | 1 Mbps，8N1 |
| 左 FOC | FDCAN1 RX=PD0，TX=PD1 | Classic CAN，1 Mbps |
| 右 FOC | FDCAN2 RX=PB5，TX=PB6 | Classic CAN，1 Mbps |
| MPU6050 | I2C2 SCL=PB10，SDA=PB11 | 100 kHz，板载 4.7 kOhm 上拉 |
| nRF24L01 | SPI1 SCK=PA5，MISO=PA6，MOSI=PA7 | Mode 0，7.5 Mbps |
| nRF24L01 控制 | CE=PC4，CSN=PC5 | CE 下拉，CSN 上拉，轮询接收 |
| SWD | SWDIO=PA13，SWCLK=PA14 | 下载与调试 |

USART1/USART10 接口是 MCU 的全双工 TX/RX。三线单总线 STS3032 不能把
TX 与 RX 直接硬短接，需使用板外半双工转换电路；舵机信号、电源地和主控地
必须共地。默认左舵机 ID=1，右舵机 ID=2。

## 指示灯

通信灯和心跳灯均为 GPIO 高电平点亮。原理图中的 10 kOhm LED 电阻装配时可换
为 1 kOhm；颜色较暗时可按 LED 额定电流继续核算，但不要直接短接电阻。

| 引脚 | 含义 |
|---|---|
| PE5 | USART10 右舵机收发活动 |
| PC7 | USART1 左舵机收发活动 |
| PD3 | CAN1 收发活动 |
| PD7 | CAN2 收发活动 |
| PD5 | 程序心跳，约 1 Hz 完整闪烁周期 |

通信灯采用 25 ms 短脉冲并限制为最快 80 ms 闪一次，避免 CAN 高频通信时常亮刺眼。
通信灯表示 MCU 确实完成了对应接口收发；舵机写命令没有应答时，只能证明发包成功。
使用 `servo ping` 或 `servo read` 才能确认舵机真实回包。

## 编译和下载

1. 用 Keil 打开 `MDK-ARM/MiniWheelLegRobot.uvprojx`。
2. 选择 `MiniWheelLegRobot` target，执行 Build。
3. 使用 ST-Link 连接 PA13、PA14、3V3、GND 和 NRST（推荐），下载 AXF/HEX。
4. UART7 接 USB-TTL：板 TX 接模块 RX，板 RX 接模块 TX，并共地。
5. VOFA+ 串口设置为 115200、8N1、无流控。

启动后 UART7 应输出：

```text
mini controller v2 init
pc: uart7 pe7/pe8 115200 8N1
servos: usart1 pa9/pa10, usart10 pe3/pe2, 1Mbps
mpu6050 i2c2=0, nrf24 spi1=0
safe boot: motors stopped, no servo command sent
```

HAL 状态 `0` 表示初始化成功。MPU6050 或 nRF24L01 未安装时会报告非零，但不会
阻止 UART7、CAN 和舵机测试。

## VOFA+ 命令

文本命令以换行结束：

```text
vel 0.2 0.0
enable 1
stop
pos 6.28 6.28
pid speed 0 0.20 0.02 0.00
pid pos 0 4.0 0.0 0.0
servo ping 1
servo read 1
servo torque 1 1
servo pos 1 2048
servo pair 1800 2296
servo shutdown
lqr enable 1
lqr k 0 0 -1.0
lqr target 0 0 0 0 0 0
lqr limit 4.0
```

`pid` 的轮编号为 0/1。`lqr k <输入> <状态> <值>` 中输入为左右轮 0/1，
状态顺序如下：

1. 车体俯仰角 rad
2. 车体俯仰角速度 rad/s
3. 左轮位置 rad
4. 左轮速度 rps
5. 右轮位置 rad
6. 右轮速度 rps

VOFA+ FireWater 每 10 ms 收到 8 个 float：左右实际轮速、左右目标轮速、
`vx`、`wz`、左右舵机目标位置，帧尾为 `00 00 80 7F`。

## nRF24L01

主控工作在接收模式，不使用 IRQ，CommandTask 每 20 ms 轮询：

- 地址：`TANK1`
- 信道：76
- 空中速率：1 Mbps
- 固定载荷：32 字节
- CRC：2 字节
- Auto ACK：开启

接收格式与 `D:\github_prj\遥控器程序\遥控器程序` 一致。左摇杆 Y 控制前后，
左摇杆 X 控制转向，右摇杆 Y 控制双舵机升降；按键 2 选择快速档，默认慢速档。
按键 1 已解析为跳跃请求，但在加入限位、姿态判定和落地保护状态机前不会执行跳跃。
遥控失联超过 300 ms 后，底盘控制自动失能并发送零输出。

## MPU6050、PID 和 LQR

MPU6050 配置为加速度 +/-4 g、陀螺仪 +/-500 deg/s，互补滤波得到俯仰角。
安装方向不一致时调整：

```c
MINI_MPU6050_PITCH_SIGN
MINI_MPU6050_PITCH_OFFSET_RAD
```

PID、LQR 初值和所有遥控映射系数都在
`Application/MiniRobot/Config/mini_robot_config.h`。LQR 的 A、B、K 默认全为
0，必须根据实车模型填写或通过 VOFA+ 在线下发后再执行 `lqr enable 1`。

## 建议上电顺序

1. 只给主控供电，确认 PD5 心跳和 UART7 启动信息。
2. 测 MPU6050 与 nRF24L01 初始化状态。
3. 分别连接左右舵机，先 `ping`、`read`，再使能扭矩和小范围位置测试。
4. 分别连接 CAN1、CAN2，悬空车轮测试反馈和低电流命令。
5. 最后落地调速度 PID，再进行位置环和 LQR 调试。

初次调试时架空车体、设置电流上限并准备断电开关。
