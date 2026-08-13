# MATLAB 单连杆轮腿建模说明

作者：Miya Zheng  
创建时间：2026-07-30

本目录新增 `single_link_wheel_leg_*` 脚本，用于小串腿机器人的单连杆轮式倒立摆建模。该模型不是五连杆模型，也不使用 VMC 腿力分配；当前目标是让 MPU6050 + 两个 FOC 轮毂电机先完成站立平衡。

## 文件说明

- `single_link_wheel_leg_params.m`：集中参数表，质量、质心、惯量、轮半径、FOC 力矩常数都在这里改。
- `single_link_wheel_leg_linear_model.m`：建立单连杆轮腿线性状态空间模型。
- `single_link_wheel_leg_lqr.m`：计算 LQR，并生成适配固件 `MINI_LQR_DEFAULT_K` 的 2x6 增益。
- `single_link_wheel_leg_export.m`：导出 `single_link_lqr_output.txt`。
- `single_link_wheel_leg_sim.m`：闭环仿真，观察 pitch 和轮位移是否收敛。

## 模型状态

MATLAB 基础模型使用 4 维状态：

```text
x = [theta; theta_dot; s; s_dot]
```

含义：

- `theta`：车身/单连杆相对直立的俯仰角，单位 rad
- `theta_dot`：俯仰角速度，单位 rad/s
- `s`：轮子地面位移，单位 m
- `s_dot`：轮子地面速度，单位 m/s

固件当前使用 6 维状态：

```text
[pitch, pitch_rate, left_pos, left_speed, right_pos, right_speed]
```

导出脚本会把 MATLAB 的 4 维 LQR 增益映射为固件需要的 2x6 增益，两行分别对应左右轮电流命令。

## 使用步骤

1. 从 SolidWorks/Inventor 的质量属性中读取整机等效质量、质心高度、pitch 转动惯量。
2. 填入 `single_link_wheel_leg_params.m`。
3. 在 MATLAB 中运行：

```matlab
result = single_link_wheel_leg_lqr();
sim = single_link_wheel_leg_sim();
```

4. 打开 `single_link_lqr_output.txt`，把 `MINI_LQR_DEFAULT_K` 复制到固件 `mini_robot_config.h`。
5. 上车前先把 `maxWheelCurrentA` 和固件 `MINI_BALANCE_LQR_OUTPUT_LIMIT_A` 调小，离地测试轮子方向。

## CAD 注意事项

`D:\github_prj\Micro-Wheeled_leg-Robot\1.RobotModel\OriginalRobotModel.stp` 是 STEP 几何文件，单位为 mm。STEP 中没有可靠导出整体质量、质心和惯量，因此动力学参数必须从 CAD 软件的质量属性或实测中回填。
