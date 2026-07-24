cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME DM-MC02)

#####################################
# ASCII Art
#####################################
message(STATUS [=[
|
|                   RobotPilots 2023 
|                       _oo0oo_
|                      o8888888o
|                      88" . "88
|                      (| -_- |)
|                      0\  =  /0
|                    ___/`---'\___
|                  .' \\|     |// '.
|                 / \\|||  :  |||// \
|                / _||||| -:- |||||- \
|               |   | \\\  -  /// |   |
|               | \_|  ''\---/''  |_/ |
|               \  .-\__  '-'  ___/-. /
|             ___'. .'  /--.--\  `. .'___
|          ."" '<  `.___\_<|>_/___.' >' "".
|         | | :  `- \`.;`\ _ /`;.`/ - ` : | |
|         \  \ `_.   \_ __\ /__ _/   .-` /  /
|     =====`-.____`.___ \_____/___.-`___.-'=====
|                       `=---='
|
|
|     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
|
|               BEST BLESS FOR NO BUG
|
|
|]=])

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    # AlgorithmLayer
    Application/AlgorithmLayer/algo.c
    Application/AlgorithmLayer/ave_filter.c
    Application/AlgorithmLayer/crc.c
    Application/AlgorithmLayer/kalman_filter.c
    Application/AlgorithmLayer/PID.c
    Application/AlgorithmLayer/rp_math.c

    # ConfigLayer
    Application/ConfigLayer/rp_config.c

    # ControlLayer
    Application/ControlLayer/Balance.c

    # DeviceLayer
    Application/DeviceLayer/buzzer.c
    Application/DeviceLayer/cap.c
    Application/DeviceLayer/chassis_motor.c
    Application/DeviceLayer/device.c
    Application/DeviceLayer/gimbal_motor.c
    Application/DeviceLayer/judge.c
    Application/DeviceLayer/shoot_motor.c

    # DeviceLayer/Imu
    Application/DeviceLayer/Imu/bmi.c
    Application/DeviceLayer/Imu/bmi_EKF.c
    Application/DeviceLayer/Imu/BMI088driver.c
    Application/DeviceLayer/Imu/BMI088Middleware.c

    # DeviceLayer/Sensor
    Application/DeviceLayer/Sensor/imu_sensor.c
    Application/DeviceLayer/Sensor/rc_sensor.c

    # DriverLayer
    Application/DriverLayer/driver.c
    Application/DriverLayer/drv_can.c
    Application/DriverLayer/drv_flash.c
    Application/DriverLayer/drv_tick.c
    Application/DriverLayer/drv_tim.c
    Application/DriverLayer/drv_uart.c

    # HardwareLayer
    Application/HardwareLayer/BRT_code.c
    Application/HardwareLayer/DM_Motor.c
    Application/HardwareLayer/HT_Motor.c
    Application/HardwareLayer/KT_motor.c
    Application/HardwareLayer/motor_def.c
    Application/HardwareLayer/RM_motor.c

    # ModuleLayer
    Application/ModuleLayer/Chassis.c
    Application/ModuleLayer/gimbal.c
    Application/ModuleLayer/shoot.c
    Application/ModuleLayer/ui.c

    # ParameterLayer
    Application/ParameterLayer/car_info.c
    Application/ParameterLayer/Filter.c
    Application/ParameterLayer/Link_instance.c
    Application/ParameterLayer/PID_Instance.c
    Application/ParameterLayer/Straight_Instance.c

    # ProtocolLayer
    Application/ProtocolLayer/can_protocol.c
    Application/ProtocolLayer/cap_protocol.c
    Application/ProtocolLayer/communicate.c
    Application/ProtocolLayer/judge_protocol.c
    Application/ProtocolLayer/rc_protocol.c
    Application/ProtocolLayer/ui_protocol.c

    # ServiceLayer
    Application/ServiceLayer/Command.c
    Application/ServiceLayer/Command_Instance.c
    Application/ServiceLayer/posture.c
    Application/ServiceLayer/RP_Log.c

    # SolvingLayer
    Application/SolvingLayer/Chassis_Posture.c
    Application/SolvingLayer/Link_Calc.c
    Application/SolvingLayer/Straight_Leg_Calc.c
    Application/SolvingLayer/ui_priority.c

    # TaskLayer
    Application/TaskLayer/Command_Task.c
    Application/TaskLayer/control_task.c
    Application/TaskLayer/monitor_task.c
    Application/TaskLayer/observe_task.c
    Application/TaskLayer/RP_Log_task.c
    Application/TaskLayer/UI_Task.c

    # Segger RTT
    Middlewares/Segger_RTT/SEGGER_RTT.c
    Middlewares/Segger_RTT/SEGGER_RTT_printf.c
)

# ----- CMSIS-DSP sources -----
file(GLOB_RECURSE CMSIS_DSP_SOURCES
    Drivers/CMSIS/DSP/Source/*.c
)
# Remove aggregate source files (avoid duplicate definitions)
list(FILTER CMSIS_DSP_SOURCES EXCLUDE REGEX
    "/(TransformFunctions|FilteringFunctions|MatrixFunctions|StatisticsFunctions|SupportFunctions|FastMathFunctions|ControllerFunctions|ComplexMathFunctions|BasicMathFunctions|BayesFunctions|CommonTables|DistanceFunctions|InterpolationFunctions|QuaternionMathFunctions|SVMFunctions|TransformFunctionsF16)\\.c$"
)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${CMSIS_DSP_SOURCES})
# -----------------------------------------------------------

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Application
    Application/AlgorithmLayer
    Application/ConfigLayer
    Application/ControlLayer
    Application/DeviceLayer
    Application/DeviceLayer/Imu
    Application/DeviceLayer/Sensor
    Application/DriverLayer
    Application/HardwareLayer
    Application/ModuleLayer
    Application/ParameterLayer
    Application/ProtocolLayer
    Application/ServiceLayer
    Application/SolvingLayer
    Application/TaskLayer
    Middlewares/Segger_RTT
    Drivers/CMSIS/DSP/Include
    Drivers/CMSIS/DSP/PrivateInclude
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ARM_MATH_CM7
    ARM_MATH_DSP
    ARM_MATH_LOOPUNROLL
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
)
