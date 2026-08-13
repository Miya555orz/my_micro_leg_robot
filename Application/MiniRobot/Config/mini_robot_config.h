/**
 ******************************************************************************
 * @file    mini_robot_config.h
 * @brief   Project parameters for the mini wheel-leg controller.
 * @author  Miya Zheng
 * @date    2026-08-02
 ******************************************************************************
 */
#ifndef MINI_ROBOT_CONFIG_H
#define MINI_ROBOT_CONFIG_H

#include <stdint.h>

#define MINI_ROBOT_CONTROL_PERIOD_MS        1U
#define MINI_ROBOT_CAN_PERIOD_MS            2U
#define MINI_ROBOT_VOFA_PERIOD_MS           10U
#define MINI_ROBOT_COMMAND_TIMEOUT_MS       300U

#define MINI_CAN_BOOT_TEST_ENABLE           0U
#define MINI_CAN_BOOT_TEST_PERIOD_MS        100U
#define MINI_CAN1_BOOT_TEST_ID              0x101U
#define MINI_CAN2_BOOT_TEST_ID              0x102U

#define MINI_LED_ACTIVITY_PULSE_MS          25U
#define MINI_LED_ACTIVITY_INTERVAL_MS       80U
#define MINI_LED_HEARTBEAT_HALF_PERIOD_MS   500U

#define MINI_ROBOT_WHEEL_COUNT              2U
#define MINI_ROBOT_WHEEL_TRACK_M            0.160f
#define MINI_ROBOT_WHEEL_RADIUS_M           0.030f
#define MINI_ROBOT_MAX_WHEEL_SPEED_RPS      120.0f
#define MINI_ROBOT_MAX_CURRENT_A            8.0f

#define MINI_PID_SPEED_KP                   0.20f
#define MINI_PID_SPEED_KI                   0.02f
#define MINI_PID_SPEED_KD                   0.0f
#define MINI_PID_SPEED_INTEGRAL_LIMIT       20.0f
#define MINI_PID_POSITION_KP                4.0f
#define MINI_PID_POSITION_KI                0.0f
#define MINI_PID_POSITION_KD                0.0f
#define MINI_PID_POSITION_INTEGRAL_LIMIT    0.0f

#define MINI_BALANCE_STAND_SETTLE_MS        300U
#define MINI_BALANCE_IMU_TIMEOUT_MS         100U
#define MINI_BALANCE_PITCH_LIMIT_RAD        0.70f
#define MINI_BALANCE_TARGET_PITCH_RAD       0.0f
#define MINI_BALANCE_LQR_OUTPUT_LIMIT_A     2.5f
#define MINI_BALANCE_FOC_GRACE_MS           500U
#define MINI_BALANCE_FOC_TIMEOUT_MS         500U
#define MINI_BALANCE_PID_KP                 4.0f
#define MINI_BALANCE_PID_KI                 0.0f
#define MINI_BALANCE_PID_KD                 0.18f
#define MINI_BALANCE_PID_INTEGRAL_LIMIT     0.20f
#define MINI_BALANCE_PID_OUTPUT_LIMIT_A     1.5f
#define MINI_BALANCE_RETURN_ENABLE          1U
#define MINI_BALANCE_RETURN_POSITION_KP     0.012f
#define MINI_BALANCE_RETURN_SPEED_KD        0.020f
#define MINI_BALANCE_RETURN_PITCH_LIMIT_RAD 0.16f
#define MINI_BALANCE_RETURN_SIGN            (-1.0f)
#define MINI_BALANCE_PITCH_OUTPUT_SIGN      1.0f
#define MINI_BALANCE_WHEEL_LEFT_SIGN        1.0f
#define MINI_BALANCE_WHEEL_RIGHT_SIGN       1.0f
#define MINI_BALANCE_ROLL_LIMIT_RAD         0.35f
#define MINI_BALANCE_ROLL_PID_KP            0.40f
#define MINI_BALANCE_ROLL_PID_KI            0.0f
#define MINI_BALANCE_ROLL_PID_KD            0.02f
#define MINI_BALANCE_ROLL_OUTPUT_LIMIT_A    0.35f
#define MINI_BALANCE_FALL_ANGLE_RAD         1.0472f
#define MINI_BALANCE_FALL_CONFIRM_MS        500U
#define MINI_BALANCE_RECOVER_CURRENT_A      1.2f
#define MINI_BALANCE_RECOVER_TIME_MS        450U
#define MINI_BALANCE_RECOVER_MAX_TRY        3U
#define MINI_BALANCE_RECOVER_SUCCESS_RAD    0.35f
#define MINI_BALANCE_FRONT_RECOVER_SIGN     (-1.0f)
#define MINI_BALANCE_BACK_RECOVER_SIGN      1.0f
#define MINI_BALANCE_SERVO_AUX_SIDE         0U
#define MINI_BALANCE_SERVO_FORWARD_POS      1780U
#define MINI_BALANCE_SERVO_BACKWARD_POS     2310U
#define MINI_BALANCE_SERVO_ROLL_RANGE       180U
#define MINI_BALANCE_SERVO_UPDATE_MS        60U
#define MINI_BLOCK_ENABLE                   0U
#define MINI_BLOCK_CURRENT_THRESHOLD_A      2.20f
#define MINI_BLOCK_SPEED_THRESHOLD_RPS      0.52f
#define MINI_BLOCK_CONFIRM_MS               200U

#define MINI_LQR_DEFAULT_A                  { \
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, \
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, \
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, \
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, \
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, \
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}  \
}
#define MINI_LQR_DEFAULT_B                  { \
    {0.0f, 0.0f}, \
    {0.0f, 0.0f}, \
    {0.0f, 0.0f}, \
    {0.0f, 0.0f}, \
    {0.0f, 0.0f}, \
    {0.0f, 0.0f}  \
}
#define MINI_LQR_DEFAULT_K                  { \
    {1.20f, 0.08f, 0.04f, 0.03f, 0.04f, 0.03f}, \
    {1.20f, 0.08f, 0.04f, 0.03f, 0.04f, 0.03f}  \
}
#define MINI_LQR_DEFAULT_X_REF              {MINI_BALANCE_TARGET_PITCH_RAD, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
#define MINI_LQR_DEFAULT_U_FF               {0.0f, 0.0f}
#define MINI_LQR_OUTPUT_LIMIT_A             MINI_BALANCE_LQR_OUTPUT_LIMIT_A
#define MINI_LQR_WHEEL_STATE_USE_METER      0U

#define MINI_FOC_CAN_ID_LEFT                1U
#define MINI_FOC_CAN_ID_RIGHT               1U
#define MINI_FOC_CMD_BASE_ID                0x210U
#define MINI_FOC_FB_BASE_ID                 0x290U

#define MINI_SERVO_UART_BAUD                1000000U
#define MINI_SERVO_LEFT_ID                  1U
#define MINI_SERVO_RIGHT_ID                 1U
#define MINI_SERVO_CENTER_POS               2048U
#define MINI_SERVO_OPEN_POS                 MINI_SERVO_CENTER_POS
#define MINI_SERVO_SHUTDOWN_POS             MINI_SERVO_CENTER_POS
#define MINI_SERVO_STAND_LEFT_POS           MINI_SERVO_CENTER_POS
#define MINI_SERVO_STAND_RIGHT_POS          MINI_SERVO_CENTER_POS
#define MINI_SERVO_STAND_TIME_MS            800U
#define MINI_SERVO_STAND_SPEED              300U
#define MINI_SERVO_POSITION_MIN             0U
#define MINI_SERVO_POSITION_MAX             4095U
#define MINI_SERVO_TEST_POSITION_MIN        MINI_SERVO_POSITION_MIN
#define MINI_SERVO_TEST_POSITION_MAX        MINI_SERVO_POSITION_MAX
#define MINI_SERVO_TEST_ANGLE_LIMIT_DEG     25.0f
#define MINI_SERVO_MOVE_TIME_MS             300U
#define MINI_SERVO_MOVE_SPEED               300U
#define MINI_SERVO_MOVE_ACC                 30U
#define MINI_SERVO_PING_PERIOD_MS           200U
#define MINI_SERVO_ONLINE_TIMEOUT_MS        600U
#define MINI_SERVO_RX_BUF_LEN               32U
#define MINI_SERVO_ONLINE_USE_PING          0U

#define MINI_VOFA_RX_BUF_LEN                400U
#define MINI_VOFA_CHANNEL_COUNT             8U
#define MINI_VOFA_TX_TIMEOUT_MS             20U

#define MINI_MPU6050_I2C_ADDRESS            (0x68U << 1U)
#define MINI_MPU6050_UPDATE_PERIOD_MS       10U
#define MINI_MPU6050_COMPLEMENTARY_ALPHA    0.98f
#define MINI_MPU6050_ROLL_SIGN              1.0f
#define MINI_MPU6050_PITCH_SIGN             1.0f
#define MINI_MPU6050_YAW_SIGN               1.0f
#define MINI_MPU6050_ROLL_OFFSET_RAD        0.0f
#define MINI_MPU6050_PITCH_OFFSET_RAD       0.0f

#define MINI_NRF24_CHANNEL                  76U
#define MINI_NRF24_PAYLOAD_SIZE             32U
#define MINI_NRF24_ADDRESS                  {'T', 'A', 'N', 'K', '1'}
#define MINI_NRF24_LINK_TIMEOUT_MS          300U
#define MINI_REMOTE_AXIS_DEADZONE           180
#define MINI_REMOTE_MAX_VX_MPS              1.0f
#define MINI_REMOTE_MAX_WZ_RPS              3.0f
#define MINI_REMOTE_SLOW_SCALE              0.40f
#define MINI_REMOTE_FAST_SCALE              1.00f
#define MINI_REMOTE_SERVO_RANGE             1000U
#define MINI_REMOTE_SERVO_MIN_PERIOD_MS     40U
#define MINI_REMOTE_SERVO_MIN_DELTA         8U
#define MINI_REMOTE_SERVO_LEFT_SIGN         1.0f
#define MINI_REMOTE_SERVO_RIGHT_SIGN        (-1.0f)
#define MINI_REMOTE_KEY_JUMP_MASK           0x01U
#define MINI_REMOTE_KEY_FAST_MASK           0x02U

#endif
