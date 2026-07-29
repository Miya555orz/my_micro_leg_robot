#include "mini_robot_app.h"

#include "cmsis_os.h"
#include "drv_can.h"
#include "drv_uart.h"
#include "fdcan.h"
#include "i2c.h"
#include "iwdg.h"
#include "mini_chassis.h"
#include "mini_foc_can.h"
#include "mini_mpu6050.h"
#include "mini_nrf24.h"
#include "mini_pid.h"
#include "mini_robot_config.h"
#include "mini_status_led.h"
#include "mini_ttl_servo.h"
#include "mini_vofa.h"
#include "spi.h"
#include "usart.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define REMOTE_PACKET_MAGIC            0xA5U
#define REMOTE_PACKET_VERSION          0x01U
#define REMOTE_PACKET_CHECKSUM_LENGTH  30U
#define REMOTE_AXIS_LIMIT              1000
#define MINI_SERVO_SIDE_LEFT           0U
#define MINI_SERVO_SIDE_RIGHT          1U
#define MINI_SERVO_SIDE_ALL            0xFEU
#define MINI_RAD_TO_DEG                57.2957795f

typedef enum {
    MINI_BALANCE_STATE_SLEEP = 0,
    MINI_BALANCE_STATE_SERVO_SETTLE,
    MINI_BALANCE_STATE_STAND,
    MINI_BALANCE_STATE_RECOVER_FRONT,
    MINI_BALANCE_STATE_RECOVER_BACK,
    MINI_BALANCE_STATE_FAULT,
} MiniBalanceState_t;

#pragma pack(push, 1)
typedef struct {
    uint8_t magic;
    uint8_t version;
    uint8_t length;
    uint8_t sequence;
    int16_t left_x;
    int16_t left_y;
    int16_t right_x;
    int16_t right_y;
    uint8_t key_bits;
    uint8_t spare1_state;
    uint8_t spare2_state;
    uint8_t flags;
    uint8_t reserved[14];
    uint16_t checksum;
} MiniRemotePacket_t;
#pragma pack(pop)

typedef char MiniRemotePacketSizeCheck[
    (sizeof(MiniRemotePacket_t) == MINI_NRF24_PAYLOAD_SIZE) ? 1 : -1];

static uint16_t servo_pos[2] = {MINI_SERVO_OPEN_POS, MINI_SERVO_OPEN_POS};
static uint8_t pending_uart_data[MINI_VOFA_RX_BUF_LEN];
static volatile uint16_t pending_uart_length;
static volatile uint8_t pending_uart_ready;
static uint32_t remote_last_servo_ms;
static uint8_t remote_jump_request;
static uint8_t telemetry_enabled = 0U;
static uint8_t can_auto_tx_enabled = 0U;
static uint8_t foc_direct_mode_enabled = 0U;
static MiniBalanceState_t balance_state = MINI_BALANCE_STATE_SLEEP;
static uint32_t balance_state_start_ms;
static MiniPid_t balance_pitch_pid;
static MiniPid_t balance_roll_pid;
static uint32_t balance_fall_start_ms;
static uint8_t balance_recover_count;
static uint32_t balance_servo_last_ms;
static uint32_t block_start_ms[MINI_ROBOT_WHEEL_COUNT];
static uint8_t block_latched[MINI_ROBOT_WHEEL_COUNT];
static uint32_t can1_boot_test_count;
static uint32_t can2_boot_test_count;
static volatile uint32_t ctrl_loop_count;

static uint16_t checksum16(const uint8_t *data, uint16_t length)
{
    uint32_t sum = 0U;

    for (uint16_t i = 0U; i < length; ++i) {
        sum += data[i];
    }
    return (uint16_t)(sum & 0xFFFFU);
}

static float normalize_axis(int16_t value)
{
    int32_t magnitude = (value < 0) ? -(int32_t)value : (int32_t)value;
    float normalized;

    if (magnitude <= MINI_REMOTE_AXIS_DEADZONE) {
        return 0.0f;
    }
    if (magnitude > REMOTE_AXIS_LIMIT) {
        magnitude = REMOTE_AXIS_LIMIT;
    }
    normalized = (float)(magnitude - MINI_REMOTE_AXIS_DEADZONE) /
                 (float)(REMOTE_AXIS_LIMIT - MINI_REMOTE_AXIS_DEADZONE);
    return (value < 0) ? -normalized : normalized;
}

static uint16_t clamp_servo_position(float value)
{
    if (value < (float)MINI_SERVO_POSITION_MIN) {
        return MINI_SERVO_POSITION_MIN;
    }
    if (value > (float)MINI_SERVO_POSITION_MAX) {
        return MINI_SERVO_POSITION_MAX;
    }
    return (uint16_t)value;
}

static uint16_t servo_delta(uint16_t a, uint16_t b)
{
    return (a > b) ? (a - b) : (b - a);
}

static uint8_t servo_side_from_command(uint8_t value)
{
    if (value == MINI_SERVO_SIDE_RIGHT || value == 2U) {
        return MINI_SERVO_SIDE_RIGHT;
    }
    if (value == MINI_SERVO_SIDE_ALL) {
        return MINI_SERVO_SIDE_ALL;
    }
    return MINI_SERVO_SIDE_LEFT;
}

static uint8_t servo_id_from_side(uint8_t side)
{
    if (side == MINI_SERVO_SIDE_ALL) {
        return 0xFEU;
    }
    return (side == MINI_SERVO_SIDE_RIGHT) ? MINI_SERVO_RIGHT_ID : MINI_SERVO_LEFT_ID;
}

static const char *servo_side_name(uint8_t side)
{
    if (side == MINI_SERVO_SIDE_RIGHT) {
        return "right";
    }
    if (side == MINI_SERVO_SIDE_ALL) {
        return "all";
    }
    return "left";
}

static void update_servo_shadow(uint8_t side, uint16_t position)
{
    if (side == MINI_SERVO_SIDE_LEFT) {
        servo_pos[0] = position;
    } else if (side == MINI_SERVO_SIDE_RIGHT) {
        servo_pos[1] = position;
    }
}

static void send_servo_status(const char *operation, uint8_t side, HAL_StatusTypeDef status)
{
    char message[80];

    snprintf(message,
             sizeof(message),
             "servo %s side=%s id=%u st=%u\r\n",
             operation,
             servo_side_name(side),
             (unsigned)servo_id_from_side(side),
             (unsigned)status);
    MiniVofa_SendText(message);
}

static void send_servo_packet(const char *label,
                              uint8_t (*getter)(uint8_t *, uint8_t))
{
    uint8_t packet[32];
    uint8_t length = getter(packet, sizeof(packet));
    char message[160];
    uint16_t used;

    if (length == 0U) {
        return;
    }
    used = (uint16_t)snprintf(message, sizeof(message), "servo %s", label);
    for (uint8_t i = 0U; i < length && used < (sizeof(message) - 5U); ++i) {
        used += (uint16_t)snprintf(&message[used],
                                   sizeof(message) - used,
                                   " %02X",
                                   packet[i]);
    }
    snprintf(&message[used], sizeof(message) - used, "\r\n");
    MiniVofa_SendText(message);
}

static void send_servo_readout(const MiniServoStatus_t *status, HAL_StatusTypeDef result)
{
    char message[160];

    if (status == 0 || result != HAL_OK) {
        snprintf(message, sizeof(message), "servo read st=%u\r\n", (unsigned)result);
    } else {
        snprintf(message,
                 sizeof(message),
                 "servo read id=%u err=%u pos=%u spd=%u load=%u volt=%.1fV temp=%u\r\n",
                 (unsigned)status->id,
                 (unsigned)status->error,
                 (unsigned)status->position,
                 (unsigned)status->speed,
                 (unsigned)status->load,
                 (double)((float)status->voltage_0v1 * 0.1f),
                 (unsigned)status->temperature_c);
    }
    MiniVofa_SendText(message);
}

static void send_servo_scan(void)
{
    HAL_StatusTypeDef left;
    HAL_StatusTypeDef right;
    char message[96];

    left = MiniServo_PingSide(MINI_SERVO_SIDE_LEFT);
    snprintf(message,
             sizeof(message),
             "servo scan left usart10 id=%u st=%u\r\n",
             (unsigned)MINI_SERVO_LEFT_ID,
             (unsigned)left);
    MiniVofa_SendText(message);
    send_servo_packet("left_tx", MiniServo_GetLastTx);
    send_servo_packet("left_rx", MiniServo_GetLastRx);

    right = MiniServo_PingSide(MINI_SERVO_SIDE_RIGHT);
    snprintf(message,
             sizeof(message),
             "servo scan right usart1 id=%u st=%u\r\n",
             (unsigned)MINI_SERVO_RIGHT_ID,
             (unsigned)right);
    MiniVofa_SendText(message);
    send_servo_packet("right_tx", MiniServo_GetLastTx);
    send_servo_packet("right_rx", MiniServo_GetLastRx);
}

static const char *balance_state_name(MiniBalanceState_t state)
{
    switch (state) {
    case MINI_BALANCE_STATE_SERVO_SETTLE:
        return "servo_settle";
    case MINI_BALANCE_STATE_STAND:
        return "stand";
    case MINI_BALANCE_STATE_RECOVER_FRONT:
        return "recover_front";
    case MINI_BALANCE_STATE_RECOVER_BACK:
        return "recover_back";
    case MINI_BALANCE_STATE_FAULT:
        return "fault";
    default:
        return "sleep";
    }
}

static float app_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float app_clampf(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static uint16_t balance_aux_servo_position(float roll_rad)
{
    float normalized = app_clampf(roll_rad / MINI_BALANCE_ROLL_LIMIT_RAD, 1.0f);
    float position = (float)MINI_SERVO_CENTER_POS +
                     normalized * (float)MINI_BALANCE_SERVO_ROLL_RANGE;

    return clamp_servo_position(position);
}

static void balance_reset_controllers(void)
{
    MiniPid_Reset(&balance_pitch_pid);
    MiniPid_Reset(&balance_roll_pid);
    balance_fall_start_ms = 0U;
    balance_recover_count = 0U;
    for (uint8_t i = 0U; i < MINI_ROBOT_WHEEL_COUNT; ++i) {
        block_start_ms[i] = 0U;
        block_latched[i] = 0U;
    }
}

static uint8_t imu_ready_for_balance(uint32_t now_ms)
{
    const MiniMpu6050Data_t *imu = MiniMpu6050_GetData();

    if (imu == 0 || imu->online == 0U) {
        return 0U;
    }
    if ((now_ms - imu->last_update_ms) > MINI_BALANCE_IMU_TIMEOUT_MS) {
        return 0U;
    }
    if (fabsf(imu->pitch_rad - MINI_BALANCE_TARGET_PITCH_RAD) > MINI_BALANCE_PITCH_LIMIT_RAD) {
        return 0U;
    }
    return 1U;
}

static uint8_t foc_feedback_ready(uint8_t index, uint32_t now_ms)
{
    const MiniFocMotor_t *motor = MiniFoc_GetMotor(index);

    if (motor == 0 || motor->online == 0U) {
        return 0U;
    }
    if ((now_ms - motor->last_rx_ms) > MINI_BALANCE_FOC_TIMEOUT_MS) {
        return 0U;
    }
    return 1U;
}

static uint8_t foc_pair_feedback_ready(uint32_t now_ms)
{
    return (foc_feedback_ready(0U, now_ms) &&
            foc_feedback_ready(1U, now_ms))
               ? 1U
               : 0U;
}

static const char *foc_feedback_fault_reason(uint32_t now_ms)
{
    uint8_t left_ok = foc_feedback_ready(0U, now_ms);
    uint8_t right_ok = foc_feedback_ready(1U, now_ms);

    if (left_ok && right_ok) {
        return "foc_feedback_lost_transient";
    }
    if (!left_ok && !right_ok) {
        return "foc_both_lost";
    }
    return left_ok ? "foc_right_lost" : "foc_left_lost";
}

static void balance_send_status(const char *reason)
{
    const MiniMpu6050Data_t *imu = MiniMpu6050_GetData();
    const MiniFocMotor_t *left = MiniFoc_GetMotor(0U);
    const MiniFocMotor_t *right = MiniFoc_GetMotor(1U);
    uint32_t now_ms = HAL_GetTick();
    char message[220];

    snprintf(message,
             sizeof(message),
             "balance %s reason=%s pitch=%.4f rate=%.4f foc_l=%u/%lums foc_r=%u/%lums\r\n",
             balance_state_name(balance_state),
             (reason != 0) ? reason : "none",
             (double)((imu != 0) ? imu->pitch_rad : 0.0f),
             (double)((imu != 0) ? imu->pitch_rate_rps : 0.0f),
             (unsigned)((left != 0) ? left->online : 0U),
             (unsigned long)((left != 0) ? (now_ms - left->last_rx_ms) : 0U),
             (unsigned)((right != 0) ? right->online : 0U),
             (unsigned long)((right != 0) ? (now_ms - right->last_rx_ms) : 0U));
    MiniVofa_SendText(message);
}

static void balance_enter_sleep(const char *reason)
{
    balance_state = MINI_BALANCE_STATE_SLEEP;
    balance_state_start_ms = HAL_GetTick();
    foc_direct_mode_enabled = 0U;
    can_auto_tx_enabled = 0U;
    balance_reset_controllers();
    MiniChassis_Sleep();
    MiniFoc_SetCommand(0U, MINI_FOC_MODE_STOP, 0.0f);
    MiniFoc_SetCommand(1U, MINI_FOC_MODE_STOP, 0.0f);
    MiniFoc_SendAll();
    (void)MiniServo_TorqueEnableSide(MINI_SERVO_SIDE_ALL, 0U);
    balance_send_status(reason);
}

static void balance_enter_fault(const char *reason)
{
    balance_state = MINI_BALANCE_STATE_FAULT;
    balance_state_start_ms = HAL_GetTick();
    foc_direct_mode_enabled = 0U;
    can_auto_tx_enabled = 0U;
    balance_reset_controllers();
    MiniChassis_Sleep();
    MiniFoc_SetCommand(0U, MINI_FOC_MODE_STOP, 0.0f);
    MiniFoc_SetCommand(1U, MINI_FOC_MODE_STOP, 0.0f);
    MiniFoc_SendAll();
    (void)MiniServo_TorqueEnableSide(MINI_SERVO_SIDE_ALL, 0U);
    balance_send_status(reason);
}

static void balance_request_stand(const char *reason)
{
    HAL_StatusTypeDef torque_status;
    HAL_StatusTypeDef position_status;

    foc_direct_mode_enabled = 0U;
    can_auto_tx_enabled = 0U;
    MiniChassis_Sleep();
    MiniFoc_SetCommand(0U, MINI_FOC_MODE_STOP, 0.0f);
    MiniFoc_SetCommand(1U, MINI_FOC_MODE_STOP, 0.0f);
    MiniFoc_SendAll();
    balance_reset_controllers();

    torque_status = MiniServo_TorqueEnableSide(MINI_SERVO_SIDE_ALL, 1U);
    position_status = MiniServo_WritePair(MINI_SERVO_STAND_LEFT_POS,
                                          MINI_SERVO_STAND_RIGHT_POS,
                                          MINI_SERVO_STAND_SPEED);
    servo_pos[0] = MINI_SERVO_STAND_LEFT_POS;
    servo_pos[1] = MINI_SERVO_STAND_RIGHT_POS;

    balance_state = MINI_BALANCE_STATE_SERVO_SETTLE;
    balance_state_start_ms = HAL_GetTick();
    balance_send_status(reason);

    {
        char message[96];
        snprintf(message,
                 sizeof(message),
                 "stand arm torque_st=%u pos_st=%u left=%u right=%u\r\n",
                 (unsigned)torque_status,
                 (unsigned)position_status,
                 (unsigned)MINI_SERVO_STAND_LEFT_POS,
                 (unsigned)MINI_SERVO_STAND_RIGHT_POS);
        MiniVofa_SendText(message);
    }
}

static uint8_t balance_is_active(void)
{
    return (balance_state == MINI_BALANCE_STATE_STAND ||
            balance_state == MINI_BALANCE_STATE_RECOVER_FRONT ||
            balance_state == MINI_BALANCE_STATE_RECOVER_BACK)
               ? 1U
               : 0U;
}

static void balance_set_wheel_current(float left_current, float right_current)
{
    MiniFoc_SetCommand(0U,
                       MINI_FOC_MODE_CURRENT,
                       app_clampf(left_current, MINI_BALANCE_PID_OUTPUT_LIMIT_A));
    MiniFoc_SetCommand(1U,
                       MINI_FOC_MODE_CURRENT,
                       app_clampf(right_current, MINI_BALANCE_PID_OUTPUT_LIMIT_A));
}

static void balance_print_blocked(uint8_t wheel)
{
    char message[40];

    snprintf(message, sizeof(message), "BLOCKED wheel=%u\r\n", (unsigned)(wheel + 1U));
    MiniVofa_SendText(message);
}

static void balance_check_blocked(uint8_t wheel, float current_cmd, float speed_rps, uint32_t now_ms)
{
    if (wheel >= MINI_ROBOT_WHEEL_COUNT || block_latched[wheel]) {
        return;
    }

    if (app_absf(current_cmd) > MINI_BLOCK_CURRENT_THRESHOLD_A &&
        app_absf(speed_rps) < MINI_BLOCK_SPEED_THRESHOLD_RPS) {
        if (block_start_ms[wheel] == 0U) {
            block_start_ms[wheel] = now_ms;
        } else if ((now_ms - block_start_ms[wheel]) >= MINI_BLOCK_CONFIRM_MS) {
            block_latched[wheel] = 1U;
            MiniFoc_SetCommand(wheel, MINI_FOC_MODE_STOP, 0.0f);
            balance_print_blocked(wheel);
            balance_enter_fault("wheel_blocked");
        }
    } else {
        block_start_ms[wheel] = 0U;
    }
}

static void balance_control_update(uint32_t now_ms, float dt_s)
{
    const MiniMpu6050Data_t *imu = MiniMpu6050_GetData();
    const MiniFocMotor_t *left_motor = MiniFoc_GetMotor(0U);
    const MiniFocMotor_t *right_motor = MiniFoc_GetMotor(1U);
    float pitch_current;
    float roll_current = 0.0f;
    float left_current;
    float right_current;

    if (imu == 0 || balance_state == MINI_BALANCE_STATE_SLEEP ||
        balance_state == MINI_BALANCE_STATE_FAULT) {
        return;
    }

    if (balance_state == MINI_BALANCE_STATE_RECOVER_FRONT ||
        balance_state == MINI_BALANCE_STATE_RECOVER_BACK) {
        float sign = (balance_state == MINI_BALANCE_STATE_RECOVER_FRONT)
                         ? MINI_BALANCE_FRONT_RECOVER_SIGN
                         : MINI_BALANCE_BACK_RECOVER_SIGN;
        float recover_current = sign * MINI_BALANCE_RECOVER_CURRENT_A;

        balance_set_wheel_current(MINI_BALANCE_WHEEL_LEFT_SIGN * recover_current,
                                  MINI_BALANCE_WHEEL_RIGHT_SIGN * recover_current);
        return;
    }

    if (balance_state != MINI_BALANCE_STATE_STAND) {
        return;
    }

    pitch_current =
        MINI_BALANCE_PITCH_OUTPUT_SIGN *
        MiniPid_Calc(&balance_pitch_pid,
                     MINI_BALANCE_TARGET_PITCH_RAD,
                     imu->pitch_rad,
                     dt_s);

    if (app_absf(imu->roll_rad) > MINI_BALANCE_ROLL_LIMIT_RAD) {
        roll_current = MiniPid_Calc(&balance_roll_pid, 0.0f, imu->roll_rad, dt_s);
    } else {
        MiniPid_Reset(&balance_roll_pid);
    }

    left_current = MINI_BALANCE_WHEEL_LEFT_SIGN * (pitch_current + roll_current);
    right_current = MINI_BALANCE_WHEEL_RIGHT_SIGN * (pitch_current - roll_current);
    balance_set_wheel_current(left_current, right_current);

    balance_check_blocked(0U,
                          left_current,
                          (left_motor != 0) ? left_motor->speed_rps : 0.0f,
                          now_ms);
    balance_check_blocked(1U,
                          right_current,
                          (right_motor != 0) ? right_motor->speed_rps : 0.0f,
                          now_ms);
}

static void balance_start_recovery(uint32_t now_ms, uint8_t front_fall)
{
    HAL_StatusTypeDef servo_status;

    if (balance_recover_count >= MINI_BALANCE_RECOVER_MAX_TRY) {
        balance_enter_fault("recover_failed");
        return;
    }

    balance_recover_count++;
    balance_state = front_fall ? MINI_BALANCE_STATE_RECOVER_FRONT : MINI_BALANCE_STATE_RECOVER_BACK;
    balance_state_start_ms = now_ms;
    can_auto_tx_enabled = 1U;
    MiniPid_Reset(&balance_pitch_pid);
    MiniPid_Reset(&balance_roll_pid);

    servo_status = MiniServo_WritePositionSide(MINI_BALANCE_SERVO_AUX_SIDE,
                                               front_fall ? MINI_BALANCE_SERVO_BACKWARD_POS
                                                          : MINI_BALANCE_SERVO_FORWARD_POS,
                                               MINI_SERVO_MOVE_TIME_MS,
                                               MINI_SERVO_MOVE_SPEED);
    {
        char message[80];
        snprintf(message,
                 sizeof(message),
                 "recover start dir=%s try=%u servo_st=%u\r\n",
                 front_fall ? "front" : "back",
                 (unsigned)balance_recover_count,
                 (unsigned)servo_status);
        MiniVofa_SendText(message);
    }
    balance_send_status("fall_detected");
}

static void balance_step(uint32_t now_ms)
{
    const MiniMpu6050Data_t *imu = MiniMpu6050_GetData();

    if (balance_state == MINI_BALANCE_STATE_SERVO_SETTLE) {
        if ((now_ms - balance_state_start_ms) < MINI_BALANCE_STAND_SETTLE_MS) {
            return;
        }
        if (!imu_ready_for_balance(now_ms)) {
            balance_enter_fault("imu_not_ready_or_tilt");
            return;
        }
        balance_reset_controllers();
        can_auto_tx_enabled = 1U;
        balance_state = MINI_BALANCE_STATE_STAND;
        balance_state_start_ms = now_ms;
        balance_send_status("pid_on");
        return;
    }

    if (balance_state == MINI_BALANCE_STATE_STAND) {
        if ((now_ms - balance_state_start_ms) > MINI_BALANCE_FOC_GRACE_MS &&
            !foc_pair_feedback_ready(now_ms)) {
            balance_enter_fault(foc_feedback_fault_reason(now_ms));
            return;
        }
        if (imu == 0 ||
            imu->online == 0U ||
            (now_ms - imu->last_update_ms) > MINI_BALANCE_IMU_TIMEOUT_MS) {
            balance_enter_fault("imu_timeout");
            return;
        }
        if (fabsf(imu->pitch_rad - MINI_BALANCE_TARGET_PITCH_RAD) > MINI_BALANCE_PITCH_LIMIT_RAD) {
            if (app_absf(imu->pitch_rad - MINI_BALANCE_TARGET_PITCH_RAD) >
                MINI_BALANCE_FALL_ANGLE_RAD) {
                if (balance_fall_start_ms == 0U) {
                    balance_fall_start_ms = now_ms;
                } else if ((now_ms - balance_fall_start_ms) >= MINI_BALANCE_FALL_CONFIRM_MS) {
                    balance_start_recovery(now_ms,
                                           (imu->pitch_rad > MINI_BALANCE_TARGET_PITCH_RAD) ? 1U : 0U);
                }
            } else {
                balance_fall_start_ms = 0U;
                balance_enter_fault("pitch_limit");
            }
            return;
        }
        balance_fall_start_ms = 0U;
        if (app_absf(imu->roll_rad) > MINI_BALANCE_ROLL_LIMIT_RAD &&
            (now_ms - balance_servo_last_ms) >= MINI_BALANCE_SERVO_UPDATE_MS) {
            balance_servo_last_ms = now_ms;
            (void)MiniServo_WritePositionSide(MINI_BALANCE_SERVO_AUX_SIDE,
                                              balance_aux_servo_position(imu->roll_rad),
                                              MINI_SERVO_MOVE_TIME_MS,
                                              MINI_SERVO_MOVE_SPEED);
        }
        return;
    }

    if (balance_state == MINI_BALANCE_STATE_RECOVER_FRONT ||
        balance_state == MINI_BALANCE_STATE_RECOVER_BACK) {
        if (imu == 0 ||
            imu->online == 0U ||
            (now_ms - imu->last_update_ms) > MINI_BALANCE_IMU_TIMEOUT_MS) {
            balance_enter_fault("imu_timeout");
            return;
        }
        if (app_absf(imu->pitch_rad - MINI_BALANCE_TARGET_PITCH_RAD) <
            MINI_BALANCE_RECOVER_SUCCESS_RAD) {
            balance_state = MINI_BALANCE_STATE_SERVO_SETTLE;
            balance_state_start_ms = now_ms;
            MiniFoc_SetCommand(0U, MINI_FOC_MODE_STOP, 0.0f);
            MiniFoc_SetCommand(1U, MINI_FOC_MODE_STOP, 0.0f);
            (void)MiniServo_WritePositionSide(MINI_BALANCE_SERVO_AUX_SIDE,
                                              MINI_SERVO_CENTER_POS,
                                              MINI_SERVO_MOVE_TIME_MS,
                                              MINI_SERVO_MOVE_SPEED);
            balance_send_status("recover_ok");
            return;
        }
        if ((now_ms - balance_state_start_ms) >= MINI_BALANCE_RECOVER_TIME_MS) {
            balance_start_recovery(now_ms,
                                   (imu->pitch_rad > MINI_BALANCE_TARGET_PITCH_RAD) ? 1U : 0U);
        }
    }
}

static void send_can_status_line(const char *name, FDCAN_HandleTypeDef *hfdcan)
{
    const CAN_DiagTypeDef *diag = CAN_GetDiag(hfdcan);
    FDCAN_ProtocolStatusTypeDef protocol = {0};
    FDCAN_ErrorCountersTypeDef errors = {0};
    char message[192];
    uint32_t free_level = 0U;

    if (diag == 0) {
        return;
    }
    free_level = HAL_FDCAN_GetTxFifoFreeLevel(hfdcan);
    (void)HAL_FDCAN_GetProtocolStatus(hfdcan, &protocol);
    (void)HAL_FDCAN_GetErrorCounters(hfdcan, &errors);
    snprintf(message,
             sizeof(message),
             "%s free=%lu txq=%lu ok=%lu fail=%lu full=%lu abort=%lu rx=%lu rxfail=%lu st=%lu err=0x%08lX lec=%lu dlec=%lu tec=%lu rec=%lu ep=%lu warn=%lu bo=%lu\r\n",
             name,
             (unsigned long)free_level,
             (unsigned long)diag->tx_attempt_count,
             (unsigned long)diag->tx_ok_count,
             (unsigned long)diag->tx_fail_count,
             (unsigned long)diag->tx_fifo_full_count,
             (unsigned long)diag->tx_abort_count,
             (unsigned long)diag->rx_count,
             (unsigned long)diag->rx_fail_count,
             (unsigned long)diag->last_tx_status,
             (unsigned long)diag->last_error_code,
             (unsigned long)protocol.LastErrorCode,
             (unsigned long)protocol.DataLastErrorCode,
             (unsigned long)errors.TxErrorCnt,
             (unsigned long)errors.RxErrorCnt,
             (unsigned long)protocol.ErrorPassive,
             (unsigned long)protocol.Warning,
             (unsigned long)protocol.BusOff);
    MiniVofa_SendText(message);

    snprintf(message,
             sizeof(message),
             "%s last_tx id=0x%03lX t=%lums data=%02X %02X %02X %02X %02X %02X %02X %02X | last_rx id=0x%03lX dlc=%lu t=%lums data=%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
             name,
             (unsigned long)diag->last_tx_id,
             (unsigned long)diag->last_tx_tick_ms,
             (unsigned)diag->last_tx_data[0],
             (unsigned)diag->last_tx_data[1],
             (unsigned)diag->last_tx_data[2],
             (unsigned)diag->last_tx_data[3],
             (unsigned)diag->last_tx_data[4],
             (unsigned)diag->last_tx_data[5],
             (unsigned)diag->last_tx_data[6],
             (unsigned)diag->last_tx_data[7],
             (unsigned long)diag->last_rx_id,
             (unsigned long)diag->last_rx_dlc,
             (unsigned long)diag->last_rx_tick_ms,
             (unsigned)diag->last_rx_data[0],
             (unsigned)diag->last_rx_data[1],
             (unsigned)diag->last_rx_data[2],
             (unsigned)diag->last_rx_data[3],
             (unsigned)diag->last_rx_data[4],
             (unsigned)diag->last_rx_data[5],
             (unsigned)diag->last_rx_data[6],
             (unsigned)diag->last_rx_data[7]);
    MiniVofa_SendText(message);
}

static void send_foc_status(void)
{
    const MiniFocMotor_t *left = MiniFoc_GetMotor(0U);
    const MiniFocMotor_t *right = MiniFoc_GetMotor(1U);
    uint32_t now_ms = HAL_GetTick();
    char message[220];

    snprintf(message,
             sizeof(message),
             "foc l:on=%u age=%lums mode=%u cmd=%.4f spd=%.4f pos=%.4f | r:on=%u age=%lums mode=%u cmd=%.4f spd=%.4f pos=%.4f\r\n",
             (unsigned)((left != 0) ? left->online : 0U),
             (unsigned long)((left != 0) ? (now_ms - left->last_rx_ms) : 0U),
             (unsigned)((left != 0) ? left->mode : MINI_FOC_MODE_STOP),
             (double)((left != 0) ? left->command : 0.0f),
             (double)((left != 0) ? left->speed_rps : 0.0f),
             (double)((left != 0) ? left->position_rad : 0.0f),
             (unsigned)((right != 0) ? right->online : 0U),
             (unsigned long)((right != 0) ? (now_ms - right->last_rx_ms) : 0U),
             (unsigned)((right != 0) ? right->mode : MINI_FOC_MODE_STOP),
             (double)((right != 0) ? right->command : 0.0f),
             (double)((right != 0) ? right->speed_rps : 0.0f),
             (double)((right != 0) ? right->position_rad : 0.0f));
    MiniVofa_SendText(message);
}

static void send_attitude_text(const MiniMpu6050Data_t *imu)
{
    char message[80];

    if (imu == 0) {
        return;
    }
    snprintf(message,
             sizeof(message),
             "x:%.2f y:%.2f z:%.2f\n",
             (double)(imu->roll_rad * MINI_RAD_TO_DEG),
             (double)(imu->pitch_rad * MINI_RAD_TO_DEG),
             (double)(imu->yaw_rad * MINI_RAD_TO_DEG));
    MiniVofa_SendText(message);
}

static void send_can_status(void)
{
    char message[96];

    snprintf(message,
             sizeof(message),
             "can task loops=%lu boot1=%lu boot2=%lu\r\n",
             (unsigned long)ctrl_loop_count,
             (unsigned long)can1_boot_test_count,
             (unsigned long)can2_boot_test_count);
    MiniVofa_SendText(message);
    send_can_status_line("can1", &hfdcan1);
    send_can_status_line("can2", &hfdcan2);
    send_foc_status();
}

static void send_can_boot_test_frame(FDCAN_HandleTypeDef *hfdcan,
                                     uint32_t std_id,
                                     uint8_t marker,
                                     uint32_t *counter)
{
    uint8_t data[8];
    uint32_t value;

    if (counter == 0) {
        return;
    }

    value = (*counter)++;
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
    data[2] = (uint8_t)((value >> 16U) & 0xFFU);
    data[3] = (uint8_t)((value >> 24U) & 0xFFU);
    data[4] = marker;
    data[5] = 0x55U;
    data[6] = 0xAAU;
    data[7] = (uint8_t)(data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5] ^ data[6]);

    if (CAN_SendData(hfdcan, std_id, data) == HAL_OK) {
        if (hfdcan == &hfdcan1) {
            MiniStatusLed_Pulse(MINI_STATUS_LED_CAN1);
        } else if (hfdcan == &hfdcan2) {
            MiniStatusLed_Pulse(MINI_STATUS_LED_CAN2);
        }
    }
}

static void send_can_boot_test_all(void)
{
#if MINI_CAN_BOOT_TEST_ENABLE
    send_can_boot_test_frame(&hfdcan1,
                             MINI_CAN1_BOOT_TEST_ID,
                             0xC1U,
                             &can1_boot_test_count);
    send_can_boot_test_frame(&hfdcan2,
                             MINI_CAN2_BOOT_TEST_ID,
                             0xC2U,
                             &can2_boot_test_count);
#endif
}

static uint8_t foc_any_direct_command_active(void)
{
    const MiniFocMotor_t *left = MiniFoc_GetMotor(0U);
    const MiniFocMotor_t *right = MiniFoc_GetMotor(1U);

    return ((left != 0 && left->mode != MINI_FOC_MODE_STOP) ||
            (right != 0 && right->mode != MINI_FOC_MODE_STOP))
               ? 1U
               : 0U;
}

static void apply_vofa_command(const MiniVofaCommand_t *cmd)
{
    uint8_t side;
    uint16_t position;
    HAL_StatusTypeDef result;

    if (cmd == 0) {
        return;
    }

    switch (cmd->type) {
    case MINI_VOFA_CMD_VEL:
        foc_direct_mode_enabled = 0U;
        MiniChassis_SetVelocity(cmd->a, cmd->b);
        can_auto_tx_enabled = 1U;
        break;
    case MINI_VOFA_CMD_ENABLE:
        foc_direct_mode_enabled = 0U;
        MiniChassis_SetEnabled(1U);
        can_auto_tx_enabled = 1U;
        break;
    case MINI_VOFA_CMD_STOP:
        foc_direct_mode_enabled = 0U;
        balance_state = MINI_BALANCE_STATE_SLEEP;
        MiniChassis_SetEnabled(0U);
        MiniFoc_SetCommand(0U, MINI_FOC_MODE_STOP, 0.0f);
        MiniFoc_SetCommand(1U, MINI_FOC_MODE_STOP, 0.0f);
        MiniFoc_SendAll();
        can_auto_tx_enabled = 0U;
        break;
    case MINI_VOFA_CMD_STAND:
        balance_request_stand("cmd");
        break;
    case MINI_VOFA_CMD_SLEEP:
        balance_enter_sleep("cmd");
        break;
    case MINI_VOFA_CMD_TELEMETRY: {
        char message[32];
        telemetry_enabled = cmd->enable;
        snprintf(message, sizeof(message), "telemetry=%u\r\n", (unsigned)telemetry_enabled);
        MiniVofa_SendText(message);
        break;
    }
    case MINI_VOFA_CMD_CAN_STAT:
        send_can_status();
        break;
    case MINI_VOFA_CMD_CAN_RESTART:
        if (cmd->index == 1U) {
            FDCAN1_Restart();
        } else if (cmd->index == 2U) {
            FDCAN2_Restart();
        }
        send_can_status();
        break;
    case MINI_VOFA_CMD_CAN_AUTO: {
        char message[32];
        can_auto_tx_enabled = cmd->enable;
        snprintf(message, sizeof(message), "can auto=%u\r\n", (unsigned)can_auto_tx_enabled);
        MiniVofa_SendText(message);
        break;
    }
    case MINI_VOFA_CMD_CAN_TX: {
        char message[40];
        HAL_StatusTypeDef status = HAL_ERROR;

        if (cmd->index >= 1U && cmd->index <= MINI_ROBOT_WHEEL_COUNT) {
            status = MiniFoc_SendIndex((uint8_t)(cmd->index - 1U));
        }
        snprintf(message,
                 sizeof(message),
                 "can tx%u st=%u\r\n",
                 (unsigned)cmd->index,
                 (unsigned)status);
        MiniVofa_SendText(message);
        send_can_status();
        break;
    }
    case MINI_VOFA_CMD_FOC_DIRECT: {
        char message[64];
        HAL_StatusTypeDef status;

        status = MiniFoc_CommandNode(cmd->index, (MiniFocMode_t)cmd->mode, cmd->a);
        if (cmd->mode == MINI_FOC_MODE_STOP) {
            foc_direct_mode_enabled =
                (cmd->index == 0U) ? 0U : foc_any_direct_command_active();
        } else {
            foc_direct_mode_enabled = 1U;
        }
        can_auto_tx_enabled = 0U;
        snprintf(message,
                 sizeof(message),
                 "foc node=%u mode=%u value=%.4f st=%u\r\n",
                 (unsigned)cmd->index,
                 (unsigned)cmd->mode,
                 (double)cmd->a,
                 (unsigned)status);
        MiniVofa_SendText(message);
        break;
    }
    case MINI_VOFA_CMD_BLOCK_RESET:
        balance_reset_controllers();
        MiniVofa_SendText("block reset ok\r\n");
        break;
    case MINI_VOFA_CMD_WHEEL_POS:
        foc_direct_mode_enabled = 0U;
        MiniChassis_SetWheelPosition(cmd->a, cmd->b);
        can_auto_tx_enabled = 1U;
        break;
    case MINI_VOFA_CMD_PID_SPEED:
        MiniChassis_SetPid(MINI_PID_SLOT_SPEED, cmd->index, cmd->a, cmd->b, cmd->c);
        break;
    case MINI_VOFA_CMD_PID_POSITION:
        MiniChassis_SetPid(MINI_PID_SLOT_POSITION, cmd->index, cmd->a, cmd->b, cmd->c);
        break;
    case MINI_VOFA_CMD_LQR_ENABLE:
        MiniChassis_SetLqrEnabled(cmd->enable);
        break;
    case MINI_VOFA_CMD_LQR_GAIN:
        MiniChassis_SetLqrGain(cmd->index, cmd->enable, cmd->a);
        break;
    case MINI_VOFA_CMD_LQR_A:
        MiniChassis_SetLqrA(cmd->index, cmd->enable, cmd->a);
        break;
    case MINI_VOFA_CMD_LQR_B:
        MiniChassis_SetLqrB(cmd->index, cmd->enable, cmd->a);
        break;
    case MINI_VOFA_CMD_LQR_STATE: {
        float x[MINI_LQR_STATE_DIM] = {cmd->a, cmd->b, cmd->c, cmd->d, cmd->e, cmd->f};
        MiniChassis_SetLqrState(x);
        break;
    }
    case MINI_VOFA_CMD_LQR_TARGET: {
        float x_ref[MINI_LQR_STATE_DIM] = {cmd->a, cmd->b, cmd->c, cmd->d, cmd->e, cmd->f};
        MiniChassis_SetLqrTarget(x_ref);
        break;
    }
    case MINI_VOFA_CMD_LQR_FEEDFORWARD:
        MiniChassis_SetLqrFeedforward(cmd->index, cmd->a);
        break;
    case MINI_VOFA_CMD_LQR_LIMIT:
        MiniChassis_SetLqrOutputLimit(cmd->a);
        break;
    case MINI_VOFA_CMD_SERVO_POS:
        side = servo_side_from_command(cmd->index);
        position = clamp_servo_position(cmd->a);
        result = MiniServo_WritePositionSide(side,
                                             position,
                                             MINI_SERVO_MOVE_TIME_MS,
                                             MINI_SERVO_MOVE_SPEED);
        if (result == HAL_OK) {
            update_servo_shadow(side, position);
        }
        send_servo_status("pos", side, result);
        send_servo_packet("tx", MiniServo_GetLastTx);
        send_servo_packet("rx", MiniServo_GetLastRx);
        break;
    case MINI_VOFA_CMD_SERVO_PING:
        side = servo_side_from_command(cmd->index);
        result = MiniServo_PingSide(side);
        send_servo_status("ping", side, result);
        send_servo_packet("tx", MiniServo_GetLastTx);
        send_servo_packet("rx", MiniServo_GetLastRx);
        break;
    case MINI_VOFA_CMD_SERVO_TORQUE:
        side = servo_side_from_command(cmd->index);
        result = MiniServo_TorqueEnableSide(side, cmd->enable);
        send_servo_status(cmd->enable ? "torque_on" : "torque_off", side, result);
        send_servo_packet("tx", MiniServo_GetLastTx);
        send_servo_packet("rx", MiniServo_GetLastRx);
        break;
    case MINI_VOFA_CMD_SERVO_READ: {
        MiniServoStatus_t status;
        side = servo_side_from_command(cmd->index);
        result = MiniServo_ReadStatusSide(side, &status);
        send_servo_packet("tx", MiniServo_GetLastTx);
        send_servo_packet("rx", MiniServo_GetLastRx);
        send_servo_readout(&status, result);
        break;
    }
    case MINI_VOFA_CMD_SERVO_BAUD: {
        char message[56];
        uint32_t baud = (cmd->a < 1.0f) ? MINI_SERVO_UART_BAUD : (uint32_t)cmd->a;
        result = MiniServo_SetBaud(baud);
        snprintf(message,
                 sizeof(message),
                 "servo baud=%lu st=%u\r\n",
                 (unsigned long)baud,
                 (unsigned)result);
        MiniVofa_SendText(message);
        break;
    }
    case MINI_VOFA_CMD_SERVO_PROBE: {
        MiniServoDebug_t debug;
        char message[192];

        side = servo_side_from_command(cmd->index);
        result = MiniServo_DebugProbeSide(side, &debug);
        snprintf(message,
                 sizeof(message),
                 "servo probe side=%s id=%u st=%u txst=%u len=%u isr0=0x%08lX isr1=0x%08lX err=0x%08lX\r\n",
                 servo_side_name(side),
                 (unsigned)servo_id_from_side(side),
                 (unsigned)result,
                 (unsigned)debug.tx_status,
                 (unsigned)debug.length,
                 (unsigned long)debug.isr_before,
                 (unsigned long)debug.isr_after,
                 (unsigned long)debug.error_flags);
        MiniVofa_SendText(message);
        send_servo_packet("tx", MiniServo_GetLastTx);
        send_servo_packet("rxraw", MiniServo_GetLastRx);
        break;
    }
    case MINI_VOFA_CMD_SERVO_PAIR: {
        uint16_t left = clamp_servo_position(cmd->a);
        uint16_t right = clamp_servo_position(cmd->b);
        result = MiniServo_WritePair(left, right, MINI_SERVO_MOVE_SPEED);
        if (result == HAL_OK) {
            servo_pos[0] = left;
            servo_pos[1] = right;
        }
        send_servo_status("pair", MINI_SERVO_SIDE_ALL, result);
        send_servo_packet("tx", MiniServo_GetLastTx);
        send_servo_packet("rx", MiniServo_GetLastRx);
        break;
    }
    case MINI_VOFA_CMD_SERVO_SCAN:
        send_servo_scan();
        break;
    case MINI_VOFA_CMD_SERVO_SHUTDOWN:
        MiniChassis_RequestServoShutdown();
        break;
    default:
        break;
    }
}

static uint8_t remote_packet_valid(const MiniRemotePacket_t *packet)
{
    return (packet->magic == REMOTE_PACKET_MAGIC &&
            packet->version == REMOTE_PACKET_VERSION &&
            packet->length == sizeof(*packet) &&
            packet->checksum ==
                checksum16((const uint8_t *)packet, REMOTE_PACKET_CHECKSUM_LENGTH))
               ? 1U
               : 0U;
}

static void apply_remote_packet(const MiniRemotePacket_t *packet, uint32_t now_ms)
{
    float speed_scale = ((packet->key_bits & MINI_REMOTE_KEY_FAST_MASK) != 0U)
                            ? MINI_REMOTE_FAST_SCALE
                            : MINI_REMOTE_SLOW_SCALE;
    float vx = normalize_axis(packet->left_y) * MINI_REMOTE_MAX_VX_MPS * speed_scale;
    float wz = normalize_axis(packet->left_x) * MINI_REMOTE_MAX_WZ_RPS * speed_scale;
    float lift = normalize_axis(packet->right_y) * (float)MINI_REMOTE_SERVO_RANGE;
    uint16_t left_position = clamp_servo_position(
        (float)MINI_SERVO_CENTER_POS + MINI_REMOTE_SERVO_LEFT_SIGN * lift);
    uint16_t right_position = clamp_servo_position(
        (float)MINI_SERVO_CENTER_POS + MINI_REMOTE_SERVO_RIGHT_SIGN * lift);

    MiniChassis_SetVelocity(vx, wz);
    remote_jump_request =
        ((packet->key_bits & MINI_REMOTE_KEY_JUMP_MASK) != 0U) ? 1U : 0U;

    if ((now_ms - remote_last_servo_ms) >= MINI_REMOTE_SERVO_MIN_PERIOD_MS &&
        (servo_delta(left_position, servo_pos[0]) >= MINI_REMOTE_SERVO_MIN_DELTA ||
         servo_delta(right_position, servo_pos[1]) >= MINI_REMOTE_SERVO_MIN_DELTA)) {
        if (MiniServo_WritePair(left_position,
                                right_position,
                                MINI_SERVO_MOVE_SPEED) == HAL_OK) {
            servo_pos[0] = left_position;
            servo_pos[1] = right_position;
            remote_last_servo_ms = now_ms;
        }
    }
}

static void poll_remote(uint32_t now_ms)
{
    uint8_t payload[MINI_NRF24_PAYLOAD_SIZE];
    MiniRemotePacket_t packet;

    while (MiniNrf24_Poll(payload)) {
        memcpy(&packet, payload, sizeof(packet));
        if (remote_packet_valid(&packet)) {
            apply_remote_packet(&packet, now_ms);
        }
    }
}

void MiniRobot_Init(void)
{
    HAL_StatusTypeDef mpu_status;
    HAL_StatusTypeDef nrf_status;
    MiniPidParam_t balance_pitch_param = {
        .kp = MINI_BALANCE_PID_KP,
        .ki = MINI_BALANCE_PID_KI,
        .kd = MINI_BALANCE_PID_KD,
        .integral_limit = MINI_BALANCE_PID_INTEGRAL_LIMIT,
        .output_limit = MINI_BALANCE_PID_OUTPUT_LIMIT_A,
    };
    MiniPidParam_t balance_roll_param = {
        .kp = MINI_BALANCE_ROLL_PID_KP,
        .ki = MINI_BALANCE_ROLL_PID_KI,
        .kd = MINI_BALANCE_ROLL_PID_KD,
        .integral_limit = MINI_BALANCE_PID_INTEGRAL_LIMIT,
        .output_limit = MINI_BALANCE_ROLL_OUTPUT_LIMIT_A,
    };
    char message[96];

    MiniStatusLed_Init();
    MiniFoc_Init();
    MiniChassis_Init();
    MiniPid_Init(&balance_pitch_pid, &balance_pitch_param);
    MiniPid_Init(&balance_roll_pid, &balance_roll_param);
    balance_reset_controllers();
    MiniServo_Init(&huart10, &huart1);

    (void)CAN1_Filter_Init();
    (void)CAN2_Filter_Init();
    (void)DRV_UART7_StartRx();

    mpu_status = MiniMpu6050_Init(&hi2c2);
    nrf_status = MiniNrf24_Init(&hspi1);

    MiniVofa_SendText("\r\nmini controller v2 init\r\n");
    MiniVofa_SendText("pc: uart7 pe7/pe8 115200 8N1\r\n");
    MiniVofa_SendText("servos swapped test: left usart10 pe3/pe2, right usart1 pa9/pa10, 1Mbps\r\n");
    MiniVofa_SendText("telemetry default off, send telemetry 1 for JustFloat\r\n");
    MiniVofa_SendText("can auto default off, use can tx 1/2 for single-frame test\r\n");
#if MINI_CAN_BOOT_TEST_ENABLE
    MiniVofa_SendText("can boot test on: CAN1 id=0x101, CAN2 id=0x102, 100ms\r\n");
#endif
    snprintf(message,
             sizeof(message),
             "mpu6050 i2c2=%u, nrf24 spi1=%u\r\n",
             (unsigned)mpu_status,
             (unsigned)nrf_status);
    MiniVofa_SendText(message);
    MiniVofa_SendText("balance cmd: stand = servo support + pitch PID wheels, sleep = motor stop + servo torque off\r\n");
#if MINI_ROBOT_AUTO_STAND_ON_BOOT
    balance_request_stand("boot");
#else
    MiniVofa_SendText("safe boot: sleep state, send stand to balance\r\n");
#endif
}

void MiniRobot_ControlStep(void)
{
    uint32_t now_ms = HAL_GetTick();

    MiniStatusLed_Update(now_ms);
    if (balance_is_active()) {
        balance_control_update(now_ms, (float)MINI_ROBOT_CONTROL_PERIOD_MS * 0.001f);
    } else if (foc_direct_mode_enabled == 0U) {
        MiniChassis_Update((float)MINI_ROBOT_CONTROL_PERIOD_MS * 0.001f);
    }
    MiniFoc_Heartbeat(now_ms);
}

void MiniRobot_TelemetryStep(void)
{
    MiniVofaTelemetry_t telemetry;
    const MiniChassisCommand_t *cmd = MiniChassis_GetCommand();
    const MiniChassisState_t *state = MiniChassis_GetState();
    const MiniFocMotor_t *left = MiniFoc_GetMotor(0U);
    const MiniFocMotor_t *right = MiniFoc_GetMotor(1U);
    const MiniMpu6050Data_t *imu;
    uint32_t now_ms = HAL_GetTick();

    if (MiniMpu6050_Update((float)MINI_MPU6050_UPDATE_PERIOD_MS * 0.001f) == HAL_OK) {
        imu = MiniMpu6050_GetData();
        MiniChassis_SetMeasuredAttitude(imu->pitch_rad, imu->pitch_rate_rps);
        if (balance_state == MINI_BALANCE_STATE_STAND) {
            send_attitude_text(imu);
        }
    }

    memset(&telemetry, 0, sizeof(telemetry));
    telemetry.wheel_speed_l = (left != 0) ? left->speed_rps : 0.0f;
    telemetry.wheel_speed_r = (right != 0) ? right->speed_rps : 0.0f;
    telemetry.wheel_target_l = state->wheel_target_rps[0];
    telemetry.wheel_target_r = state->wheel_target_rps[1];
    telemetry.chassis_vx = cmd->vx_mps;
    telemetry.chassis_wz = cmd->wz_rps;
    telemetry.servo_pos_l = (float)servo_pos[0];
    telemetry.servo_pos_r = (float)servo_pos[1];
    if (telemetry_enabled) {
        MiniVofa_SendTelemetry(&telemetry);
    }

    MiniStatusLed_Update(now_ms);
#if MINI_IWDG_ENABLE
    HAL_IWDG_Refresh(&hiwdg1);
#endif
}

void MiniRobot_CommandStep(void)
{
    MiniVofaCommand_t command;
    static uint8_t uart_snapshot[MINI_VOFA_RX_BUF_LEN];
    uint16_t uart_length = 0U;
    uint32_t now_ms = HAL_GetTick();

    __disable_irq();
    if (pending_uart_ready) {
        uart_length = pending_uart_length;
        memcpy(uart_snapshot, pending_uart_data, uart_length);
        pending_uart_ready = 0U;
    }
    __enable_irq();

    if (uart_length > 0U) {
        if (MiniVofa_ParseCommand(uart_snapshot, uart_length, &command)) {
            apply_vofa_command(&command);
            MiniVofa_SendText("ack cmd\r\n");
        } else {
            MiniVofa_SendText("ack rx parse fail\r\n");
        }
    }

    poll_remote(now_ms);
    MiniServo_Poll(now_ms);
    balance_step(now_ms);

    if (MiniChassis_GetCommand()->servo_shutdown_request) {
        if (MiniServo_ShutdownPose() == HAL_OK) {
            servo_pos[0] = MINI_SERVO_SHUTDOWN_POS;
            servo_pos[1] = MINI_SERVO_SHUTDOWN_POS;
        }
        MiniChassis_ClearServoShutdownRequest();
    }

    /* Jump is deliberately reserved until a protected jump state machine exists. */
    (void)remote_jump_request;
}

void CAN1_rxDataHandler(uint32_t rx_id, uint8_t *rx_data)
{
    MiniFoc_OnCanRx(&hfdcan1, rx_id, rx_data);
}

void CAN2_rxDataHandler(uint32_t rx_id, uint8_t *rx_data)
{
    MiniFoc_OnCanRx(&hfdcan2, rx_id, rx_data);
}

void UART7_rxDataHandler(const uint8_t *rx_data, uint16_t length)
{
    if (rx_data == 0 || length == 0U) {
        return;
    }
    if (length > sizeof(pending_uart_data)) {
        length = sizeof(pending_uart_data);
    }
    memcpy(pending_uart_data, rx_data, length);
    pending_uart_length = length;
    pending_uart_ready = 1U;
}

void StartCtrlTask(void *argument)
{
    uint32_t tick = osKernelGetTickCount();
    uint32_t can_elapsed_ms = 0U;
    uint32_t can_boot_test_elapsed_ms = 0U;

    (void)argument;
    for (;;) {
        ctrl_loop_count++;
        MiniRobot_ControlStep();
        can_boot_test_elapsed_ms += MINI_ROBOT_CONTROL_PERIOD_MS;
        if (can_boot_test_elapsed_ms >= MINI_CAN_BOOT_TEST_PERIOD_MS) {
            can_boot_test_elapsed_ms = 0U;
            send_can_boot_test_all();
        }
        can_elapsed_ms += MINI_ROBOT_CONTROL_PERIOD_MS;
        if (can_elapsed_ms >= MINI_ROBOT_CAN_PERIOD_MS) {
            can_elapsed_ms = 0U;
            if (can_auto_tx_enabled) {
                MiniFoc_SendAll();
            } else if (foc_direct_mode_enabled) {
                MiniFoc_SendAll();
            }
        }
        tick += MINI_ROBOT_CONTROL_PERIOD_MS;
        osDelayUntil(tick);
    }
}

void StartCommandTask(void *argument)
{
    (void)argument;
    for (;;) {
        MiniRobot_CommandStep();
        osDelay(20U);
    }
}

void StartMonitorTask(void *argument)
{
    (void)argument;
    for (;;) {
        MiniRobot_TelemetryStep();
        osDelay(MINI_ROBOT_VOFA_PERIOD_MS);
    }
}

void StartRP_LogTask(void *argument)
{
    (void)argument;
    for (;;) {
        osDelay(1000U);
    }
}
