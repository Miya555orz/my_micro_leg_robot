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
#include "mini_robot_config.h"
#include "mini_status_led.h"
#include "mini_ttl_servo.h"
#include "mini_vofa.h"
#include "spi.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

#define REMOTE_PACKET_MAGIC            0xA5U
#define REMOTE_PACKET_VERSION          0x01U
#define REMOTE_PACKET_CHECKSUM_LENGTH  30U
#define REMOTE_AXIS_LIMIT              1000

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

static uint8_t servo_id_from_command(uint8_t value)
{
    return (value == 0U) ? MINI_SERVO_LEFT_ID : value;
}

static void update_servo_shadow(uint8_t id, uint16_t position)
{
    if (id == MINI_SERVO_LEFT_ID) {
        servo_pos[0] = position;
    } else if (id == MINI_SERVO_RIGHT_ID) {
        servo_pos[1] = position;
    }
}

static void send_servo_status(const char *operation, uint8_t id, HAL_StatusTypeDef status)
{
    char message[56];

    snprintf(message,
             sizeof(message),
             "servo %s id=%u st=%u\r\n",
             operation,
             (unsigned)id,
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

static void apply_vofa_command(const MiniVofaCommand_t *cmd)
{
    uint8_t id;
    uint16_t position;
    HAL_StatusTypeDef result;

    if (cmd == 0) {
        return;
    }

    switch (cmd->type) {
    case MINI_VOFA_CMD_VEL:
        MiniChassis_SetVelocity(cmd->a, cmd->b);
        break;
    case MINI_VOFA_CMD_ENABLE:
        MiniChassis_SetEnabled(1U);
        break;
    case MINI_VOFA_CMD_STOP:
        MiniChassis_SetEnabled(0U);
        break;
    case MINI_VOFA_CMD_WHEEL_POS:
        MiniChassis_SetWheelPosition(cmd->a, cmd->b);
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
        id = servo_id_from_command(cmd->index);
        position = clamp_servo_position(cmd->a);
        result = MiniServo_WritePosition(id,
                                         position,
                                         MINI_SERVO_MOVE_TIME_MS,
                                         MINI_SERVO_MOVE_SPEED);
        if (result == HAL_OK) {
            update_servo_shadow(id, position);
        }
        send_servo_status("pos", id, result);
        send_servo_packet("tx", MiniServo_GetLastTx);
        break;
    case MINI_VOFA_CMD_SERVO_PING:
        id = servo_id_from_command(cmd->index);
        result = MiniServo_Ping(id);
        send_servo_status("ping", id, result);
        send_servo_packet("tx", MiniServo_GetLastTx);
        send_servo_packet("rx", MiniServo_GetLastRx);
        break;
    case MINI_VOFA_CMD_SERVO_TORQUE:
        id = servo_id_from_command(cmd->index);
        result = MiniServo_TorqueEnable(id, cmd->enable);
        send_servo_status(cmd->enable ? "torque_on" : "torque_off", id, result);
        send_servo_packet("tx", MiniServo_GetLastTx);
        break;
    case MINI_VOFA_CMD_SERVO_READ: {
        MiniServoStatus_t status;
        id = servo_id_from_command(cmd->index);
        result = MiniServo_ReadStatus(id, &status);
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
    case MINI_VOFA_CMD_SERVO_PAIR: {
        uint16_t left = clamp_servo_position(cmd->a);
        uint16_t right = clamp_servo_position(cmd->b);
        result = MiniServo_WritePair(left, right, MINI_SERVO_MOVE_SPEED);
        if (result == HAL_OK) {
            servo_pos[0] = left;
            servo_pos[1] = right;
        }
        send_servo_status("pair", 0xFEU, result);
        send_servo_packet("tx", MiniServo_GetLastTx);
        break;
    }
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
    char message[96];

    MiniStatusLed_Init();
    MiniFoc_Init();
    MiniChassis_Init();
    MiniServo_Init(&huart1, &huart10);

    (void)CAN1_Filter_Init();
    (void)CAN2_Filter_Init();
    (void)DRV_UART7_StartRx();

    mpu_status = MiniMpu6050_Init(&hi2c2);
    nrf_status = MiniNrf24_Init(&hspi1);

    MiniVofa_SendText("\r\nmini controller v2 init\r\n");
    MiniVofa_SendText("pc: uart7 pe7/pe8 115200 8N1\r\n");
    MiniVofa_SendText("servos: usart1 pa9/pa10, usart10 pe3/pe2, 1Mbps\r\n");
    snprintf(message,
             sizeof(message),
             "mpu6050 i2c2=%u, nrf24 spi1=%u\r\n",
             (unsigned)mpu_status,
             (unsigned)nrf_status);
    MiniVofa_SendText(message);
    MiniVofa_SendText("safe boot: motors stopped, no servo command sent\r\n");
}

void MiniRobot_ControlStep(void)
{
    MiniChassis_Update((float)MINI_ROBOT_CONTROL_PERIOD_MS * 0.001f);
    MiniFoc_Heartbeat(HAL_GetTick());
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
    MiniVofa_SendTelemetry(&telemetry);

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

    (void)argument;
    for (;;) {
        MiniRobot_ControlStep();
        can_elapsed_ms += MINI_ROBOT_CONTROL_PERIOD_MS;
        if (can_elapsed_ms >= MINI_ROBOT_CAN_PERIOD_MS) {
            can_elapsed_ms = 0U;
            MiniFoc_SendAll();
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
