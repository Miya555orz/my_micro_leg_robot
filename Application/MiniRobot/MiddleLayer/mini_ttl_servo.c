/**
 ******************************************************************************
 * @file    mini_ttl_servo.c
 * @brief   Feetech STS serial servo driver.
 * @author  Miya Zheng
 * @date    2026-08-02
 ******************************************************************************
 */
#include "mini_ttl_servo.h"

#include "mini_robot_config.h"
#include "mini_status_led.h"
#include <string.h>

#define STS_HEADER                         0xFFU
#define STS_INST_PING                      0x01U
#define STS_INST_READ                      0x02U
#define STS_INST_WRITE                     0x03U
#define STS_REG_TORQUE_ENABLE              0x28U
#define STS_REG_ACC                        0x29U
#define STS_REG_PRESENT_POS                0x38U
#define STS_RX_TIMEOUT_MS                  6U
#define STS_TX_TIMEOUT_MS                  10U
#define STS_PACKET_MAX_LEN                 32U
#define STS_HEADER_SCAN_LIMIT              8U

typedef struct
{
    UART_HandleTypeDef *uart;
    uint8_t id;
    uint8_t online;
    uint32_t last_rx_ms;
} MiniServoBus_t;

static MiniServoBus_t servo_buses[2];
static uint8_t last_tx_packet[STS_PACKET_MAX_LEN];
static uint8_t last_rx_packet[STS_PACKET_MAX_LEN];
static uint8_t last_tx_len;
static uint8_t last_rx_len;
static MiniServoSafetyState_t safety_state;
static uint32_t last_safe_hold_ms;

static uint8_t sts_checksum(uint8_t id, uint8_t length, uint8_t instruction, const uint8_t *params)
{
    uint16_t sum = (uint16_t)id + length + instruction;
    uint8_t i;

    for (i = 0U; i < (uint8_t)(length - 2U); ++i)
    {
        sum += params[i];
    }

    return (uint8_t)(~sum);
}

static MiniServoBus_t *get_bus(uint8_t side)
{
    if (side == MINI_SERVO_SIDE_LEFT)
    {
        return &servo_buses[0];
    }

    if (side == MINI_SERVO_SIDE_RIGHT)
    {
        return &servo_buses[1];
    }

    return NULL;
}

static uint16_t side_min(uint8_t side)
{
    return (side == MINI_SERVO_SIDE_RIGHT) ? MINI_SERVO_RIGHT_POSITION_MIN : MINI_SERVO_LEFT_POSITION_MIN;
}

static uint16_t side_max(uint8_t side)
{
    return (side == MINI_SERVO_SIDE_RIGHT) ? MINI_SERVO_RIGHT_POSITION_MAX : MINI_SERVO_LEFT_POSITION_MAX;
}

static uint16_t side_safe(uint8_t side)
{
    return (side == MINI_SERVO_SIDE_RIGHT) ? MINI_SERVO_RIGHT_SAFE_POSITION : MINI_SERVO_LEFT_SAFE_POSITION;
}

static uint16_t clamp_side_position(uint8_t side, uint16_t target)
{
    const uint16_t min_pos = side_min(side);
    const uint16_t max_pos = side_max(side);

    if (target < min_pos)
    {
        safety_state.fault_flags |= MINI_SERVO_FAULT_POSITION_RANGE;
        return min_pos;
    }
    if (target > max_pos)
    {
        safety_state.fault_flags |= MINI_SERVO_FAULT_POSITION_RANGE;
        return max_pos;
    }
    return target;
}

static uint8_t side_position_in_range(uint8_t side, uint16_t position)
{
    return (position >= side_min(side) && position <= side_max(side)) ? 1U : 0U;
}

static uint16_t limit_step(uint8_t side, uint16_t target)
{
    uint16_t current = safety_state.last_position[side];

    if (side > MINI_SERVO_SIDE_RIGHT || safety_state.position_valid[side] == 0U)
    {
        safety_state.fault_flags |= MINI_SERVO_FAULT_COMM_CHECK;
        return side_safe(side);
    }

    if (target > current && (target - current) > MINI_SERVO_MAX_STEP_PER_CMD)
    {
        return (uint16_t)(current + MINI_SERVO_MAX_STEP_PER_CMD);
    }
    if (current > target && (current - target) > MINI_SERVO_MAX_STEP_PER_CMD)
    {
        return (uint16_t)(current - MINI_SERVO_MAX_STEP_PER_CMD);
    }
    return target;
}

static void clear_uart_error(UART_HandleTypeDef *uart)
{
    if (uart == NULL)
    {
        return;
    }

    __HAL_UART_CLEAR_OREFLAG(uart);
    __HAL_UART_CLEAR_FEFLAG(uart);
    __HAL_UART_CLEAR_NEFLAG(uart);
    __HAL_UART_CLEAR_PEFLAG(uart);
}

static void drain_uart_rx(UART_HandleTypeDef *uart)
{
    uint8_t byte;
    uint8_t guard = STS_PACKET_MAX_LEN;

    if (uart == NULL)
    {
        return;
    }

    clear_uart_error(uart);
    while (guard > 0U && HAL_UART_Receive(uart, &byte, 1U, 0U) == HAL_OK)
    {
        --guard;
    }
    clear_uart_error(uart);
}

static void mark_rx_fault(HAL_StatusTypeDef status)
{
    if (status == HAL_TIMEOUT)
    {
        safety_state.fault_flags |= MINI_SERVO_FAULT_RX_TIMEOUT;
    }
    else
    {
        safety_state.fault_flags |= MINI_SERVO_FAULT_INVALID_PACKET;
    }
}

static HAL_StatusTypeDef send_packet(MiniServoBus_t *bus, uint8_t instruction, const uint8_t *params, uint8_t param_len)
{
    uint8_t tx_len;

    if (bus == NULL || bus->uart == NULL || param_len > (STS_PACKET_MAX_LEN - 6U))
    {
        return HAL_ERROR;
    }

    tx_len = (uint8_t)(param_len + 6U);
    last_tx_packet[0] = STS_HEADER;
    last_tx_packet[1] = STS_HEADER;
    last_tx_packet[2] = bus->id;
    last_tx_packet[3] = (uint8_t)(param_len + 2U);
    last_tx_packet[4] = instruction;

    if (param_len > 0U)
    {
        memcpy(&last_tx_packet[5], params, param_len);
    }

    last_tx_packet[tx_len - 1U] = sts_checksum(bus->id, last_tx_packet[3], instruction, params);
    last_tx_len = tx_len;
    drain_uart_rx(bus->uart);

    if (HAL_UART_Transmit(bus->uart, last_tx_packet, tx_len, STS_TX_TIMEOUT_MS) != HAL_OK)
    {
        return HAL_ERROR;
    }

    MiniStatusLed_Pulse((bus == &servo_buses[0]) ? MINI_STATUS_LED_USART10 : MINI_STATUS_LED_USART1);
    return HAL_OK;
}

static HAL_StatusTypeDef receive_packet(MiniServoBus_t *bus, uint8_t *rx_len)
{
    uint8_t b;
    uint8_t header_state = 0U;
    uint8_t scanned = 0U;
    uint8_t remain;
    uint16_t sum;
    HAL_StatusTypeDef ret;

    if (bus == NULL || bus->uart == NULL || rx_len == NULL)
    {
        return HAL_ERROR;
    }

    memset(last_rx_packet, 0, sizeof(last_rx_packet));
    last_rx_len = 0U;

    while (scanned < STS_HEADER_SCAN_LIMIT)
    {
        ret = HAL_UART_Receive(bus->uart, &b, 1U, STS_RX_TIMEOUT_MS);
        if (ret != HAL_OK)
        {
            mark_rx_fault(ret);
            return ret;
        }
        ++scanned;

        if (header_state == 0U)
        {
            header_state = (b == STS_HEADER) ? 1U : 0U;
        }
        else
        {
            if (b == STS_HEADER)
            {
                last_rx_packet[0] = STS_HEADER;
                last_rx_packet[1] = STS_HEADER;
                break;
            }
            header_state = 0U;
        }
    }

    if (last_rx_packet[0] != STS_HEADER || last_rx_packet[1] != STS_HEADER)
    {
        safety_state.fault_flags |= MINI_SERVO_FAULT_INVALID_PACKET;
        return HAL_ERROR;
    }

    if (HAL_UART_Receive(bus->uart, &last_rx_packet[2], 2U, STS_RX_TIMEOUT_MS) != HAL_OK)
    {
        safety_state.fault_flags |= MINI_SERVO_FAULT_RX_TIMEOUT;
        return HAL_TIMEOUT;
    }

    if (last_rx_packet[2] != bus->id)
    {
        safety_state.fault_flags |= MINI_SERVO_FAULT_INVALID_PACKET;
        return HAL_ERROR;
    }

    remain = last_rx_packet[3];
    if (remain < 2U || remain > (STS_PACKET_MAX_LEN - 4U))
    {
        safety_state.fault_flags |= MINI_SERVO_FAULT_INVALID_PACKET;
        return HAL_ERROR;
    }

    if (HAL_UART_Receive(bus->uart, &last_rx_packet[4], remain, STS_RX_TIMEOUT_MS) != HAL_OK)
    {
        safety_state.fault_flags |= MINI_SERVO_FAULT_RX_TIMEOUT;
        return HAL_TIMEOUT;
    }

    last_rx_len = (uint8_t)(remain + 4U);
    sum = 0U;
    for (uint8_t i = 2U; i < (uint8_t)(last_rx_len - 1U); ++i)
    {
        sum += last_rx_packet[i];
    }
    if ((uint8_t)(~sum) != last_rx_packet[last_rx_len - 1U])
    {
        safety_state.fault_flags |= MINI_SERVO_FAULT_INVALID_PACKET;
        return HAL_ERROR;
    }

    *rx_len = last_rx_len;
    bus->online = 1U;
    bus->last_rx_ms = HAL_GetTick();
    MiniStatusLed_Pulse((bus == &servo_buses[0]) ? MINI_STATUS_LED_USART10 : MINI_STATUS_LED_USART1);
    return HAL_OK;
}

static HAL_StatusTypeDef write_byte(MiniServoBus_t *bus, uint8_t address, uint8_t value, uint8_t wait_ack)
{
    uint8_t params[2];
    HAL_StatusTypeDef status;
    uint8_t rx_len;

    params[0] = address;
    params[1] = value;
    status = send_packet(bus, STS_INST_WRITE, params, sizeof(params));
    if (status != HAL_OK || wait_ack == 0U)
    {
        return status;
    }

    return receive_packet(bus, &rx_len);
}

static HAL_StatusTypeDef write_position(MiniServoBus_t *bus, uint16_t position, uint16_t time_ms, uint16_t speed, uint8_t wait_ack)
{
    uint8_t params[8];
    HAL_StatusTypeDef status;
    uint8_t rx_len;

    if (position > 4095U)
    {
        position = 4095U;
    }

    params[0] = STS_REG_ACC;
    params[1] = MINI_SERVO_MOVE_ACC;
    params[2] = (uint8_t)(position & 0xFFU);
    params[3] = (uint8_t)(position >> 8U);
    params[4] = (uint8_t)(time_ms & 0xFFU);
    params[5] = (uint8_t)(time_ms >> 8U);
    params[6] = (uint8_t)(speed & 0xFFU);
    params[7] = (uint8_t)(speed >> 8U);

    status = send_packet(bus, STS_INST_WRITE, params, sizeof(params));
    if (status != HAL_OK || wait_ack == 0U)
    {
        return status;
    }

    return receive_packet(bus, &rx_len);
}

void MiniServo_Init(UART_HandleTypeDef *left_uart, UART_HandleTypeDef *right_uart)
{
    memset(servo_buses, 0, sizeof(servo_buses));
    servo_buses[0].uart = left_uart;
    servo_buses[0].id = MINI_SERVO_LEFT_ID;
    servo_buses[1].uart = right_uart;
    servo_buses[1].id = MINI_SERVO_RIGHT_ID;
    (void)MiniServo_SetBaud(MINI_SERVO_UART_BAUD);
    MiniServo_SafetyInit();
}

HAL_StatusTypeDef MiniServo_SetBaud(uint32_t baud)
{
    uint8_t i;

    for (i = 0U; i < 2U; ++i)
    {
        if (servo_buses[i].uart != NULL)
        {
            servo_buses[i].uart->Init.BaudRate = baud;
            if (HAL_UART_Init(servo_buses[i].uart) != HAL_OK)
            {
                return HAL_ERROR;
            }
        }
    }

    return HAL_OK;
}

HAL_StatusTypeDef MiniServo_PingSide(uint8_t side)
{
    MiniServoBus_t *bus = get_bus(side);
    HAL_StatusTypeDef status;
    uint8_t rx_len;

    status = send_packet(bus, STS_INST_PING, NULL, 0U);
    if (status != HAL_OK)
    {
        return status;
    }

    return receive_packet(bus, &rx_len);
}

HAL_StatusTypeDef MiniServo_TorqueEnableSide(uint8_t side, uint8_t enable)
{
    if (side == MINI_SERVO_SIDE_ALL)
    {
        HAL_StatusTypeDef left_status = MiniServo_TorqueEnableSide(MINI_SERVO_SIDE_LEFT, enable);
        HAL_StatusTypeDef right_status = MiniServo_TorqueEnableSide(MINI_SERVO_SIDE_RIGHT, enable);
        return (left_status == HAL_OK && right_status == HAL_OK) ? HAL_OK : HAL_ERROR;
    }

    return write_byte(get_bus(side), STS_REG_TORQUE_ENABLE, enable ? 1U : 0U, 1U);
}

HAL_StatusTypeDef MiniServo_TorqueEnableSideNoAck(uint8_t side, uint8_t enable)
{
    if (side == MINI_SERVO_SIDE_ALL)
    {
        HAL_StatusTypeDef left_status = MiniServo_TorqueEnableSideNoAck(MINI_SERVO_SIDE_LEFT, enable);
        HAL_StatusTypeDef right_status = MiniServo_TorqueEnableSideNoAck(MINI_SERVO_SIDE_RIGHT, enable);
        return (left_status == HAL_OK && right_status == HAL_OK) ? HAL_OK : HAL_ERROR;
    }

    return write_byte(get_bus(side), STS_REG_TORQUE_ENABLE, enable ? 1U : 0U, 0U);
}

HAL_StatusTypeDef MiniServo_SetPositionModeSideNoAck(uint8_t side)
{
    (void)side;
    return HAL_OK;
}

HAL_StatusTypeDef MiniServo_ReadStatusSide(uint8_t side, MiniServoStatus_t *status)
{
    MiniServoBus_t *bus = get_bus(side);
    uint8_t params[2];
    uint8_t rx_len;
    HAL_StatusTypeDef ret;

    if (status == NULL)
    {
        return HAL_ERROR;
    }

    params[0] = STS_REG_PRESENT_POS;
    params[1] = 8U;
    ret = send_packet(bus, STS_INST_READ, params, sizeof(params));
    if (ret != HAL_OK)
    {
        return ret;
    }

    ret = receive_packet(bus, &rx_len);
    if (ret != HAL_OK || rx_len < 14U)
    {
        return ret;
    }

    status->id = last_rx_packet[2];
    status->error = last_rx_packet[4];
    status->position = (uint16_t)last_rx_packet[5] | ((uint16_t)last_rx_packet[6] << 8U);
    status->speed = (uint16_t)last_rx_packet[7] | ((uint16_t)last_rx_packet[8] << 8U);
    status->load = (uint16_t)last_rx_packet[9] | ((uint16_t)last_rx_packet[10] << 8U);
    status->voltage_0v1 = last_rx_packet[11];
    status->temperature_c = last_rx_packet[12];
    status->moving = 0U;
    status->current = 0U;
    if (side <= MINI_SERVO_SIDE_RIGHT)
    {
        safety_state.last_position[side] = status->position;
        safety_state.position_valid[side] = 1U;
    }
    return HAL_OK;
}

HAL_StatusTypeDef MiniServo_WritePositionSide(uint8_t side, uint16_t position, uint16_t time_ms, uint16_t speed)
{
    if (side == MINI_SERVO_SIDE_ALL)
    {
        return MiniServo_WritePair(position, position, speed);
    }

    return write_position(get_bus(side), position, time_ms, speed, 1U);
}

HAL_StatusTypeDef MiniServo_WritePositionSideNoAck(uint8_t side, uint16_t position, uint16_t time_ms, uint16_t speed)
{
    if (side == MINI_SERVO_SIDE_ALL)
    {
        return MiniServo_WritePairNoAck(position, position, speed);
    }

    return write_position(get_bus(side), position, time_ms, speed, 0U);
}

HAL_StatusTypeDef MiniServo_Ping(uint8_t id)
{
    (void)id;
    return MiniServo_PingSide(MINI_SERVO_SIDE_RIGHT);
}

HAL_StatusTypeDef MiniServo_TorqueEnable(uint8_t id, uint8_t enable)
{
    (void)id;
    return MiniServo_TorqueEnableSide(MINI_SERVO_SIDE_ALL, enable);
}

HAL_StatusTypeDef MiniServo_ReadStatus(uint8_t id, MiniServoStatus_t *status)
{
    (void)id;
    return MiniServo_ReadStatusSide(MINI_SERVO_SIDE_RIGHT, status);
}

HAL_StatusTypeDef MiniServo_WritePosition(uint8_t id, uint16_t position, uint16_t time_ms, uint16_t speed)
{
    (void)id;
    return MiniServo_WritePositionSide(MINI_SERVO_SIDE_ALL, position, time_ms, speed);
}

HAL_StatusTypeDef MiniServo_WritePair(uint16_t left_pos, uint16_t right_pos, uint16_t speed)
{
    HAL_StatusTypeDef left_status = write_position(&servo_buses[0], left_pos, MINI_SERVO_MOVE_TIME_MS, speed, 1U);
    HAL_StatusTypeDef right_status = write_position(&servo_buses[1], right_pos, MINI_SERVO_MOVE_TIME_MS, speed, 1U);

    return (left_status == HAL_OK && right_status == HAL_OK) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef MiniServo_WritePairNoAck(uint16_t left_pos, uint16_t right_pos, uint16_t speed)
{
    HAL_StatusTypeDef left_status = write_position(&servo_buses[0], left_pos, MINI_SERVO_MOVE_TIME_MS, speed, 0U);
    HAL_StatusTypeDef right_status = write_position(&servo_buses[1], right_pos, MINI_SERVO_MOVE_TIME_MS, speed, 0U);

    return (left_status == HAL_OK && right_status == HAL_OK) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef MiniServo_ShutdownPose(void)
{
    return MiniServo_EnterSafePose();
}

uint8_t MiniServo_GetLastTx(uint8_t *out, uint8_t max_len)
{
    uint8_t len = (last_tx_len < max_len) ? last_tx_len : max_len;

    if (out == NULL)
    {
        return 0U;
    }

    memcpy(out, last_tx_packet, len);
    return len;
}

uint8_t MiniServo_GetLastRx(uint8_t *out, uint8_t max_len)
{
    uint8_t len = (last_rx_len < max_len) ? last_rx_len : max_len;

    if (out == NULL)
    {
        return 0U;
    }

    memcpy(out, last_rx_packet, len);
    return len;
}

void MiniServo_Poll(uint32_t now_ms)
{
    uint8_t i;

    for (i = 0U; i < 2U; ++i)
    {
        if ((now_ms - servo_buses[i].last_rx_ms) > MINI_SERVO_ONLINE_TIMEOUT_MS)
        {
            servo_buses[i].online = 0U;
        }
    }
    MiniServo_SafetyPoll(now_ms);
}

uint8_t MiniServo_IsOnline(void)
{
    return (uint8_t)(servo_buses[0].online && servo_buses[1].online);
}

void MiniServo_SafetyInit(void)
{
    memset(&safety_state, 0, sizeof(safety_state));
    safety_state.last_position[MINI_SERVO_SIDE_LEFT] = MINI_SERVO_LEFT_SAFE_POSITION;
    safety_state.last_position[MINI_SERVO_SIDE_RIGHT] = MINI_SERVO_RIGHT_SAFE_POSITION;
    safety_state.target_position[MINI_SERVO_SIDE_LEFT] = MINI_SERVO_LEFT_SAFE_POSITION;
    safety_state.target_position[MINI_SERVO_SIDE_RIGHT] = MINI_SERVO_RIGHT_SAFE_POSITION;
    last_safe_hold_ms = 0U;
}

HAL_StatusTypeDef MiniServo_CheckCommunication(void)
{
    MiniServoStatus_t left_status;
    MiniServoStatus_t right_status;
    HAL_StatusTypeDef left_ret = MiniServo_ReadStatusSide(MINI_SERVO_SIDE_LEFT, &left_status);
    HAL_StatusTypeDef right_ret = MiniServo_ReadStatusSide(MINI_SERVO_SIDE_RIGHT, &right_status);

    if (left_ret != HAL_OK || right_ret != HAL_OK)
    {
        safety_state.fault_flags |= MINI_SERVO_FAULT_COMM_CHECK;
        return HAL_ERROR;
    }
    if (side_position_in_range(MINI_SERVO_SIDE_LEFT, left_status.position) == 0U ||
        side_position_in_range(MINI_SERVO_SIDE_RIGHT, right_status.position) == 0U)
    {
        safety_state.fault_flags |= MINI_SERVO_FAULT_POSITION_RANGE;
        return HAL_ERROR;
    }

    safety_state.fault_flags = MINI_SERVO_FAULT_NONE;
    return HAL_OK;
}

HAL_StatusTypeDef MiniServo_SetPositionSafeSide(uint8_t side, uint16_t target, uint16_t speed)
{
    uint16_t limited_target;
    uint16_t step_target;
    HAL_StatusTypeDef ret;

    if (side > MINI_SERVO_SIDE_RIGHT)
    {
        return HAL_ERROR;
    }
    if (safety_state.position_valid[side] == 0U)
    {
        safety_state.fault_flags |= MINI_SERVO_FAULT_COMM_CHECK;
        return HAL_ERROR;
    }
    if (side_position_in_range(side, safety_state.last_position[side]) == 0U)
    {
        safety_state.fault_flags |= MINI_SERVO_FAULT_POSITION_RANGE;
        return HAL_ERROR;
    }

    limited_target = clamp_side_position(side, target);
    safety_state.target_position[side] = limited_target;
    step_target = limit_step(side, limited_target);
    ret = MiniServo_WritePositionSide(side,
                                      step_target,
                                      MINI_SERVO_MOVE_TIME_MS,
                                      (speed > MINI_SERVO_MOVE_SPEED) ? MINI_SERVO_MOVE_SPEED : speed);
    if (ret == HAL_OK)
    {
        safety_state.last_position[side] = step_target;
        safety_state.position_valid[side] = 1U;
    }
    else
    {
        mark_rx_fault(ret);
    }
    return ret;
}

HAL_StatusTypeDef MiniServo_SetPairSafe(uint16_t left_pos, uint16_t right_pos, uint16_t speed)
{
    HAL_StatusTypeDef left_ret = MiniServo_SetPositionSafeSide(MINI_SERVO_SIDE_LEFT, left_pos, speed);
    HAL_StatusTypeDef right_ret = MiniServo_SetPositionSafeSide(MINI_SERVO_SIDE_RIGHT, right_pos, speed);

    return (left_ret == HAL_OK && right_ret == HAL_OK) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef MiniServo_EnterSafePose(void)
{
    HAL_StatusTypeDef ret;

    safety_state.safe_pose_active = 1U;
    if (safety_state.position_valid[MINI_SERVO_SIDE_LEFT] == 0U ||
        safety_state.position_valid[MINI_SERVO_SIDE_RIGHT] == 0U)
    {
        if (MiniServo_CheckCommunication() != HAL_OK)
        {
            (void)MiniServo_TorqueEnableSideNoAck(MINI_SERVO_SIDE_ALL, 0U);
            return HAL_ERROR;
        }
    }

    ret = MiniServo_SetPairSafe(MINI_SERVO_LEFT_SAFE_POSITION,
                                MINI_SERVO_RIGHT_SAFE_POSITION,
                                MINI_SERVO_STAND_SPEED);
    if (ret != HAL_OK)
    {
        (void)MiniServo_TorqueEnableSideNoAck(MINI_SERVO_SIDE_ALL, 0U);
    }
    return ret;
}

void MiniServo_SafetyPoll(uint32_t now_ms)
{
    uint16_t left_target;
    uint16_t right_target;

    if (safety_state.safe_pose_active == 0U)
    {
        return;
    }
    if ((now_ms - last_safe_hold_ms) < MINI_SERVO_SAFE_HOLD_PERIOD_MS)
    {
        return;
    }

    last_safe_hold_ms = now_ms;
    if (safety_state.position_valid[MINI_SERVO_SIDE_LEFT] == 0U ||
        safety_state.position_valid[MINI_SERVO_SIDE_RIGHT] == 0U)
    {
        return;
    }

    left_target = limit_step(MINI_SERVO_SIDE_LEFT, MINI_SERVO_LEFT_SAFE_POSITION);
    right_target = limit_step(MINI_SERVO_SIDE_RIGHT, MINI_SERVO_RIGHT_SAFE_POSITION);
    if (write_position(&servo_buses[MINI_SERVO_SIDE_LEFT],
                       left_target,
                       MINI_SERVO_MOVE_TIME_MS,
                       MINI_SERVO_STAND_SPEED,
                       0U) == HAL_OK)
    {
        safety_state.last_position[MINI_SERVO_SIDE_LEFT] = left_target;
    }
    if (write_position(&servo_buses[MINI_SERVO_SIDE_RIGHT],
                       right_target,
                       MINI_SERVO_MOVE_TIME_MS,
                       MINI_SERVO_STAND_SPEED,
                       0U) == HAL_OK)
    {
        safety_state.last_position[MINI_SERVO_SIDE_RIGHT] = right_target;
    }
}

uint8_t MiniServo_HasFault(void)
{
    return (safety_state.fault_flags != MINI_SERVO_FAULT_NONE) ? 1U : 0U;
}

uint32_t MiniServo_GetFaultFlags(void)
{
    return safety_state.fault_flags;
}

const MiniServoSafetyState_t *MiniServo_GetSafetyState(void)
{
    return &safety_state;
}

void MiniServo_ClearFaults(void)
{
    safety_state.fault_flags = MINI_SERVO_FAULT_NONE;
}
