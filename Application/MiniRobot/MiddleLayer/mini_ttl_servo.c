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
    clear_uart_error(bus->uart);

    if (HAL_UART_Transmit(bus->uart, last_tx_packet, tx_len, STS_TX_TIMEOUT_MS) != HAL_OK)
    {
        return HAL_ERROR;
    }

    MiniStatusLed_Pulse((bus == &servo_buses[0]) ? MINI_STATUS_LED_USART10 : MINI_STATUS_LED_USART1);
    return HAL_OK;
}

static HAL_StatusTypeDef receive_packet(MiniServoBus_t *bus, uint8_t *rx_len)
{
    uint8_t head[4];
    uint8_t remain;

    if (bus == NULL || bus->uart == NULL || rx_len == NULL)
    {
        return HAL_ERROR;
    }

    memset(last_rx_packet, 0, sizeof(last_rx_packet));
    last_rx_len = 0U;

    if (HAL_UART_Receive(bus->uart, head, sizeof(head), STS_RX_TIMEOUT_MS) != HAL_OK)
    {
        return HAL_TIMEOUT;
    }

    if (head[0] != STS_HEADER || head[1] != STS_HEADER || head[2] != bus->id)
    {
        return HAL_ERROR;
    }

    remain = head[3];
    if (remain > (STS_PACKET_MAX_LEN - 4U))
    {
        return HAL_ERROR;
    }

    memcpy(last_rx_packet, head, sizeof(head));
    if (HAL_UART_Receive(bus->uart, &last_rx_packet[4], remain, STS_RX_TIMEOUT_MS) != HAL_OK)
    {
        return HAL_TIMEOUT;
    }

    last_rx_len = (uint8_t)(remain + 4U);
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

    if (position > MINI_SERVO_POSITION_MAX)
    {
        position = MINI_SERVO_POSITION_MAX;
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
    return MiniServo_WritePair(MINI_SERVO_SHUTDOWN_POS, MINI_SERVO_SHUTDOWN_POS, MINI_SERVO_MOVE_SPEED);
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
}

uint8_t MiniServo_IsOnline(void)
{
    return (uint8_t)(servo_buses[0].online && servo_buses[1].online);
}
