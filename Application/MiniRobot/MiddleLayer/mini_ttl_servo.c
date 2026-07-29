#include "mini_ttl_servo.h"

#include "mini_robot_config.h"
#include "mini_status_led.h"
#include <string.h>

#define STS_INST_PING          0x01U
#define STS_INST_READ          0x02U
#define STS_INST_WRITE         0x03U
#define STS_INST_SYNC_WRITE    0x83U

#define STS_REG_TORQUE_ENABLE  40U
#define STS_REG_ACC            41U
#define STS_REG_PRESENT_POS    56U

#define STS_STATUS_READ_LEN    8U
#define STS_MAX_PACKET_LEN     32U
#define STS_RX_TOTAL_TIMEOUT_MS 35U

typedef struct {
    UART_HandleTypeDef *uart;
    uint8_t id;
    MiniStatusLedId_t led;
    HAL_StatusTypeDef last_status;
    uint32_t last_tx_ms;
    uint32_t last_rx_ms;
} MiniServoBus_t;

static MiniServoBus_t buses[2];
static uint8_t last_tx_packet[STS_MAX_PACKET_LEN];
static uint8_t last_tx_len;
static uint8_t last_rx_packet[STS_MAX_PACKET_LEN];
static uint8_t last_rx_len;

static uint8_t sts_checksum(const uint8_t *packet, uint8_t length)
{
    uint16_t sum = 0U;

    for (uint8_t i = 2U; i < (uint8_t)(length - 1U); ++i) {
        sum += packet[i];
    }
    return (uint8_t)(~sum);
}

static void put_u16_le(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)value;
    dst[1] = (uint8_t)(value >> 8U);
}

static uint16_t get_u16_le(const uint8_t *src)
{
    return (uint16_t)((uint16_t)src[0] | ((uint16_t)src[1] << 8U));
}

static MiniServoBus_t *bus_from_id(uint8_t id)
{
    if (id == buses[0].id) {
        return &buses[0];
    }
    if (id == buses[1].id) {
        return &buses[1];
    }
    return 0;
}

static MiniServoBus_t *bus_from_side(uint8_t side)
{
    if (side == 0U) {
        return &buses[0];
    }
    if (side == 1U) {
        return &buses[1];
    }
    return 0;
}

static uint8_t elapsed_ms(uint32_t start, uint32_t timeout_ms)
{
    return ((HAL_GetTick() - start) >= timeout_ms) ? 1U : 0U;
}

static void save_last_rx(const uint8_t *packet, uint8_t length)
{
    if (length > STS_MAX_PACKET_LEN) {
        length = STS_MAX_PACKET_LEN;
    }
    memcpy(last_rx_packet, packet, length);
    last_rx_len = length;
}

static void uart_clear_errors(UART_HandleTypeDef *huart)
{
    uint32_t isr;

    if (huart == 0 || huart->Instance == 0) {
        return;
    }

    isr = READ_REG(huart->Instance->ISR);
    if ((isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)) != 0U) {
        WRITE_REG(huart->Instance->ICR,
                  USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF);
        huart->ErrorCode = HAL_UART_ERROR_NONE;
    }
}

static void uart_drain_rx_ready(UART_HandleTypeDef *huart)
{
    uint32_t guard = 64U;

    if (huart == 0 || huart->Instance == 0) {
        return;
    }

    uart_clear_errors(huart);
    while (((READ_REG(huart->Instance->ISR) & USART_ISR_RXNE_RXFNE) != 0U) && guard > 0U) {
        (void)READ_REG(huart->Instance->RDR);
        --guard;
        uart_clear_errors(huart);
    }
}

static HAL_StatusTypeDef rx_byte(MiniServoBus_t *bus, uint8_t *byte, uint32_t start)
{
    uint32_t isr;

    while (elapsed_ms(start, STS_RX_TOTAL_TIMEOUT_MS) == 0U) {
        isr = READ_REG(bus->uart->Instance->ISR);
        if ((isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)) != 0U) {
            uart_clear_errors(bus->uart);
            continue;
        }
        if ((isr & USART_ISR_RXNE_RXFNE) != 0U) {
            *byte = (uint8_t)READ_REG(bus->uart->Instance->RDR);
            return HAL_OK;
        }
    }
    return HAL_TIMEOUT;
}

static HAL_StatusTypeDef receive_status(MiniServoBus_t *bus,
                                        uint8_t expected_id,
                                        uint8_t expected_param_len,
                                        uint8_t *error_out)
{
    uint8_t packet[STS_MAX_PACKET_LEN];
    uint8_t byte;
    uint8_t state = 0U;
    uint8_t length;
    uint8_t total;
    uint32_t start;

    if (bus == 0 || bus->uart == 0) {
        return HAL_ERROR;
    }

    memset(last_rx_packet, 0, sizeof(last_rx_packet));
    last_rx_len = 0U;
    start = HAL_GetTick();

    while (elapsed_ms(start, STS_RX_TOTAL_TIMEOUT_MS) == 0U) {
        if (rx_byte(bus, &byte, start) != HAL_OK) {
            break;
        }

        if (state == 0U) {
            if (byte == 0xFFU) {
                packet[0] = byte;
                state = 1U;
            }
            continue;
        }
        if (state == 1U) {
            if (byte == 0xFFU) {
                packet[1] = byte;
                state = 2U;
            } else {
                state = 0U;
            }
            continue;
        }

        packet[2] = byte;
        if (rx_byte(bus, &packet[3], start) != HAL_OK) {
            break;
        }

        length = packet[3];
        total = (uint8_t)(length + 4U);
        if (length < 2U || total > STS_MAX_PACKET_LEN) {
            state = 0U;
            continue;
        }

        for (uint8_t i = 4U; i < total; ++i) {
            if (rx_byte(bus, &packet[i], start) != HAL_OK) {
                return HAL_TIMEOUT;
            }
        }

        if (sts_checksum(packet, total) != packet[total - 1U]) {
            save_last_rx(packet, total);
            state = 0U;
            continue;
        }

        save_last_rx(packet, total);
        if (packet[2] != expected_id) {
            state = 0U;
            continue;
        }
        if (expected_param_len != 0xFFU &&
            length != (uint8_t)(expected_param_len + 2U)) {
            state = 0U;
            continue;
        }
        if (expected_param_len == 0U && packet[4] == STS_INST_PING) {
            state = 0U;
            continue;
        }

        bus->last_status = (packet[4] == 0U) ? HAL_OK : HAL_ERROR;
        bus->last_rx_ms = HAL_GetTick();
        if (error_out != 0) {
            *error_out = packet[4];
        }
        MiniStatusLed_Pulse(bus->led);
        return bus->last_status;
    }

    bus->last_status = HAL_TIMEOUT;
    return HAL_TIMEOUT;
}

static HAL_StatusTypeDef bus_transmit(MiniServoBus_t *bus, const uint8_t *packet, uint8_t length)
{
    HAL_StatusTypeDef status;

    if (bus == 0 || bus->uart == 0 || packet == 0 || length > STS_MAX_PACKET_LEN) {
        return HAL_ERROR;
    }

    (void)HAL_UART_AbortReceive(bus->uart);
    uart_drain_rx_ready(bus->uart);
    memcpy(last_tx_packet, packet, length);
    last_tx_len = length;
    status = HAL_UART_Transmit(bus->uart, (uint8_t *)packet, length, 20U);
    bus->last_status = status;
    if (status == HAL_OK) {
        bus->last_tx_ms = HAL_GetTick();
        MiniStatusLed_Pulse(bus->led);
    }
    return status;
}

static uint8_t collect_raw_rx(MiniServoBus_t *bus,
                              uint8_t *out,
                              uint8_t max_len,
                              uint32_t timeout_ms,
                              uint32_t *isr_after,
                              uint32_t *error_flags)
{
    uint32_t start = HAL_GetTick();
    uint8_t length = 0U;

    if (isr_after != 0) {
        *isr_after = 0U;
    }
    if (error_flags != 0) {
        *error_flags = 0U;
    }
    if (bus == 0 || bus->uart == 0 || bus->uart->Instance == 0 || out == 0) {
        return 0U;
    }

    while (elapsed_ms(start, timeout_ms) == 0U) {
        uint32_t isr = READ_REG(bus->uart->Instance->ISR);
        uint32_t errors = isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE);

        if (errors != 0U) {
            if (error_flags != 0) {
                *error_flags |= errors;
            }
            if ((isr & USART_ISR_RXNE_RXFNE) != 0U && length < max_len) {
                out[length++] = (uint8_t)READ_REG(bus->uart->Instance->RDR);
            }
            uart_clear_errors(bus->uart);
            continue;
        }

        if ((isr & USART_ISR_RXNE_RXFNE) != 0U) {
            uint8_t byte = (uint8_t)READ_REG(bus->uart->Instance->RDR);
            if (length < max_len) {
                out[length++] = byte;
            }
        }
    }

    if (isr_after != 0) {
        *isr_after = READ_REG(bus->uart->Instance->ISR);
    }
    return length;
}

static HAL_StatusTypeDef send_instruction(MiniServoBus_t *bus,
                                          uint8_t instruction,
                                          const uint8_t *params,
                                          uint8_t param_length,
                                          uint8_t response_param_len,
                                          uint8_t expect_response)
{
    uint8_t packet[STS_MAX_PACKET_LEN];
    uint8_t total = (uint8_t)(param_length + 6U);

    if (bus == 0 || total > sizeof(packet)) {
        return HAL_ERROR;
    }

    packet[0] = 0xFFU;
    packet[1] = 0xFFU;
    packet[2] = bus->id;
    packet[3] = (uint8_t)(param_length + 2U);
    packet[4] = instruction;
    if (param_length > 0U) {
        memcpy(&packet[5], params, param_length);
    }
    packet[total - 1U] = sts_checksum(packet, total);

    if (bus_transmit(bus, packet, total) != HAL_OK) {
        return HAL_ERROR;
    }
    return expect_response ? receive_status(bus, bus->id, response_param_len, 0) : HAL_OK;
}

static HAL_StatusTypeDef write_u8(MiniServoBus_t *bus, uint8_t address, uint8_t value, uint8_t expect_ack)
{
    const uint8_t params[2] = {address, value};
    return send_instruction(bus, STS_INST_WRITE, params, sizeof(params), 0U, expect_ack);
}

static HAL_StatusTypeDef write_position(MiniServoBus_t *bus,
                                        uint16_t position,
                                        uint16_t time_ms,
                                        uint16_t speed)
{
    uint8_t params[8];

    if (position > MINI_SERVO_POSITION_MAX) {
        position = MINI_SERVO_POSITION_MAX;
    }
    params[0] = STS_REG_ACC;
    params[1] = MINI_SERVO_MOVE_ACC;
    put_u16_le(&params[2], position);
    put_u16_le(&params[4], time_ms);
    put_u16_le(&params[6], speed);
    return send_instruction(bus, STS_INST_WRITE, params, sizeof(params), 0U, 1U);
}

void MiniServo_Init(UART_HandleTypeDef *left_uart, UART_HandleTypeDef *right_uart)
{
    memset(buses, 0, sizeof(buses));
    buses[0].uart = left_uart;
    buses[0].id = MINI_SERVO_LEFT_ID;
    buses[0].led = MINI_STATUS_LED_USART10;
    buses[0].last_status = HAL_ERROR;
    buses[1].uart = right_uart;
    buses[1].id = MINI_SERVO_RIGHT_ID;
    buses[1].led = MINI_STATUS_LED_USART1;
    buses[1].last_status = HAL_ERROR;
    last_tx_len = 0U;
    last_rx_len = 0U;
}

HAL_StatusTypeDef MiniServo_SetBaud(uint32_t baud)
{
    for (uint8_t i = 0U; i < 2U; ++i) {
        if (buses[i].uart == 0 || HAL_UART_DeInit(buses[i].uart) != HAL_OK) {
            return HAL_ERROR;
        }
        buses[i].uart->Init.BaudRate = baud;
        if (HAL_UART_Init(buses[i].uart) != HAL_OK) {
            return HAL_ERROR;
        }
    }
    return HAL_OK;
}

HAL_StatusTypeDef MiniServo_Ping(uint8_t id)
{
    MiniServoBus_t *bus = bus_from_id(id);
    return send_instruction(bus, STS_INST_PING, 0, 0U, 0U, 1U);
}

HAL_StatusTypeDef MiniServo_PingSide(uint8_t side)
{
    MiniServoBus_t *bus = bus_from_side(side);
    return send_instruction(bus, STS_INST_PING, 0, 0U, 0U, 1U);
}

HAL_StatusTypeDef MiniServo_TorqueEnable(uint8_t id, uint8_t enable)
{
    if (id == 0xFEU) {
        HAL_StatusTypeDef left = write_u8(&buses[0], STS_REG_TORQUE_ENABLE, enable ? 1U : 0U, 1U);
        HAL_StatusTypeDef right = write_u8(&buses[1], STS_REG_TORQUE_ENABLE, enable ? 1U : 0U, 1U);
        return (left == HAL_OK && right == HAL_OK) ? HAL_OK : HAL_ERROR;
    }
    return write_u8(bus_from_id(id), STS_REG_TORQUE_ENABLE, enable ? 1U : 0U, 1U);
}

HAL_StatusTypeDef MiniServo_TorqueEnableSide(uint8_t side, uint8_t enable)
{
    if (side == 0xFEU) {
        HAL_StatusTypeDef left = write_u8(&buses[0], STS_REG_TORQUE_ENABLE, enable ? 1U : 0U, 1U);
        HAL_StatusTypeDef right = write_u8(&buses[1], STS_REG_TORQUE_ENABLE, enable ? 1U : 0U, 1U);
        return (left == HAL_OK && right == HAL_OK) ? HAL_OK : HAL_ERROR;
    }
    return write_u8(bus_from_side(side), STS_REG_TORQUE_ENABLE, enable ? 1U : 0U, 1U);
}

HAL_StatusTypeDef MiniServo_ReadStatus(uint8_t id, MiniServoStatus_t *status)
{
    MiniServoBus_t *bus = bus_from_id(id);
    uint8_t params[2] = {STS_REG_PRESENT_POS, STS_STATUS_READ_LEN};
    uint8_t packet[STS_MAX_PACKET_LEN];
    uint8_t length;

    if (bus == 0 || status == 0) {
        return HAL_ERROR;
    }
    memset(status, 0, sizeof(*status));
    if (send_instruction(bus, STS_INST_READ, params, sizeof(params), 0U, 0U) != HAL_OK) {
        return HAL_ERROR;
    }
    if (receive_status(bus, id, STS_STATUS_READ_LEN, &status->error) != HAL_OK) {
        return HAL_TIMEOUT;
    }

    length = MiniServo_GetLastRx(packet, sizeof(packet));
    if (length != (uint8_t)(STS_STATUS_READ_LEN + 6U)) {
        return HAL_ERROR;
    }

    status->id = packet[2];
    status->error = packet[4];
    status->position = get_u16_le(&packet[5]);
    status->speed = get_u16_le(&packet[7]);
    status->load = get_u16_le(&packet[9]);
    status->voltage_0v1 = packet[11];
    status->temperature_c = packet[12];
    return (status->error == 0U) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef MiniServo_ReadStatusSide(uint8_t side, MiniServoStatus_t *status)
{
    MiniServoBus_t *bus = bus_from_side(side);
    uint8_t params[2] = {STS_REG_PRESENT_POS, STS_STATUS_READ_LEN};
    uint8_t packet[STS_MAX_PACKET_LEN];
    uint8_t length;

    if (bus == 0 || status == 0) {
        return HAL_ERROR;
    }
    memset(status, 0, sizeof(*status));
    if (send_instruction(bus, STS_INST_READ, params, sizeof(params), 0U, 0U) != HAL_OK) {
        return HAL_ERROR;
    }
    if (receive_status(bus, bus->id, STS_STATUS_READ_LEN, &status->error) != HAL_OK) {
        return HAL_TIMEOUT;
    }

    length = MiniServo_GetLastRx(packet, sizeof(packet));
    if (length != (uint8_t)(STS_STATUS_READ_LEN + 6U)) {
        return HAL_ERROR;
    }

    status->id = packet[2];
    status->error = packet[4];
    status->position = get_u16_le(&packet[5]);
    status->speed = get_u16_le(&packet[7]);
    status->load = get_u16_le(&packet[9]);
    status->voltage_0v1 = packet[11];
    status->temperature_c = packet[12];
    return (status->error == 0U) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef MiniServo_WritePosition(uint8_t id, uint16_t position, uint16_t time_ms, uint16_t speed)
{
    return write_position(bus_from_id(id), position, time_ms, speed);
}

HAL_StatusTypeDef MiniServo_WritePositionSide(uint8_t side, uint16_t position, uint16_t time_ms, uint16_t speed)
{
    return write_position(bus_from_side(side), position, time_ms, speed);
}

HAL_StatusTypeDef MiniServo_DebugProbeSide(uint8_t side, MiniServoDebug_t *debug)
{
    MiniServoBus_t *bus = bus_from_side(side);
    uint8_t raw[STS_MAX_PACKET_LEN];
    HAL_StatusTypeDef tx_status;

    if (bus == 0 || debug == 0) {
        return HAL_ERROR;
    }

    memset(debug, 0, sizeof(*debug));
    memset(last_rx_packet, 0, sizeof(last_rx_packet));
    last_rx_len = 0U;

    debug->isr_before = READ_REG(bus->uart->Instance->ISR);
    tx_status = send_instruction(bus, STS_INST_PING, 0, 0U, 0U, 0U);
    debug->tx_status = tx_status;
    if (tx_status != HAL_OK) {
        debug->isr_after = READ_REG(bus->uart->Instance->ISR);
        return tx_status;
    }

    debug->length = collect_raw_rx(bus,
                                   raw,
                                   sizeof(raw),
                                   STS_RX_TOTAL_TIMEOUT_MS,
                                   &debug->isr_after,
                                   &debug->error_flags);
    save_last_rx(raw, debug->length);
    return (debug->length > 0U) ? HAL_OK : HAL_TIMEOUT;
}

HAL_StatusTypeDef MiniServo_WritePair(uint16_t left_pos, uint16_t right_pos, uint16_t speed)
{
    HAL_StatusTypeDef left = write_position(&buses[0], left_pos, MINI_SERVO_MOVE_TIME_MS, speed);
    HAL_StatusTypeDef right = write_position(&buses[1], right_pos, MINI_SERVO_MOVE_TIME_MS, speed);
    return (left == HAL_OK && right == HAL_OK) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef MiniServo_ShutdownPose(void)
{
    return MiniServo_WritePair(MINI_SERVO_SHUTDOWN_POS,
                               MINI_SERVO_SHUTDOWN_POS,
                               MINI_SERVO_MOVE_SPEED);
}

uint8_t MiniServo_GetLastTx(uint8_t *out, uint8_t max_len)
{
    if (out == 0 || max_len < last_tx_len) {
        return 0U;
    }
    memcpy(out, last_tx_packet, last_tx_len);
    return last_tx_len;
}

uint8_t MiniServo_GetLastRx(uint8_t *out, uint8_t max_len)
{
    if (out == 0 || max_len < last_rx_len) {
        return 0U;
    }
    memcpy(out, last_rx_packet, last_rx_len);
    return last_rx_len;
}

void MiniServo_Poll(uint32_t now_ms)
{
    (void)now_ms;
}

uint8_t MiniServo_IsOnline(void)
{
    const uint32_t now = HAL_GetTick();

    for (uint8_t i = 0U; i < 2U; ++i) {
        if (buses[i].last_rx_ms == 0U ||
            buses[i].last_status != HAL_OK ||
            (now - buses[i].last_rx_ms) > MINI_SERVO_ONLINE_TIMEOUT_MS) {
            return 0U;
        }
    }
    return 1U;
}
