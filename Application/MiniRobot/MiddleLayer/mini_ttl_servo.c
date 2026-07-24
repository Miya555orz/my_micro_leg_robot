#include "mini_ttl_servo.h"

#include "mini_robot_config.h"
#include "mini_status_led.h"
#include <string.h>

#define STS_INST_PING          0x01U
#define STS_INST_READ          0x02U
#define STS_INST_WRITE         0x03U
#define STS_REG_TORQUE_ENABLE  40U
#define STS_REG_GOAL_POS       42U
#define STS_REG_PRESENT_POS    56U
#define STS_STATUS_READ_LEN    8U
#define STS_MAX_PACKET_LEN     32U
#define STS_RX_TIMEOUT_MS      30U

typedef struct {
    UART_HandleTypeDef *uart;
    uint8_t id;
    MiniStatusLedId_t led;
    HAL_StatusTypeDef last_status;
    uint32_t last_success_ms;
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
    uint8_t i;

    for (i = 2U; i < (uint8_t)(length - 1U); ++i) {
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

static HAL_StatusTypeDef bus_transmit(MiniServoBus_t *bus, const uint8_t *packet, uint8_t length)
{
    HAL_StatusTypeDef status;

    if (bus == 0 || bus->uart == 0 || packet == 0 || length > STS_MAX_PACKET_LEN) {
        return HAL_ERROR;
    }

    memcpy(last_tx_packet, packet, length);
    last_tx_len = length;
    status = HAL_UART_Transmit(bus->uart, (uint8_t *)packet, length, 20U);
    bus->last_status = status;
    if (status == HAL_OK) {
        bus->last_success_ms = HAL_GetTick();
        MiniStatusLed_Pulse(bus->led);
    }
    return status;
}

static HAL_StatusTypeDef bus_receive(MiniServoBus_t *bus, uint8_t *packet, uint8_t length)
{
    HAL_StatusTypeDef status;

    if (bus == 0 || bus->uart == 0 || packet == 0 || length > STS_MAX_PACKET_LEN) {
        return HAL_ERROR;
    }

    memset(last_rx_packet, 0, sizeof(last_rx_packet));
    last_rx_len = 0U;
    status = HAL_UART_Receive(bus->uart, packet, length, STS_RX_TIMEOUT_MS);
    if (status == HAL_OK) {
        memcpy(last_rx_packet, packet, length);
        last_rx_len = length;
        bus->last_rx_ms = HAL_GetTick();
        bus->last_success_ms = bus->last_rx_ms;
        bus->last_status = HAL_OK;
        MiniStatusLed_Pulse(bus->led);
    }
    return status;
}

static HAL_StatusTypeDef receive_ack(MiniServoBus_t *bus)
{
    uint8_t response[6];
    HAL_StatusTypeDef status = bus_receive(bus, response, sizeof(response));

    if (status != HAL_OK) {
        return status;
    }
    if (response[0] != 0xFFU || response[1] != 0xFFU ||
        response[2] != bus->id || response[3] != 0x02U ||
        sts_checksum(response, sizeof(response)) != response[5]) {
        return HAL_ERROR;
    }
    return (response[4] == 0U) ? HAL_OK : HAL_ERROR;
}

static HAL_StatusTypeDef send_instruction(MiniServoBus_t *bus,
                                          uint8_t instruction,
                                          const uint8_t *params,
                                          uint8_t param_length,
                                          uint8_t expect_ack)
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
    return expect_ack ? receive_ack(bus) : HAL_OK;
}

static HAL_StatusTypeDef write_u8(MiniServoBus_t *bus, uint8_t address, uint8_t value, uint8_t expect_ack)
{
    const uint8_t params[2] = {address, value};
    return send_instruction(bus, STS_INST_WRITE, params, sizeof(params), expect_ack);
}

static HAL_StatusTypeDef write_position(MiniServoBus_t *bus,
                                        uint16_t position,
                                        uint16_t time_ms,
                                        uint16_t speed)
{
    uint8_t params[7];

    if (position > MINI_SERVO_POSITION_MAX) {
        position = MINI_SERVO_POSITION_MAX;
    }
    params[0] = STS_REG_GOAL_POS;
    put_u16_le(&params[1], position);
    put_u16_le(&params[3], time_ms);
    put_u16_le(&params[5], speed);
    /* Position writes are fire-and-forget; ping/read are used when a real
       response is required. This keeps the control task free of UART timeouts. */
    return send_instruction(bus, STS_INST_WRITE, params, sizeof(params), 0U);
}

void MiniServo_Init(UART_HandleTypeDef *left_uart, UART_HandleTypeDef *right_uart)
{
    memset(buses, 0, sizeof(buses));
    buses[0].uart = left_uart;
    buses[0].id = MINI_SERVO_LEFT_ID;
    buses[0].led = MINI_STATUS_LED_USART1;
    buses[0].last_status = HAL_ERROR;
    buses[1].uart = right_uart;
    buses[1].id = MINI_SERVO_RIGHT_ID;
    buses[1].led = MINI_STATUS_LED_USART10;
    buses[1].last_status = HAL_ERROR;
    last_tx_len = 0U;
    last_rx_len = 0U;
}

HAL_StatusTypeDef MiniServo_SetBaud(uint32_t baud)
{
    uint8_t i;

    for (i = 0U; i < 2U; ++i) {
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
    return send_instruction(bus, STS_INST_PING, 0, 0U, 1U);
}

HAL_StatusTypeDef MiniServo_TorqueEnable(uint8_t id, uint8_t enable)
{
    if (id == 0xFEU) {
        HAL_StatusTypeDef left = write_u8(&buses[0], STS_REG_TORQUE_ENABLE, enable ? 1U : 0U, 0U);
        HAL_StatusTypeDef right = write_u8(&buses[1], STS_REG_TORQUE_ENABLE, enable ? 1U : 0U, 0U);
        return (left == HAL_OK && right == HAL_OK) ? HAL_OK : HAL_ERROR;
    }
    return write_u8(bus_from_id(id), STS_REG_TORQUE_ENABLE, enable ? 1U : 0U, 0U);
}

HAL_StatusTypeDef MiniServo_ReadStatus(uint8_t id, MiniServoStatus_t *status)
{
    MiniServoBus_t *bus = bus_from_id(id);
    uint8_t params[2] = {STS_REG_PRESENT_POS, STS_STATUS_READ_LEN};
    uint8_t response[STS_STATUS_READ_LEN + 6U];

    if (bus == 0 || status == 0) {
        return HAL_ERROR;
    }
    memset(status, 0, sizeof(*status));
    if (send_instruction(bus, STS_INST_READ, params, sizeof(params), 0U) != HAL_OK) {
        return HAL_ERROR;
    }
    if (bus_receive(bus, response, sizeof(response)) != HAL_OK) {
        return HAL_TIMEOUT;
    }
    if (response[0] != 0xFFU || response[1] != 0xFFU ||
        response[2] != id || response[3] != (STS_STATUS_READ_LEN + 2U) ||
        sts_checksum(response, sizeof(response)) != response[sizeof(response) - 1U]) {
        return HAL_ERROR;
    }

    status->id = response[2];
    status->error = response[4];
    status->position = get_u16_le(&response[5]);
    status->speed = get_u16_le(&response[7]);
    status->load = get_u16_le(&response[9]);
    status->voltage_0v1 = response[11];
    status->temperature_c = response[12];
    return (status->error == 0U) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef MiniServo_WritePosition(uint8_t id, uint16_t position, uint16_t time_ms, uint16_t speed)
{
    return write_position(bus_from_id(id), position, time_ms, speed);
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
    uint8_t i;

    for (i = 0U; i < 2U; ++i) {
        if (buses[i].last_success_ms == 0U ||
            buses[i].last_status != HAL_OK ||
            (now - buses[i].last_success_ms) > MINI_SERVO_ONLINE_TIMEOUT_MS) {
            return 0U;
        }
    }
    return 1U;
}
