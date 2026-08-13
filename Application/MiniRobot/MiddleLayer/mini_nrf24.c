/**
 ******************************************************************************
 * @file    mini_nrf24.c
 * @brief   nRF24L01 SPI driver for wireless command packet reception.
 * @author  Miya Zheng
 * @date    2026-07-29
 ******************************************************************************
 */
#include "mini_nrf24.h"

#include "main.h"
#include "mini_robot_config.h"
#include <string.h>

#define NRF_CMD_R_REGISTER    0x00U
#define NRF_CMD_W_REGISTER    0x20U
#define NRF_CMD_R_RX_PAYLOAD  0x61U
#define NRF_CMD_FLUSH_RX      0xE2U
#define NRF_CMD_NOP           0xFFU

#define NRF_REG_CONFIG        0x00U
#define NRF_REG_EN_AA         0x01U
#define NRF_REG_EN_RXADDR     0x02U
#define NRF_REG_SETUP_AW      0x03U
#define NRF_REG_SETUP_RETR    0x04U
#define NRF_REG_RF_CH         0x05U
#define NRF_REG_RF_SETUP      0x06U
#define NRF_REG_STATUS        0x07U
#define NRF_REG_RX_ADDR_P0    0x0AU
#define NRF_REG_TX_ADDR       0x10U
#define NRF_REG_RX_PW_P0      0x11U
#define NRF_REG_FIFO_STATUS   0x17U
#define NRF_REG_DYNPD         0x1CU
#define NRF_REG_FEATURE       0x1DU

#define NRF_STATUS_RX_DR      0x40U
#define NRF_STATUS_IRQ_MASK   0x70U
#define NRF_FIFO_RX_EMPTY     0x01U

static SPI_HandleTypeDef *nrf_spi;
static uint8_t nrf_online;
static const uint8_t nrf_address[5] = MINI_NRF24_ADDRESS;

static uint8_t spi_byte(uint8_t tx)
{
    uint8_t rx = 0U;
    if (HAL_SPI_TransmitReceive(nrf_spi, &tx, &rx, 1U, 10U) != HAL_OK)
    {
        nrf_online = 0U;
    }
    return rx;
}

static void csn(GPIO_PinState state)
{
    HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, state);
}

static uint8_t read_reg(uint8_t reg)
{
    uint8_t value;
    csn(GPIO_PIN_RESET);
    (void)spi_byte((uint8_t)(NRF_CMD_R_REGISTER | (reg & 0x1FU)));
    value = spi_byte(NRF_CMD_NOP);
    csn(GPIO_PIN_SET);
    return value;
}

static void write_reg(uint8_t reg, uint8_t value)
{
    csn(GPIO_PIN_RESET);
    (void)spi_byte((uint8_t)(NRF_CMD_W_REGISTER | (reg & 0x1FU)));
    (void)spi_byte(value);
    csn(GPIO_PIN_SET);
}

static void read_buffer(uint8_t command, uint8_t *data, uint8_t length)
{
    uint8_t i;
    csn(GPIO_PIN_RESET);
    (void)spi_byte(command);
    for (i = 0U; i < length; ++i)
    {
        data[i] = spi_byte(NRF_CMD_NOP);
    }
    csn(GPIO_PIN_SET);
}

static void write_buffer(uint8_t reg, const uint8_t *data, uint8_t length)
{
    uint8_t i;
    csn(GPIO_PIN_RESET);
    (void)spi_byte((uint8_t)(NRF_CMD_W_REGISTER | (reg & 0x1FU)));
    for (i = 0U; i < length; ++i)
    {
        (void)spi_byte(data[i]);
    }
    csn(GPIO_PIN_SET);
}

static void command(uint8_t cmd)
{
    csn(GPIO_PIN_RESET);
    (void)spi_byte(cmd);
    csn(GPIO_PIN_SET);
}

static uint8_t check_module(void)
{
    const uint8_t test[5] = {0xA5U, 0x5AU, 0xC3U, 0x3CU, 0x69U};
    uint8_t readback[5];

    write_buffer(NRF_REG_TX_ADDR, test, sizeof(test));
    read_buffer(NRF_REG_TX_ADDR, readback, sizeof(readback));
    write_buffer(NRF_REG_TX_ADDR, nrf_address, sizeof(nrf_address));
    return (memcmp(test, readback, sizeof(test)) == 0) ? 1U : 0U;
}

HAL_StatusTypeDef MiniNrf24_Init(SPI_HandleTypeDef *hspi)
{
    if (hspi == 0)
    {
        return HAL_ERROR;
    }

    nrf_spi = hspi;
    nrf_online = 1U;
    HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);
    HAL_Delay(5U);

    if (!check_module())
    {
        nrf_online = 0U;
        return HAL_ERROR;
    }

    write_reg(NRF_REG_CONFIG, 0x0FU);
    write_reg(NRF_REG_EN_AA, 0x01U);
    write_reg(NRF_REG_EN_RXADDR, 0x01U);
    write_reg(NRF_REG_SETUP_AW, 0x03U);
    write_reg(NRF_REG_SETUP_RETR, 0x3FU);
    write_reg(NRF_REG_RF_CH, MINI_NRF24_CHANNEL);
    write_reg(NRF_REG_RF_SETUP, 0x06U);
    write_buffer(NRF_REG_RX_ADDR_P0, nrf_address, sizeof(nrf_address));
    write_buffer(NRF_REG_TX_ADDR, nrf_address, sizeof(nrf_address));
    write_reg(NRF_REG_RX_PW_P0, MINI_NRF24_PAYLOAD_SIZE);
    write_reg(NRF_REG_DYNPD, 0x00U);
    write_reg(NRF_REG_FEATURE, 0x00U);
    write_reg(NRF_REG_STATUS, NRF_STATUS_IRQ_MASK);
    command(NRF_CMD_FLUSH_RX);
    HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_SET);
    HAL_Delay(2U);

    nrf_online = (read_reg(NRF_REG_RF_CH) == MINI_NRF24_CHANNEL &&
                  read_reg(NRF_REG_RX_PW_P0) == MINI_NRF24_PAYLOAD_SIZE) ? 1U : 0U;
    return nrf_online ? HAL_OK : HAL_ERROR;
}

uint8_t MiniNrf24_Poll(uint8_t payload[32])
{
    uint8_t status;

    if (!nrf_online || payload == 0)
    {
        return 0U;
    }
    status = read_reg(NRF_REG_STATUS);
    if ((status & NRF_STATUS_RX_DR) == 0U &&
        (read_reg(NRF_REG_FIFO_STATUS) & NRF_FIFO_RX_EMPTY) != 0U)
    {
        return 0U;
    }

    read_buffer(NRF_CMD_R_RX_PAYLOAD, payload, MINI_NRF24_PAYLOAD_SIZE);
    write_reg(NRF_REG_STATUS, NRF_STATUS_RX_DR);
    return 1U;
}

uint8_t MiniNrf24_IsOnline(void)
{
    return nrf_online;
}

