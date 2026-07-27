#include "drv_can.h"

#include "fdcan.h"
#include <string.h>

static CAN_RxFrameTypeDef can1_rx_frame;
static CAN_RxFrameTypeDef can2_rx_frame;
static CAN_DiagTypeDef can1_diag;
static CAN_DiagTypeDef can2_diag;

__WEAK void CAN1_rxDataHandler(uint32_t rx_id, uint8_t *rx_buf)
{
    (void)rx_id;
    (void)rx_buf;
}

__WEAK void CAN2_rxDataHandler(uint32_t rx_id, uint8_t *rx_buf)
{
    (void)rx_id;
    (void)rx_buf;
}

static void can_filter_init(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef filter = {0};

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x000U;
    filter.FilterID2 = 0x000U;
    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_FDCAN_Start(hfdcan) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U) != HAL_OK) {
        Error_Handler();
    }
}

void CAN1_Filter_Init(void)
{
    can_filter_init(&hfdcan1);
}

void CAN2_Filter_Init(void)
{
    can_filter_init(&hfdcan2);
}

void FDCAN1_Restart(void)
{
    (void)HAL_FDCAN_Stop(&hfdcan1);
    (void)HAL_FDCAN_DeInit(&hfdcan1);
    MX_FDCAN1_Init();
    CAN1_Filter_Init();
}

void FDCAN2_Restart(void)
{
    (void)HAL_FDCAN_Stop(&hfdcan2);
    (void)HAL_FDCAN_DeInit(&hfdcan2);
    MX_FDCAN2_Init();
    CAN2_Filter_Init();
}

static CAN_DiagTypeDef *can_diag_from_handle(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == &hfdcan1) {
        return &can1_diag;
    }
    if (hfdcan == &hfdcan2) {
        return &can2_diag;
    }
    return 0;
}

static void can_abort_pending_tx(FDCAN_HandleTypeDef *hfdcan, CAN_DiagTypeDef *diag)
{
    uint32_t buffer_count;

    if (hfdcan == 0 || diag == 0) {
        return;
    }

    buffer_count = hfdcan->Init.TxFifoQueueElmtsNbr;
    if (buffer_count > 8U) {
        buffer_count = 8U;
    }
    for (uint32_t i = 0U; i < buffer_count; ++i) {
        if (HAL_FDCAN_AbortTxRequest(hfdcan, (FDCAN_TX_BUFFER0 << i)) == HAL_OK) {
            diag->tx_abort_count++;
        }
    }
}

static uint32_t fdcan_dlc_to_bytes(uint32_t dlc)
{
    if (dlc <= FDCAN_DLC_BYTES_8) {
        return dlc;
    }
    return 8U;
}

HAL_StatusTypeDef CAN_SendData(FDCAN_HandleTypeDef *hfdcan, uint32_t std_id, const uint8_t *data)
{
    FDCAN_TxHeaderTypeDef header = {0};
    CAN_DiagTypeDef *diag;
    HAL_StatusTypeDef status;

    if (hfdcan == 0 || data == 0 || std_id > 0x7FFU) {
        return HAL_ERROR;
    }

    diag = can_diag_from_handle(hfdcan);
    if (diag != 0) {
        diag->tx_attempt_count++;
        diag->last_tx_id = std_id;
        diag->last_tx_tick_ms = HAL_GetTick();
        memcpy(diag->last_tx_data, data, sizeof(diag->last_tx_data));
        diag->last_tx_free_level = HAL_FDCAN_GetTxFifoFreeLevel(hfdcan);
        diag->last_error_code = hfdcan->ErrorCode;
        if (diag->last_tx_free_level == 0U) {
            diag->tx_fifo_full_count++;
            can_abort_pending_tx(hfdcan, diag);
            diag->last_tx_free_level = HAL_FDCAN_GetTxFifoFreeLevel(hfdcan);
        }
    }

    header.Identifier = std_id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;
    status = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &header, (uint8_t *)data);
    if (diag != 0) {
        diag->last_tx_status = (uint32_t)status;
        diag->last_error_code = hfdcan->ErrorCode;
        if (status == HAL_OK) {
            diag->tx_ok_count++;
        } else {
            diag->tx_fail_count++;
        }
    }
    return status;
}

const CAN_DiagTypeDef *CAN_GetDiag(FDCAN_HandleTypeDef *hcan)
{
    return can_diag_from_handle(hcan);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t flags)
{
    CAN_RxFrameTypeDef *frame;
    CAN_DiagTypeDef *diag;
    uint32_t rx_len;

    if ((flags & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) {
        return;
    }

    if (hfdcan == &hfdcan1) {
        frame = &can1_rx_frame;
        diag = &can1_diag;
    } else if (hfdcan == &hfdcan2) {
        frame = &can2_rx_frame;
        diag = &can2_diag;
    } else {
        return;
    }

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &frame->header, frame->data) != HAL_OK) {
        diag->rx_fail_count++;
        diag->last_error_code = hfdcan->ErrorCode;
        return;
    }

    rx_len = fdcan_dlc_to_bytes(frame->header.DataLength);
    diag->rx_count++;
    diag->last_rx_id = frame->header.Identifier;
    diag->last_rx_dlc = rx_len;
    diag->last_rx_tick_ms = HAL_GetTick();
    memset(diag->last_rx_data, 0, sizeof(diag->last_rx_data));
    memcpy(diag->last_rx_data, frame->data, rx_len);

    if (hfdcan == &hfdcan1) {
        CAN1_rxDataHandler(frame->header.Identifier, frame->data);
    } else {
        CAN2_rxDataHandler(frame->header.Identifier, frame->data);
    }
}
