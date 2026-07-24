#include "drv_can.h"

#include "fdcan.h"

static CAN_RxFrameTypeDef can1_rx_frame;
static CAN_RxFrameTypeDef can2_rx_frame;

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

HAL_StatusTypeDef CAN_SendData(FDCAN_HandleTypeDef *hfdcan, uint32_t std_id, const uint8_t *data)
{
    FDCAN_TxHeaderTypeDef header = {0};

    if (hfdcan == 0 || data == 0 || std_id > 0x7FFU) {
        return HAL_ERROR;
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
    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &header, (uint8_t *)data);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t flags)
{
    CAN_RxFrameTypeDef *frame;

    if ((flags & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) {
        return;
    }

    if (hfdcan == &hfdcan1) {
        frame = &can1_rx_frame;
    } else if (hfdcan == &hfdcan2) {
        frame = &can2_rx_frame;
    } else {
        return;
    }

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &frame->header, frame->data) != HAL_OK) {
        return;
    }

    if (hfdcan == &hfdcan1) {
        CAN1_rxDataHandler(frame->header.Identifier, frame->data);
    } else {
        CAN2_rxDataHandler(frame->header.Identifier, frame->data);
    }
}
