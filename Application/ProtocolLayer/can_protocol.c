#include "can_protocol.h"
#include "Chassis_Motor.h"
#include "cap.h"
#include "cap_protocol.h"
#include "gimbal_Motor.h"

/**
 *  @brief  CAN1 接收数据
 */
void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
    switch (rxId)
    {
    case RXID_R_F_Sd_M:
        Sd_Group.motor[R_F_Sd_M]->rx(Sd_Group.motor[R_F_Sd_M], rxBuf);
        break;
    case RXID_R_B_Sd_M:
        Sd_Group.motor[R_B_Sd_M]->rx(Sd_Group.motor[R_B_Sd_M], rxBuf);
        break;
    case R_Wheel_RX_ID:
        Wheel_Group.motor[R_WHEEL_M]->rx(Wheel_Group.motor[R_WHEEL_M], rxBuf);
        break;
    case DAIL_MOTOR_ID:
        dail_motor.get_info(&dail_motor, rxBuf);
        break;
    default:
        break;
    }
}

/**
 *  @brief  CAN2 接收数据
 */
void CAN2_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
    switch (rxId)
    {
    case L_Wheel_RX_ID:
        Wheel_Group.motor[L_WHEEL_M]->rx(Wheel_Group.motor[L_WHEEL_M], rxBuf);
        break;
    case YAW_RX_ID:
        Yaw_Motor.rx(&Yaw_Motor, rxBuf); // 确定
        break;
    case RXID_L_F_Sd_M:
        Sd_Group.motor[L_F_Sd_M]->rx(Sd_Group.motor[L_F_Sd_M], rxBuf);
        break;
    case RXID_L_B_Sd_M:
        Sd_Group.motor[L_B_Sd_M]->rx(Sd_Group.motor[L_B_Sd_M], rxBuf);
        break;
    default:
        break;
    }
}

/**
 *  @brief  CAN3 接收数据
 */
void CAN3_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
    switch (rxId)
    {
    case CAP_TO_MASTER_ID:
        cap.update(&cap, rxBuf, CAP_TO_MASTER_ID);
        break;
    case WIRELESS_ID:
        cap.update(&cap, rxBuf, WIRELESS_ID);
        break;
    case 0xC1:
        Board_Rx_C1(rxBuf);
        break;

    case 0xC2:
        Board_Rx_C2(rxBuf);
        break;

    case 0xC3:
        Board_Rx_C3(rxBuf);
        break;

    case 0xC4:
        Board_Rx_C4(rxBuf);
        break;
        
    default:
        break;
    }
}