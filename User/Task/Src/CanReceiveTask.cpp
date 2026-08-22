#include "CanReceiveTask.hpp"

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) // CAN1
{
    static CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_buff[8];
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_buff);
    switch (rx_header.StdId)
    {
    case YAW_RX_ID:
        YAW_Motor.GetInfo(rx_buff);
        break;
    case FEEDER_RX_ID:
        FEEDER_Motor.GetInfo(rx_buff);
        break;
    case COM_RX_ID:
        break;
    default:
        break;
    }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) // CAN2
{
    static CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_buff[8];
    HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO1, &rx_header, rx_buff);
    switch (rx_header.StdId)
    {
    case PITCH_RX_ID:
        PITCH_Motor.GetInfo(rx_buff);
        break;
    case LEFT_FRICTION_RX_ID:
        LEFT_FRICTION_Motor.GetInfo(rx_buff);
        break;
    case RIGHT_FRICTION_RX_ID:
        RIGHT_FRICTION_Motor.GetInfo(rx_buff);
        break;
    default:
        break;
    }
}