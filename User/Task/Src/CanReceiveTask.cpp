#include "CanReceiveTask.hpp"

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) // CAN1
{
    static CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_buff[8];
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_buff);

    /*
     * CAN1：YAW GM6020 反馈（0x206）+ 拨弹 DM 电机反馈（0x30，按
     * rx_id 匹配）。板间通讯帧 DR16_COM(0x300) / YAW_COM(0x302)
     * 也会到达本总线，接入 BoardCom 时在下方追加分发。
     */
    uint16_t rx_id = rx_header.StdId;
    if (rx_id == YAW_Motor.RxId())
    {
        YAW_Motor.GetInfo(rx_buff);
    }
    else if (rx_id == FEEDER_Motor.rx_id)
    {
        FEEDER_Motor.DecodeFeedback(rx_buff);
    }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) // CAN2
{
    static CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_buff[8];
    HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO1, &rx_header, rx_buff);

    /*
     * 电机反馈帧 ID 由各电机对象自己报（DJI 用 RxId()，DM 用 rx_id
     * 成员），电机身份仍然只在各自的登记表设一遍；这里不做
     * switch/case，因为反馈 ID 是运行期值，不能做 case 标签。
     */
    uint16_t rx_id = rx_header.StdId;
    if (rx_id == PITCH_Motor.RxId())
    {
        PITCH_Motor.GetInfo(rx_buff);
    }
    else if (rx_id == LEFT_FRICTION_Motor.RxId())
    {
        LEFT_FRICTION_Motor.GetInfo(rx_buff);
    }
    else if (rx_id == RIGHT_FRICTION_Motor.RxId())
    {
        RIGHT_FRICTION_Motor.GetInfo(rx_buff);
    }
}
