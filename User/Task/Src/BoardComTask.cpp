#include "BoardComTask.hpp"

void Board_Dr16_Data_Sent()
{
    uint64_t packed = 0; // 用 64 位整数暂存
    uint8_t Tx_data[8];

    // 1. 通道 0~3：各取低 11 位
    packed |= (uint64_t)(dr16_remote.ch0 & 0x7FF) << 0;
    packed |= (uint64_t)(dr16_remote.ch1 & 0x7FF) << 11;
    packed |= (uint64_t)(dr16_remote.ch2 & 0x7FF) << 22;
    packed |= (uint64_t)(dr16_remote.ch3 & 0x7FF) << 33;

    // 2. 开关 S1, S2：各取低 2 位
    packed |= (uint64_t)(dr16_remote.s1 & 0x03) << 44;
    packed |= (uint64_t)(dr16_remote.s2 & 0x03) << 46;

    // 3. 拨轮：取低 16 位（实际有效 11 位，但保留 16 位无妨）
    packed |= (uint64_t)(dr16_remote.wheel & 0xFFFF) << 48;

    // 将 64 位整数按小端顺序写入字节数组
    for (int i = 0; i < 8; i++)
    {
        Tx_data[i] = (packed >> (i * 8)) & 0xFF;
    }
    CANSend(DR16_COM_CAN, DR16_COM_TX_ID, Tx_data, 8);
}

void Board_Yaw_Data_Sent()
{
    uint8_t Tx_data[8];

    memcpy(Tx_data, &YAW_Motor.accumulate_angle, sizeof(float));
    Tx_data[4] = YAW_Motor.speed >> 8;
    Tx_data[5] = YAW_Motor.speed & 0xFF;

    CANSend(YAW_COM_CAN, YAW_COM_TX_ID, Tx_data, 8);
}