#include "DJI_Motor.hpp"

// 电流缓存（静态全局，仅在内部使用）
static uint8_t GM6020_Current[8];
static uint8_t M3508_Current[8];

// 全局电机对象定义
DJIMotor YAW_Motor;
DJIMotor PITCH_Motor;
DJIMotor LEFT_FRICTION_Motor;
DJIMotor RIGHT_FRICTION_Motor;

void DJIMotor::GetInfo(uint8_t *rx_buff)
{
    last_encoder = encoder;

    encoder = (rx_buff[0] << 8) | rx_buff[1];
    speed = (rx_buff[2] << 8) | rx_buff[3];
    current = (rx_buff[4] << 8) | rx_buff[5];
    temperature = rx_buff[6];

    if (encoder - last_encoder < -4096)
        circle_number++;
    else if (encoder - last_encoder > 4096)
        circle_number--;

    accumulate_angle = (float)(circle_number * 8192 + encoder) / 8192.0f * 360.0f;
}

void DJIMotor::SetCurrent(MotorModel model, uint8_t id, int16_t current)
{
    if (model == MotorModel::GM6020)
    {
        GM6020_Current[(id - 1) * 2] = current >> 8;
        GM6020_Current[(id - 1) * 2 + 1] = current & 0xFF;
    }
    else if (model == MotorModel::M3508)
    {
        M3508_Current[(id - 1) * 2] = current >> 8;
        M3508_Current[(id - 1) * 2 + 1] = current & 0xFF;
    }
}

void DJIMotor::StartMotor(MotorModel model, CAN_HandleTypeDef *phcan)
{
    if (model == MotorModel::GM6020)
    {
        CANSend(phcan, GM6020_TX_ID, GM6020_Current, 8);
    }
    else if (model == MotorModel::M3508)
    {
        CANSend(phcan, M3508_TX_ID, M3508_Current, 8);
    }
}