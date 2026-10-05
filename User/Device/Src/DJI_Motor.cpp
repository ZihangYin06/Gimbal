/**
 ******************************************************************************
 * @file    DJI_Motor.cpp
 * @brief   大疆电机（GM6020 / M3508）驱动实现（CAN1 / CAN2 各 4 台布局）
 ******************************************************************************
 */
#include "DJI_Motor.hpp"

/* 总线数量：与 BusIndexOf() 的映射一一对应 */
#define BUS_NUM 2U

/* 每帧最多 4 台电机，每台占 2 字节（大端）：[id1_H,id1_L,id2_H,id2_L,...] */
static const uint8_t MOTOR_NUM_PER_FRAME = 4U;

/*
 * 电流指令发送缓存：每条总线、每个型号各一份 8 字节帧
 * （0x200 / 0x1FF 各带 4 台；当前布局每总线只有 ID 1~4，仅用 0x200。
 *   将来同一总线挂第 5~8 台电机时，需扩展 0x1FF 帧缓冲）
 */
static uint8_t GM6020_Current[BUS_NUM][8];
static uint8_t M3508_Current[BUS_NUM][8];

/*
 * 总线索引映射：hcan1 -> 0，hcan2 -> 1。
 * 工程若增加新的 CAN 外设需在此登记；未知总线返回 BUS_NUM，
 * 调用方据此忽略本次写入/发送，防止数组越界。
 */
static uint8_t BusIndexOf(CAN_HandleTypeDef *phcan)
{
    if (phcan == &hcan1)
    {
        return 0U;
    }
    if (phcan == &hcan2)
    {
        return 1U;
    }
    return BUS_NUM;
}

// 全局电机对象定义（云台：YAW/PITCH 各一台 GM6020，摩擦轮两台 M3508）
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
    temperature = rx_buff[6]; /* rx_buff[7] 为保留字节 */

    /*
     * 过零换圈：encoder/last_encoder 为 uint16_t，相减提升为 int，
     * 结果范围 -65535~65535。正转跨 8191->0 时差值约 -8191，
     * 反转跨 0->8191 时差值约 +8191，以半圈 4096 为阈值判方向。
     */
    if (encoder - last_encoder < -4096)
        circle_number++;
    else if (encoder - last_encoder > 4096)
        circle_number--;

    accumulate_angle = (float)(circle_number * 8192 + encoder) / 8192.0f * 360.0f;
}

void DJIMotor::SetCurrent(MotorModel model, CAN_HandleTypeDef *phcan, uint8_t id, int16_t cmd_current)
{
    uint8_t bus = BusIndexOf(phcan);
    if (bus >= BUS_NUM || id < 1U || id > MOTOR_NUM_PER_FRAME)
    {
        return; /* 未知总线或 id 超出一帧容量，直接忽略 */
    }

    uint8_t index = (id - 1U) * 2U;

    if (model == MotorModel::GM6020)
    {
        GM6020_Current[bus][index] = cmd_current >> 8;
        GM6020_Current[bus][index + 1] = cmd_current & 0xFF;
    }
    else if (model == MotorModel::M3508)
    {
        M3508_Current[bus][index] = cmd_current >> 8;
        M3508_Current[bus][index + 1] = cmd_current & 0xFF;
    }
}

void DJIMotor::StartMotor(MotorModel model, CAN_HandleTypeDef *phcan)
{
    uint8_t bus = BusIndexOf(phcan);
    if (bus >= BUS_NUM)
    {
        return; /* 未知总线，忽略 */
    }

    /* 只发送 phcan 自己的总线缓存，两条总线互不干扰 */
    if (model == MotorModel::GM6020)
    {
        CANSend(phcan, GM6020_TX_ID, GM6020_Current[bus], 8);
    }
    else if (model == MotorModel::M3508)
    {
        CANSend(phcan, M3508_TX_ID, M3508_Current[bus], 8);
    }
}
