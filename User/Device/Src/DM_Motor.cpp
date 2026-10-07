/**
 ******************************************************************************
 * @file    DM_Motor.cpp
 * @brief   达妙 DM 系列电机驱动实现（MIT 协议编解码 + 多圈解环绕）
 ******************************************************************************
 */
#include "DM_Motor.hpp"

#include "cmsis_os.h"

/* MIT 协议各物理量的编码量程（与电机调试助手配置一致） */
static constexpr float P_MIN = -12.5f;
static constexpr float P_MAX = 12.5f;
static constexpr float V_MIN = -30.0f;
static constexpr float V_MAX = 30.0f;
static constexpr float KP_MIN = 0.0f;
static constexpr float KP_MAX = 500.0f;
static constexpr float KD_MIN = 0.0f;
static constexpr float KD_MAX = 5.0f;
static constexpr float T_MIN = -7.0f;
static constexpr float T_MAX = 7.0f;

static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

static float clampf(float x, float low, float high)
{
    if (x < low)
        return low;
    if (x > high)
        return high;
    return x;
}

/* 使能帧：FF FF FF FF FF FF FF FC */
void DMMotor::Enable(void)
{
    uint8_t tx_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    CANSend(phcan, tx_id, tx_data, 8);
}

/* 失能帧：FF FF FF FF FF FF FF FD */
void DMMotor::Disable(void)
{
    uint8_t tx_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    CANSend(phcan, tx_id, tx_data, 8);
}

/*
 * MIT 控制帧：目标位置/速度/Kp/Kd/前馈力矩按固定位域打包。
 * 入参先钳位到协议量程——超范围值经 float_to_uint 会回绕成
 * 完全错误的命令（比如巨大的位置跳变），宁可饱和也不可回绕。
 */
void DMMotor::MITCmd(float pos, float vel, float KP, float KD, float torq)
{
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    pos_tmp = (uint16_t)float_to_uint(clampf(pos, P_MIN, P_MAX), P_MIN, P_MAX, 16);
    vel_tmp = (uint16_t)float_to_uint(clampf(vel, V_MIN, V_MAX), V_MIN, V_MAX, 12);
    kp_tmp = (uint16_t)float_to_uint(clampf(KP, KP_MIN, KP_MAX), KP_MIN, KP_MAX, 12);
    kd_tmp = (uint16_t)float_to_uint(clampf(KD, KD_MIN, KD_MAX), KD_MIN, KD_MAX, 12);
    tor_tmp = (uint16_t)float_to_uint(clampf(torq, T_MIN, T_MAX), T_MIN, T_MAX, 12);

    uint8_t tx_data[8];
    tx_data[0] = (uint8_t)(pos_tmp >> 8);
    tx_data[1] = (uint8_t)(pos_tmp);
    tx_data[2] = (uint8_t)(vel_tmp >> 4);
    tx_data[3] = (uint8_t)(((vel_tmp & 0xF) << 4) | (kp_tmp >> 8));
    tx_data[4] = (uint8_t)(kp_tmp);
    tx_data[5] = (uint8_t)(kd_tmp >> 4);
    tx_data[6] = (uint8_t)(((kd_tmp & 0xF) << 4) | (tor_tmp >> 8));
    tx_data[7] = (uint8_t)(tor_tmp);

    CANSend(phcan, tx_id, tx_data, 8);
}

/*
 * 反馈帧解码 + 多圈解环绕。
 * pos 量化周期为 (P_MAX - P_MIN) = 25 rad，用单帧增量折叠到
 * ±半周期内累加，任意连续转速下 accumulate_angle 都保持连续。
 */
void DMMotor::DecodeFeedback(uint8_t *rx_buff)
{
    state = (rx_buff[0]) >> 4;
    pos_last = pos;
    pos = uint_to_float((rx_buff[1] << 8) | rx_buff[2], P_MIN, P_MAX, 16);
    vel = uint_to_float((rx_buff[3] << 4) | (rx_buff[4] >> 4), V_MIN, V_MAX, 12);
    tor = uint_to_float(((rx_buff[4] & 0xF) << 8) | rx_buff[5], T_MIN, T_MAX, 12);
    Tmos = (float)rx_buff[6];
    Tcoil = (float)rx_buff[7];

    float delta = pos - pos_last;
    if (delta > 0.5f * (P_MAX - P_MIN))
        delta -= (P_MAX - P_MIN);
    else if (delta < -0.5f * (P_MAX - P_MIN))
        delta += (P_MAX - P_MIN);
    accumulate_angle += delta;
}

/* 全局 DM 电机对象定义（身份登记表，原 define.h 的 FEEDER_* 并入于此）：
 * 拨弹轮，CAN1，控制帧 TX 0x03 / 反馈帧 RX 0x30 */
DMMotor FEEDER_Motor(&hcan1, 0x03U, 0x30U);
