#include "DM_Motor.hpp"

static constexpr float P_MIN = -5.23599f;
static constexpr float P_MAX = 5.23599f;
static constexpr float V_MIN = -30.0f;
static constexpr float V_MAX = 30.0f;
static constexpr float KP_MIN = 0.0f;
static constexpr float KP_MAX = 500.0f;
static constexpr float KD_MIN = 0.0f;
static constexpr float KD_MAX = 5.0f;
static constexpr float T_MIN = -10.0f;
static constexpr float T_MAX = 10.0f;

// 全局拨弹电机对象定义
DMMotor FEEDER_Motor;

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

void DMMotor::Enable(CAN_HandleTypeDef *phcan, uint32_t id)
{
    uint8_t txDATA[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    CANSend(phcan, id, txDATA, 8);
}

void DMMotor::Disable(CAN_HandleTypeDef *phcan, uint32_t id)
{
    uint8_t txDATA[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    CANSend(phcan, id, txDATA, 8);
}

void DMMotor::MITCmd(CAN_HandleTypeDef *phcan, uint32_t id,
                     float pos, float vel, float KP, float KD, float torq)
{
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    pos_tmp = float_to_uint(pos, P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(vel, V_MIN, V_MAX, 12);
    kp_tmp = float_to_uint(KP, KP_MIN, KP_MAX, 12);
    kd_tmp = float_to_uint(KD, KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(torq, T_MIN, T_MAX, 12);

    uint8_t txDATA[8];
    txDATA[0] = (pos_tmp >> 8);
    txDATA[1] = pos_tmp;
    txDATA[2] = (vel_tmp >> 4);
    txDATA[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    txDATA[4] = kp_tmp;
    txDATA[5] = (kd_tmp >> 4);
    txDATA[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    txDATA[7] = tor_tmp;

    CANSend(phcan, id, txDATA, 8);
}

void DMMotor::GetInfo(uint8_t *rx_buff)
{
    state = (rx_buff[0]) >> 4;
    pos_last = pos;
    p_int = (rx_buff[1] << 8) | rx_buff[2];
    v_int = (rx_buff[3] << 4) | (rx_buff[4] >> 4);
    t_int = ((rx_buff[4] & 0xF) << 8) | rx_buff[5];

    pos = uint_to_float(p_int, P_MIN, P_MAX, 16);
    vel = uint_to_float(v_int, V_MIN, V_MAX, 12);
    tor = uint_to_float(t_int, T_MIN, T_MAX, 12);
    Tmos = (float)rx_buff[6];
    Tcoil = (float)rx_buff[7];

    if ((pos - pos_last) > 5.23599f)
        circle_number--;
    else if ((pos - pos_last) < -5.23599f)
        circle_number++;

    accumulate_angle = circle_number * 10.47198f + pos;
}