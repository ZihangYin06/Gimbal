#ifndef _DM_MOTOR_HPP_
#define _DM_MOTOR_HPP_

#include <stdint.h>
#include "bsp_can.h"

struct DMMotor
{
    uint16_t state = 0;
    int p_int = 0, v_int = 0, t_int = 0;
    float pos = 0.0f, vel = 0.0f, tor = 0.0f;
    float Tmos = 0.0f, Tcoil = 0.0f;
    float pos_last = 0.0f;
    int32_t circle_number = 0;
    float accumulate_angle = 0.0f;

    void GetInfo(uint8_t *rx_buff);
    void Enable(CAN_HandleTypeDef *phcan, uint32_t id);
    void Disable(CAN_HandleTypeDef *phcan, uint32_t id);
    void MITCmd(CAN_HandleTypeDef *phcan, uint32_t id,
                float pos, float vel, float KP, float KD, float torq);
};

// 全局拨弹电机对象声明（定义在 .cpp 中）
extern DMMotor FEEDER_Motor;

#endif