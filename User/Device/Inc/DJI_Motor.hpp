#ifndef _DJI_MOTOR_HPP_
#define _DJI_MOTOR_HPP_

#include <stdint.h>
#include "bsp_can.h"
#include "define.h"

enum class MotorModel : uint8_t
{
    GM6020,
    M3508
};

struct DJIMotor
{
    uint16_t encoder = 0;
    int16_t speed = 0;
    int16_t current = 0;
    uint8_t temperature = 0;

    uint16_t last_encoder = 0;
    int32_t circle_number = 0;
    float accumulate_angle = 0.0f;

    void GetInfo(uint8_t *rx_buff);
    void SetCurrent(MotorModel model, uint8_t id, int16_t current);
    void StartMotor(MotorModel model, CAN_HandleTypeDef *phcan);
};

// 全局电机对象声明（定义在 .cpp 中）
extern DJIMotor YAW_Motor;
extern DJIMotor PITCH_Motor;
extern DJIMotor LEFT_FRICTION_Motor;
extern DJIMotor RIGHT_FRICTION_Motor;

#endif
