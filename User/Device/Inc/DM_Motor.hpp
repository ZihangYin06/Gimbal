/**
 ******************************************************************************
 * @file    DM_Motor.hpp
 * @brief   达妙 DM 系列电机驱动（CAN + MIT 协议帧）
 *
 * 反馈帧解码出单圈角度 pos 与多圈累积角 accumulate_angle（解环绕按
 * 25 rad 协议周期增量累积）；MITCmd 按协议定长位域打包下发，
 * 入参越界会在驱动内钳位到协议量程，不会回绕成错误值。
 * 另提供六关节批量操作（逐轴使能握手 / 全轴安全阻尼态）。
 ******************************************************************************
 */
#ifndef _DM_MOTOR_HPP_
#define _DM_MOTOR_HPP_

#include <stdint.h>
#include "bsp_can.h"

struct DMMotor
{
    /* 反馈状态 */
    uint16_t state = 0; /* 使能/错误状态字（反馈帧 byte0 高 4 位） */
    float pos = 0.0f;   /* 单圈角度 rad，协议量程 ±12.5 */
    float vel = 0.0f;   /* 角速度 rad/s，量程 ±30 */
    float tor = 0.0f;   /* 反馈力矩 N·m，量程 ±7 */
    float Tmos = 0.0f;  /* MOS 温度 °C */
    float Tcoil = 0.0f; /* 绕组温度 °C */

    /* 多圈累积 */
    float pos_last = 0.0f;         /* 上一帧单圈角，解环绕用 */
    float accumulate_angle = 0.0f; /* 多圈累积角 rad，重力模型输入 */

    void DecodeFeedback(uint8_t *rx_buff); /* 反馈帧 8 字节解码 */
    void Enable(CAN_HandleTypeDef *phcan, uint32_t id);
    void Disable(CAN_HandleTypeDef *phcan, uint32_t id);
    void MITCmd(CAN_HandleTypeDef *phcan, uint32_t id,
                float pos, float vel, float KP, float KD, float torq);
};

/* 全局 DM 电机对象声明（定义在 .cpp 中，命名与 define.h 的 ID/CAN 宏对应） */
extern DMMotor FEEDER_Motor; /* 拨弹轮（define.h: FEEDER_*，CAN1） */

#endif
