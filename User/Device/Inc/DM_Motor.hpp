/**
 ******************************************************************************
 * @file    DM_Motor.hpp
 * @brief   达妙 DM 系列电机驱动（CAN + MIT 协议帧）
 *
 * 反馈帧解码出单圈角度 pos 与多圈累积角 accumulate_angle（解环绕按
 * 25 rad 协议周期增量累积）；MITCmd 按协议定长位域打包下发，
 * 入参越界会在驱动内钳位到协议量程，不会回绕成错误值。
 *
 * 身份绑定说明：
 *   与 DJI 电机四台共用一帧不同，DM 电机每台都有独立的控制帧（TX）
 *   和反馈帧（RX）编号。总线 + TX/RX 编号在构造时绑定一次（拨弹
 *   电机的登记见 DM_Motor.cpp），之后所有接口不再传 CAN / ID 参数。
 ******************************************************************************
 */
#ifndef _DM_MOTOR_HPP_
#define _DM_MOTOR_HPP_

#include <stdint.h>
#include "bsp_can.h"

struct DMMotor
{
    /* ---- 电机身份（构造时绑定，之后不变） ---- */
    CAN_HandleTypeDef *phcan; /* 所属总线 */
    uint32_t tx_id;           /* 控制帧 StdId（使能/失能/MIT 指令帧） */
    uint32_t rx_id;           /* 反馈帧 StdId（每台各不相同） */

    /* ---- 反馈状态（由 DecodeFeedback 解析更新） ---- */
    uint16_t state = 0; /* 使能/错误状态字（反馈帧 byte0 高 4 位） */
    float pos = 0.0f;   /* 单圈角度 rad，协议量程 ±12.5 */
    float vel = 0.0f;   /* 角速度 rad/s，量程 ±30 */
    float tor = 0.0f;   /* 反馈力矩 N·m，量程 ±7 */
    float Tmos = 0.0f;  /* MOS 温度 ℃ */
    float Tcoil = 0.0f; /* 绕组温度 ℃ */

    /* ---- 多圈累积 ---- */
    float pos_last = 0.0f;         /* 上一帧单圈角，解环绕用 */
    float accumulate_angle = 0.0f; /* 多圈累积角 rad，重力模型输入 */

    /**
     * @brief  构造即绑定身份，此后总线 / TX / RX 编号不再作为参数出现
     * @param  phcan: 所属总线
     * @param  tx_id: 控制帧 StdId
     * @param  rx_id: 反馈帧 StdId（每台各不相同，登记时逐台核对）
     */
    DMMotor(CAN_HandleTypeDef *phcan, uint32_t tx_id, uint32_t rx_id)
        : phcan(phcan), tx_id(tx_id), rx_id(rx_id) {}

    void DecodeFeedback(uint8_t *rx_buff); /* 反馈帧 8 字节解码 */
    void Enable(void);                     /* 使能帧，发往本机 TX 编号 */
    void Disable(void);                    /* 失能帧，发往本机 TX 编号 */
    void MITCmd(float pos, float vel, float KP, float KD, float torq);
};

/* 全局 DM 电机对象声明（定义与身份绑定在 .cpp 中） */
extern DMMotor FEEDER_Motor; /* 拨弹轮（CAN1，TX 0x03 / RX 0x30） */

#endif
