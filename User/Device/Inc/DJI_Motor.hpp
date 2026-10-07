/**
 ******************************************************************************
 * @file    DJI_Motor.hpp
 * @brief   大疆电机（GM6020 / M3508）驱动 + 电机单元（驱动 + 双环 PID）
 *
 * 发送模型说明（重要）：
 *   同一型号电机在一条总线上共用一帧电流 CAN（8 字节，每台电机占
 *   2 字节大端）。CAN1 / CAN2 各有一份独立的发送缓存，按 (总线, id)
 *   定位槽位：SetCurrent() 把指令写入本电机所属总线缓存的对应槽位，
 *   StartMotor() 把该总线整帧缓存发出。
 *
 * 身份绑定说明：
 *   型号 / 总线 / 编号是每台电机的固定属性，构造对象时绑定一次
 *   （全局电机的"身份登记表"在 DJI_Motor.cpp 中），之后所有控制、
 *   发送、反馈接口都不再传身份参数。
 *   新增电机：在 DJI_Motor.cpp 的登记表中仿照现有写法定义对象。
 ******************************************************************************
 */
#ifndef DJI_MOTOR_HPP_
#define DJI_MOTOR_HPP_

#include <stdint.h>
#include "bsp_can.h"
#include "PID.hpp"

/* 电机型号：决定电流指令帧的 CAN ID 与协议 */
enum class MotorModel : uint8_t
{
    GM6020,
    M3508
};

/* ---- 纯驱动层：身份 + 反馈 + 收发，不含控制器 ---- */
struct DJIMotor
{
    /* ---- 电机身份（构造时绑定，之后不变） ---- */
    MotorModel model;         /* 电机型号，决定共用哪份电流帧 */
    CAN_HandleTypeDef *phcan; /* 所属总线（&hcan1 / &hcan2） */
    uint8_t id;               /* 总线上的电机编号 1~4 */

    /* ---- 反馈数据（由 GetInfo 解析 CAN 反馈帧更新） ---- */
    uint16_t encoder = 0;       /* 机械角度原始值 0~8191（一圈 8192） */
    int16_t speed = 0;          /* 转速 rpm */
    int16_t current = 0;        /* 实际转矩电流反馈 */
    uint8_t temperature = 0;    /* 电机温度 ℃ */

    /* ---- 多圈角度累计 ---- */
    uint16_t last_encoder = 0;      /* 上一拍机械角度，用于过零检测 */
    int32_t circle_number = 0;      /* 累计圈数，正转为正、反转为负 */
    float accumulate_angle = 0.0f;  /* 多圈累计角度（度），可超出 0~360、可为负 */

    /**
     * @brief  构造即绑定身份，此后型号 / 总线 / 编号不再作为参数出现
     * @param  model: 电机型号
     * @param  phcan: 所属总线（&hcan1 / &hcan2）
     * @param  id: 该总线上的电机编号 1~4
     */
    DJIMotor(MotorModel model, CAN_HandleTypeDef *phcan, uint8_t id)
        : model(model), phcan(phcan), id(id) {}

    /**
     * @brief  将电流指令写入本电机在所属总线发送缓存中的槽位
     * @param  current: 电流指令（M3508 范围 -16384~16384，GM6020 为电压指令）
     * @note   身份非法（未知总线 / id 超出 1~4）时直接忽略；
     *         需先对所有电机调用 SetCurrent，再调用 StartMotor 发整帧
     */
    void SetCurrent(int16_t current);

    /**
     * @brief  将本电机所属总线、所属型号的整帧电流缓存发出
     * @note   需先对所有电机调用 SetCurrent 再调用本函数，
     *         否则未更新的槽位会把旧值/0 发出去
     */
    void StartMotor(void);

    /**
     * @brief  解析 8 字节 CAN 反馈帧，更新本对象全部反馈字段
     * @param  rx_buff: 反馈帧数据（GM6020 为 0x205+id-1，M3508 为 0x201+id-1）
     * @note   过零换圈假设相邻两拍角度变化小于半圈（控制周期足够短）
     */
    void GetInfo(uint8_t *rx_buff);

    /**
     * @brief  本电机的反馈帧 CAN ID（M3508: 0x200+id，GM6020: 0x204+id）
     * @return 反馈帧 ID；id 非法时返回 0
     * @note   供 CAN 接收分发用：StdId == xx_Motor.RxId() 即为本电机反馈
     */
    uint16_t RxId(void) const;
};

/* ---- 电机单元：纯驱动 + 角度外环 / 速度内环，反馈取自本电机自身 ---- */
struct MotorUnit : DJIMotor
{
    PID angle_pid; /* 角度外环 */
    PID speed_pid; /* 速度内环 */

    MotorUnit(MotorModel model, CAN_HandleTypeDef *phcan, uint8_t id)
        : DJIMotor(model, phcan, id) {}

    /**
     * @brief  角度-速度双环级联计算并写入电流指令
     * @param  target_angle: 目标角度（度，与 accumulate_angle 同单位）
     * @note   反馈自动取本电机 accumulate_angle / speed，无需传参；
     *         电流只写入发送缓存，需等同一总线的电机都算完后统一 StartMotor()
     */
    void AngleControl(float target_angle)
    {
        angle_pid.AngleCalc(speed_pid, target_angle, accumulate_angle, speed);
        SetCurrent((int16_t)speed_pid.GetOutput());
    }

    /**
     * @brief  单速度环计算并写入电流指令（反馈自动取本电机 speed）
     */
    void SpeedControl(float target_speed)
    {
        speed_pid.SpeedCalc(target_speed, speed);
        SetCurrent((int16_t)speed_pid.GetOutput());
    }

    /**
     * @brief  电流指令清零（停机保持，同样只写缓存不发包）
     */
    void StopCurrent(void)
    {
        SetCurrent(0);
    }
};

// 全局电机对象声明（定义与身份绑定在 .cpp 中）
extern DJIMotor YAW_Motor;            /* 云台偏航 GM6020（CAN1） */
extern DJIMotor PITCH_Motor;          /* 云台俯仰 GM6020（CAN2） */
extern DJIMotor LEFT_FRICTION_Motor;  /* 左摩擦轮 M3508（CAN2） */
extern DJIMotor RIGHT_FRICTION_Motor; /* 右摩擦轮 M3508（CAN2） */

#endif
