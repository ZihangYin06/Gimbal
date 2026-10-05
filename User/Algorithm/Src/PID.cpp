/**
 ******************************************************************************
 * @file    PID.cpp
 * @brief   位置式 PID 控制器实现
 ******************************************************************************
 */
#include "PID.hpp"

/* 圆周率（仅重力项角度换算使用，常量放在 .cpp 内避免全局宏污染） */
static constexpr float PI = 3.14159265358979f;

void PID::SetParams(float kp, float ki, float kd, float max_integral, float max_output)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    max_integral_ = max_integral;
    max_output_ = max_output;
}

void PID::Calc(float target, float feedback)
{
    last_error_ = now_error_;
    now_error_ = target - feedback;

    /* 积分项：先累加误差，输出时再乘 ki；超限即截断（抗积分饱和） */
    integral_ += now_error_;
    if (integral_ > max_integral_)
        integral_ = max_integral_;
    else if (integral_ < -max_integral_)
        integral_ = -max_integral_;

    output_ = kp_ * now_error_ + ki_ * integral_ + kd_ * (now_error_ - last_error_);
    if (output_ > max_output_)
        output_ = max_output_;
    else if (output_ < -max_output_)
        output_ = -max_output_;
}

void PID::SpeedCalc(float target, float speed_feedback)
{
    Calc(target, speed_feedback);
}

void PID::AngleCalc(PID &speed_pid, float target, float angle_feedback, float speed_feedback)
{
    /* 外环：角度环，输出作为内环速度目标 */
    Calc(target, angle_feedback);
    /* 内环：速度环，输出即最终控制量 */
    speed_pid.SpeedCalc(output_, speed_feedback);
}

void PID::GravityAngleCalc(PID &speed_pid, float kf, float offset_rad,
                           float target, float angle_feedback, float speed_feedback)
{
    /* 外环：角度环 */
    Calc(target, angle_feedback);
    /* 重力前馈：角度反馈为度，转为弧度后代入 sin */
    float gravity_out = Gravity_PID_Calc(kf, (angle_feedback)*PI / 180.0f, offset_rad);
    /* 内环：速度环，目标 = 角度环输出 + 重力前馈 */
    speed_pid.SpeedCalc(output_ + gravity_out, speed_feedback);
}

void PID::Reset()
{
    now_error_ = 0.0f;
    last_error_ = 0.0f;
    integral_ = 0.0f;
    output_ = 0.0f;
}
