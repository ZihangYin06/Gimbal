#include "PID.hpp"

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
    Calc(target, angle_feedback);
    speed_pid.SpeedCalc(output_, speed_feedback);
}

void PID::GravityAngleCalc(PID &speed_pid, float kf, float offset_rad,
                           float target, float angle_feedback, float speed_feedback)
{
    Calc(target, angle_feedback);
    float gravity_out = Gravity_PID_Calc(kf, angle_feedback, offset_rad);
    speed_pid.SpeedCalc(output_ + gravity_out, speed_feedback);
}

void PID::Reset()
{
    now_error_ = 0.0f;
    last_error_ = 0.0f;
    integral_ = 0.0f;
    output_ = 0.0f;
}