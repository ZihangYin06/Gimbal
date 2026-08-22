#ifndef _PID_HPP_
#define _PID_HPP_

#include <cmath>

class PID
{
private:
    float kp_ = 0.0f;
    float ki_ = 0.0f;
    float kd_ = 0.0f;
    float now_error_ = 0.0f;
    float last_error_ = 0.0f;
    float integral_ = 0.0f;
    float max_integral_ = 0.0f;
    float output_ = 0.0f;
    float max_output_ = 0.0f;

public:
    PID() = default;
    static float Gravity_PID_Calc(float kf, float angle_feedback, float offset_rad)
    {
        return kf * std::sinf(angle_feedback + offset_rad);
    }

    void SetParams(float kp, float ki, float kd, float max_integral, float max_output);

    void Calc(float target, float feedback);

    void SpeedCalc(float target, float speed_feedback);

    void AngleCalc(PID &speed_pid, float target, float angle_feedback, float speed_feedback);

    void GravityAngleCalc(PID &speed_pid, float kf, float offset_rad,
                          float target, float angle_feedback, float speed_feedback);

    void Reset();

    float GetOutput() const { return output_; }
};

#endif
