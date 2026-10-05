/**
 ******************************************************************************
 * @file    PID.hpp
 * @brief   位置式 PID 控制器（支持单环、角度-速度双环级联、重力前馈补偿）
 *
 * 使用约定：
 *   - 控制周期由调用方决定（如 LIFT 任务 2ms 调用一次），内部不做 dt 缩放，
 *     因此 ki/kd 的量纲与控制周期绑定，改任务周期需重新整定参数
 *   - 一个 PID 对象同一时间只服务一个控制环，级联时角度环与速度环
 *     需要两个独立对象
 ******************************************************************************
 */
#ifndef PID_HPP_
#define PID_HPP_

#include <cmath>

class PID
{
private:
    /* PID 三参数 */
    float kp_ = 0.0f;
    float ki_ = 0.0f;
    float kd_ = 0.0f;

    /* 误差历史（用于微分项） */
    float now_error_ = 0.0f;
    float last_error_ = 0.0f;

    /* 误差积分值（先累加误差，输出时再乘 ki） */
    float integral_ = 0.0f;
    float max_integral_ = 0.0f; /* 积分限幅，取正值，0 表示不限幅 */

    float output_ = 0.0f;
    float max_output_ = 0.0f; /* 输出限幅，取正值，0 表示不限幅 */

public:
    PID() = default;

    /**
     * @brief  重力补偿前馈：kf * sin(角度反馈 + 偏置)
     * @param  kf: 重力补偿系数
     * @param  angle_feedback: 角度反馈，单位弧度
     * @param  offset_rad: 结构零位与重力方向对齐所需的角度偏置，单位弧度
     * @note   纯函数，不依赖对象状态，可直接静态调用
     */
    static float Gravity_PID_Calc(float kf, float angle_feedback, float offset_rad)
    {
        return kf * std::sinf(angle_feedback + offset_rad);
    }

    /**
     * @brief  设置 PID 参数
     * @param  kp/ki/kd: 三项增益
     * @param  max_integral: 积分限幅（对累加后的误差值限幅），0 表示不限制
     * @param  max_output: 输出限幅，0 表示不限制
     */
    void SetParams(float kp, float ki, float kd, float max_integral, float max_output);

    /**
     * @brief  基础位置式 PID 计算（含积分限幅和输出限幅）
     * @param  target: 目标值
     * @param  feedback: 反馈值（与 target 同单位）
     * @note   首次调用时 last_error_ 为 0，微分项会有一拍冲击，
     *         可先调用 Reset() 或忽略首拍输出
     */
    void Calc(float target, float feedback);

    /**
     * @brief  速度环计算，语义上等同 Calc（单环）
     */
    void SpeedCalc(float target, float speed_feedback);

    /**
     * @brief  角度-速度双环级联：本对象作角度外环，
     *         外环输出作为 speed_pid 速度内环的目标
     * @param  speed_pid: 速度内环对象（需已单独 SetParams）
     * @param  target: 目标角度
     * @param  angle_feedback: 角度反馈（度）
     * @param  speed_feedback: 速度反馈（与内环目标同单位）
     */
    void AngleCalc(PID &speed_pid, float target, float angle_feedback, float speed_feedback);

    /**
     * @brief  在角度环基础上叠加重力补偿前馈后再进速度环
     * @param  kf: 重力补偿系数
     * @param  offset_rad: 重力零位偏置，单位弧度
     * @note   注意单位：target/angle_feedback 为度（供 Calc 用），
     *         仅重力项内部转弧度
     */
    void GravityAngleCalc(PID &speed_pid, float kf, float offset_rad,
                          float target, float angle_feedback, float speed_feedback);

    /**
     * @brief  清除历史状态（误差/积分/输出），参数保留；
     *         切换目标或重新上电时建议调用，避免旧积分引起跳变
     */
    void Reset();

    /**
     * @brief  获取最近一次计算的输出（已限幅）
     */
    float GetOutput() const { return output_; }
};

#endif
