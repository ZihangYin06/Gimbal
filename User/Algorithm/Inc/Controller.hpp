#ifndef _CONTROLLER_HPP_
#define _CONTROLLER_HPP_

#include <cstdint>
#include <cmath>
#include "LeastSquares.hpp"

namespace Algorithm
{

// ========================== 模糊 PID 规则 ==========================
#define NB -3
#define NM -2
#define NS -1
#define ZE 0
#define PS 1
#define PM 2
#define PB 3

    /**
     * @brief 模糊 PID 规则表
     */
    struct FuzzyRule
    {
        float KpFuzzy;
        float KiFuzzy;
        float KdFuzzy;

        const float (*FuzzyRuleKp)[7];
        const float (*FuzzyRuleKi)[7];
        const float (*FuzzyRuleKd)[7];

        float KpRatio;
        float KiRatio;
        float KdRatio;

        float eStep;
        float ecStep;

        float e;
        float ec;
        float eLast;

        float dt;

        void Init(const float (*ruleKp)[7], const float (*ruleKi)[7], const float (*ruleKd)[7],
                  float kpRatio, float kiRatio, float kdRatio,
                  float eStep, float ecStep);

        void Update(float measure, float ref, float dt);
    };

    // ========================== PID 控制器（增强版） ==========================
    enum PIDImprovement
    {
        NONE = 0x00,
        INTEGRAL_LIMIT = 0x01,
        DERIVATIVE_ON_MEASUREMENT = 0x02,
        TRAPEZOID_INTEGRAL = 0x04,
        OUTPUT_FILTER = 0x10,
        DERIVATIVE_FILTER = 0x40,
    };

    class PIDController
    {
    public:
        struct Params
        {
            float kp = 0.0f, ki = 0.0f, kd = 0.0f;
            float max_out = 0.0f;
            float integral_limit = 0.0f;
            float deadband = 0.0f;
            float output_lpf_rc = 0.0f;
            float derivative_lpf_rc = 0.0f;
            uint8_t improve = 0;
            FuzzyRule *fuzzy_rule = nullptr;
            LeastSquares *ols = nullptr;
        };

        PIDController() = default;
        void Init(const Params &params);

        float Calculate(float measure, float ref, float dt);

        float GetOutput() const { return output_; }
        void Reset();

    private:
        Params params_;
        float ref_ = 0.0f, measure_ = 0.0f;
        float err_ = 0.0f, last_err_ = 0.0f;
        float pout_ = 0.0f, iout_ = 0.0f, dout_ = 0.0f;
        float integral_ = 0.0f;
        float output_ = 0.0f, last_output_ = 0.0f;
        float last_dout_ = 0.0f;
        bool first_run_ = true;

        void UpdateIntegral(float dt);
        void UpdateDerivative(float dt);
        void ApplyLimits();
    };

    // ========================== 前馈控制器 ==========================
    class Feedforward
    {
    public:
        void Init(float max_out, float c0, float c1, float c2, float lpf_rc);
        float Calculate(float ref, float dt);

    private:
        float max_out_ = 0.0f;
        float c_[3] = {0, 0, 0};
        float lpf_rc_ = 0.0f;
        float ref_filtered_ = 0.0f;
        float last_ref_ = 0.0f;
        float ref_dot_ = 0.0f, last_ref_dot_ = 0.0f;
        float ref_ddot_ = 0.0f;
        float output_ = 0.0f;
        LeastSquares ols_dot_, ols_ddot_;
        bool ols_inited_ = false;
    };

    // ========================== 线性扰动观测器 (LDOB) ==========================
    class LDOB
    {
    public:
        void Init(float max_disturbance, float deadband, float c0, float c1, float c2, float lpf_rc);
        float Calculate(float measure, float u, float dt);

    private:
        float max_d_ = 0.0f, deadband_ = 0.0f;
        float c_[3] = {0, 0, 0};
        float lpf_rc_ = 0.0f;
        float measure_ = 0.0f, last_measure_ = 0.0f;
        float measure_dot_ = 0.0f, last_measure_dot_ = 0.0f;
        float measure_ddot_ = 0.0f;
        float disturbance_ = 0.0f, last_disturbance_ = 0.0f;
        float output_ = 0.0f;
        LeastSquares ols_dot_, ols_ddot_;
        bool ols_inited_ = false;
    };

    // ========================== 跟踪微分器 (TD) ==========================
    class TrackingDifferentiator
    {
    public:
        void Init(float r, float h0);
        float Calculate(float input, float dt);

    private:
        float r_ = 0.0f, h0_ = 0.0f;
        float x_ = 0.0f, dx_ = 0.0f, ddx_ = 0.0f;
        float last_dx_ = 0.0f, last_ddx_ = 0.0f;
    };

} // namespace Algorithm

#endif