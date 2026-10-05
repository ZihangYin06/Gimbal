#include "Controller.hpp"
#include "MathUtils.hpp"

namespace Algorithm
{

    // ========================== FuzzyRule ==========================
    void FuzzyRule::Init(const float (*ruleKp)[7], const float (*ruleKi)[7], const float (*ruleKd)[7],
                         float kpRatio, float kiRatio, float kdRatio,
                         float eStep, float ecStep)
    {
        FuzzyRuleKp = ruleKp;
        FuzzyRuleKi = ruleKi;
        FuzzyRuleKd = ruleKd;
        KpRatio = kpRatio;
        KiRatio = kiRatio;
        KdRatio = kdRatio;
        this->eStep = (eStep < 1e-6f) ? 1.0f : eStep;
        this->ecStep = (ecStep < 1e-6f) ? 1.0f : ecStep;
        e = 0.0f;
        ec = 0.0f;
        eLast = 0.0f;
    }

    void FuzzyRule::Update(float measure, float ref, float dt)
    {
        this->dt = dt;
        eLast = e;
        e = ref - measure;
        ec = (dt > 1e-6f) ? (e - eLast) / dt : 0.0f;

        // 查表
        int eLeftIdx = (e >= 3 * eStep) ? 6 : (e <= -3 * eStep) ? 0
                                                                : (e >= 0 ? ((int)(e / eStep) + 3) : ((int)(e / eStep) + 2));
        int eRightIdx = eLeftIdx + 1;
        int ecLeftIdx = (ec >= 3 * ecStep) ? 6 : (ec <= -3 * ecStep) ? 0
                                                                     : (ec >= 0 ? ((int)(ec / ecStep) + 3) : ((int)(ec / ecStep) + 2));
        int ecRightIdx = ecLeftIdx + 1;

        float eLeftW = (e >= 3 * eStep) ? 0 : (e <= -3 * eStep) ? 1
                                                                : (eRightIdx - e / eStep - 3);
        float eRightW = (e >= 3 * eStep) ? 1 : (e <= -3 * eStep) ? 0
                                                                 : (e / eStep - eLeftIdx + 3);
        float ecLeftW = (ec >= 3 * ecStep) ? 0 : (ec <= -3 * ecStep) ? 1
                                                                     : (ecRightIdx - ec / ecStep - 3);
        float ecRightW = (ec >= 3 * ecStep) ? 1 : (ec <= -3 * ecStep) ? 0
                                                                      : (ec / ecStep - ecLeftIdx + 3);

        KpFuzzy = eLeftW * ecLeftW * FuzzyRuleKp[eLeftIdx][ecLeftIdx] + eLeftW * ecRightW * FuzzyRuleKp[eRightIdx][ecLeftIdx] + eRightW * ecLeftW * FuzzyRuleKp[eLeftIdx][ecRightIdx] + eRightW * ecRightW * FuzzyRuleKp[eRightIdx][ecRightIdx];

        KiFuzzy = eLeftW * ecLeftW * FuzzyRuleKi[eLeftIdx][ecLeftIdx] + eLeftW * ecRightW * FuzzyRuleKi[eRightIdx][ecLeftIdx] + eRightW * ecLeftW * FuzzyRuleKi[eLeftIdx][ecRightIdx] + eRightW * ecRightW * FuzzyRuleKi[eRightIdx][ecRightIdx];

        KdFuzzy = eLeftW * ecLeftW * FuzzyRuleKd[eLeftIdx][ecLeftIdx] + eLeftW * ecRightW * FuzzyRuleKd[eRightIdx][ecLeftIdx] + eRightW * ecLeftW * FuzzyRuleKd[eLeftIdx][ecRightIdx] + eRightW * ecRightW * FuzzyRuleKd[eRightIdx][ecRightIdx];
    }

    // ========================== PIDController ==========================
    void PIDController::Init(const Params &params)
    {
        params_ = params;
        Reset();
    }

    void PIDController::Reset()
    {
        err_ = 0.0f;
        last_err_ = 0.0f;
        integral_ = 0.0f;
        output_ = 0.0f;
        last_output_ = 0.0f;
        pout_ = 0.0f;
        iout_ = 0.0f;
        dout_ = 0.0f;
        last_dout_ = 0.0f;
        first_run_ = true;
    }

    float PIDController::Calculate(float measure, float ref, float dt)
    {
        if (dt <= 0.0f)
            dt = 0.001f;

        measure_ = measure;
        ref_ = ref;
        err_ = ref - measure;

        // 死区
        if (std::fabs(err_) < params_.deadband)
        {
            return output_;
        }

        // 模糊 PID 更新
        if (params_.fuzzy_rule)
        {
            params_.fuzzy_rule->Update(measure, ref, dt);
        }

        // P
        float kp = params_.kp;
        if (params_.fuzzy_rule)
            kp += params_.fuzzy_rule->KpFuzzy * params_.fuzzy_rule->KpRatio;
        pout_ = kp * err_;

        // I
        float ki = params_.ki;
        if (params_.fuzzy_rule)
            ki += params_.fuzzy_rule->KiFuzzy * params_.fuzzy_rule->KiRatio;

        if (params_.improve & TRAPEZOID_INTEGRAL)
        {
            integral_ += ki * (err_ + last_err_) * dt * 0.5f;
        }
        else
        {
            integral_ += ki * err_ * dt;
        }

        if (params_.improve & INTEGRAL_LIMIT)
        {
            integral_ = Constrain(integral_, -params_.integral_limit, params_.integral_limit);
        }
        iout_ = integral_;

        // D
        float kd = params_.kd;
        if (params_.fuzzy_rule)
            kd += params_.fuzzy_rule->KdFuzzy * params_.fuzzy_rule->KdRatio;

        float raw_derivative;
        if (params_.improve & DERIVATIVE_ON_MEASUREMENT)
        {
            raw_derivative = (last_err_ - (ref - measure_)) / dt;
        }
        else
        {
            raw_derivative = (err_ - last_err_) / dt;
        }

        // 使用最小二乘法提取微分（如果提供了 OLS 对象）
        if (params_.ols)
        {
            params_.ols->Update(dt, err_);
            dout_ = kd * params_.ols->GetDerivative();
        }
        else
        {
            dout_ = kd * raw_derivative;
        }

        if (params_.improve & DERIVATIVE_FILTER)
        {
            float rc = params_.derivative_lpf_rc;
            dout_ = dout_ * dt / (rc + dt) + last_dout_ * rc / (rc + dt);
            last_dout_ = dout_;
        }

        // 输出
        output_ = pout_ + iout_ + dout_;

        if (params_.improve & OUTPUT_FILTER)
        {
            float rc = params_.output_lpf_rc;
            output_ = output_ * dt / (rc + dt) + last_output_ * rc / (rc + dt);
            last_output_ = output_;
        }

        output_ = Constrain(output_, -params_.max_out, params_.max_out);

        last_err_ = err_;
        return output_;
    }

    // ========================== Feedforward ==========================
    void Feedforward::Init(float max_out, float c0, float c1, float c2, float lpf_rc)
    {
        max_out_ = max_out;
        c_[0] = c0;
        c_[1] = c1;
        c_[2] = c2;
        lpf_rc_ = lpf_rc;
        ref_filtered_ = 0.0f;
        last_ref_ = 0.0f;
        output_ = 0.0f;
        ols_inited_ = false;
    }

    float Feedforward::Calculate(float ref, float dt)
    {
        if (dt <= 0.0f)
            return 0.0f;

        ref_filtered_ = ref * dt / (lpf_rc_ + dt) + ref_filtered_ * lpf_rc_ / (lpf_rc_ + dt);

        if (!ols_inited_)
        {
            ols_dot_.Init(10);
            ols_ddot_.Init(10);
            ols_inited_ = true;
        }

        ols_dot_.Update(dt, ref_filtered_);
        float ref_dot = ols_dot_.GetDerivative();

        ols_ddot_.Update(dt, ref_dot);
        float ref_ddot = ols_ddot_.GetDerivative();

        output_ = c_[0] * ref_filtered_ + c_[1] * ref_dot + c_[2] * ref_ddot;
        output_ = Constrain(output_, -max_out_, max_out_);

        return output_;
    }

    // ========================== LDOB ==========================
    void LDOB::Init(float max_disturbance, float deadband, float c0, float c1, float c2, float lpf_rc)
    {
        max_d_ = max_disturbance;
        deadband_ = deadband;
        c_[0] = c0;
        c_[1] = c1;
        c_[2] = c2;
        lpf_rc_ = lpf_rc;
        measure_ = 0.0f;
        last_measure_ = 0.0f;
        disturbance_ = 0.0f;
        last_disturbance_ = 0.0f;
        output_ = 0.0f;
        ols_inited_ = false;
    }

    float LDOB::Calculate(float measure, float u, float dt)
    {
        if (dt <= 0.0f)
            return 0.0f;

        last_measure_ = measure_;
        measure_ = measure;

        if (!ols_inited_)
        {
            ols_dot_.Init(10);
            ols_ddot_.Init(10);
            ols_inited_ = true;
        }

        ols_dot_.Update(dt, measure_);
        float measure_dot = ols_dot_.GetDerivative();

        ols_ddot_.Update(dt, measure_dot);
        float measure_ddot = ols_ddot_.GetDerivative();

        float raw_disturbance = c_[0] * measure_ + c_[1] * measure_dot + c_[2] * measure_ddot - u;

        disturbance_ = raw_disturbance * dt / (lpf_rc_ + dt) + last_disturbance_ * lpf_rc_ / (lpf_rc_ + dt);
        disturbance_ = Constrain(disturbance_, -max_d_, max_d_);

        if (std::fabs(disturbance_) > deadband_ * max_d_)
        {
            output_ = disturbance_;
        }
        else
        {
            output_ = 0.0f;
        }

        last_disturbance_ = disturbance_;
        return output_;
    }

    // ========================== TrackingDifferentiator ==========================
    void TrackingDifferentiator::Init(float r, float h0)
    {
        r_ = r;
        h0_ = h0;
        x_ = 0.0f;
        dx_ = 0.0f;
        ddx_ = 0.0f;
        last_dx_ = 0.0f;
        last_ddx_ = 0.0f;
    }

    float TrackingDifferentiator::Calculate(float input, float dt)
    {
        if (dt > 0.5f)
            return 0.0f;

        float d = r_ * h0_ * h0_;
        float a0 = dx_ * h0_;
        float y = x_ - input + a0;
        float a1 = std::sqrt(d * (d + 8.0f * std::fabs(y)));
        float a2 = a0 + Sign(y) * (a1 - d) / 2.0f;
        float a = (a0 + y) * (Sign(y + d) - Sign(y - d)) / 2.0f + a2 * (1.0f - (Sign(y + d) - Sign(y - d)) / 2.0f);
        float fhan = -r_ * a / d * (Sign(a + d) - Sign(a - d)) / 2.0f - r_ * Sign(a) * (1.0f - (Sign(a + d) - Sign(a - d)) / 2.0f);

        ddx_ = fhan;
        dx_ += (ddx_ + last_ddx_) * dt / 2.0f;
        x_ += (dx_ + last_dx_) * dt / 2.0f;

        last_ddx_ = ddx_;
        last_dx_ = dx_;

        return x_;
    }

} // namespace Algorithm