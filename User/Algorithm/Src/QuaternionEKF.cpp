/**
 * @file  QuaternionEKF.cpp
 * @brief 四元数姿态解算 EKF 实现
 *
 * 算法流程(每步):
 *   Prepare      陀螺去零偏 → F 阵 4x4 分块 → 加速度低通 → 量测 z(归一化重力) → Q/R
 *   黄金五式     式1/2 预测(含 OnPredict 的 F 线性化、渐消)
 *                式3/4 由 OnCorrect 替换(卡方检验 + 自适应增益)
 *                式5 协方差更新
 *   PostProcess  四元数归一化 → 欧拉角 → yaw 累计圈数
 */
#include "QuaternionEKF.hpp"
#include <cmath>

namespace algo
{
    void QuaternionEKF::Init(const Params &params)
    {
        params_ = params;
        Reset(); // 清零状态/协方差并回调 OnReset() 设初值
        initialized_ = true;
    }

    void QuaternionEKF::Update(float gx, float gy, float gz, float ax, float ay, float az, float dt)
    {
        if (!initialized_)
            Init(Params{}); // 兜底:未显式初始化时使用默认参数

        Prepare(gx, gy, gz, ax, ay, az, dt);
        KalmanFilter<6, 0, 3>::Update(); // 黄金五式(内部回调 OnPredict/OnPreCorrect/OnCorrect)
        PostProcess();
    }

    // ==================== 复位 ====================

    void QuaternionEKF::OnReset()
    {
        // 四元数初值 [1,0,0,0],零偏初值 0
        memset(xhat_data_, 0, sizeof(xhat_data_));
        xhat_data_[0] = 1.0f;

        memcpy(P_data_, kPInit, sizeof(kPInit));

        // 式 3/4 由 OnCorrect() 自定义实现
        skip_eq1_ = false;
        skip_eq2_ = false;
        skip_eq3_ = true;
        skip_eq4_ = true;
        skip_eq5_ = false;

        // 清空输出与收敛状态
        q_[0] = 1.0f;
        q_[1] = q_[2] = q_[3] = 0.0f;
        memset(gyro_bias_, 0, sizeof(gyro_bias_));
        yaw_ = pitch_ = roll_ = yaw_total_ = yaw_last_ = 0.0f;
        yaw_round_count_ = 0;
        converge_flag_ = false;
        chi_square_ = 0.0f;
        adaptive_gain_scale_ = 1.0f;
        error_count_ = 0;
        update_count_ = 0; // 让下一次 Update 重新用当前原始加速度播种低通
        stable_flag_ = false;
        chi_square_data_[0] = 0.0f;
        arm_mat_init_f32(&chi_square_mat_, 1, 1, chi_square_data_);
    }

    // ==================== 预处理 ====================

    void QuaternionEKF::Prepare(float gx, float gy, float gz, float ax, float ay, float az, float dt)
    {
        dt_ = dt;

        // 1. 陀螺去零偏
        gyro_[0] = gx - gyro_bias_[0];
        gyro_[1] = gy - gyro_bias_[1];
        gyro_[2] = gz - gyro_bias_[2];

        // 2. F 阵左上 4x4 分块: I + 0.5·Ω·dt(先恢复单位模板再写入时变项)
        memcpy(F_data_, kFIdentity, sizeof(kFIdentity));

        float halfgxdt = 0.5f * gyro_[0] * dt_;
        float halfgydt = 0.5f * gyro_[1] * dt_;
        float halfgzdt = 0.5f * gyro_[2] * dt_;

        F_data_[1] = -halfgxdt;
        F_data_[2] = -halfgydt;
        F_data_[3] = -halfgzdt;

        F_data_[6] = halfgxdt;
        F_data_[8] = halfgzdt;
        F_data_[9] = -halfgydt;

        F_data_[12] = halfgydt;
        F_data_[13] = -halfgzdt;
        F_data_[15] = halfgxdt;

        F_data_[18] = halfgzdt;
        F_data_[19] = halfgydt;
        F_data_[20] = -halfgxdt;

        // 3. 加速度低通滤波(降低撞击/振动对量测的影响)
        accel_raw_[0] = ax;
        accel_raw_[1] = ay;
        accel_raw_[2] = az;
        if (update_count_ == 0 || params_.acc_lpf_tau <= 0.0f || dt_ <= 0.0f)
        {
            accel_[0] = ax;
            accel_[1] = ay;
            accel_[2] = az;
        }
        else
        {
            const float tau = params_.acc_lpf_tau;
            for (uint8_t i = 0; i < 3; ++i)
                accel_[i] = accel_[i] * tau / (dt_ + tau) + accel_raw_[i] * dt_ / (dt_ + tau);
        }

        // 4. 量测 z = 归一化加速度(近似重力方向)
        float accel_sq = accel_[0] * accel_[0] + accel_[1] * accel_[1] + accel_[2] * accel_[2];
        float accel_inv_norm = 1.0f / sqrtf(accel_sq);
        if (!std::isfinite(accel_inv_norm)) // 加速度全零等异常:沿用上一步量测
            accel_inv_norm = 0.0f;
        accel_norm_ = (accel_inv_norm > 0.0f) ? 1.0f / accel_inv_norm : accel_norm_;
        if (accel_inv_norm > 0.0f)
        {
            z_data_[0] = accel_[0] * accel_inv_norm;
            z_data_[1] = accel_[1] * accel_inv_norm;
            z_data_[2] = accel_[2] * accel_inv_norm;
        }

        // 5. 运动状态判定: 角速度小且加速度模长接近 g,认为静止可用加速度修正
        gyro_norm_ = sqrtf(gyro_[0] * gyro_[0] + gyro_[1] * gyro_[1] + gyro_[2] * gyro_[2]);
        stable_flag_ = (gyro_norm_ < 0.3f &&
                        accel_norm_ > 9.8f - 0.5f &&
                        accel_norm_ < 9.8f + 0.5f);

        // 6. 过程噪声 Q 与量测噪声 R
        Q_data_[0] = params_.process_noise_q * dt_;
        Q_data_[7] = params_.process_noise_q * dt_;
        Q_data_[14] = params_.process_noise_q * dt_;
        Q_data_[21] = params_.process_noise_q * dt_;
        Q_data_[28] = params_.process_noise_bias * dt_;
        Q_data_[35] = params_.process_noise_bias * dt_;

        R_data_[0] = params_.measure_noise_acc;
        R_data_[4] = params_.measure_noise_acc;
        R_data_[8] = params_.measure_noise_acc;
    }

    // ==================== 预测阶段扩展 ====================

    void QuaternionEKF::OnPredict()
    {
        // 1. 先验四元数归一化
        float q0 = xhatminus_data_[0];
        float q1 = xhatminus_data_[1];
        float q2 = xhatminus_data_[2];
        float q3 = xhatminus_data_[3];

        float q_inv_norm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        if (std::isfinite(q_inv_norm))
        {
            xhatminus_data_[0] = q0 * q_inv_norm;
            xhatminus_data_[1] = q1 * q_inv_norm;
            xhatminus_data_[2] = q2 * q_inv_norm;
            xhatminus_data_[3] = q3 * q_inv_norm;
            q0 = xhatminus_data_[0];
            q1 = xhatminus_data_[1];
            q2 = xhatminus_data_[2];
            q3 = xhatminus_data_[3];
        }

        // 2. F 阵右上 4x2 线性化分块(∂q̇/∂bias,协方差预测需要)
        F_data_[4] = q1 * dt_ / 2.0f;
        F_data_[5] = q2 * dt_ / 2.0f;

        F_data_[10] = -q0 * dt_ / 2.0f;
        F_data_[11] = q3 * dt_ / 2.0f;

        F_data_[16] = -q3 * dt_ / 2.0f;
        F_data_[17] = -q0 * dt_ / 2.0f;

        F_data_[22] = q2 * dt_ / 2.0f;
        F_data_[23] = -q1 * dt_ / 2.0f;

        // 3. 零偏方差渐消(防过度收敛)并限幅(防发散)
        const float lambda = (params_.fading_factor > 1.0f) ? 1.0f : params_.fading_factor;
        P_data_[28] /= lambda;
        P_data_[35] /= lambda;
        if (P_data_[28] > 10000.0f)
            P_data_[28] = 10000.0f;
        if (P_data_[35] > 10000.0f)
            P_data_[35] = 10000.0f;
    }

    // ==================== 校正阶段扩展 ====================

    void QuaternionEKF::OnPreCorrect()
    {
        // 观测函数 h(x) = 重力方向预测值在工作点处的雅可比 H
        float dq0 = 2.0f * xhatminus_data_[0];
        float dq1 = 2.0f * xhatminus_data_[1];
        float dq2 = 2.0f * xhatminus_data_[2];
        float dq3 = 2.0f * xhatminus_data_[3];

        memset(H_data_, 0, sizeof(H_data_));

        H_data_[0] = -dq2;
        H_data_[1] = dq3;
        H_data_[2] = -dq0;
        H_data_[3] = dq1;

        H_data_[6] = dq1;
        H_data_[7] = dq0;
        H_data_[8] = dq3;
        H_data_[9] = dq2;

        H_data_[12] = dq0;
        H_data_[13] = -dq1;
        H_data_[14] = -dq2;
        H_data_[15] = dq3;
    }

    void QuaternionEKF::OnCorrect()
    {
        // ---- 1. 新息协方差 S = H·P⁻·Hᵀ + R 及其逆 ----
        arm_mat_trans_f32(&H_, &HT_);

        temp_mat1_.numRows = H_.numRows;
        temp_mat1_.numCols = Pminus_.numCols;
        arm_mat_mult_f32(&H_, &Pminus_, &temp_mat1_); // H·P⁻ (3x6)

        temp_mat2_.numRows = temp_mat1_.numRows;
        temp_mat2_.numCols = HT_.numCols;
        arm_mat_mult_f32(&temp_mat1_, &HT_, &temp_mat2_); // H·P⁻·Hᵀ (3x3)

        S_.numRows = R_.numRows;
        S_.numCols = R_.numCols;
        arm_mat_add_f32(&temp_mat2_, &R_, &S_); // + R

        arm_mat_inverse_f32(&S_, &temp_mat2_); // temp_mat2_ = inv(S) (3x3)

        // ---- 2. 残差 r = z − h(xhat⁻) ----
        float q0 = xhatminus_data_[0];
        float q1 = xhatminus_data_[1];
        float q2 = xhatminus_data_[2];
        float q3 = xhatminus_data_[3];

        temp_vec1_.numRows = 3;
        temp_vec1_.numCols = 1;
        // 预测的重力方向(由先验四元数得到)
        tempv1_data_[0] = 2.0f * (q1 * q3 - q0 * q2);
        tempv1_data_[1] = 2.0f * (q0 * q1 + q2 * q3);
        tempv1_data_[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        // 各轴方向余弦(用于零偏增益修正)
        for (uint8_t i = 0; i < 3; ++i)
            orientation_cosine_[i] = acosf(fabsf(tempv1_data_[i]));

        temp_vec2_.numRows = z_.numRows;
        temp_vec2_.numCols = 1;
        arm_mat_sub_f32(&z_, &temp_vec1_, &temp_vec2_); // r (3x1)

        // ---- 3. 卡方统计量 χ² = rᵀ·inv(S)·r ----
        temp_mat1_.numRows = temp_vec2_.numRows;
        temp_mat1_.numCols = 1;
        arm_mat_mult_f32(&temp_mat2_, &temp_vec2_, &temp_mat1_); // inv(S)·r (3x1)

        temp_vec1_.numRows = 1;
        temp_vec1_.numCols = temp_vec2_.numRows;
        arm_mat_trans_f32(&temp_vec2_, &temp_vec1_); // rᵀ (1x3)

        arm_mat_mult_f32(&temp_vec1_, &temp_mat1_, &chi_square_mat_); // χ² (1x1)
        chi_square_ = chi_square_data_[0];

        // ---- 4. 卡方检验: 收敛判定 / 异常量测剔除 / 发散保护 ----
        if (chi_square_ < 0.5f * kChiSquareThreshold)
        {
            converge_flag_ = true; // 残差很小,滤波器已收敛
        }

        if (chi_square_ > kChiSquareThreshold && converge_flag_)
        {
            if (stable_flag_)
                error_count_++; // 静止时仍无法通过卡方检验
            else
                error_count_ = 0;

            if (error_count_ > 50)
            {
                // 滤波器发散:强制执行量测更新以恢复
                converge_flag_ = false;
                skip_eq5_ = false;
            }
            else
            {
                // 剔除本次量测,仅预测: xhat = xhat⁻, P = P⁻
                memcpy(xhat_data_, xhatminus_data_, sizeof(xhat_data_));
                memcpy(P_data_, Pminus_data_, sizeof(P_data_));
                skip_eq5_ = true;
                return;
            }
        }
        else
        {
            // 增益自适应: χ² 越小增益越大,越相信量测;反之相信预测
            if (chi_square_ > 0.1f * kChiSquareThreshold && converge_flag_)
                adaptive_gain_scale_ = (kChiSquareThreshold - chi_square_) / (0.9f * kChiSquareThreshold);
            else
                adaptive_gain_scale_ = 1.0f;
            error_count_ = 0;
            skip_eq5_ = false;
        }

        // ---- 5. 卡尔曼增益 K = P⁻·Hᵀ·inv(S) ----
        temp_mat1_.numRows = Pminus_.numRows;
        temp_mat1_.numCols = HT_.numCols;
        arm_mat_mult_f32(&Pminus_, &HT_, &temp_mat1_); // P⁻·Hᵀ (6x3)
        arm_mat_mult_f32(&temp_mat1_, &temp_mat2_, &K_); // K (6x3)

        // 增益自适应缩放
        for (uint8_t i = 0; i < StateDim * MeasureDim; ++i)
            K_data_[i] *= adaptive_gain_scale_;

        // 零偏行的增益按对应轴方向余弦缩放(重力方向附近的轴零偏更可信)
        for (uint8_t i = 4; i < 6; ++i)
        {
            for (uint8_t j = 0; j < 3; ++j)
                K_data_[i * MeasureDim + j] *= orientation_cosine_[i - 4] / 1.5707963f; // 1 rad
        }

        // ---- 6. 校正量 dx = K·r ----
        temp_vec1_.numRows = K_.numRows;
        temp_vec1_.numCols = 1;
        arm_mat_mult_f32(&K_, &temp_vec2_, &temp_vec1_); // dx (6x1)

        // 收敛后对零偏校正限幅(正常漂移不会太大)
        if (converge_flag_)
        {
            for (uint8_t i = 4; i < 6; ++i)
            {
                const float limit = 1e-2f * dt_;
                if (tempv1_data_[i] > limit)
                    tempv1_data_[i] = limit;
                if (tempv1_data_[i] < -limit)
                    tempv1_data_[i] = -limit;
            }
        }

        // 不修正与 yaw 耦合的四元数第 4 分量(yaw 不可观)
        tempv1_data_[3] = 0.0f;

        arm_mat_add_f32(&xhatminus_, &temp_vec1_, &xhat_);
    }

    // ==================== 后处理 ====================

    void QuaternionEKF::PostProcess()
    {
        // 1. 取滤波结果(四元数 + xy 零偏)
        q_[0] = filtered_data_[0];
        q_[1] = filtered_data_[1];
        q_[2] = filtered_data_[2];
        q_[3] = filtered_data_[3];
        gyro_bias_[0] = filtered_data_[4];
        gyro_bias_[1] = filtered_data_[5];
        gyro_bias_[2] = 0.0f; // z 轴通常通天,零偏不可观

        // 2. 四元数归一化(NaN/退化兜底)
        float sumsq = q_[0] * q_[0] + q_[1] * q_[1] + q_[2] * q_[2] + q_[3] * q_[3];
        if (!std::isfinite(sumsq) || sumsq <= 1e-12f)
        {
            q_[0] = 1.0f;
            q_[1] = q_[2] = q_[3] = 0.0f;
        }
        else
        {
            const float inv = 1.0f / sqrtf(sumsq);
            q_[0] *= inv;
            q_[1] *= inv;
            q_[2] *= inv;
            q_[3] *= inv;
        }

        // 3. 欧拉角(异常值保持上一拍)
        const float prev_yaw = yaw_;
        const float prev_pitch = pitch_;
        const float prev_roll = roll_;
        const float prev_total = yaw_total_;

        float yaw = atan2f(2.0f * (q_[0] * q_[3] + q_[1] * q_[2]),
                           2.0f * (q_[0] * q_[0] + q_[1] * q_[1]) - 1.0f) *
                    kDegPerRad;
        float pitch = atan2f(2.0f * (q_[0] * q_[1] + q_[2] * q_[3]),
                             2.0f * (q_[0] * q_[0] + q_[3] * q_[3]) - 1.0f) *
                      kDegPerRad;
        float roll;
        {
            float sin_roll = -2.0f * (q_[1] * q_[3] - q_[0] * q_[2]);
            sin_roll = fminf(1.0f, fmaxf(-1.0f, sin_roll));
            roll = asinf(sin_roll) * kDegPerRad;
        }

        if (!std::isfinite(yaw))
            yaw = prev_yaw;
        if (!std::isfinite(pitch))
            pitch = prev_pitch;
        if (!std::isfinite(roll))
            roll = prev_roll;

        yaw_ = yaw;
        pitch_ = pitch;
        roll_ = roll;

        // 4. 累计 yaw(处理 ±180° 跳变,方便小陀螺等连续角度应用)
        if (yaw_ - yaw_last_ > 180.0f)
            yaw_round_count_--;
        else if (yaw_ - yaw_last_ < -180.0f)
            yaw_round_count_++;

        float yaw_total = 360.0f * yaw_round_count_ + yaw_;
        if (!std::isfinite(yaw_total))
            yaw_total = prev_total;

        yaw_total_ = yaw_total;
        yaw_last_ = yaw_;

        update_count_++;
    }

} // namespace algo
