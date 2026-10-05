/**
 * @file  QuaternionEKF.hpp
 * @brief 四元数姿态解算 EKF(基于通用 KalmanFilter 框架)
 *
 * 状态向量(6 维):
 *   x = [q0, q1, q2, q3, bx, by]ᵀ
 *   q      —— 姿态四元数(机体系 → 导航系)
 *   bx, by —— 陀螺仪 x/y 轴零偏估计(z 轴与重力平行,通常不可观,不估计)
 *
 * 量测向量(3 维):
 *   z = 归一化加速度向量(近似导航系重力方向在机体系下的投影)
 *
 * 在通用 KF 黄金五式基础上的扩展:
 *   - 预测阶段: 先验四元数归一化 + F 阵 4x2 线性化分块 + 零偏方差渐消/限幅
 *   - 校正阶段: 卡方检验剔除异常量测(撞击/甩动保护) + 增益自适应
 *               + 零偏校正限幅 + 不修正与 yaw 耦合的四元数分量
 */
#ifndef _ALGO_QUATERNION_EKF_HPP_
#define _ALGO_QUATERNION_EKF_HPP_

#include "KalmanFilter.hpp"

namespace algo
{
    class QuaternionEKF : public KalmanFilter<6, 0, 3>
    {
    public:
        struct Params
        {
            float process_noise_q = 10.0f;    // Q1: 四元数过程噪声
            float process_noise_bias = 0.001f; // Q2: 陀螺零偏过程噪声
            float measure_noise_acc = 1e7f;   // R: 加速度计量测噪声
            float fading_factor = 1.0f;       // 渐消因子 λ(0.9996 表示缓慢渐消;≥1 不渐消)
            float acc_lpf_tau = 0.0085f;      // 加速度低通时间常数 s(0 表示不滤波)
        };

        /// 卡方检验阈值(残差加权范数超过它则剔除本次量测)
        static constexpr float kChiSquareThreshold = 1e-8f;
        static constexpr float kDegPerRad = 57.295779513f;

        /**
         * @brief 初始化(须在首次 Update 前调用;未调用时首次 Update 用默认参数兜底)
         */
        void Init(const Params &params);
        void Init() { Init(Params{}); }

        /**
         * @brief 姿态解算一步
         * @param gx,gy,gz 陀螺角速度 rad/s(机体系)
         * @param ax,ay,az 加速度 m/s²(机体系,静止时 ≈ +g 方向)
         * @param dt       更新周期 s
         */
        void Update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

        // ---------- 解算结果 ----------
        const float *Quaternion() const { return q_; }             // 归一化四元数
        float Yaw() const { return yaw_; }                         // deg
        float Pitch() const { return pitch_; }                     // deg
        float Roll() const { return roll_; }                       // deg
        float YawTotalAngle() const { return yaw_total_; }         // 连续累计 yaw,deg
        const float *GyroBias() const { return gyro_bias_; }       // 估计零偏 rad/s(z 恒 0)
        const float *GyroCorrected() const { return gyro_; }       // 去零偏角速度 rad/s(供控制环)
        const float *AccelFiltered() const { return accel_; }      // 低通后的加速度 m/s²
        bool IsConverged() const { return converge_flag_; }        // 卡方检验收敛标志
        bool IsStable() const { return stable_flag_; }             // 角速度小且加速度模长正常
        float ChiSquare() const { return chi_square_; }            // 最近一次卡方统计量
        uint64_t UpdateCount() const { return update_count_; }     // 更新次数

    protected:
        void OnReset() override;
        void OnPredict() override;    ///< 先验归一化 + F 线性化 + P 渐消
        void OnPreCorrect() override; ///< 观测雅可比 H
        void OnCorrect() override;    ///< 卡方检验 + 自适应增益 + 自定义校正

    private:
        void Prepare(float gx, float gy, float gz, float ax, float ay, float az, float dt);
        void PostProcess();

        Params params_{};
        bool initialized_ = false;
        float dt_ = 0.0f;

        // 传感器数据(去零偏/滤波后)
        float gyro_[3] = {0};    // 去零偏角速度 rad/s
        float accel_[3] = {0};   // 低通滤波后加速度 m/s²
        float accel_raw_[3] = {0};
        float gyro_norm_ = 0.0f;
        float accel_norm_ = 0.0f;
        bool stable_flag_ = false;

        // 解算输出
        float q_[4] = {1.0f, 0.0f, 0.0f, 0.0f};
        float gyro_bias_[3] = {0};
        float yaw_ = 0.0f;
        float pitch_ = 0.0f;
        float roll_ = 0.0f;
        float yaw_total_ = 0.0f;
        float yaw_last_ = 0.0f;
        int16_t yaw_round_count_ = 0;

        // 收敛/自适应状态
        bool converge_flag_ = false;
        float chi_square_ = 0.0f;
        float adaptive_gain_scale_ = 1.0f;
        float orientation_cosine_[3] = {0}; // 加速度预测方向与三轴的夹角 rad
        uint64_t error_count_ = 0;
        uint64_t update_count_ = 0;

        // 卡方统计量(1x1)
        Mat32 chi_square_mat_{};
        float chi_square_data_[1] = {0};

        // F 阵单位模板(每步在其上写入 4x4 时变分块)
        static constexpr float kFIdentity[36] = {
            1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1};

        // P 阵初值: 四元数大不确定 + 零偏 100
        static constexpr float kPInit[36] = {
            100000, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f,
            0.1f, 100000, 0.1f, 0.1f, 0.1f, 0.1f,
            0.1f, 0.1f, 100000, 0.1f, 0.1f, 0.1f,
            0.1f, 0.1f, 0.1f, 100000, 0.1f, 0.1f,
            0.1f, 0.1f, 0.1f, 0.1f, 100, 0.1f,
            0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 100};
    };

} // namespace algo

#endif // _ALGO_QUATERNION_EKF_HPP_
