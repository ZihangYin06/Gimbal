/**
 * @file  KalmanFilter.hpp
 * @brief 通用离散卡尔曼滤波框架(黄金五式),C++ 模板实现
 *
 * 设计要点:
 *  1. 全静态存储 —— 所有矩阵空间随模板维度在编译期确定,
 *     不使用堆(无 malloc/pvPortMalloc),可直接作为成员或静态对象;
 *  2. 虚函数钩子 —— 五个环节前后均留有扩展点,派生类按需重写即可
 *     实现 EKF / 带卡方检验的自适应滤波等(见 QuaternionEKF);
 *  3. 矩阵运算基于 CMSIS-DSP(arm_math),FPU 硬件加速。
 *
 * 系统模型:
 *   状态方程: x(k) = F·x(k-1) + B·u(k) + w,  w ~ N(0, Q)
 *   量测方程: z(k) = H·x(k) + v,             v ~ N(0, R)
 *
 * 黄金五式:
 *   1. 先验状态:   xhat⁻ = F·xhat + B·u
 *   2. 先验协方差: P⁻    = F·P·Fᵀ + Q
 *   3. 卡尔曼增益: K     = P⁻·Hᵀ·(H·P⁻·Hᵀ + R)⁻¹
 *   4. 后验状态:   xhat  = xhat⁻ + K·(z − H·xhat⁻)
 *   5. 后验协方差: P     = (I − K·H)·P⁻ = P⁻ − K·H·P⁻
 *
 * 使用方法(直接作普通 KF 用):
 * @code
 *   algo::KalmanFilter<2, 0, 2> kf;          // 二维状态、无量测控制
 *   kf.SetF(F2x2);  kf.SetQ(...);  kf.SetRDiagonal(...);  kf.SetH(...);
 *   memcpy(kf.Measurement(), z, sizeof(z));
 *   const float *x = kf.Update();
 * @endcode
 */
#ifndef _ALGO_KALMAN_FILTER_HPP_
#define _ALGO_KALMAN_FILTER_HPP_

#include "arm_math.h"

namespace algo
{
    using Mat32 = arm_matrix_instance_f32;

    template <uint8_t Nx, uint8_t Nu, uint8_t Nz>
    class KalmanFilter
    {
    public:
        static_assert(Nx >= 1 && Nz >= 1, "KalmanFilter: 状态维数与量测维数必须 >= 1");

        static constexpr uint8_t StateDim = Nx;   // 状态维数
        static constexpr uint8_t ControlDim = Nu; // 控制维数
        static constexpr uint8_t MeasureDim = Nz; // 量测维数

        KalmanFilter()
        {
            InitMatrixHandles();
            ZeroAll();
        }

        virtual ~KalmanFilter() = default;

        /**
         * @brief 复位滤波器:状态与协方差清零后回调 OnReset() 供派生类设初值
         */
        virtual void Reset()
        {
            memset(xhat_data_, 0, sizeof(xhat_data_));
            memset(xhatminus_data_, 0, sizeof(xhatminus_data_));
            memset(filtered_data_, 0, sizeof(filtered_data_));
            memset(P_data_, 0, sizeof(P_data_));
            memset(Pminus_data_, 0, sizeof(Pminus_data_));
            OnReset();
        }

        /**
         * @brief 执行一步滤波(黄金五式)
         * @return 滤波后的状态向量(长度 Nx,内部存储,勿手动释放)
         */
        const float *Update()
        {
            // 0. 量测预处理(装填 z、按需更新 F/Q/R 等)
            OnMeasure();

            // 1. 先验状态: xhat⁻ = F·xhat + B·u
            if (!skip_eq1_)
                PredictState();

            // 钩子:先验状态后(F 线性化、渐消因子等)
            OnPredict();

            // 2. 先验协方差: P⁻ = F·P·Fᵀ + Q
            if (!skip_eq2_)
                PredictCovariance();

            // 钩子:协方差预测后(观测雅可比 H 计算等)
            OnPreCorrect();

            if (HasMeasurement())
            {
                if (skip_eq3_ || skip_eq4_)
                {
                    // 自定义校正(替代式 3+4,如 EKF),内部可修改 skip_eq5_
                    OnCorrect();
                }
                else
                {
                    // 3. 卡尔曼增益: K = P⁻·Hᵀ·(H·P⁻·Hᵀ+R)⁻¹
                    ComputeGain();
                    // 4. 后验状态: xhat = xhat⁻ + K·(z − H·xhat⁻)
                    CorrectState();
                }

                // 5. 后验协方差: P = P⁻ − K·H·P⁻
                if (!skip_eq5_)
                    UpdateCovariance();
            }
            else
            {
                // 无有效量测:仅预测
                memcpy(xhat_data_, xhatminus_data_, sizeof(xhat_data_));
                memcpy(P_data_, Pminus_data_, sizeof(P_data_));
            }

            // 钩子:校正完成后(后处理)
            OnUpdate();

            // 抑制方差过度收敛
            for (uint8_t i = 0; i < Nx; ++i)
            {
                if (P_data_[i * Nx + i] < min_variance_[i])
                    P_data_[i * Nx + i] = min_variance_[i];
            }

            memcpy(filtered_data_, xhat_data_, sizeof(filtered_data_));
            return filtered_data_;
        }

        // ==================== 数据装填与访问 ====================

        /// 量测向量 z(Update 前装填)
        float *Measurement() { return z_data_; }
        /// 控制向量 u(Nu == 0 时为 nullptr 语义,勿使用)
        float *Control() { return u_data_; }
        /// 当前状态估计(最近一次 Update 的结果,与 Update 返回值相同)
        const float *State() const { return filtered_data_; }

        void SetF(const float *f) { memcpy(F_data_, f, sizeof(F_data_)); }
        void SetQ(const float *q) { memcpy(Q_data_, q, sizeof(Q_data_)); }
        void SetH(const float *h) { memcpy(H_data_, h, sizeof(H_data_)); }
        void SetB(const float *b)
        {
            if (Nu > 0)
                memcpy(B_data_, b, sizeof(float) * Nx * Nu);
        }
        /// 只装填 R 的对角线(量测互相独立时够用)
        void SetRDiagonal(const float *r_diag)
        {
            memset(R_data_, 0, sizeof(R_data_));
            for (uint8_t i = 0; i < Nz; ++i)
                R_data_[i * Nz + i] = r_diag[i];
        }
        /// 状态 i 的最小方差(防止滤波器过度收敛),传 0 关闭
        void SetMinVariance(uint8_t i, float variance) { min_variance_[i] = variance; }

        // 跳过标准环节(扩展为 EKF 时,替换式 3/4 需同时置位)
        void SkipStatePredict(bool skip) { skip_eq1_ = skip; } // 式 1
        void SkipCovPredict(bool skip) { skip_eq2_ = skip; }   // 式 2
        void SkipGainCompute(bool skip) { skip_eq3_ = skip; }  // 式 3
        void SkipStateCorrect(bool skip) { skip_eq4_ = skip; } // 式 4
        void SkipCovCorrect(bool skip) { skip_eq5_ = skip; }   // 式 5

    protected:
        // ==================== 扩展点(派生类按需重写) ====================

        virtual void OnReset() {}      ///< Reset 后:设置初值 xhat0/P0
        virtual void OnMeasure() {}    ///< 式 1 前:量测预处理/装填
        virtual void OnPredict() {}    ///< 式 1 后:如 EKF 的 F 阵线性化、渐消因子
        virtual void OnPreCorrect() {} ///< 式 2 后、式 3 前:如 EKF 的观测雅可比 H
        virtual void OnCorrect() {}    ///< 替代式 3+4 的自定义校正(需 Skip 式 3/4)
        virtual void OnUpdate() {}     ///< 式 5 后:后处理
        /// 当前是否有有效量测(默认有;无则本步退化为纯预测)
        virtual bool HasMeasurement() const { return true; }

        // ==================== 黄金五式(供派生类复用) ====================

        void PredictState()
        {
            if (Nu > 0)
            {
                temp_vec1_.numRows = Nx;
                temp_vec1_.numCols = 1;
                status_ = arm_mat_mult_f32(&F_, &xhat_, &temp_vec1_); // F·xhat
                temp_vec2_.numRows = Nx;
                temp_vec2_.numCols = 1;
                status_ = arm_mat_mult_f32(&B_, &u_, &temp_vec2_); // B·u
                status_ = arm_mat_add_f32(&temp_vec1_, &temp_vec2_, &xhatminus_);
            }
            else
            {
                status_ = arm_mat_mult_f32(&F_, &xhat_, &xhatminus_);
            }
        }

        void PredictCovariance()
        {
            status_ = arm_mat_trans_f32(&F_, &FT_);
            status_ = arm_mat_mult_f32(&F_, &P_, &Pminus_); // F·P
            temp_mat1_.numRows = Pminus_.numRows;
            temp_mat1_.numCols = FT_.numCols;
            status_ = arm_mat_mult_f32(&Pminus_, &FT_, &temp_mat1_); // F·P·Fᵀ
            status_ = arm_mat_add_f32(&temp_mat1_, &Q_, &Pminus_);   // + Q
        }

        void ComputeGain()
        {
            status_ = arm_mat_trans_f32(&H_, &HT_);
            temp_mat1_.numRows = H_.numRows;
            temp_mat1_.numCols = Pminus_.numCols;
            status_ = arm_mat_mult_f32(&H_, &Pminus_, &temp_mat1_); // H·P⁻
            temp_mat2_.numRows = temp_mat1_.numRows;
            temp_mat2_.numCols = HT_.numCols;
            status_ = arm_mat_mult_f32(&temp_mat1_, &HT_, &temp_mat2_); // H·P⁻·Hᵀ
            S_.numRows = R_.numRows;
            S_.numCols = R_.numCols;
            status_ = arm_mat_add_f32(&temp_mat2_, &R_, &S_); // + R
            status_ = arm_mat_inverse_f32(&S_, &temp_mat2_);  // inv(S)
            temp_mat1_.numRows = Pminus_.numRows;
            temp_mat1_.numCols = HT_.numCols;
            status_ = arm_mat_mult_f32(&Pminus_, &HT_, &temp_mat1_);   // P⁻·Hᵀ
            status_ = arm_mat_mult_f32(&temp_mat1_, &temp_mat2_, &K_); // K
        }

        void CorrectState()
        {
            temp_vec1_.numRows = H_.numRows;
            temp_vec1_.numCols = 1;
            status_ = arm_mat_mult_f32(&H_, &xhatminus_, &temp_vec1_); // H·xhat⁻
            temp_vec2_.numRows = z_.numRows;
            temp_vec2_.numCols = 1;
            status_ = arm_mat_sub_f32(&z_, &temp_vec1_, &temp_vec2_); // z − H·xhat⁻
            temp_vec1_.numRows = K_.numRows;
            temp_vec1_.numCols = 1;
            status_ = arm_mat_mult_f32(&K_, &temp_vec2_, &temp_vec1_); // K·(...)
            status_ = arm_mat_add_f32(&xhatminus_, &temp_vec1_, &xhat_);
        }

        void UpdateCovariance()
        {
            temp_mat1_.numRows = K_.numRows;
            temp_mat1_.numCols = H_.numCols;
            status_ = arm_mat_mult_f32(&K_, &H_, &temp_mat1_); // K·H
            temp_mat2_.numRows = temp_mat1_.numRows;
            temp_mat2_.numCols = Pminus_.numCols;
            status_ = arm_mat_mult_f32(&temp_mat1_, &Pminus_, &temp_mat2_); // K·H·P⁻
            status_ = arm_mat_sub_f32(&Pminus_, &temp_mat2_, &P_);
        }

        // ==================== 成员数据(派生类可直接访问) ====================

        static constexpr uint16_t kMaxSq = (Nx * Nx > Nz * Nz) ? static_cast<uint16_t>(Nx * Nx) : static_cast<uint16_t>(Nz * Nz);
        static constexpr uint16_t kMaxDim = (Nx > Nz) ? static_cast<uint16_t>(Nx) : static_cast<uint16_t>(Nz);

        // 状态与结果
        float xhat_data_[Nx];      // 后验状态 xhat
        float xhatminus_data_[Nx]; // 先验状态 xhat⁻
        float filtered_data_[Nx];  // Update() 的返回缓存

        // 量测与控制
        float z_data_[Nz];
        float u_data_[Nu > 0 ? Nu : 1];
        float B_data_[Nu > 0 ? Nu * Nx : 1];

        // 主要矩阵
        float P_data_[Nx * Nx];
        float Pminus_data_[Nx * Nx];
        float F_data_[Nx * Nx];
        float FT_data_[Nx * Nx];
        float Q_data_[Nx * Nx];
        float H_data_[Nz * Nx];
        float HT_data_[Nx * Nz];
        float R_data_[Nz * Nz];
        float K_data_[Nx * Nz];

        // 公共中间量(派生类的自定义校正可直接复用)
        float S_data_[kMaxSq];       // 新息协方差 S = H·P⁻·Hᵀ + R
        float temp1_data_[kMaxSq];   // 中间矩阵 1
        float temp2_data_[kMaxSq];   // 中间矩阵 2
        float tempv1_data_[kMaxDim]; // 中间向量 1
        float tempv2_data_[kMaxDim]; // 中间向量 2

        float min_variance_[Nx]; // 对角方差下限(防过度收敛)

        // CMSIS 矩阵句柄
        Mat32 xhat_, xhatminus_, z_, u_, B_;
        Mat32 P_, Pminus_, F_, FT_, Q_, H_, HT_, R_, K_, S_;
        Mat32 temp_mat1_, temp_mat2_, temp_vec1_, temp_vec2_;

        arm_status status_ = ARM_MATH_SUCCESS;

        // 环节跳过标志
        bool skip_eq1_ = false;
        bool skip_eq2_ = false;
        bool skip_eq3_ = false;
        bool skip_eq4_ = false;
        bool skip_eq5_ = false;

    private:
        void InitMatrixHandles()
        {
            arm_mat_init_f32(&xhat_, Nx, 1, xhat_data_);
            arm_mat_init_f32(&xhatminus_, Nx, 1, xhatminus_data_);
            arm_mat_init_f32(&z_, Nz, 1, z_data_);
            arm_mat_init_f32(&u_, Nu > 0 ? Nu : 1, 1, u_data_);
            arm_mat_init_f32(&B_, Nx, Nu > 0 ? Nu : 1, B_data_);
            arm_mat_init_f32(&P_, Nx, Nx, P_data_);
            arm_mat_init_f32(&Pminus_, Nx, Nx, Pminus_data_);
            arm_mat_init_f32(&F_, Nx, Nx, F_data_);
            arm_mat_init_f32(&FT_, Nx, Nx, FT_data_);
            arm_mat_init_f32(&Q_, Nx, Nx, Q_data_);
            arm_mat_init_f32(&H_, Nz, Nx, H_data_);
            arm_mat_init_f32(&HT_, Nx, Nz, HT_data_);
            arm_mat_init_f32(&R_, Nz, Nz, R_data_);
            arm_mat_init_f32(&K_, Nx, Nz, K_data_);
            arm_mat_init_f32(&S_, kMaxSq > Nz ? Nz : kMaxSq, kMaxSq > Nz ? Nz : kMaxSq, S_data_);
            arm_mat_init_f32(&temp_mat1_, Nx, Nx, temp1_data_);
            arm_mat_init_f32(&temp_mat2_, Nx, Nx, temp2_data_);
            arm_mat_init_f32(&temp_vec1_, kMaxDim, 1, tempv1_data_);
            arm_mat_init_f32(&temp_vec2_, kMaxDim, 1, tempv2_data_);
        }

        void ZeroAll()
        {
            memset(xhat_data_, 0, sizeof(xhat_data_));
            memset(xhatminus_data_, 0, sizeof(xhatminus_data_));
            memset(filtered_data_, 0, sizeof(filtered_data_));
            memset(z_data_, 0, sizeof(z_data_));
            memset(u_data_, 0, sizeof(u_data_));
            memset(B_data_, 0, sizeof(B_data_));
            memset(P_data_, 0, sizeof(P_data_));
            memset(Pminus_data_, 0, sizeof(Pminus_data_));
            memset(F_data_, 0, sizeof(F_data_));
            memset(FT_data_, 0, sizeof(FT_data_));
            memset(Q_data_, 0, sizeof(Q_data_));
            memset(H_data_, 0, sizeof(H_data_));
            memset(HT_data_, 0, sizeof(HT_data_));
            memset(R_data_, 0, sizeof(R_data_));
            memset(K_data_, 0, sizeof(K_data_));
            memset(S_data_, 0, sizeof(S_data_));
            memset(temp1_data_, 0, sizeof(temp1_data_));
            memset(temp2_data_, 0, sizeof(temp2_data_));
            memset(tempv1_data_, 0, sizeof(tempv1_data_));
            memset(tempv2_data_, 0, sizeof(tempv2_data_));
            memset(min_variance_, 0, sizeof(min_variance_));
        }
    };

} // namespace algo

#endif // _ALGO_KALMAN_FILTER_HPP_
