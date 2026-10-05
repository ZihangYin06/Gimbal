#ifndef _ALGORITHM_LEAST_SQUARES_HPP_
#define _ALGORITHM_LEAST_SQUARES_HPP_

#include <cstdint>
#include <cmath>
#include <cstring>
#include "cmsis_os.h" // FreeRTOS 内存管理

namespace Algorithm
{
    class LeastSquares
    {
    public:
        LeastSquares() = default;

        ~LeastSquares()
        {
            Deinit();
        }

        /**
         * @brief 初始化最小二乘法
         * @param order  样本数（至少 2）
         * @return true 成功，false 失败（内存不足）
         */
        bool Init(uint16_t order)
        {
            if (order < 2)
                return false;

            // 释放之前分配的内存
            Deinit();

            order_ = order;
            count_ = 0;

            x_ = (float *)pvPortMalloc(sizeof(float) * order);
            y_ = (float *)pvPortMalloc(sizeof(float) * order);

            if (x_ == nullptr || y_ == nullptr)
            {
                Deinit();
                return false;
            }

            std::memset(x_, 0, sizeof(float) * order);
            std::memset(y_, 0, sizeof(float) * order);

            k_ = 0.0f;
            b_ = 0.0f;
            std_dev_ = 0.0f;
            inited_ = true;

            return true;
        }

        /**
         * @brief 释放内存
         */
        void Deinit()
        {
            if (x_ != nullptr)
            {
                vPortFree(x_);
                x_ = nullptr;
            }
            if (y_ != nullptr)
            {
                vPortFree(y_);
                y_ = nullptr;
            }
            order_ = 0;
            count_ = 0;
            inited_ = false;
        }

        /**
         * @brief 添加新样本
         * @param deltax  x 增量（时间间隔）
         * @param y       新样本值
         */
        void Update(float deltax, float y)
        {
            if (!inited_ || order_ == 0)
                return;

            // 滑动窗口
            for (uint16_t i = 0; i < order_ - 1; ++i)
            {
                x_[i] = x_[i + 1] - x_[0];
                y_[i] = y_[i + 1];
            }
            x_[order_ - 1] = x_[order_ - 2] + deltax;
            y_[order_ - 1] = y;

            if (count_ < order_)
                count_++;

            // 计算统计量
            float t0 = 0.0f, t1 = 0.0f, t2 = 0.0f, t3 = 0.0f;
            uint16_t start = order_ - count_;
            for (uint16_t i = start; i < order_; ++i)
            {
                t0 += x_[i] * x_[i];
                t1 += x_[i];
                t2 += x_[i] * y_[i];
                t3 += y_[i];
            }

            float n = (float)count_;
            float denom = t0 * n - t1 * t1;
            if (std::fabs(denom) < 1e-12f)
            {
                k_ = 0.0f;
                b_ = 0.0f;
            }
            else
            {
                k_ = (t2 * n - t1 * t3) / denom;
                b_ = (t0 * t3 - t1 * t2) / denom;
            }

            // 标准差
            std_dev_ = 0.0f;
            for (uint16_t i = start; i < order_; ++i)
            {
                float err = k_ * x_[i] + b_ - y_[i];
                std_dev_ += std::fabs(err);
            }
            std_dev_ /= (count_ > 0) ? count_ : 1;
        }

        // ---- 获取结果 ----
        float GetDerivative() const { return k_; }
        float GetIntercept() const { return b_; }
        float GetSmoothed() const
        {
            if (!inited_ || count_ == 0 || order_ == 0)
                return 0.0f;
            return k_ * x_[order_ - 1] + b_;
        }
        float GetStdDev() const { return std_dev_; }
        bool IsInited() const { return inited_; }

    private:
        uint16_t order_ = 0;
        uint16_t count_ = 0;
        float *x_ = nullptr;
        float *y_ = nullptr;
        float k_ = 0.0f;
        float b_ = 0.0f;
        float std_dev_ = 0.0f;
        bool inited_ = false;
    };

}

#endif
