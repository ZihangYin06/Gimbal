/**
 * @file  QuaternionUtils.hpp
 * @brief 四元数姿态变换工具(纯数学,无任何硬件依赖)
 *
 * 约定: 四元数 q = [w, x, y, z],表示 机体系 → 导航系 的旋转,
 *       与 QuaternionEKF 输出的四元数同一约定。
 */
#ifndef _ALGO_QUATERNION_UTILS_HPP_
#define _ALGO_QUATERNION_UTILS_HPP_

#include <cmath>
#include <cstdint>

namespace algo
{
    /// 机体/导航系坐标轴下标(用于 vector[3] 的可读索引)
    enum Axis : int
    {
        kX = 0,
        kY = 1,
        kZ = 2
    };

    /**
     * @brief 四元数归一化(零范数/非有限值时回退为单位四元数)
     */
    inline void NormalizeQuaternion(float q[4])
    {
        float norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        if (norm < 1e-12f || !std::isfinite(norm))
        {
            q[0] = 1.0f;
            q[1] = 0.0f;
            q[2] = 0.0f;
            q[3] = 0.0f;
            return;
        }
        float inv = 1.0f / norm;
        q[0] *= inv;
        q[1] *= inv;
        q[2] *= inv;
        q[3] *= inv;
    }

    /**
     * @brief 机体系向量 → 导航系向量 (v_e = R(q)·v_b)
     * @param b 机体系输入向量
     * @param e 导航系输出向量
     * @param q 四元数 [w,x,y,z]
     */
    inline void BodyToEarth(const float b[3], float e[3], const float q[4])
    {
        float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

        e[0] = 2.0f * ((0.5f - q2 * q2 - q3 * q3) * b[0] + (q1 * q2 - q0 * q3) * b[1] + (q1 * q3 + q0 * q2) * b[2]);

        e[1] = 2.0f * ((q1 * q2 + q0 * q3) * b[0] + (0.5f - q1 * q1 - q3 * q3) * b[1] + (q2 * q3 - q0 * q1) * b[2]);

        e[2] = 2.0f * ((q1 * q3 - q0 * q2) * b[0] + (q2 * q3 + q0 * q1) * b[1] + (0.5f - q1 * q1 - q2 * q2) * b[2]);
    }

    /**
     * @brief 导航系向量 → 机体系向量 (v_b = R(q)⁻¹·v_e)
     */
    inline void EarthToBody(const float e[3], float b[3], const float q[4])
    {
        float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

        b[0] = 2.0f * ((0.5f - q2 * q2 - q3 * q3) * e[0] + (q1 * q2 + q0 * q3) * e[1] + (q1 * q3 - q0 * q2) * e[2]);

        b[1] = 2.0f * ((q1 * q2 - q0 * q3) * e[0] + (0.5f - q1 * q1 - q3 * q3) * e[1] + (q2 * q3 + q0 * q1) * e[2]);

        b[2] = 2.0f * ((q1 * q3 + q0 * q2) * e[0] + (q2 * q3 - q0 * q1) * e[1] + (0.5f - q1 * q1 - q2 * q2) * e[2]);
    }

} // namespace algo

#endif // _ALGO_QUATERNION_UTILS_HPP_
