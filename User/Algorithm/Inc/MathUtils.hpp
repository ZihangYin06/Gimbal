#ifndef _MATH_UTILS_HPP_
#define _MATH_UTILS_HPP_

#include <cmath>
#include <cstdint>

namespace Algorithm
{

    constexpr float MATH_PI = 3.141592653589793f;
    constexpr float RADIAN_COEF = 57.295779513f; // 180/PI

    // ========================== 限幅 ==========================
    template <typename T>
    inline T Constrain(T value, T min, T max)
    {
        return (value < min) ? min : (value > max) ? max
                                                   : value;
    }

    template <typename T>
    inline T AbsLimit(T value, T limit)
    {
        return (value > limit) ? limit : (value < -limit) ? -limit
                                                          : value;
    }

    // ========================== 死区 ==========================
    template <typename T>
    inline T Deadband(T value, T min, T max)
    {
        return (value > min && value < max) ? T(0) : value;
    }

    // ========================== 符号函数 ==========================
    template <typename T>
    inline T Sign(T value)
    {
        return (value >= T(0)) ? T(1) : T(-1);
    }

    // ========================== 循环限幅（角度格式化） ==========================
    inline float LoopConstrain(float value, float min, float max)
    {
        if (max <= min)
            return value;
        float range = max - min;
        while (value > max)
            value -= range;
        while (value < min)
            value += range;
        return value;
    }

    inline float RadFormat(float rad)
    {
        return LoopConstrain(rad, -MATH_PI, MATH_PI);
    }

    inline float DegFormat(float deg)
    {
        return LoopConstrain(deg, -180.0f, 180.0f);
    }

    // ========================== 快速平方根 ==========================
    inline float FastSqrt(float x)
    {
        return std::sqrt(x);
    }

    // ========================== 四舍五入 ==========================
    inline int Round(float value)
    {
        return static_cast<int>(std::round(value));
    }

    // ========================== 浮点比较 ==========================
    inline bool IsZero(float value, float epsilon = 1e-6f)
    {
        return std::fabs(value) < epsilon;
    }

} // namespace Algorithm

#endif