#include "INSTask.hpp"
#include "MathUtils.hpp"
#include "QuaternionUtils.hpp"
#include <cstring>
#include <cmath>

// ========================== 模块状态 ==========================
static InsHandler g_ins_handler;

// 全局姿态快照:INS_Update 每周期刷新,控制任务直接读(声明见 INSTask.hpp)
volatile INS_Data_t INS = {0};
// 最近一次 IMU 初始化错误码(声明见 INSTask.hpp)
volatile uint8_t g_debug_ins_err = 0;

// ========================== 常量 ==========================
static const float GRAVITY[3] = {0.0f, 0.0f, 9.81f};
static const float X_AXIS[3] = {1.0f, 0.0f, 0.0f};
static const float Y_AXIS[3] = {0.0f, 1.0f, 0.0f};
static const float Z_AXIS[3] = {0.0f, 0.0f, 1.0f};

// ========================== 任务级接口 ==========================
bool INS_Init(SPI_HandleTypeDef *hspi, bool calibrate)
{
    if (g_ins_handler.Init(hspi, calibrate))
    {
        g_debug_ins_err = 0;
        return true;
    }
    g_debug_ins_err = g_ins_handler.GetIMUInitError();
    return false;
}

void INS_Update(float dt)
{
    g_ins_handler.Update(dt);

    // 刷新全局快照,供控制任务直接读取
    memcpy(const_cast<INS_Data_t *>(&INS),
           g_ins_handler.GetData(), sizeof(INS_Data_t));
}

const INS_Data_t *GetINSData(void)
{
    return g_ins_handler.GetData();
}

// ========================== 初始化 ==========================
bool InsHandler::Init(SPI_HandleTypeDef *hspi, bool calibrate)
{
    // 1. 初始化 IMU 设备
    if (!imu_.Init(hspi, calibrate))
    {
        ready_ = false;
        return false;
    }

    // 2. 初始化四元数 EKF
    //    Q1=10, Q2=0.001, R=1e7, lambda=1(不渐消), 加速度低通 tau=0.0085s
    algo::QuaternionEKF::Params params;
    params.process_noise_q = 10.0f;
    params.process_noise_bias = 0.001f;
    params.measure_noise_acc = 10000000.0f;
    params.fading_factor = 1.0f;
    params.acc_lpf_tau = 0.0085f;
    qekf_.Init(params);

    // 3. 清空数据
    std::memset(&data_, 0, sizeof(data_));
    data_.q[0] = 1.0f;
    data_.valid = false;
    yaw_zero_offset_ = 0.0f;

    ready_ = true;
    return true;
}

// ========================== 核心更新 ==========================
void InsHandler::Update(float dt)
{
    if (!ready_)
        return;

    // ----- 1. 读取 IMU -----
    imu_.Update();

    // ----- 2. 拷贝原始数据 -----
    data_.Accel[algo::kX] = imu_.GetAccelX();
    data_.Accel[algo::kY] = imu_.GetAccelY();
    data_.Accel[algo::kZ] = imu_.GetAccelZ();
    data_.Gyro[algo::kX] = imu_.GetGyroX();
    data_.Gyro[algo::kY] = imu_.GetGyroY();
    data_.Gyro[algo::kZ] = imu_.GetGyroZ();

    // ----- 3. 计算加速度与重力的夹角(辅助变量)-----
    data_.atanxz = -std::atan2(data_.Accel[algo::kX], data_.Accel[algo::kZ]) * 180.0f / Algorithm::MATH_PI;
    data_.atanyz = std::atan2(data_.Accel[algo::kY], data_.Accel[algo::kZ]) * 180.0f / Algorithm::MATH_PI;

    // ----- 4. EKF 姿态解算 -----
    qekf_.Update(
        data_.Gyro[algo::kX], data_.Gyro[algo::kY], data_.Gyro[algo::kZ],
        data_.Accel[algo::kX], data_.Accel[algo::kY], data_.Accel[algo::kZ],
        dt);

    // ----- 5. 复制 EKF 结果 -----
    std::memcpy(data_.q, qekf_.Quaternion(), sizeof(data_.q));
    algo::NormalizeQuaternion(data_.q);
    std::memcpy(data_.GyroCorrected, qekf_.GyroCorrected(), sizeof(data_.GyroCorrected));

    // ----- 6. 更新方向余弦 -----
    UpdateOrientationVectors();

    // ----- 7. 计算运动加速度(扣除重力)-----
    float gravity_b[3];
    algo::EarthToBody(GRAVITY, gravity_b, data_.q);
    UpdateMotionAccel(gravity_b, dt);

    // ----- 8. 获取姿态角(带有效性检查,叠加 yaw 软件零点)-----
    data_.Yaw = std::isfinite(qekf_.Yaw())
                    ? Algorithm::DegFormat(qekf_.Yaw() - yaw_zero_offset_)
                    : data_.Yaw;
    // IMU 安装方向导致俯仰"向上为负",此处取反,统一为云台约定"向上为正"
    data_.Pitch = std::isfinite(qekf_.Pitch()) ? -qekf_.Pitch() : data_.Pitch;
    data_.Roll = std::isfinite(qekf_.Roll()) ? qekf_.Roll() : data_.Roll;
    data_.YawTotalAngle = std::isfinite(qekf_.YawTotalAngle())
                              ? qekf_.YawTotalAngle() - yaw_zero_offset_
                              : data_.YawTotalAngle;

    data_.ekf_converged = qekf_.IsConverged();
    data_.ekf_stable = qekf_.IsStable();
    data_.chi_square = qekf_.ChiSquare();
    data_.valid = true;
}

// ========================== 姿态复位 ==========================
void InsHandler::ResetAttitude()
{
    yaw_zero_offset_ = 0.0f;
    qekf_.Reset(); // 四元数回 [1,0,0,0],EKF 重新收敛
}

// ========================== 内部工具函数 ==========================
void InsHandler::UpdateOrientationVectors()
{
    algo::BodyToEarth(X_AXIS, data_.xn, data_.q);
    algo::BodyToEarth(Y_AXIS, data_.yn, data_.q);
    algo::BodyToEarth(Z_AXIS, data_.zn, data_.q);
}

void InsHandler::UpdateMotionAccel(const float gravity_b[3], float dt)
{
    float lpf = accel_lpf_;

    for (int i = 0; i < 3; i++)
    {
        float raw = data_.Accel[i] - gravity_b[i];
        data_.MotionAccel_b[i] = raw * dt / (lpf + dt) + data_.MotionAccel_b[i] * lpf / (lpf + dt);
    }

    algo::BodyToEarth(data_.MotionAccel_b, data_.MotionAccel_n, data_.q);
}
