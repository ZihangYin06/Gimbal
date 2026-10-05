#ifndef _INSTASK_HPP_
#define _INSTASK_HPP_

#include "IMU.hpp"
#include "QuaternionEKF.hpp"

/**
 * @brief INS 姿态数据(输出结构体)
 *        包含姿态角、四元数、运动加速度等
 */
struct INS_Data_t
{
    // ---- 姿态数据 ----
    float q[4];          // 四元数
    float Roll;          // 横滚角 (度)
    float Pitch;         // 俯仰角 (度)
    float Yaw;           // 偏航角 (度,已叠加 yaw 零点偏移)
    float YawTotalAngle; // 累计偏航角 (度,连续不跳变)

    // ---- 传感器原始数据 ----
    float Gyro[3];          // 角速度原始值 (rad/s)
    float GyroCorrected[3]; // 去零偏角速度 (rad/s,控制环直接使用)
    float Accel[3];         // 加速度 (m/s²)

    // ---- 运动加速度 ----
    float MotionAccel_b[3]; // 机体坐标系运动加速度 (m/s²)
    float MotionAccel_n[3]; // 导航坐标系运动加速度 (m/s²)

    // ---- 方向余弦 ----
    float xn[3]; // 机体 X 轴在导航系中的方向向量
    float yn[3]; // 机体 Y 轴在导航系中的方向向量
    float zn[3]; // 机体 Z 轴在导航系中的方向向量

    // ---- 辅助变量 ----
    float atanxz; // 加速度 X-Z 夹角 (度)
    float atanyz; // 加速度 Y-Z 夹角 (度)

    // ---- 有效性 / EKF 状态 ----
    bool valid;         // 数据有效(IMU 已初始化且完成至少一次解算)
    bool ekf_converged; // EKF 卡方检验收敛标志
    bool ekf_stable;    // 运动稳定(角速度小且加速度模长正常)
    float chi_square;   // 最近一次卡方统计量(调试用)
};

/**
 * @brief INS 业务逻辑控制器(TASK 层)
 *        不包含任何 RTOS API,负责:IMU 读取 → 四元数 EKF 姿态解算 → 数据输出
 *        经任务级接口 INS_Init / INS_Update 由 APP 层调度调用
 */
class InsHandler
{
public:
    InsHandler() = default;
    ~InsHandler() = default;

    /**
     * @brief 初始化 INS 控制器
     * @param hspi      SPI 句柄(传递给 IMU 设备)
     * @param calibrate 是否执行 IMU 自动校准
     * @return true 成功,false 失败
     */
    bool Init(SPI_HandleTypeDef *hspi, bool calibrate);

    /**
     * @brief 更新 INS(业务流程核心)
     * @param dt 时间间隔(秒),由调用方传入
     */
    void Update(float dt);

    // ---- 数据获取接口 ----
    const INS_Data_t *GetData() const { return &data_; }
    const IMU *GetIMU() const { return &imu_; }
    const algo::QuaternionEKF *GetEKF() const { return &qekf_; }
    /// 最近一次 IMU 初始化错误码(0=成功; 0x01=加速度计ID失败; 0x02=陀螺仪ID失败)
    uint8_t GetIMUInitError() const { return imu_.GetInitError(); }

    // ---- 状态查询 ----
    bool IsReady() const { return ready_; }

    // ---- 云台应用接口 ----
    /**
     * @brief 把当前朝向设为 yaw 零点(云台对心/上电对正后调用)
     *        之后 GetData()->Yaw 相对该零点输出,YawTotalAngle 同步平移
     */
    void SetYawZero() { yaw_zero_offset_ = qekf_.Yaw(); }
    /// 清除 yaw 零点偏移,恢复绝对航向输出
    void ClearYawZero() { yaw_zero_offset_ = 0.0f; }
    /**
     * @brief 复位姿态解算(跌落/翻滚后调用):
     *        四元数回 [1,0,0,0],零点偏移清零,EKF 重新收敛
     */
    void ResetAttitude();

private:
    IMU imu_;                      // IMU 设备
    algo::QuaternionEKF qekf_;     // 四元数姿态 EKF(内含全部滤波器存储,静态分配)
    INS_Data_t data_;              // 姿态数据
    float yaw_zero_offset_ = 0.0f; // yaw 软件零点偏移 (度)

    bool ready_ = false;

    // 运动加速度低通系数
    float accel_lpf_ = 0.0085f;

    // 内部工具函数
    void UpdateMotionAccel(const float gravity_b[3], float dt);
    void UpdateOrientationVectors();
};

// ---- 任务级接口:由 APP 层(StartINSTask)调度调用 ----

/**
 * @brief 初始化 INS(IMU + EKF),同时维护 g_debug_ins_err 错误码
 * @return true 成功,false 失败(APP 层应延时后重试)
 */
bool INS_Init(SPI_HandleTypeDef *hspi, bool calibrate);

/**
 * @brief 执行一次姿态解算并刷新全局快照 INS(APP 层以 500Hz 调用)
 * @param dt 时间间隔(秒)
 */
void INS_Update(float dt);

/* 全局姿态快照:INS 任务以 500Hz 更新,控制任务直接读(如 INS.YawTotalAngle) */
extern volatile INS_Data_t INS;

/* 最近一次 IMU 初始化错误码: 0=正常; 0x01=加速度计ID失败; 0x02=陀螺仪ID失败; 0x03=都失败 */
extern volatile uint8_t g_debug_ins_err;

#ifdef __cplusplus
extern "C"
{
#endif

    /* C 接口:获取姿态数据指针 */
    const INS_Data_t *GetINSData(void);

#ifdef __cplusplus
}
#endif

#endif
