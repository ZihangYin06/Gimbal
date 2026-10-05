#ifndef _IMU_HPP_
#define _IMU_HPP_

#include "bsp_bmi088.h"

class IMU
{
public:
    IMU() = default;

    /**
     * @brief 初始化 IMU
     * @param hspi       SPI 句柄
     * @param calibrate  是否执行自动零偏校准
     * @return true 成功，false 失败(错误码见 GetInitError)
     */
    bool Init(SPI_HandleTypeDef *hspi, bool calibrate);

    /**
     * @brief 读取最新传感器数据（更新内部缓存）
     */
    void Update(void);

    /**
     * @brief 最近一次初始化的错误码(BSP_BMI088_Init 返回值)
     *        0=成功; 0x01=加速度计ID失败; 0x02=陀螺仪ID失败; 0x03=都失败
     */
    uint8_t GetInitError(void) const { return init_err_; }

    // ---- 物理量获取接口 ----
    float GetAccelX(void) const { return raw_.Accel[0]; }
    float GetAccelY(void) const { return raw_.Accel[1]; }
    float GetAccelZ(void) const { return raw_.Accel[2]; }

    float GetGyroX(void) const { return raw_.Gyro[0]; }
    float GetGyroY(void) const { return raw_.Gyro[1]; }
    float GetGyroZ(void) const { return raw_.Gyro[2]; }

    float GetTemperature(void) const { return raw_.Temperature; }

    /**
     * @brief 获取原始数据结构体指针（用于批量读取）
     */
    const BSP_IMU_Data_t *GetRawDataPtr(void) const { return &raw_; }

    /**
     * @brief 获取当前 IMU 是否已初始化
     */
    bool IsReady(void) const { return ready_; }

private:
    BSP_IMU_Data_t raw_;
    uint8_t init_err_ = 0;
    bool ready_ = false;
};

#endif
