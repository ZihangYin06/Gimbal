#include "IMU.hpp"

bool IMU::Init(SPI_HandleTypeDef *hspi, bool calibrate)
{
    init_err_ = BSP_BMI088_Init(hspi, calibrate);
    ready_ = (init_err_ == 0);
    return ready_;
}

void IMU::Update(void)
{
    if (ready_)
    {
        BSP_BMI088_Read(&raw_);
    }
}