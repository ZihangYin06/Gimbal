#ifndef _BSP_BMI088_H_
#define _BSP_BMI088_H_

#include "BMI088reg.h"
#include "bsp_spi.h"
#include "bsp_dwt.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>     

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float Accel[3];      // 加速度 (m/s²)
    float Gyro[3];       // 角速度 (rad/s)
    float Temperature;   // 温度 (℃)
} BSP_IMU_Data_t;

uint8_t BSP_BMI088_Init(SPI_HandleTypeDef *hspi, bool calibrate);
void BSP_BMI088_Read(BSP_IMU_Data_t *data);
void BSP_BMI088_SetGyroOffset(float ox, float oy, float oz);
void BSP_BMI088_SetAccelScale(float scale);
void BSP_BMI088_GetGyroOffset(float *ox, float *oy, float *oz);

#ifdef __cplusplus
}
#endif

#endif
