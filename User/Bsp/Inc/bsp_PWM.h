#ifndef __BSP_IMU_PWM_H
#define __BSP_IMU_PWM_H

#include "stdint.h"
#include "tim.h"

#ifdef __cplusplus
extern "C" {
#endif

void TIM_Set_PWM(TIM_HandleTypeDef *tim_pwmHandle, uint8_t Channel, uint16_t value);

#ifdef __cplusplus
}
#endif

#endif