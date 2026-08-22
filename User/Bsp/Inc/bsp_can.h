#ifndef _BSP_CAN_H
#define _BSP_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "can.h"

void CAN1_Init(void);
void CAN2_Init(void);
void CANSend(CAN_HandleTypeDef* _phcan, uint32_t _idx, uint8_t* _data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif
