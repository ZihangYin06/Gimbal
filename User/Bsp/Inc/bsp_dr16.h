// bsp_dr16.h
#ifndef __BSP_DR16_H
#define __BSP_DR16_H

#include <stdint.h>
#include "usart.h" 
#include "string.h"  

#define DR16_CH_MIN  364
#define DR16_CH_MAX  1684
#define DR16_CH_MID  1024

typedef struct {
    uint16_t ch0;
    uint16_t ch1;
    uint16_t ch2;
    uint16_t ch3;
    uint8_t  s1;        // 1, 2, 3
    uint8_t  s2;        // 1, 2, 3
    uint16_t wheel;     // 拨轮值
    uint8_t  valid;     // 0=无效，1=有效
} DR16_Remote_t;

extern DR16_Remote_t dr16_remote;

void DR16_Init(void);        
void DR16_StartDMA(void);    

#endif
