/**
 ******************************************************************************
 * @file    bsp_dr16.h
 * @brief   大疆 DT7 遥控器接收机驱动（DR16 协议，USART3 + DMA 空闲中断接收）
 *
 * 数据流：DT7 接收机 -> USART3 -> DMA 空闲事件 -> HAL_UARTEx_RxEventCallback
 *       -> DR16_Decode() 解析 18 字节帧 -> dr16_remote 全局结构体
 * 协议帧：18 字节 = 4 x 11bit 通道 + 2 x 2bit 拨杆 + 拨轮 2 字节
 ******************************************************************************
 */
#ifndef BSP_DR16_H
#define BSP_DR16_H

#include <stdint.h>
#include "usart.h"
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 通道原始值范围（DT7 出厂标定），中位为 1024 */
#define DR16_CH_MIN 364
#define DR16_CH_MAX 1684
#define DR16_CH_MID 1024

/* 判定遥控器在线的最长间隔：超过该时长未收到完整帧视为离线 */
#define DR16_ONLINE_TIMEOUT_MS 100U

typedef struct {
    uint16_t ch0;       /* 右手横向通道原始值 364~1684，中位 1024 */
    uint16_t ch1;       /* 右手纵向通道 */
    uint16_t ch2;       /* 左手纵向通道 */
    uint16_t ch3;       /* 左手横向通道 */
    uint8_t  s1;        /* 右侧拨杆：1=上 2=下 3=中 */
    uint8_t  s2;        /* 左侧拨杆：1=上 2=下 3=中 */
    uint16_t wheel;     /* 拨轮原始值，中位 1024（与通道同范围） */
    uint8_t  valid;     /* 最近一帧校验结果：0=无效 1=有效 */
} DR16_Remote_t;

/* 解析结果全局共享（中断里写，任务里读） */
extern DR16_Remote_t dr16_remote;

/**
 * @brief  启动遥控器接收（内部即开启一轮 DMA 空闲接收）
 */
void DR16_Init(void);

/**
 * @brief  重新开启一轮"DMA + 空闲中断"接收
 * @note   每收完一帧须重新调用；对外也暴露给错误恢复使用
 */
void DR16_StartDMA(void);

/**
 * @brief  查询遥控器是否在线（定期在任务里调用）
 * @retval 1=在线（DR16_ONLINE_TIMEOUT_MS 内收到过完整帧） 0=离线
 * @note   上电后未收到过任何帧时返回 0；
 *         注意 valid 只反映"最近一帧是否通过校验"，
 *         失联超时须用本函数判断
 */
uint8_t DR16_IsOnline(void);

#ifdef __cplusplus
}
#endif

#endif
