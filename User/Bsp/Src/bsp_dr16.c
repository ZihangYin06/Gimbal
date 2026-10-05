/**
 ******************************************************************************
 * @file    bsp_dr16.c
 * @brief   大疆 DT7 遥控器接收机驱动实现（DR16 协议解析）
 *
 * 18 字节帧位布局（小端字节序打包成 64 位位流后按位取出）：
 *   bit  0~10 : ch0    bit 11~21 : ch1
 *   bit 22~32 : ch2    bit 33~43 : ch3       （各 11 位，364~1684）
 *   bit 44~45 : s1     bit 46~47 : s2        （各 2 位，1=上 2=下 3=中）
 *   字节 16~17 : 拨轮（16 为低字节，17 为高字节，中位 1024）
 ******************************************************************************
 */
#include "bsp_dr16.h"

/* 一帧固定 18 字节 */
#define DR16_RX_BUF_LEN 18U

static uint8_t rx_buffer[DR16_RX_BUF_LEN] = {0};

DR16_Remote_t dr16_remote = {0};

/* 最近一次收到完整帧的系统 tick（0 = 上电至今未收到过帧） */
static uint32_t last_update_tick = 0;

void DR16_Init(void)
{
    DR16_StartDMA();
}

void DR16_StartDMA(void)
{
    /* 空闲事件 + DMA：一帧发完总线空闲即触发回调，无需逐字节中断 */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buffer, DR16_RX_BUF_LEN);
    /* 关闭半传输中断，一帧只进一次回调 */
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
}

uint8_t DR16_IsOnline(void)
{
    return (last_update_tick != 0U) &&
           ((HAL_GetTick() - last_update_tick) < DR16_ONLINE_TIMEOUT_MS);
}

static void DR16_Decode(void)
{
    uint64_t bit_stream = 0;
    uint16_t raw_ch0, raw_ch1, raw_ch2, raw_ch3;

    /* 只拼前 8 字节：解码只用到 bit 0..47，拨轮单独读字节 16/17。
     * 若拼满 18 字节，i>=8 时移位量 >=64 是未定义行为，
     * ARM 上按 mod 64 回卷会把 byte8 的开关值污染进 ch0 */
    for (int i = 0; i < 8; i++)
    {
        bit_stream |= ((uint64_t)rx_buffer[i]) << (8 * i);
    }

    /* 4 个 11 位通道依次排列，每通道留 1 位符号余量 */
    raw_ch0 = (bit_stream >> 0)  & 0x07FF;
    raw_ch1 = (bit_stream >> 11) & 0x07FF;
    raw_ch2 = (bit_stream >> 22) & 0x07FF;
    raw_ch3 = (bit_stream >> 33) & 0x07FF;

    uint8_t raw_s1 = (bit_stream >> 44) & 0x03;
    uint8_t raw_s2 = (bit_stream >> 46) & 0x03;

    uint16_t raw_wheel = (uint16_t)(((uint16_t)rx_buffer[17] << 8U) | (uint16_t)rx_buffer[16]);

    /* 通道值超出标定范围说明帧损坏（如上电错位、干扰），整帧丢弃 */
    if (raw_ch0 < DR16_CH_MIN || raw_ch0 > DR16_CH_MAX ||
        raw_ch1 < DR16_CH_MIN || raw_ch1 > DR16_CH_MAX ||
        raw_ch2 < DR16_CH_MIN || raw_ch2 > DR16_CH_MAX ||
        raw_ch3 < DR16_CH_MIN || raw_ch3 > DR16_CH_MAX)
    {
        dr16_remote.valid = 0;
        return;
    }

    dr16_remote.ch0   = raw_ch0;
    dr16_remote.ch1   = raw_ch1;
    dr16_remote.ch2   = raw_ch2;
    dr16_remote.ch3   = raw_ch3;
    dr16_remote.s1    = raw_s1;
    dr16_remote.s2    = raw_s2;
    dr16_remote.wheel = raw_wheel;
    dr16_remote.valid = 1;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART3)
    {
        if (Size == DR16_RX_BUF_LEN)
        {
            DR16_Decode();
            /* 只要收到完整帧就刷新在线时间戳（通道校验失败只影响 valid） */
            last_update_tick = HAL_GetTick();
        }
        else
        {
            /* 帧长异常：总线错位/干扰，丢弃并标记失效 */
            dr16_remote.valid = 0;
        }
        memset(rx_buffer, 0, DR16_RX_BUF_LEN);
        DR16_StartDMA();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        /* 溢出/噪声等错误会中止 DMA 传输，若不在此重启，
         * 遥控器将永久断流直至复位（热插拔接收机时必现） */
        dr16_remote.valid = 0;
        HAL_UART_AbortReceive(huart);
        memset(rx_buffer, 0, DR16_RX_BUF_LEN);
        DR16_StartDMA();
    }
}
