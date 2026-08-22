#include "bsp_dr16.h"

static uint8_t rx_buffer[18] = {0};
static const uint16_t RX_SIZE = 18;

DR16_Remote_t dr16_remote = {0};

void DR16_Init(void)
{
    DR16_StartDMA();
}

void DR16_StartDMA(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buffer, RX_SIZE);
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
}

static void DR16_Decode(void)
{
    uint64_t bit_stream = 0;
    uint16_t raw_ch0, raw_ch1, raw_ch2, raw_ch3;

    for (int i = 0; i < 18; i++) 
    {
        bit_stream |= ((uint64_t)rx_buffer[i]) << (8 * i);
    }

    raw_ch0 = (bit_stream >> 0)  & 0x07FF;
    raw_ch1 = (bit_stream >> 11) & 0x07FF;
    raw_ch2 = (bit_stream >> 22) & 0x07FF;
    raw_ch3 = (bit_stream >> 33) & 0x07FF;

    uint8_t raw_s1 = (bit_stream >> 44) & 0x03;
    uint8_t raw_s2 = (bit_stream >> 46) & 0x03;

    uint16_t raw_wheel = (uint16_t)(((uint16_t)rx_buffer[17] << 8U) | (uint16_t)rx_buffer[16]);

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
        if (Size == RX_SIZE) 
        {
            DR16_Decode();
        }
        else 
        {
            dr16_remote.valid = 0;
        }
        memset(rx_buffer, 0, RX_SIZE);
        DR16_StartDMA();
    }
}