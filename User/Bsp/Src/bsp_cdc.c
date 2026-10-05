/**
 ******************************************************************************
 * @file    bsp_cdc.c
 * @brief   USB CDC 虚拟串口板级封装（收发均线程安全，调试输出用）
 *
 * 数据流：
 *   发送：任务 -> BSP_CDC_Send() -> 中转缓冲 -> CDC_Transmit_FS -> USB
 *   接收：USB 中断 -> CDC_Receive_FS(usbd_cdc_if.c) -> BSP_CDC_RxCallback()
 *         -> 接收环形缓冲 -> 任务用 BSP_CDC_Read() 取出
 *
 * 移植说明：仅依赖 USB CDC 类（usbd_cdc_if.h）与 FreeRTOS（临界区/互斥锁），
 * 不依赖任何具体任务或外设，可在其它工程直接复用。
 ******************************************************************************
 */
#include "bsp_cdc.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_def.h"          /* 保险起见，确保 USBD_HandleTypeDef 可见 */

/* 该变量定义在 usbd_cdc_if.c，未在头文件中导出，这里手动 extern */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* 发送等待超时（ms）：等待上一次 USB 发送完成的最长时间 */
#define BSP_CDC_TX_TIMEOUT_MS 20U

/* 接收环形缓冲 */
static uint8_t  s_rx_buffer[BSP_CDC_RX_BUFFER_SIZE];
static volatile uint16_t s_rx_write_index = 0U;
static volatile uint16_t s_rx_read_index  = 0U;
static volatile uint32_t s_rx_overflow_count = 0U;

/*
 * 发送中转缓冲：
 * CubeMX 的 CDC_Transmit_FS 只是把指针存进 USB 句柄，
 * 真正的数据搬运在后续 USB 中断里完成。
 * 因此调用者传入的 data 必须保持有效直到发送完成。
 * 这里用静态缓冲中转，并在启动新发送前等待上一次发送结束，
 * 保证 s_tx_buffer 不会被并发覆盖。
 */
static uint8_t  s_tx_buffer[BSP_CDC_TX_BUFFER_SIZE];

/*
 * 发送互斥锁：保证"等 TxState 空闲 -> 拷贝到中转缓冲 -> 发起发送"
 * 整个过程原子。没有它，两个任务可能同时通过 TxState==0 检查，
 * 先后覆盖 s_tx_buffer，发出交错的坏帧。
 */
static SemaphoreHandle_t s_tx_mutex = NULL;

void BSP_CDC_Init(void)
{
    MX_USB_DEVICE_Init();

    taskENTER_CRITICAL();
    s_rx_write_index    = 0U;
    s_rx_read_index     = 0U;
    s_rx_overflow_count = 0U;
    taskEXIT_CRITICAL();

    if (s_tx_mutex == NULL)
    {
        s_tx_mutex = xSemaphoreCreateMutex();
    }

}

uint8_t BSP_CDC_Send(const uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0U || length > BSP_CDC_TX_BUFFER_SIZE)
    {
        return USBD_FAIL;
    }

    USBD_CDC_HandleTypeDef *hcdc =
        (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

    if (hcdc == NULL)
    {
        return USBD_FAIL; /* USB 未枚举或未调用 MX_USB_DEVICE_Init() */
    }

    /* 未调用 BSP_CDC_Init() 时退化为无锁模式（仅限单任务使用） */
    if (s_tx_mutex != NULL &&
        xSemaphoreTake(s_tx_mutex, pdMS_TO_TICKS(BSP_CDC_TX_TIMEOUT_MS)) != pdTRUE)
    {
        return USBD_BUSY;
    }

    /*
     * 等待上一次发送彻底完成（TxState 由 USB 中断清零）。
     * 按 1ms 步进让出 CPU，避免忙等拖慢同优先级及以下的任务；
     * 只有确认底层空闲，才能安全覆盖 s_tx_buffer。
     */
    uint32_t waited_ms = 0U;
    uint8_t result = USBD_BUSY;

    for (;;)
    {
        /* 每轮重取句柄：等待期间设备可能被拔出，pClassData 会被清空 */
        hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
        if (hcdc == NULL)
        {
            result = USBD_FAIL;
            break;
        }

        if (hcdc->TxState == 0U)
        {
            memcpy(s_tx_buffer, data, length);
            result = CDC_Transmit_FS(s_tx_buffer, length);
            break;
        }

        if (++waited_ms > BSP_CDC_TX_TIMEOUT_MS)
        {
            result = USBD_BUSY;
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(1U));
    }

    if (s_tx_mutex != NULL)
    {
        xSemaphoreGive(s_tx_mutex);
    }

    return result;
}

void BSP_CDC_SendVofaJustFloat(const float *channels, uint8_t count)
{
    if (channels == NULL || count == 0U || count > BSP_CDC_VOFA_MAX_CHANNELS)
    {
        return;
    }

    /*
     * JustFloat 帧 = N 个小端 float + 4 字节帧尾（+Inf，VOFA+ 以此切帧）。
     * Cortex-M 为小端，float 内存布局即协议字节序，直接逐字节发送。
     * 打包在栈上完成后再整体走 BSP_CDC_Send，保证一帧原子发出。
     */
    uint8_t frame[BSP_CDC_VOFA_MAX_CHANNELS * 4U + 4U];

    memcpy(frame, channels, (uint16_t)count * 4U);

    static const uint8_t tail[4] = {0x00U, 0x00U, 0x80U, 0x7FU};
    memcpy(&frame[count * 4U], tail, 4U);

    (void)BSP_CDC_Send(frame, (uint16_t)(count * 4U + 4U));
}

uint8_t BSP_CDC_TxReady(void)
{
    USBD_CDC_HandleTypeDef *hcdc =
        (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    return (hcdc != NULL && hcdc->TxState == 0U) ? 1U : 0U;
}

uint16_t BSP_CDC_Available(void)
{
    uint16_t write_index;
    uint16_t read_index;

    taskENTER_CRITICAL();
    write_index = s_rx_write_index;
    read_index  = s_rx_read_index;
    taskEXIT_CRITICAL();

    return (uint16_t)((write_index - read_index) &
                      (BSP_CDC_RX_BUFFER_SIZE - 1U));
}

uint16_t BSP_CDC_Read(uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0U)
    {
        return 0U;
    }

    uint16_t available = BSP_CDC_Available();
    if (length > available)
    {
        length = available;
    }

    if (length == 0U)
    {
        return 0U;
    }

    /*
     * 只在任务上下文推进 read_index，写指针由 USB 中断改。
     * 拷贝过程中包临界区，避免写指针回绕导致读到未写入区域。
     */
    taskENTER_CRITICAL();
    for (uint16_t i = 0U; i < length; i++)
    {
        data[i] = s_rx_buffer[s_rx_read_index];
        s_rx_read_index = (uint16_t)((s_rx_read_index + 1U) &
                                     (BSP_CDC_RX_BUFFER_SIZE - 1U));
    }
    taskEXIT_CRITICAL();

    return length;
}

uint32_t BSP_CDC_GetRxOverflowCount(void)
{
    uint32_t count;

    taskENTER_CRITICAL();
    count = s_rx_overflow_count;
    taskEXIT_CRITICAL();

    return count;
}

void BSP_CDC_RxCallback(const uint8_t *data, uint32_t length)
{
    if (data == NULL || length == 0U)
    {
        return;
    }

    /*
     * 本函数在 USB 中断上下文执行，不调用 FreeRTOS API。
     * 只写 write_index、只读 read_index，环形算法本身无锁安全。
     */
    for (uint32_t i = 0U; i < length; i++)
    {
        uint16_t next_write_index =
            (uint16_t)((s_rx_write_index + 1U) &
                       (BSP_CDC_RX_BUFFER_SIZE - 1U));

        if (next_write_index == s_rx_read_index)
        {
            s_rx_overflow_count++;
            break;
        }

        s_rx_buffer[s_rx_write_index] = data[i];
        s_rx_write_index = next_write_index;
    }
}
