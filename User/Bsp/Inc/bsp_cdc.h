#ifndef BSP_CDC_H
#define BSP_CDC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define BSP_CDC_RX_BUFFER_SIZE 4096U
#define BSP_CDC_TX_BUFFER_SIZE 2048U

/* 环形缓冲实现要求 SIZE 为 2 的幂 */
#if ((BSP_CDC_RX_BUFFER_SIZE & (BSP_CDC_RX_BUFFER_SIZE - 1U)) != 0U)
#error "BSP_CDC_RX_BUFFER_SIZE must be a power of two"
#endif
#if ((BSP_CDC_TX_BUFFER_SIZE & (BSP_CDC_TX_BUFFER_SIZE - 1U)) != 0U)
#error "BSP_CDC_TX_BUFFER_SIZE must be a power of two"
#endif

/**
 * @brief  初始化 bsp_cdc：清空接收环形缓冲 + 创建发送互斥锁
 * @note   在 MX_USB_DEVICE_Init() 之后、任务上下文中调用一次
 */
void BSP_CDC_Init(void);

/**
 * @brief  发送一段数据（线程安全）
 * @param  data: 数据指针（内容会被拷贝到内部中转缓冲，返回后即可释放）
 * @param  length: 字节数，最大 BSP_CDC_TX_BUFFER_SIZE
 * @retval USBD_OK 成功；USBD_BUSY 等待底层空闲超时；USBD_FAIL 参数错误
 *         或 USB 未就绪（未枚举/未调用 MX_USB_DEVICE_Init）
 * @note   阻塞等待期间按 1ms 步进让出 CPU，最多等 BSP_CDC_TX_TIMEOUT_MS
 */
uint8_t BSP_CDC_Send(const uint8_t *data, uint16_t length);

/**
 * @brief  查询发送底层是否空闲（非阻塞、不动锁）
 * @retval 1=可立即发送 0=忙或 USB 未就绪
 * @note   控制回路里的低价值遥测先用它守门，忙时丢帧，
 *         避免 BSP_CDC_Send 最坏 20ms 的等待拖慢控制周期
 */
uint8_t BSP_CDC_TxReady(void);

/* 接收：环形缓冲，线程安全 */
uint16_t BSP_CDC_Available(void);
uint16_t BSP_CDC_Read(uint8_t *data, uint16_t length);
uint32_t BSP_CDC_GetRxOverflowCount(void);

/* 由 usbd_cdc_if.c 的 CDC_Receive_FS 调用（USB 中断上下文） */
void BSP_CDC_RxCallback(const uint8_t *data, uint32_t length);

/* VOFA+ JustFloat 单帧最大通道数（帧长 = 通道数*4 字节 + 4 字节帧尾） */
#define BSP_CDC_VOFA_MAX_CHANNELS 16U

/**
 * @brief  以 VOFA+ JustFloat 协议发送一帧多通道浮点数据（PID 调参示波用）
 * @param  channels: 浮点数组，如 PID 调参时传 {目标值, 反馈值, PID输出}
 * @param  count: 通道数，0 或超过 BSP_CDC_VOFA_MAX_CHANNELS 时忽略本次发送
 * @note   帧格式 = count 个 4 字节小端 float + 帧尾 {0x00 0x00 0x80 0x7F}(+Inf)，
 *         VOFA+ 上位机协议选 JustFloat，每个通道对应一条曲线；
 *         每次调用为一帧，调用频率即示波器刷新率，建议放在固定周期的
 *         控制任务里（如 LIFT 任务 2ms 一拍）。
 *         上位机通道顺序与数组顺序一致，建议用注释固定各通道含义
 */
void BSP_CDC_SendVofaJustFloat(const float *channels, uint8_t count);

#ifdef __cplusplus
}
#endif

#endif /* BSP_CDC_H */
