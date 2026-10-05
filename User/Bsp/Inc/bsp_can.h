/**
 ******************************************************************************
 * @file    bsp_can.h
 * @brief   CAN 总线板级支持：启动收发 + 统一发送接口
 *
 * 初始化分工：CAN 外设本身（波特率、引脚）由 CubeMX 生成的
 * CAN1_Init/CAN2_Init（Core/Src/can.c）完成，本模块负责过滤器配置、
 * 启动总线和打开接收中断；CAN1 数据走 FIFO0，CAN2 数据走 FIFO1，
 * 关节电机反馈回调见 Device 层 DM_Motor.cpp
 ******************************************************************************
 */
#ifndef BSP_CAN_H
#define BSP_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "can.h"

/**
 * @brief  配置 CAN1 过滤器（接收所有 ID -> FIFO0）、启动总线并打开接收中断
 * @note   须在 CubeMX 生成的 MX_CAN1_Init() 之后调用
 */
void CAN1_Init(void);

/**
 * @brief  配置 CAN2 过滤器（接收所有 ID -> FIFO1）、启动总线并打开接收中断
 * @note   须在 CubeMX 生成的 MX_CAN2_Init() 之后调用
 */
void CAN2_Init(void);

/**
 * @brief  发送一帧标准数据帧
 * @param  phcan: 目标总线（&hcan1 / &hcan2）
 * @param  std_id: 标准帧 ID（11 位）
 * @param  data: 数据指针
 * @param  len: 数据长度，大于 8 会被钳位到 8（CAN 经典帧 DLC 上限）
 * @retval HAL_CAN_AddTxMessage 的返回值；
 *         3 个发送邮箱全满时返回 HAL_ERROR 且本帧丢弃（不阻塞等待）
 */
HAL_StatusTypeDef CANSend(CAN_HandleTypeDef *phcan, uint32_t std_id,
                          const uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif
