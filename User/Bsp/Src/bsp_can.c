/**
 ******************************************************************************
 * @file    bsp_can.c
 * @brief   CAN 总线板级支持：过滤器配置、总线启动、统一发送接口
 ******************************************************************************
 */
#include "bsp_can.h"

void CAN1_Init(void)
{
  CAN_FilterTypeDef hcan1_filter;
  /* F4 双 CAN 共用 28 个过滤器组：CAN1 使用 0~13，CAN2 使用 14~27
   * （SlaveStartFilterBank = 14 即从第 14 组起划给 CAN2） */
  hcan1_filter.FilterBank = 0;
  /* 掩码模式且掩码全 0：所有位均不比较，即接收总线上所有 ID */
  hcan1_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  hcan1_filter.FilterScale = CAN_FILTERSCALE_16BIT;
  hcan1_filter.FilterIdHigh = 0x0000U << 5;
  hcan1_filter.FilterIdLow = 0x0000U << 5;
  hcan1_filter.FilterMaskIdHigh = 0x0000U << 5;
  hcan1_filter.FilterMaskIdLow = 0x0000U << 5;
  hcan1_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0; /* 与 RxFifo0 回调对应 */
  hcan1_filter.FilterActivation = ENABLE;
  hcan1_filter.SlaveStartFilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan1, &hcan1_filter);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void CAN2_Init(void)
{
  CAN_FilterTypeDef hcan2_filter;
  /* CAN2 只能使用第 14 组之后的过滤器，取第 14 组 */
  hcan2_filter.FilterBank = 14;
  /* 掩码全 0：接收所有 ID */
  hcan2_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  hcan2_filter.FilterScale = CAN_FILTERSCALE_16BIT;
  hcan2_filter.FilterIdHigh = 0x0000U << 5;
  hcan2_filter.FilterIdLow = 0x0000U << 5;
  hcan2_filter.FilterMaskIdHigh = 0x0000U << 5;
  hcan2_filter.FilterMaskIdLow = 0x0000U << 5;
  hcan2_filter.FilterFIFOAssignment = CAN_FILTER_FIFO1; /* 与 RxFifo1 回调对应 */
  hcan2_filter.FilterActivation = ENABLE;
  hcan2_filter.SlaveStartFilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan2, &hcan2_filter);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}

HAL_StatusTypeDef CANSend(CAN_HandleTypeDef *phcan, uint32_t std_id,
                          const uint8_t *data, uint8_t len)
{
  uint32_t tx_mailbox = 0;
  CAN_TxHeaderTypeDef tx_conf;
  uint8_t tx_data[8] = {0};

  if (len > 8U)
  {
    len = 8U; /* 经典 CAN 单帧上限 8 字节，钳位防止后续拷贝越界 */
  }

  tx_conf.StdId = std_id;
  tx_conf.IDE = CAN_ID_STD;
  tx_conf.RTR = CAN_RTR_DATA;
  tx_conf.DLC = len;

  for (uint8_t i = 0; i < len; ++i)
  {
    tx_data[i] = data[i];
  }

  return HAL_CAN_AddTxMessage(phcan, &tx_conf, tx_data, &tx_mailbox);
}
