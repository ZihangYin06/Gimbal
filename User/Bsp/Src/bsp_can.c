#include "bsp_can.h"

void CAN1_Init(void)
{
  CAN_FilterTypeDef hcan1_filter;
  hcan1_filter.FilterBank = 0;
  hcan1_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  hcan1_filter.FilterScale = CAN_FILTERSCALE_16BIT;
  hcan1_filter.FilterIdHigh = 0x0000U << 5;
  hcan1_filter.FilterIdLow = 0x0000U << 5;
  hcan1_filter.FilterMaskIdHigh = 0x0000U << 5;
  hcan1_filter.FilterMaskIdLow = 0x0000U << 5;
  hcan1_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  hcan1_filter.FilterActivation = ENABLE;
  hcan1_filter.SlaveStartFilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan1, &hcan1_filter);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void CAN2_Init(void)
{
  CAN_FilterTypeDef hcan2_filter;
  hcan2_filter.FilterBank = 14;
  hcan2_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  hcan2_filter.FilterScale = CAN_FILTERSCALE_16BIT;
  hcan2_filter.FilterIdHigh = 0x0000U << 5;
  hcan2_filter.FilterIdLow = 0x0000U << 5;
  hcan2_filter.FilterMaskIdHigh = 0x0000U << 5;
  hcan2_filter.FilterMaskIdLow = 0x0000U << 5;
  hcan2_filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
  hcan2_filter.FilterActivation = ENABLE;
  hcan2_filter.SlaveStartFilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan2, &hcan2_filter);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}

void CANSend(CAN_HandleTypeDef* _phcan, uint32_t _idx, uint8_t* _data, uint8_t len)
{
  uint32_t tx_mailbox = 0;
  CAN_TxHeaderTypeDef tx_conf;
  uint8_t tx_data[8] = {0};

  tx_conf.StdId = _idx;
  tx_conf.IDE = CAN_ID_STD;
  tx_conf.RTR = CAN_RTR_DATA;
  tx_conf.DLC = len;

  for (uint8_t i = 0; i < len; ++i)
{
  tx_data[i] = _data[i];
}

  HAL_CAN_AddTxMessage(_phcan, &tx_conf, tx_data, &tx_mailbox);
}