#ifndef _DEFINE_H_
#define _DEFINE_H_

/* 板间通讯帧（CAN1，与 Chassis 工程对应） */
#define DR16_COM_RX_ID 0x300
#define DR16_COM_TX_ID 0x301
#define DR16_COM_CAN (&hcan1)

#define YAW_COM_RX_ID 0x302
#define YAW_COM_TX_ID 0x303
#define YAW_COM_CAN (&hcan1)

/* DM 拨弹电机的总线与 TX/RX 编号已并入 DM_Motor.cpp 的身份登记表 */

#endif
