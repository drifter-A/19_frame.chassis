/*******************************************************************************
 * Copyright:		BUPT
 * File Name:		can_utils.h
 * Description:		CAN 工具函数
 * Author:			ZeroVoid
 * Version:			0.1
 * Data:			2019/09/23 Mon 14:00
 *******************************************************************************/
#ifndef __CAN_UTILS_H
#define __CAN_UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "can.h"
#include "simplelib_config.h"

typedef union can_msg_union{
    char ch[8];
    uint8_t ui8[8];
    uint16_t ui16[4];
    int16_t i16[4];
    int in[2];
    float fl[2];
    double df;
}can_msg;

extern CAN_HandleTypeDef HCAN;
extern CAN_TxHeaderTypeDef TxHeader;
extern CAN_RxHeaderTypeDef RxHeader;
extern uint32_t TxMailbokx;
extern can_msg can_rx_data;
extern can_msg can_tx_data;
extern int can_exc_callback_flag;

void can_init(CAN_HandleTypeDef *hcan);
int can_send_msg(uint16_t id, can_msg *msg);
void can_callback_add(const uint32_t id, void (*callback)(can_msg *data));
void can_exc_callback(void);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan);

#ifdef DEBUG
void can_send_test(void);
#endif //DEBUG

#ifdef __cplusplus
}
#endif

#endif /* __CAN_UTILS_H */