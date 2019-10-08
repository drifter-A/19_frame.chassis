#ifndef __maxon_H
#define __maxon_H
#ifdef __cplusplus
 extern "C" {
#endif
#include "stm32f4xx.h"
#include "main.h"
#include "usart.h"
#include "can.h"
#include "simplelib.h"

#define NUMBER    2
#define DATA_4    5
#define DATA_3    6
#define DATA_2    7
#define DATA_1    8
#define CHECK     9
#define MOTOR0_ID huart4
#define MOTOR1_ID huart1
#define MOTOR3_ID huart3
#define MAX_MOTOR_SPEED 630

//void maxon_newsetSpeed(uint32_t can_ID, int speed);
void maxon_setSpeed_i(UART_HandleTypeDef* USARTx , int si);
void maxon_setSpeed_p(UART_HandleTypeDef* USARTx , int p);
//void maxon_setSpeed(UART_HandleTypeDef* USARTx, int speed);
void maxon_save(UART_HandleTypeDef* USARTx);
void maxon_canset3speed(int s1,int s2,int s0);

#ifdef __cplusplus
}
#endif
#endif /*__ maxon_H */