#ifndef __vega_action_H
#define __vega_action_H
#ifdef __cplusplus
 extern "C" {
#endif
#include "stm32f4xx_hal.h"
#include "main.h"
#include "usart.h"
#include "stdlib.h"
#include "string.h"
#include "stdarg.h"
#include "usart.h"

void vega_action_setPos(float pos_x, float pos_y);//重新设置坐标值
void vega_action_setAngle(float angle);//重新设置角度值
void vega_action_setAll(float pos_x, float pos_y, float angle);
void vega_action_reset();//将坐标和角度都清零
void vega_action_init();
	










#ifdef __cplusplus
}
#endif
#endif /*__ vega_action_H */