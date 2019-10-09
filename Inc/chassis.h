/*******************************************************************************
Copyright:      BUPT
File name:      chassis.c
Description:    全向轮底盘控制代码
Author:         ZX 
Version：       1.1 更新了对于19年frame的支持
Data:           2019/10/9
History:        1.0 见老版代码
*******************************************************************************/
#ifndef __chassis_H
#define __chassis_H
#ifdef __cplusplus
extern "C" {
#endif
/*Include Area*/
#include <math.h>

#include "simplelib.h"
#include "stm32f4xx_hal.h"
#include "main.h"
#include "vec.h"
#include "maxon.h"
#include "configure.h"
#include "point_zx.h"
#include "utils.h"
/*Define Area*/
#define MAX_SPEED_ZX 850
#define PI 3.1415926535
#define EXP 2.718281828
#define ARRIVE_DISTANCE 0.02

/*Struct Area*/

/*Function Area*/
void chassis_zero();
void chassis_init(void);
void chassis_update(void);
void chassis_init_pos(float x,float y);

//Until Functions:
float chassis_angle_subtract(float a, float b);

/*Variable Area*/


#ifdef __cplusplus
}
#endif
#endif /*__ chassis_H */