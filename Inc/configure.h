/*******************************************************************************
Copyright:      BUPT
File name:      configure.c
Description:    所有的配置，例如整合chassis等需要用到的结构体等，目的是简化后续代码和
方便封装
Author:         ZX 
Version：       1.0
Data:           2019/10/9
History:        none
*******************************************************************************/
#ifndef __CONFIGURE_H
#define __CONFIGURE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
//*Define Area*/
#define MAX_SPEED_ZX 850
#define PI 3.1415926535
#define EXP 2.718281828
#define ARRIVE_DISTANCE 0.02

#define VEGA_USART huart1
/*Struct Area*/

typedef struct
{
    float g_vega_pos_x; 
    float g_vega_pos_y;   
    float g_vega_angle;
	
    float pos_x,pos_y;
    float angle;

    int dis_vertical_camera;
    int dis_horizone_camera;
    int ccd_pos;
    
    int g_ispeed;
    float g_fangle;
    float g_fturn;
    //int now_speed[3];
    float speed_x;
    float speed_y;
    float last_pos_x;
    float last_pos_y;
    float now_speed;
    
    float origin_angle;
} Chassis; 

typedef struct{
    int count;  //跑点的计数
    int point_tracer_flag;  //为1时：可以执行point_tracer；为0时：禁用point_tracer PS:此变量主要为调试时方便控制
    int total_line_control_flag;    //为0时禁用line_control，为1时启用
    int ENBALE_POINT_COLLECTION_TRACER;     //为1时开启跑点，为0时禁用跑点
    int go_to_point_test_flag;      //go_to_point_for_test函数控制变量，1为开启，0为关闭
} Chassis_Status;

typedef struct can_id_send{
    uint32_t motor0_id;//电机id
    uint32_t motor1_id;
    uint32_t motor2_id;
}Can_id_send;

/*
typedef struct can_id_recive{
    //结构体内内容等待添加
}Can_id_recive;
*/     

/*Function Area*/
void chassis_zero();
void chassis_init(void);
void chassis_update(void);
void chassis_init_pos(float x,float y);

//Until Functions:
float chassis_angle_subtract(float a, float b);

/*Variable Area*/
extern Chassis chassis;
extern Chassis_Status chassis_status;
extern Can_id_send send_id;
#ifdef __cplusplus
}
#endif

#endif /* __SIMPLELIB_H */