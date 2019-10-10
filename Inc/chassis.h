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
#include "point_zx.h"
#include "utils.h"
/*Define Area*/
#define MAX_SPEED_ZX 850
#define PI 3.1415926535
#define EXP 2.718281828
#define ARRIVE_DISTANCE 0.02

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

    int vega_is_ready;              //全场定位是否初始化成功变量，1为已经初始化，0为未初始化
} Chassis_Status;

/*Function Area*/
void chassis_zero();
void chassis_init(void);
void chassis_update(void);
void chassis_init_pos(float x,float y);

//Until Functions:
float chassis_angle_subtract(float a, float b);
void chassis_gostraight_zx(int speed , float angle, float turn, int is_handle);
void point_collection_tracer(int point_num);
void go_to_point_for_test(float point_x , float point_y);
void state_reset();
/*Variable Area*/
extern Chassis chassis;
extern Chassis_Status chassis_status;
extern int first_time_controler;    //一个调试用的变量，//值为1时：对一个新点使用此函数（不用多次调用BoostAndSlowUpdate）
extern PID_Struct line_control_PID; //line_control用的PID结构体
#ifdef __cplusplus
}
#endif
#endif /*__ chassis_H */