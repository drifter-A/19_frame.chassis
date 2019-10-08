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

/*Struct Area*/
extern Chassis chassis;
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

#endif /* __SIMPLELIB_H */