/*******************************************************************************
Copyright:      2018/12/18
File name:      vega_action.c
Description:    action全场定位，使用前初始化十秒
Author:         徐铭远
Version：       1.0
Data:           2018/12/18 22:36
History:        无
Bug:            无
*******************************************************************************/
#include "vega_action.h"

void vega_action_reset()
{
        //HAL_Delay(12000);
        for(int i = 0;i<=20;i++)
        {
            uprintf(ACTION_USART,"ACT0");
        }
        //chassis_init_pos(0,0);
        //uprintf(CMD_USART,"复位：x = %f   y = %f   angle = %f\r\n",chassis.g_vega_pos_x,chassis.g_vega_pos_y,chassis.g_vega_angle);
}
void vega_action_setAll(float pos_x, float pos_y, float angle)
{
	uprintf(ACTION_USART,"ACTA%f%f%f\n",angle,pos_x,pos_y);	
}
void vega_action_setPos(float pos_x, float pos_y)
{
	uprintf(ACTION_USART,"ACTD%f%f\n",pos_x,pos_y);
}

void vega_action_setAngle(float angle)
{
	uprintf(ACTION_USART,"ACTJ%f\n",angle);
}


void vega_action_init()
{
    //HAL_Delay(12000);
    vega_action_reset();
}

void vega_action_cherk()
{
        //HAL_Delay(12000);
	uprintf(ACTION_USART,"ACTR");
        //uprintf(CMD_USART,"复位：x = %f   y = %f   angle = %f\r\n",chassis.g_vega_pos_x,chassis.g_vega_pos_y,chassis.g_vega_angle);
}
