/*******************************************************************************
Copyright:      BUPT
File name:      configure.c
Description:    所有的配置外部配置
Author:         ZX 
Version：       1.0
Data:           2019/10/9
History:        none
*******************************************************************************/
#include "configure.h"


//Can_id_recive recive_id;
Can_id_send send_id;
Can_id_recive recive_id;
void can_id_init()
{
    //目前只定义了发送的id结构体，之后可能会需要接受的结构体
    send_id.motor0_id = 400;
    send_id.motor1_id = 401;
    send_id.motor2_id = 402;  
    
    recive_id.handle_button_id = 325;
    recive_id.handle_rocker_id = 324;    
}


