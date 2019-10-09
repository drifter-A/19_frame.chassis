/*******************************************************************************
Copyright:      2019/10/8
File name:      maxon.c
Description:    maxon电机控制，设置速度，设置PI系数，保存设置系数
Author:         徐铭远，王云轩 & ZX 
Version：       1.1 更新了对于19年frame的支持
Data:           2018/12/18 22:36
History:        1.0 见老版代码
*******************************************************************************/
#include "maxon.h"

can_msg can_TX_data;

//速度发送协议，第三个元素为节点，最后一个为校验位
uint8_t sendToSetSpeed[10] = {0x55,0xAA,0x01,0x00,0x04,0x00,0x00,0x00,0x00,0x00};
uint8_t sendToSetSpeedPI[10] = {0x55,0xAA,0x01,0x00,0x84,0x00,0x00,0x00,0x00,0x00};

/**maxon电机设置速度
*参数：串口号 速度值
*返回值： 无
*/
void maxon_canset3speed(int s1,int s2,int s0)
{
    s1 = s1 * MAX_MOTOR_SPEED / 2100;
    s2 = s2 * MAX_MOTOR_SPEED / 2100;
    s0 = s0 * MAX_MOTOR_SPEED / 2100;
    if(s1 > MAX_MOTOR_SPEED) s1 = MAX_MOTOR_SPEED;
    if(s1 < -MAX_MOTOR_SPEED) s1 = -MAX_MOTOR_SPEED;
    if(s2 > MAX_MOTOR_SPEED) s2 = MAX_MOTOR_SPEED;
    if(s2 < -MAX_MOTOR_SPEED) s2 = -MAX_MOTOR_SPEED;
    if(s0 > MAX_MOTOR_SPEED) s0 = MAX_MOTOR_SPEED;
    if(s0 < -MAX_MOTOR_SPEED) s0= -MAX_MOTOR_SPEED;
    
    if(s1 < 0) s1 = MAX_MOTOR_SPEED - s1;
    if(s2 < 0) s2 = MAX_MOTOR_SPEED - s2;
    if(s0 < 0) s0 = MAX_MOTOR_SPEED - s0;
    
    can_TX_data.ui16[0] = (uint16_t)s0;
    can_TX_data.ui16[1] = (uint16_t)s1;
    can_TX_data.ui16[2] = (uint16_t)s2;
    
    can_send_msg(send_id.motor0_id, &can_TX_data);
}

/**maxon电机设置KP
*参数：串口号 P值
*返回值： 无
*/
void maxon_setSpeed_p(UART_HandleTypeDef* USARTx , int p){
    int i;
    int check = 0;
    sendToSetSpeedPI[5] = 0x02;//命令偏移量
    sendToSetSpeedPI[6] = 0x02;//数据长度
    sendToSetSpeedPI[7] = (uint8_t)(p>>8);
    sendToSetSpeedPI[8] = (uint8_t)(p>>0);
    
    for(i = 0; i<9 ; i++){
        check += sendToSetSpeedPI[i];
    }
    sendToSetSpeedPI[CHECK] = (uint8_t)check; 
    
    HAL_UART_Transmit(USARTx,(uint8_t *) sendToSetSpeedPI,sizeof(sendToSetSpeedPI),1000);
    
}

/**maxon电机设置KI
*参数：串口号 KI
*返回值： 无
*/
void maxon_setSpeed_i(UART_HandleTypeDef* USARTx , int si){
    int i;
    int check = 0;
    
    sendToSetSpeedPI[5] = 0x03;//命令偏移量
    sendToSetSpeedPI[6] = 0x02;//数据长度
    sendToSetSpeedPI[7] = (uint8_t)(si>>8);
    sendToSetSpeedPI[8] = (uint8_t)(si>>0);
    
    for(i = 0; i<9 ; i++){
        check += sendToSetSpeedPI[i];
    }
    sendToSetSpeedPI[CHECK] = (uint8_t)check; 
                                                     
    HAL_UART_Transmit(USARTx,(uint8_t *) sendToSetSpeedPI,sizeof(sendToSetSpeedPI),1000);
}

/**maxon电机保存参数
*参数：串口号
*返回值： 无
*/
void maxon_save(UART_HandleTypeDef* USARTx){
    uint8_t saveinfo[8] = {0x55 , 0xAA , 0x01 , 0x00 , 0x03 , 0x01 ,0x00 , 0x04};
    HAL_UART_Transmit(USARTx,(uint8_t *) saveinfo,sizeof(saveinfo),1000);
}
