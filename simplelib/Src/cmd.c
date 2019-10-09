/*******************************************************************************
 * Copyright:		BUPT
 * File Name:		cmd.c
 * Description:		指令功能实现 + 加入全场定位部分
 * Author:			ZeroVoid & ZX
 * Version:			0.2
 * Data:			2019/10/8 Tue 21:30
 *******************************************************************************/

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "cmd.h"
#include "hash.h"
/* 变量定义 -----------------------------------------------------*/
static const char *delim = ", \r\n\0";
static HashTable cmd_table;

UART_HandleTypeDef CMD_USART;
char *cmd_argv[MAX_ARGC]; 
uint8_t DMAaRxBuffer[DMA_BUFFER_SIZE];
char DMAUSART_RX_BUF[DMA_BUFFER_SIZE];
int DMA_RxOK_Flag = 0;
int buffer_count = 0;
char uart_buffer[DMA_BUFFER_SIZE];

static union
{
    uint8_t data[24];
    float ActVal[6];
}posture;   //用于全场定位传输数据的结构体
int DMA_RxOK_Flag_vega=0;       //vega接收buffer
char DMAUSART_RX_BUF_vega[99];  //全场定位接收buffer
uint8_t DMAaRxBuffer_vega[99];  //全场定位接收buffer
void usart_exc_DMA_vega();      //全场定位：将读到的数据存入（结合IDLE中断使用）
/* private function -----------------------------------------------------*/
static int str_cmp(const void *a, const void *b);
static void _cmd_help(const void *key, void **value, void *c1);


/* cmd实现函数定义 -----------------------------------------------------*/

void usart_DMA_init(UART_HandleTypeDef *cmd_usart) {
    CMD_USART = *cmd_usart;
    HAL_UART_Receive_DMA(&CMD_USART, (uint8_t *)&DMAaRxBuffer, 99);
    cmd_init();
    __HAL_UART_ENABLE_IT(&CMD_USART,UART_IT_IDLE); // 开启空闲中断
}

/**
 * @brief	指令初始化函数，仅供模块初始化调用
 * @return	None
 */
void cmd_init(void) {
    if (cmd_table == NULL) {
        cmd_table = HashTable_create(str_cmp, hashStr, NULL);
    }
    cmd_add("help", "show cmd usage", cmd_help_func);
}

void usart_exc_DMA() {
    int cmd_argc;
    int erro_n;
    erro_n = cmd_parse((char *)DMAUSART_RX_BUF, &cmd_argc, cmd_argv);  //解析命令
    erro_n = cmd_exec(cmd_argc, cmd_argv);                             //执行命令
    UNUSED(erro_n);
    memset(DMAUSART_RX_BUF, 0, 98);
    buffer_count = 0;
}

void HAL_UART_IDLECallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == CMD_USART.Instance) {
        uint8_t temp;
        __HAL_UART_CLEAR_IDLEFLAG(huart);  //清除函数空闲标志
        temp = huart->Instance->SR;
        temp = huart->Instance->DR;  //读出串口的数据，防止在关闭DMA期间有数据进来，造成ORE错误
        UNUSED(temp);
        // temp = hdma_usart3_rx.Instance->CNDTR;
        // huart->hdmarx->XferCpltCallback(huart->hdmarx);
        // //调用DMA接受完毕后的回调函数，最主要的目的是要将串口的状态设置为Ready，否则无法开启下一次DMA
        HAL_UART_DMAStop(&huart3);  //停止本次DMA

        uint8_t *clr = DMAaRxBuffer;
        while(*(clr++) == '\0' && clr < DMAaRxBuffer+DMA_BUFFER_SIZE);
        strcpy((char *)DMAUSART_RX_BUF,(char *)(clr-1));
        if (DMAUSART_RX_BUF[0] != '\0') {
            DMA_RxOK_Flag = 1;
        }
        memset(DMAaRxBuffer, 0, 98);
        HAL_UART_Receive_DMA(&CMD_USART, (uint8_t *)&DMAaRxBuffer, 99);
    }
    if(huart->Instance==VEGA_USART.Instance)
    {
        uint8_t temp;
        __HAL_UART_CLEAR_IDLEFLAG(huart);   //清除函数空闲标志
        temp= huart->Instance->SR;
        temp= huart->Instance->DR;//读出串口的数据，防止在关闭DMA期间有数据进来，造成ORE错误
        //temp = hdma_usart3_rx.Instance->CNDTR; 
        //huart->hdmarx->XferCpltCallback(huart->hdmarx); //调用DMA接受完毕后的回调函数，最主要的目的是要将串口的状态设置为Ready，否则无法开启下一次DMA
        HAL_UART_DMAStop(&huart1);      //停止本次DMA
        UNUSED(temp);
        strcpy((char *)DMAUSART_RX_BUF_vega,(char *)DMAaRxBuffer_vega);
        if(DMAUSART_RX_BUF_vega[0]!='\0')
            DMA_RxOK_Flag_vega=1;
        
        usart_exc_DMA_vega();
        memset(DMAaRxBuffer_vega,0,98);
        HAL_UART_Receive_DMA(&huart1,(uint8_t *)&DMAaRxBuffer_vega, 99); //zx:开启DMA？
    }
}

/**
 * @brief	将输入分割，并记录参数个数
 * @param	cmd_line	输入指令字符串
 *          argc        指令个数
 *          argv        分割后参数列表
 * @return	None
 */
int cmd_parse(char *cmd_line,int *argc,char *argv[]){
    char *token = strtok(cmd_line, delim);
    int arg_index = 0;

    while(token && arg_index <= MAX_ARGC) {
        argv[arg_index++] = token;
        token = strtok(NULL, delim);
    }
    *argc = arg_index;
    return 0;
}

/**
 * @brief	指令执行函数
 * @param	argc    参数个数
 *          argv    参数列表 
 * @return	0   正常执行返回
 *          1   未找到指令
 */
int cmd_exec(int argc,char *argv[]){
    struct cmd_info *cmd = (struct cmd_info*)HashTable_get(cmd_table, argv[0]);
    if (cmd != NULL) {
        cmd->cmd_func(argc, argv);
        return 0;
    }
    uprintf("cmd not find\r\n");
    return 1;
}

/**
 * @brief	指令帮助函数
 * @param	忽略参数
 * @return	None
 */
void cmd_help_func(int argc,char *argv[]){
    // FIXME: ZeroVoid	2019/09/23	 dma usage 输出不完整，调试输出没问题
    uprintf("help:\r\n");
    HashTable_map(cmd_table, _cmd_help, NULL);
}

/**
 * @brief	指令添加函数
 * @param	cmd_name    指令名称
 * @param   cmd_usage   指令使用说明
 * @param   cmd_func    指令函数指针 argc 参数个数(含指令名称), argv 参数字符串数组
 * @return	None
 */
void cmd_add(char *cmd_name, char *cmd_usage, void (*cmd_func)(int argc, char *argv[])) {
    // FIXME: ZeroVoid	2019/9/23	 name or usage too long
    struct cmd_info *new_cmd = (struct cmd_info*)malloc(sizeof(struct cmd_info)); 
    char *name = (char*) malloc(strlen(cmd_name)+1);
    char *usage = (char*) malloc(strlen(cmd_usage)+1);
    strcpy(name, cmd_name);
    strcpy(usage, cmd_usage);
    new_cmd->cmd_func = cmd_func;
    new_cmd->cmd_usage = usage;
    HashTable_insert(cmd_table, name, new_cmd);
}

char print_buffer[PRINT_BUFFER_SIZE];
void uprintf(char *fmt, ...) {
    int size;
    va_list arg_ptr;
    va_start(arg_ptr, fmt);
    size = vsnprintf(print_buffer, PRINT_BUFFER_SIZE, fmt, arg_ptr);
    va_end(arg_ptr);

    // 重新设置USART准备状态，并解锁串口,否则无法再次输出
    CMD_USART.gState = HAL_UART_STATE_READY;
    //__HAL_UNLOCK(&CMD_USART);
    if (HAL_UART_Transmit_DMA(&CMD_USART, (uint8_t *)print_buffer, size) != HAL_OK) {
        HAL_Delay(10);
    }
    // TODO:	ZeroVoid	due:10/7	优化输出，异步输出，可能纯在busy时再次调用，会被忽略，输出缺失
    while(CMD_USART.hdmatx->State != HAL_DMA_STATE_READY);
    //HAL_UART_Transmit(&CMD_USART, (uint8_t*)uart_buffer, size, 1000);
}

void uprintf_to(UART_HandleTypeDef *huart, char *fmt, ...) {
    int size;
    va_list arg_ptr;
    va_start(arg_ptr, fmt);
    size = vsnprintf(print_buffer, PRINT_BUFFER_SIZE, fmt, arg_ptr);
    va_end(arg_ptr);

    HAL_UART_Transmit_DMA(huart, (uint8_t *)print_buffer, size);
    // HAL_UART_Transmit(huart,(uint8_t *)uart_buffer,size,1000);
}

static char s[22] = {'b', 'y', 16, 6};
void send_wave(float arg1, float arg2, float arg3, float arg4) {
    //s[2] = 16;  // length
    //s[3] = 6;   // type
    s[20] = '\r';
    s[21] = '\n';
    memcpy(s + 4, &arg1, sizeof(arg1));
    memcpy(s + 8, &arg2, sizeof(arg1));
    memcpy(s + 12, &arg3, sizeof(arg1));
    memcpy(s + 16, &arg4, sizeof(arg1));
    HAL_UART_Transmit_DMA(&CMD_USART, (uint8_t *)s, 22);
}

/* private function defined -----------------------------------------------------*/
/**
 * @brief	字符串比较函数，hash表中使用
 */
static int str_cmp(const void *a, const void *b) {
    return strcmp((char*)a, (char*)b) != 0;
}

/**
 * @brief	输出函数使用说明，遍历hash表中使用
 */
static void _cmd_help(const void *key, void **value, void *c1) {
    UNUSED(c1);
    char *usage = ((struct cmd_info*)(*value))->cmd_usage;
    uprintf("%s: %s\r\n", key, usage);
}


/**将全场定位从串口收到的数据存入
*参数：void
*返回值： void 注：直接将结果写入了底盘结构体中
*说明: 更新底盘目前位置的函数
*移植者: zx
*/
void usart_exc_DMA_vega()
{
    if(DMA_RxOK_Flag_vega)
    {
        for(int j = 0; j < 99; j++)
        {
            if(DMAaRxBuffer_vega[j] == 0x0d)
            {
                if(DMAaRxBuffer_vega[j + 1] == 0x0a && j < 73 && DMAaRxBuffer_vega[j + 26] == 0x0a && DMAaRxBuffer_vega[j + 27] == 0x0d)
                {
                    for(int k = 0; k < 24 ; k++)
                    {
                      posture.data[k] = DMAaRxBuffer_vega[j + 2 + k];
                    }
                    chassis.g_vega_angle = posture.ActVal[0];
                    chassis.g_vega_pos_x = posture.ActVal[3];
                    chassis.g_vega_pos_y = posture.ActVal[4];
                    chassis_update();
                    DMA_RxOK_Flag_vega = 0;
                    memset(DMAUSART_RX_BUF_vega,0,98);
                    return;
                }
            }
        }
        DMA_RxOK_Flag_vega = 0;
        memset(DMAUSART_RX_BUF_vega,0,98);        
    }
}