#ifndef __SIMPLELIB_H
#define __SIMPLELIB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usart.h"

#include "cmd.h"
#include "cmd_func.h"
#include "can_utils.h"
#include "can_func.h"

void simplelib_init(UART_HandleTypeDef *cmd_usart, CAN_HandleTypeDef *hcan);
void simplelib_run(void);

#ifdef __cplusplus
}
#endif

#endif /* __SIMPLELIB_H */