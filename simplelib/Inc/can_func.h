#ifndef __can_func_H
#define __can_func_H
#ifdef __cplusplus
extern "C" {
#endif
#include "can_utils.h"
#include "can.h"
    
void can_func_init();
void can_suc_rx(union can_msg_union *data);
    
#ifdef __cplusplus
}
#endif
#endif /*__ can_func_H */