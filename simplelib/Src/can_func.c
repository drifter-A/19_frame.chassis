#include "can_func.h"
#include "cmd.h"
#include "Simplelib_config.h"




void can_func_init() {
    can_callback_add(recive_id.handle_button_id,chassis_handle);
    can_callback_add(recive_id.handle_rocker_id,chassis_rocker);
    #ifdef DEBUG
    can_callback_add(1, can_suc_rx);
    #endif
}

void can_suc_rx(can_msg *data) 
{
    uprintf("can rx ok\r\n");
}