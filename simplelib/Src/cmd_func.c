#include "cmd_func.h"
#include "can_utils.h"

void cmd_func_init(void) {
    cmd_add("hello", "just", cmd_hello_func);

    #ifdef DEBUG
    cmd_add("can_test", "test can", cmd_can_test);
    #endif
}

void cmd_hello_func(int argc, char *argv[]) {
    uprintf("hello world\r\n");
}

void cmd_can_test(int argc, char *argv[]) {
    uprintf("can send test\r\n");
    can_send_test();
}