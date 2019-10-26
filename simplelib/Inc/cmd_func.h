#ifndef __CMD_FUNC_H
#define __CMD_FUNC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cmd.h"
#include "simplelib_config.h"
#include "configure.h"

void cmd_func_init(void);
void cmd_hello_func(int argc,char *argv[]);  
void cmd_can_test(int argc, char *argv[]);

/*User's*/
extern float go_to_point_x,go_to_point_y;
void cmd_print_pos(int argc, char *argv[]);
void cmd_stop(int argc,char *argv[]);
void cmd_y_control_PID(int argc,char *argv[]);
void cmd_go_to_point_for_test(int argc,char *argv[]);
void cmd_run_point(int argc,char *argv[]);
void cmd_status();

#ifdef __cplusplus
}
#endif

#endif /* __CMD_FUNC_H */