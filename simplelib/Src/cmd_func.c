#include "cmd_func.h"
#include "can_utils.h"

void cmd_func_init(void) {
    cmd_add("hello", "just", cmd_hello_func);
    cmd_add("print_pos","打印全场定位位置",cmd_print_pos);
    cmd_add("d_PID","position_y_dir_pid",cmd_y_control_PID);
    cmd_add("go_to"," go_to_some_point",cmd_go_to_point_for_test);
    cmd_add("run_point"," start_run_point",cmd_run_point);
    cmd_add("stop","停车",cmd_stop);
    cmd_add("status","查看车状态",cmd_status);
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

void cmd_print_pos(int argc, char *argv[])
{
  uprintf("x = %f, y = %f,  angle = %f\r\n",chassis.pos_x,chassis.pos_y,chassis.angle);
}

float point_x,point_y;
void cmd_stop(int argc,char *argv[])
{
    chassis_status.count = 0;
    chassis_status.run_point_test = 0;
    chassis_status.trace_count = 0;
    chassis_status.go_to_point_test_flag = 0;
    chassis_gostraight(0,0,chassis.angle,0);
    uprintf(" stop : %d %d\r\n" , chassis_status.run_point,chassis_status.run_point_test);
}

void cmd_y_control_PID(int argc,char *argv[])
{
  position_y_dir_pid.KP = atof(argv[1]);
  position_y_dir_pid.KI = atof(argv[2]);
  position_y_dir_pid.KD = atof(argv[3]);
  uprintf("KP: %f , KI: %f , KD: %f\r\n" ,position_y_dir_pid.KP , position_y_dir_pid.KI , position_y_dir_pid.KD);
}

float go_to_point_x = 0,go_to_point_y = 0;  //调试用代码
void cmd_go_to_point_for_test(int argc,char *argv[])
{
  chassis_status.go_to_point_test_flag = atoi(argv[1]);
  go_to_point_x = atof(argv[2]);
  go_to_point_y = atof(argv[3]);

  if(chassis_status.go_to_point_test_flag==0)
  {
    chassis_gostraight( 0 , 0 , 0 , 0);
    uprintf("stop!now\r\n");
  }
  else
  {
    uprintf("go to : %f , %f\r\n",go_to_point_x ,go_to_point_y);
  }
}

void cmd_run_point(int argc,char *argv[])
{
  chassis_status.run_point = atoi(argv[1]);
  uprintf("RUN ! %d",chassis_status.run_point);
}

void cmd_status()
{
  uprintf("run:%d,go_to:%d,count:%d,trace:%d\r\n",chassis_status.run_point,chassis_status.go_to_point_test_flag,chassis_status.count,chassis_status.trace_count);
}