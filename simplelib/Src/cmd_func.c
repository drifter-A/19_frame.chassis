#include "cmd_func.h"
#include "can_utils.h"

void cmd_func_init(void) {
    cmd_add("hello", "just", cmd_hello_func);
    cmd_add("print_pos","打印全场定位位置",cmd_print_pos);
    cmd_add("line_PID","line_control_PID",cmd_line_control_PID);
    cmd_add("go_to"," go_to_some_point",cmd_line_control_PID);
    cmd_add("collect_tracer"," start_collect_tracer",cmd_point_collection_tracer);
    
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
void cmd_stop_flag(int argc,char *argv[]){
  chassis_status.point_tracer_flag = atoi(argv[1]);
  first_time_controler = 1;
  if(chassis_status.point_tracer_flag == 0){
    chassis_status.count = 0;
    chassis_status.ENBALE_POINT_COLLECTION_TRACER = 0;
    state_reset();
    chassis_gostraight_zx(0,0,chassis.angle,0);
  }
  uprintf(" stop_flag : %d \r\n" , chassis_status.point_tracer_flag);
}

void cmd_line_control_PID(int argc,char *argv[]){
  line_control_PID.KP = atof(argv[1]);
  line_control_PID.KI = atof(argv[2]);
  line_control_PID.KD = atof(argv[3]);
  uprintf("KP: %f , KI: %f , KD: %f\r\n" ,line_control_PID.KP , line_control_PID.KI , line_control_PID.KD);
}

float go_to_point_x = 0,go_to_point_y = 0;  //调试用代码
void cmd_go_to_point_for_test(int argc,char *argv[]){
  chassis_status.go_to_point_test_flag = atoi(argv[1]);
  go_to_point_x = atof(argv[2]);
  go_to_point_y = atof(argv[3]);
  if(chassis_status.go_to_point_test_flag==0){
    chassis_gostraight_zx( 0 , 0 , 0 , 0);
    uprintf("stop!now\r\n");
  }else{
    uprintf("go to : %f , %f\r\n",go_to_point_x ,go_to_point_y);
  }
}

void cmd_point_collection_tracer(int argc,char *argv[]){
  chassis_status.ENBALE_POINT_COLLECTION_TRACER = atoi(argv[1]);
  if(chassis_status.ENBALE_POINT_COLLECTION_TRACER == 0){
    chassis_gostraight_zx(0,0,chassis.angle,0);
    state_reset();
  }
}