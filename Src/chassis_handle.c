#include "chassis_handle.h"

CHASSIS_HANDLE ChassisHandle;

void chassis_handle(can_msg *data)
{
  if(0 == main_flag.main_run_flag) return;
  
  /*if(Data[0] >= '0' && Data[0] <= '9' && (Data[1] == 'u' || Data[1] == 'd'))
  {
  if(Data[1] == 'u')
  {
  ChassisHandle.btstate[(int)(Data[0] - '0')] = 1;
}
  else if(Data[1] == 'd')
  {
  ChassisHandle.btstate[(int)(Data[0] - '0')] = 0;
}
}*/
  uint8_t id = (uint8_t)((data->ui8[0]) * 10 + (data->ui8[1]));
  switch(id)
  {
  case 0:
    main_flag.chassis_handle_flag = 1;
    main_flag.chassis_automate_flag = 0;
    ChassisHandle.handle_max_speed = 600;
    break;
  case 8:
    main_flag.chassis_handle_flag = 0;
    main_flag.chassis_automate_flag = 1;
    break;
  case 1:
    ChassisHandle.mode = 1;
    break;
  case 2:
    ChassisHandle.mode = 2;
    break;
  case 3:
    ChassisHandle.mode = 3;
    break;
  }
}

void chassis_rocker(can_msg *data)
{
  if(0 == main_flag.main_run_flag || main_flag.chassis_handle_flag == 0) return;
  
  //常数修改零偏
  ChassisHandle.ry = (int)data->i16[0] - 14;
  ChassisHandle.rx = (int)data->i16[1] - 4;
  ChassisHandle.ly = (int)data->i16[2] - 4;
  ChassisHandle.lx = (int)data->i16[3] - 5;
  
  //变换坐标系
  ChassisHandle.rx *= -1;
  ChassisHandle.lx *= -1;
  
  chassis.g_fangle = atan2(ChassisHandle.ly, ChassisHandle.lx) + chassis.angle;
  chassis.g_ispeed = (int)( ChassisHandle.mode * sqrt(ChassisHandle.ly * ChassisHandle.ly + ChassisHandle.lx * ChassisHandle.lx) );
  
  if(chassis.g_ispeed < 20) chassis.g_ispeed = 0;
  if(chassis.g_ispeed > ChassisHandle.handle_max_speed) chassis.g_ispeed = ChassisHandle.handle_max_speed;
  
  //旋转速度
  int rotate = (int)sqrt(ChassisHandle.ry * ChassisHandle.ry + ChassisHandle.rx * ChassisHandle.rx);
  
  if(rotate > 90)
  chassis.g_fturn = 100 * chassis_angle_subtract(atan2(ChassisHandle.ry, ChassisHandle.rx), PI/2) * (-1);
  else 
  chassis.g_fturn = 0; 
  
  
}

void chassis_handle_control()
{
  if(0 == main_flag.main_run_flag || main_flag.chassis_handle_flag == 0) return;
  
  chassis_gostraight(chassis.g_ispeed,chassis.g_fangle,chassis.g_fturn,1);  
}