#include "chassis.h"

Chassis chassis;

/**底盘初始化
*参数：无
*返回值： 无
*/
void chassis_zero()
{
    chassis.g_fangle = 0;
    chassis.g_fturn = 0;
    chassis.g_ispeed = 0;
    chassis.origin_angle = 0;
    /*
    ChassisSignal.m_DriveMode = _FIXED_TRACK_MODE;
    ChassisSignal.m_SpeedMode = _SPEED_SHEET;
    ChassisSignal.m_CtrlFlag._vega_ready_flag = 0;
    ChassisSignal.m_FinishFlag._delay_flag = 0;
    ChassisSignal.m_FinishFlag._position_flag = 0;
    ChassisSignal.m_FinishFlag._rotate_flag = 0;
    ChassisSignal.m_CtrlFlag._routeflag = 0;
    ChassisSignal.m_CtrlFlag._sensor_flag = 0;
    ChassisSignal.m_CtrlFlag._handle_flag = 0;
    */
}

void chassis_init(void)
{
    chassis_init_pos(zx_points_pos_x[0],zx_points_pos_y[0]);
    chassis_zero();
}

//以下变量为 chassis_init_pos 中使用的
float action_init_posx = 0;
float action_init_posy = 0;
float action_init_angle = 0;
/**底盘重设坐标
*参数：无
*返回值： 无
*/
void chassis_init_pos(float x,float y)
{
    action_init_posx += x - chassis.pos_x;
    action_init_posy += y - chassis.pos_y;
}

/**底盘更新坐标
*参数：无
*返回值： 无
*/
void chassis_update(void)
{ 
    chassis.angle = (chassis.g_vega_angle / 180.f) * PI;
    chassis.pos_x = chassis.g_vega_pos_x/1000 + action_init_posx;
    chassis.pos_y = chassis.g_vega_pos_y/1000 + action_init_posy;
    chassis.speed_x = (chassis.pos_x - chassis.last_pos_x) / 0.005;
    chassis.speed_y = (chassis.pos_y - chassis.last_pos_y) / 0.005;//除时间 m/s
    chassis.last_pos_x = chassis.pos_x;
    chassis.last_pos_y = chassis.pos_y;
    chassis.now_speed = vec_model(vec_create(chassis.speed_x,chassis.speed_y));
}

/*
角度减法函数
*/
float chassis_angle_subtract(float a, float b)
{
    float out = a - b;
    while(out > PI)
    {
        out -= 2 * PI;
    }
    while(out < - PI)
    {
        out += 2 * PI;
    }
    return out;
}

/*
角度pid控制
*/
float chassis_PID_Angle_Control(float target_angle){
    
    float angle_err = chassis_angle_subtract(target_angle, chassis.angle); 
    static float angle_last_err = 0;
    float P_out = angle_err * chassis_turn_angle_KP;
    float D_out = (angle_last_err - angle_err) * chassis_turn_angle_KD;
    angle_last_err = angle_err;
    return P_out + D_out;
}

/**底盘驱动
*参数：float angle 	方向角
*      int   speed    速度
float turn  自转方位角
*返回值： 无
*说明:
*/
void chassis_gostraight_zx(int speed , float angle, float turn, int is_handle)
{
    float Chassis_motor0 = -(speed*cos((ERR_angle_m0 + chassis.angle) - angle));
    float Chassis_motor1 = -(speed*cos((ERR_angle_m1 + chassis.angle) - angle));
    float Chassis_motor3 = -(speed*cos((ERR_angle_m3 + chassis.angle) - angle));
    
    float turn_output = 0;
    if(is_handle)
    {
        turn_output = -turn;//全场定位方向环  
    }
    else
    {
        turn_output = chassis_PID_Angle_Control(turn);
    }
    if(turn_output >350)//陀螺仪角度PID
    {
        turn_output = 350;
    }
    if(turn_output < -350)
    {
        turn_output = -350;
    }
    
    
    maxon_canset3speed((int)(Chassis_motor0 + turn_output),
                       (int)(Chassis_motor3 + turn_output),
                       (int)(Chassis_motor1 + turn_output));
}