/*******************************************************************************
Copyright:      BUPT
File name:      chassis.c
Description:    全向轮底盘控制代码
Author:         ZX 
Version：       1.1 更新了对于19年frame的支持
Data:           2019/10/9
History:        1.0 见老版代码
*******************************************************************************/
#include "chassis.h"
/*Variable*/
//底盘自转pid参数
float chassis_turn_angle_KP = -1000;
float chassis_turn_angle_KD = 0;
//三轮与全场定位模块安装偏角
float catch_bone_modify_x = 0;
float catch_bone_modify_y = 0;
float ERR_angle_m3 = -PI/3 , ERR_angle_m1 = -PI/3 + 1*2*PI/3 , ERR_angle_m0 = -PI/3 + 2*2*PI/3;

/*Variable Area*/
Chassis chassis;
Chassis_Status chassis_status;

static int point_retrack_first_ref_flag = 1;    //重新找点的控制变量

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
    
    chassis_status.count = 0;               //跑点计数，初试为0
    chassis_status.point_tracer_flag = 0;   //目前调试阶段 一开始关闭跑点
    chassis_status.ENBALE_POINT_COLLECTION_TRACER=0;    //为1时开启跑点，为0时禁用跑点
    chassis_status.go_to_point_test_flag = 0;           //go_to_point_for_test函数控制变量，1为开启，0为关闭

    chassis_status.vega_is_ready = 0;       //初始化前不ready
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


/**速度矢量相加，返回值为合速度矢量
*参数：vec1和vec2为待相加的矢量
*返回值： 合速度矢量
*说明: 用于point_tracer使用
*作者: zx
*/
vec speed_vec_add(vec vec1 , vec vec2){
  float v1_x , v1_y , v2_x , v2_y,speed,angle;
  v1_x = cosf(vec1.y)* vec1.x;
  v1_y = sinf(vec1.y)* vec1.x;
  v2_x = cosf(vec2.y)* vec2.x;
  v2_y = sinf(vec2.y)* vec2.x;
  speed = sqrtf( (v1_x+v2_x) * (v1_x+v2_x) + (v1_y+v2_y)*(v1_y+v2_y));
  angle = atan2f( v2_y + v1_y , v1_x + v2_x );
  return vec_create(speed,angle);
}

vec speed_vec_mul(vec *vec1 ,float k){
  vec1->x = vec1->x * k;
  return vec_create(vec1->x , vec1->y);
}


/*line_control函数中的PID*/
PID_Struct line_control_PID = {
5000,   //kp
0,      //Ki
50,      //Kd
0,      //i （中间变量）
0,      //last_error
500,    //i_max
0,      //last_d
0.005   //I_TIME
};
//以下变量为line_control内部调用
static int line_control_flag_first = 1;    //为1时：为第一次跑一个新点（设置这个flag的目的主要为减少计算次数）
static float origin_distance,origin_angle;
/**根据与轨迹（直线）的误差生成纠正的速度矢量
*参数：float p1_x，p1_y 地面坐标系下开始点的位置
*参数：float p2_x，p2_y 地面坐标系下开始点的位置
*参数：float max_speed 最大速度（限幅使用）
*返回值： 底盘驱动的方向矢量 vec.x 为速度大小，vec.y为速度方向（方向为：-PI~PI)
*说明: 纠正轨迹使用，配合point_tracer使用
*作者: zx
*/
vec line_control(float p1_x , float p1_y ,float p2_x,float p2_y, float max_speed){  
  float current_agnle , current_distance , error_vertical , error_on_origin ,angle_sub;
  float reference_point_x , reference_point_y;
  float target_angle , target_speed;
  //float out_put_speed = 0.0f;
  if(chassis_status.total_line_control_flag == 1){
    if(line_control_flag_first == 1){
    origin_angle = atan2f( p2_y - p1_y , p2_x - p1_x );    
    origin_distance = sqrtf( (p2_x - p1_x)*(p2_x - p1_x) + (p2_y - p1_y)*(p2_y - p1_y) );
    line_control_flag_first = 0;
    }  
    
    current_agnle = atan2f( p2_y - chassis.pos_y , p2_x - chassis.pos_x );
    current_distance = sqrtf( (p2_x - chassis.pos_x)*(p2_x - chassis.pos_x) + (p2_y - chassis.pos_y)*(p2_y - chassis.pos_y) );
    angle_sub = fabsf(origin_angle - current_agnle);
    error_on_origin = cosf(angle_sub)*current_distance;
    error_vertical = fabsf(sinf(angle_sub)*current_distance);  //控制量

    reference_point_x = (origin_distance - error_on_origin)*cosf(origin_angle) + p1_x;
    reference_point_y = (origin_distance - error_on_origin)*sinf(origin_angle) + p1_y;  

    target_angle = atan2f(reference_point_y - chassis.pos_y , reference_point_x - chassis.pos_x);   //用来矫正的速度矢量的方向
    
    if(error_vertical <= 1e-3){
      target_speed = 0;
    }else
    target_speed = fabsf(PID_Release(&line_control_PID , 0 , error_vertical));

    if(target_speed >= max_speed) target_speed = max_speed;
    if(target_speed <= -max_speed) target_speed = -max_speed;
  
    // what's beneath is the test progarm
    //uprintf("error_vertical : %f , speed : %f , angle : %f\r\n" ,error_vertical , target_speed , target_angle);
    //chassis_gostraight_zx( (int)target_speed , target_angle , 0 , 0);
  }
  return vec_create(target_speed , target_angle);
}


/**根据目标点更新新的速度方向
*参数：float point_y 	 point_x  地面坐标系下目标点的位置
*返回值： 底盘的速度方向角
*说明: by zx
*/
float point_tracer_angle_return( float point_x , float point_y ){
  //float origin_distance = sqrtf( (chassis.pos_x - point_x)*(chassis.pos_x - point_x) + (chassis.pos_y - point_y)*(chassis.pos_y - point_y) );
  //float out_put_speed = 0.0f;
  //float origin_angle = atan2f( point_y - chassis.pos_y , point_x - chassis.pos_x );
  //float current_agnle , current_distance , error_vertical;
  
  float chassis_target_x = point_x - chassis.pos_x;  
  float chassis_target_y = point_y - chassis.pos_y;   //坐标系变换，在保证角度环（小车坐标系不变的情况下），讲目标点由地面坐标系变换到小车坐标系中
  float mid = atan2f(chassis_target_y , chassis_target_x);
  float angle = chassis_angle_subtract( mid , chassis.angle);
  return angle;
}

float point_tracer_angle_return_static(float start_x, float start_y , float end_x , float end_y){
  float angle = atan2f( end_y-start_y , end_x-start_x );
  return angle;
}



//以下为 speed_trapezium 和 BoostAndSlowUpdate 用到的变量和参数
//目前先定义为静态，需要调整时在取消静态
//static float Boost_Slow_Period = 0.105;
static float Boost_Period = 0.1;
static float Slow_Period = 0.16;
int first_time_controler = 1; //值为1时：对一个新点使用此函数（不用多次调用BoostAndSlowUpdate）
static float last_point_x,last_point_y,origin_distance;

/**梯形速度曲线 加速阶段和减速阶段根据不同距离和速度进行自动调整 
*参数：float total_distance 起始点到终点的总距离 start_speed 开始时速度 final_speed结束时速度 max_speed最大速度
*返回值： 无
*说明: by zx 结合 speed_trapezium使用，speed_trapezium需要的函数
*/
void BoostAndSlowUpdate(float total_distance , int start_speed , int final_speed , int max_speed){
  Slow_Period = -0.195*total_distance + 0.00205*(max_speed - final_speed + 50) -0.7925;
  
  if(Slow_Period >= 0.6) Slow_Period = 0.6;  //Slow_Period 限制幅度
  else if(Slow_Period <= 0.1) Slow_Period = 0.1;
  else Slow_Period = Slow_Period;

  if(total_distance >= 5){    //boost_period 根据经验误差估计
    Boost_Period = 0.08;
  }
  else if(total_distance < 5 && total_distance > 1.5){
    Boost_Period = 0.12;
  }
  else{ //小于1.5m
    if(max_speed - start_speed >300)
    Boost_Period = 0.3;
    else Boost_Period = 0.1;
  }
}


/**速度曲线生成 梯形曲线
*参数：float point_y 	 point_x  地面坐标系下目标点的位置 start_speed 开始时速度 final_speed结束时速度 max_speed最大速度
*返回值： 电机当前的速度
*说明: by zx
*/
int speed_trapezium (float point_x , float point_y , int start_speed , int final_speed , int max_speed){
  float distance_to_target,speed;
  int int_speed;
  // if(first_time_controler == 0){
  //   this_time_pos = chassis.pos_y;
  //   uprintf(CMD_USART , "speed is : %f , %f , %f\r\n", (last_pos - this_time_pos)/0.005, chassis.pos_x, chassis.pos_y);
  //   last_pos = this_time_pos;
  // } //this part of code were uesd to test this function
  
  if(first_time_controler == 1){
    last_point_x = chassis.pos_x;
    last_point_y = chassis.pos_y;
    first_time_controler = 0;
    origin_distance = sqrtf( (point_x - last_point_x)*(point_x - last_point_x) + (point_y - last_point_y)*(point_y - last_point_y) );
    BoostAndSlowUpdate(origin_distance , start_speed , final_speed , max_speed);
    //last_pos = chassis.pos_y;
  }
  distance_to_target = sqrtf( (point_x - chassis.pos_x)*(point_x - chassis.pos_x) + (point_y - chassis.pos_y)*(point_y - chassis.pos_y) );

  distance_to_target =  1 - distance_to_target / origin_distance ; //正则化，使距离都限制在0~1范围内
  if(distance_to_target >=  1) distance_to_target = 1;
  if(distance_to_target <=  0) distance_to_target = 0;

  if(origin_distance <= 0.5) //如果距离小于50cm不进行梯形速度规划 使用从起始速度到最终速度的直接线性规划（一条连接起始速度和最终速度的直线）
  {
    speed = (final_speed - start_speed)*distance_to_target + start_speed;
  }
  else{
    //以下为一个分段函数，用这个函数计算速度 （梯形速度控制）
    if(distance_to_target <= Boost_Period){     //对于max_speed==start_speed 时进行优化
      if(start_speed == max_speed) speed = max_speed;
      else
      speed = (max_speed - start_speed)* distance_to_target/Boost_Period + start_speed; 
    }
    else if(distance_to_target > Slow_Period && distance_to_target <= 1 - Slow_Period){
      speed = max_speed;
    }
    else{
      if(final_speed == max_speed) int_speed = max_speed; //对于max_speed==final_speed 时进行优化
      else
      speed = (final_speed - max_speed)/ Slow_Period * distance_to_target + ( Slow_Period * final_speed - final_speed + max_speed)/ Slow_Period;
    }
  }
  int_speed = (int)speed;

  if(int_speed > max_speed) //速度限制
    int_speed = max_speed;
  else if(int_speed <= 0) 
    int_speed = 0;

  //uprintf(CMD_USART,"speed : %d  " , int_speed);
  return int_speed;
}


static int point_arrived_flag = 0; //到达点后为1
/**到达点后，将各种flag置位(重置至跑点前的状态)
*参数：void
*返回值： void
*说明: 为point_tracer内部函数，配合point_tracer使用
*作者: zx
*/
void point_arrive(){
  point_arrived_flag = 1;
  chassis_status.point_tracer_flag = 1;
  first_time_controler = 1;
  line_control_flag_first = 1;
  point_retrack_first_ref_flag = 1;  
  uprintf("\r\nArrived !!%d \r\n",chassis_status.count);
}

/**【核心函数】 point_tracer,跑点函数
*参数：float start_x , float start_y 为起始点地面坐标系坐标
*参数：float point_x , float point_y 为最终点地面坐标系坐标
*参数：int start_speed , int final_speed , int max_speed; start_speed 开始时速度，max_speed最大速度，final_speed到达终点时速度
*返回值： int; <1> 返回值为0时：可以开始跑点，并且未到达目标点
*返回值： int; <2> 返回值为-1时：目前不程序不允许跑点
*返回值： int; <3> 返回值为1时：到达目标点
*说明: 核心函数，用于跑点，最终配合point_collection_tracer（跑点集）使用
*作者: zx
*/
vec combine_vec , line_control_vec;   //point_tracer内部变量
int point_tracer (float start_x , float start_y ,float point_x , float point_y , int start_speed , int final_speed , int max_speed){
  float toward_angle,toward_speed;
  float distance = sqrtf( (chassis.pos_x - point_x)*(chassis.pos_x - point_x) + (chassis.pos_y - point_y)*(chassis.pos_y - point_y) );
  if( chassis_status.point_tracer_flag == 1 && distance >= ARRIVE_DISTANCE ) //可以开始跑点，并且未到达目标点
  { 
    //toward_angle = point_tracer_angle_return(point_x , point_y);
    toward_angle = point_tracer_angle_return_static(start_x , start_y , point_x , point_y);
    toward_speed = speed_trapezium (point_x , point_y , start_speed , final_speed , max_speed);
    
    //combine_vec = vec_create(toward_speed , toward_angle);
    line_control_vec = line_control(start_x , start_y , point_x , point_y , 700);
    //float k1 = toward_speed /1500;
    speed_vec_mul( &line_control_vec , 1 /*+ k1*/);
    combine_vec = speed_vec_add( vec_create(toward_speed , toward_angle) , line_control_vec);
    //chassis_update();
    chassis_gostraight_zx( (int)combine_vec.x , combine_vec.y , 0 , 0);
    return 0;
  }
  else {
    if(chassis_status.point_tracer_flag != 1) //不可跑点
    return -1;

    if(distance < ARRIVE_DISTANCE) //到达点
    {
      chassis_gostraight_zx( final_speed , combine_vec.y , 0 , 0);
      point_arrive();
      return 1;
    }
    
  }
  return -1;
}



static int point_count_control_flag = 0;    //点++的控制变量
/*
还是一个不成熟的功能
*/
int point_retrack(float start_x ,float start_y , float final_X , float final_y){
  static float total_distance;
  float distance_now;
  if(point_retrack_first_ref_flag == 1){
    total_distance = sqrtf( (final_X - start_x)*(final_X - start_x) + (final_y - start_y)*(final_y - start_y) );
    point_retrack_first_ref_flag = 0;
    return 1;
  }
  distance_now = sqrtf( (chassis.pos_x - start_x)*(chassis.pos_x - start_x) + (chassis.pos_y - start_y)*(chassis.pos_y - start_y) );
  if(distance_now > total_distance + ARRIVE_DISTANCE*1.5) 
  {
    chassis_status.count++;
    point_retrack_first_ref_flag = 0;
    return 0;
  }
  else return 1;
}
/**point_collection_tracer 跑点集函数：点集位于point_zx.c 文件中（具体点集一些的使用约定见point_zx.c）
*参数：int point_num; 跑点集中定义的点的数量
*说明: 最终目标函数，用于跑集，主要使用了point_tracer这个函数
*作者: zx
*/
void point_collection_tracer(int point_num){
  int mid_control = 0;
  if(chassis_status.ENBALE_POINT_COLLECTION_TRACER == 1 && chassis_status.count < point_num ){
    if(chassis_status.count < point_num-1){
      if(chassis_status.count == 0){
        //被注释的下一行为：正常跑点集使用的函数  
        //mid_control = point_tracer(zx_points_pos_x[chassis_status.count],zx_points_pos_y[chassis_status.count],zx_points_pos_x[chassis_status.count+1],zx_points_pos_y[chassis_status.count+1],speed_zx[chassis_status.count],speed_zx[chassis_status.count + 1] , max_speed_zx[chassis_status.count]);
        mid_control = point_tracer(zx_points_pos_x[chassis_status.count],zx_points_pos_y[chassis_status.count],zx_points_pos_x[chassis_status.count+1],zx_points_pos_y[chassis_status.count+1],650,650 , 500);
        //point_retrack(zx_points_pos_x[chassis_status.count],zx_points_pos_y[chassis_status.count],zx_points_pos_x[chassis_status.count+1],zx_points_pos_y[chassis_status.count+1]);
      }  
      if(chassis_status.count != 0){
        if(point_arrived_flag == 1){
          //被注释的下一行为：正常跑点集使用的函数        
          //mid_control = point_tracer(zx_points_pos_x[chassis_status.count],zx_points_pos_y[chassis_status.count],zx_points_pos_x[chassis_status.count+1],zx_points_pos_y[chassis_status.count+1],speed_zx[chassis_status.count],speed_zx[chassis_status.count + 1], max_speed_zx[chassis_status.count]);
          mid_control = point_tracer(zx_points_pos_x[chassis_status.count],zx_points_pos_y[chassis_status.count],zx_points_pos_x[chassis_status.count+1],zx_points_pos_y[chassis_status.count+1],650,650, 500);
          //point_retrack(zx_points_pos_x[chassis_status.count],zx_points_pos_y[chassis_status.count],zx_points_pos_x[chassis_status.count+1],zx_points_pos_y[chassis_status.count+1]);
        }
      }
    }
    else{   //chassis_status.count == point_num-1
      mid_control = point_tracer(zx_points_pos_x[chassis_status.count-1],zx_points_pos_y[chassis_status.count-1],zx_points_pos_x[chassis_status.count] ,zx_points_pos_y[chassis_status.count], 150 , 0 , 200);
      //point_retrack(zx_points_pos_x[chassis_status.count-1],zx_points_pos_y[chassis_status.count-1],zx_points_pos_x[chassis_status.count] ,zx_points_pos_y[chassis_status.count]);
    }
    /* 以下为chassis_status.count++的控制 */
    if(mid_control == 1){
      if(point_count_control_flag<=2){    //防止数值加的过大（实际上只需要加一次）
      point_count_control_flag ++;    
      }
    }
    else point_count_control_flag = 0;
    if(point_count_control_flag == 1){
      chassis_status.count++;
    }
  }
  
  if(chassis_status.count >= point_num){   //chassis_status.ENBALE_POINT_COLLECTION_TRACER == 0 && chassis_status.count>= point_num
    chassis_status.count = 0;    //跑点集计数清零
    chassis_gostraight_zx(0,0,chassis.angle,0); //停止的指令
    chassis_status.ENBALE_POINT_COLLECTION_TRACER = 0;   //关闭跑点集
    uprintf("END TRACER !");
  }
}

/**go_to_point_for_test go_to函数，用于测试中的回到初始位置或者一些特殊位置
*参数：float point_X , float point_y 最终目标点的位置
*说明: 应该只会用于调试中，速度矢量方向改变的算法，其余逻辑与point_tracer完全一致。
*作者: zx
*/
void go_to_point_for_test(float point_x , float point_y){
  float toward_angle,toward_speed;
  float distance = sqrtf( (chassis.pos_x - point_x)*(chassis.pos_x - point_x) + (chassis.pos_y - point_y)*(chassis.pos_y - point_y) );
  if( chassis_status.go_to_point_test_flag == 1 && distance >= ARRIVE_DISTANCE ) //可以开始跑点，并且未到达目标点
  { 
    toward_angle = point_tracer_angle_return(point_x , point_y);
    
    toward_speed = speed_trapezium (point_x , point_y , 125 , 50 , 850);
    combine_vec = vec_create(toward_speed , toward_angle);
    //chassis_update();
    chassis_gostraight_zx( (int)combine_vec.x , combine_vec.y , 0 , 0);
    return;
  }
  else {
    if(chassis_status.go_to_point_test_flag != 1){ //不可跑点
      chassis_gostraight_zx( 0 , combine_vec.y , chassis.angle , 0);
      return;
    }
    if(distance < ARRIVE_DISTANCE) //到达点
    {
      //chassis_gostraight_zx( 50 , combine_vec.y , 0 , 0);
      //point_arrived_flag = 1;
      //chassis_status.point_tracer_flag = 1;
      first_time_controler = 1;
      chassis_status.go_to_point_test_flag = 0;
      //chassis_status.go_to_point_test_flag = 1;
      //point_retrack_first_ref_flag = 1;  
      uprintf("\r\ngo_point_arrived %f,%f\r\n",chassis.pos_x , chassis.pos_y);
      chassis_gostraight_zx(0 , 0 , 0 , 0);
      return;
    }    
  }
  return;
}
/**state_reset 状态清零：用于跑点集结束后的状态清零，恢复到跑点前的状态
*参数：void
*说明: 调试时使用，用cmd_func中的函数调用
*作者: zx
*/
void state_reset(){
  chassis_status.count = 0;    //跑点集计数清零
  point_count_control_flag = 0;   //控制count++的变量
  chassis_status.ENBALE_POINT_COLLECTION_TRACER = 0;   //禁止跑点
  chassis_status.point_tracer_flag = 0;    //禁用point_tracer
  //以下逻辑等同于point_arrive 
  point_arrived_flag = 0;
  chassis_status.point_tracer_flag = 0;
  first_time_controler = 1;
  line_control_flag_first = 1;
  point_retrack_first_ref_flag = 1;
}