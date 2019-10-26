#ifndef __point_H
#define __point_H
#ifdef __cplusplus
extern "C" {
#endif
    
#include <math.h>
#include "chassis.h"

#define POINTS_NUM 200
#define LINE_NUM 20

typedef struct
{
  float points_pos_x[LINE_NUM][POINTS_NUM];
  float points_pos_y[LINE_NUM][POINTS_NUM];
  int trace_count;
}Trace;
    

extern float points_pos_x[POINTS_NUM];
extern float points_pos_y[POINTS_NUM];
extern int speed_each_point[POINTS_NUM + 1];
extern int max_speed_eachpoint[POINTS_NUM];

extern float points_pos_x1[POINTS_NUM];
extern float points_pos_y1[POINTS_NUM];
extern int speed_each_point1[POINTS_NUM + 1];
extern int max_speed_eachpoint1[POINTS_NUM];

extern float points_pos_x2[POINTS_NUM];
extern float points_pos_y2[POINTS_NUM];
extern int speed_each_point2[POINTS_NUM + 1];
extern int max_speed_eachpoint2[POINTS_NUM];

extern float points_pos_x3[POINTS_NUM];
extern float points_pos_y3[POINTS_NUM];
extern int speed_each_point3[POINTS_NUM + 1];
extern int max_speed_eachpoint3[POINTS_NUM];

extern float points_pos_x4[POINTS_NUM];
extern float points_pos_y4[POINTS_NUM];
extern int speed_each_point4[POINTS_NUM + 1];
extern int max_speed_eachpoint4[POINTS_NUM];

extern float points_pos_x5[POINTS_NUM];
extern float points_pos_y5[POINTS_NUM];
extern int speed_each_point5[POINTS_NUM + 1];
extern int max_speed_eachpoint5[POINTS_NUM];

extern float points_pos_x6[POINTS_NUM];
extern float points_pos_y6[POINTS_NUM];
extern int speed_each_point6[POINTS_NUM + 1];
extern int max_speed_eachpoint6[POINTS_NUM];

extern float points_pos_x7[POINTS_NUM];
extern float points_pos_y7[POINTS_NUM];
extern int speed_each_point7[POINTS_NUM + 1];
extern int max_speed_eachpoint7[POINTS_NUM];

extern float points_pos_x8[POINTS_NUM];
extern float points_pos_y8[POINTS_NUM];
extern int speed_each_point8[POINTS_NUM + 1];
extern int max_speed_eachpoint8[POINTS_NUM];

extern float points_pos_x9[POINTS_NUM];
extern float points_pos_y9[POINTS_NUM];
extern int speed_each_point9[POINTS_NUM + 1];
extern int max_speed_eachpoint9[POINTS_NUM];

extern float direct0[POINTS_NUM],direct1[POINTS_NUM],direct2[POINTS_NUM],direct3[POINTS_NUM],direct4[POINTS_NUM],direct5[POINTS_NUM],direct6[POINTS_NUM],direct7[POINTS_NUM],direct8[POINTS_NUM],direct9[POINTS_NUM];

void trace_init();
void calculate_the_direct(float points_pos_x[],float points_pos_y[],float direct[],int point_num);
void direct_init(int point_num);
    
#ifdef __cplusplus
}
#endif
#endif /*__ points_H */