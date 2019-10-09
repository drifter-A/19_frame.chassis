#ifndef __point_zx_H
#define __point_zx_H
#ifdef __cplusplus
extern "C" {
#endif
    
#include <math.h>
#include "chassis.h"

#define ZX_POINTS_NUM 200
    

extern float zx_points_pos_x[ZX_POINTS_NUM];

extern float zx_points_pos_y[ZX_POINTS_NUM];

extern int speed_zx[ZX_POINTS_NUM + 1];
extern int max_speed_zx[ZX_POINTS_NUM];
    
#ifdef __cplusplus
}
#endif
#endif /*__ points_H */