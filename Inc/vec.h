#ifndef __vec_H
#define __vec_H
#ifdef __cplusplus
 extern "C" {
#endif
     
typedef struct
{
    float x;
    float y;
}vec;
vec vec_add(vec a, vec b);      
double vec_mul(vec a, vec b);
double vec_model(vec a);
vec vec_create(float x, float y);
vec vec_mul_i(vec a, double b);
vec vec_normal(vec a);
vec vec_unit(vec a);    
int vec_is_zero(vec a);
#ifdef __cplusplus
}
#endif
#endif
