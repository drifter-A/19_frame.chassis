#include "vec.h"
#include "math.h"

vec vec_add(vec a, vec b)
{
	vec tmp;
	tmp.x = a.x + b.x;
	tmp.y = a.y + b.y;
	return tmp;
}

double vec_mul(vec a, vec b)
{
	return a.x * b.x + a.y * b.y;
}

double vec_model(vec a)
{
	return sqrt(a.x * a.x + a.y * a.y);
}

vec vec_create(float x, float y)
{
	vec tmp;
	tmp.x = x;
	tmp.y = y;
	return tmp;
}

vec vec_mul_i(vec a, double b)
{
	a.x *= b;
	a.y *= b;
	return a;
}

vec vec_normal(vec a)//顺时针90度法向 
{
	vec tmp;
	tmp.x = a.y;
	tmp.y = -a.x;
	return vec_mul_i(tmp,1/vec_model(tmp));
}

vec vec_unit(vec a)
{
	double b = vec_model(a);
	return vec_mul_i(a,1/b);
}

int vec_is_zero(vec a)
{
    if(fabs(a.x) < 1e-6 && fabs(a.y) < 1e-6)
        return 1;
    return 0;
}