/*
 * @Description:
 * @Author: kx zhang
 * @Date: 2022-09-20 09:46:42
 * @LastEditTime: 2022-09-20 11:23:43
 */
#ifndef __MATH_OPS_H
#define __MATH_OPS_H

//#ifndef PI
//#define PI 3.14159265359f
//#endif

#include <stdio.h>

int float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);


#endif