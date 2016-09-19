#ifndef __Algorithm_math_H
#define	__Algorithm_math_H

#include "arm_math.h"
#include <math.h>

#define M_PI  (float)3.1415926535
#define squa( Sq )        (((float)Sq)*((float)Sq))
#define toRad( Math_D )	  ((float)(Math_D)*0.0174532925f)
#define toDeg( Math_R )	  ((float)(Math_R)*57.2957795f)
#define absu16( Math_X )  (Math_X<0? -Math_X:Math_X)
#define absFloat( Math_X )(Math_X<0? -Math_X:Math_X)


float Q_rsqrt(float number);
float VariableParameter(float error);
void array_assign(uint16_t *array,int16_t value,uint16_t length);
void array_astrict(int16_t *array,int16_t lower,int16_t upper);
float data_limit(float data,float toplimit,float lowerlimit);
float Degree(float rad);
float Rad(float angle);
#endif /* __Algorithm_math_H */
