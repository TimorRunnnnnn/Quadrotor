#ifndef __Algorithm_quaternion_H
#define	__Algorithm_quaternion_H

#include "Algorithm_math.h"
typedef  struct {
  float Pitch;
  float Roll;
  float Yaw;
} EulerAngle;

typedef  struct {
  float q0;
  float q1;
  float q2;
  float q3;
} Quaternion;

typedef  struct {
  float x;
  float y;
  float z;
} Gravity;


extern EulerAngle AngE;

Gravity Quaternion_vectorGravity( Quaternion *pNumQ );
void Quaternion_ToNumQ( Quaternion *pNumQ, EulerAngle *pAngE );
void Quaternion_ToAngE( Quaternion *pNumQ, EulerAngle *pAngE );
Quaternion Quaternion_Multiply_( Quaternion NowQ, Quaternion OldQ );
void Quaternion_Normalize_( Quaternion *pNumQ );
void Quaternion_RungeKutta( Quaternion *pNumQ, float GyrX, float GyrY, float GyrZ, float helfTimes );
#endif /* __Algorithm_quaternion_H */
