#ifndef _CONTROL_H_
#define _CONTROL_H_
#include "Global.h"

struct _PIDStruct {
	float kp;
	float kd;
	float ki;

	float integral;
	float pidOut;
};
struct _Attitude {
	float yaw;
	float pitch;
	float roll;
	float altitude;
};
struct _Gyroscope {
	float pitch;
	float roll;
	float yaw;
};
struct _Outer {
	struct _PIDStruct pitch;
	struct _PIDStruct roll;
	struct _PIDStruct yaw;
};
struct _Inner {
	struct _PIDStruct pitch;
	struct _PIDStruct roll;
	struct _PIDStruct yaw;
};
struct _Altitude {
	float kp;
	float kd;
	float ki;

	float integral;
	float pidOut;

	float currentAltitude;
	float targetAltitude;
};

struct _ControlStruct {
	float limitRollDegree;
	float limitPitchDegree;
	float limitYawDPS;
	//float limitPIDOut;
	float limitInnerIntegral;
	float limitOutterIntegral;
	float limitYawIntegral;
	//float MotorMAX;

	short yawMode;
	short flag_Lock;
	//short flag_Flying;
	short PWMCapture[7];
	short PWMOutput[4];//us

	short motorOut[4];

	struct _Altitude altitude;
	struct _Gyroscope gyro;
	struct _Gyroscope lastGyro;
	struct _Attitude targetAttitude;
	struct _Attitude currentAttitude;
	struct _Outer outer;
	struct _Inner inner;
};

void ControlTask(void *parg);

extern short needLoadParameterFromFile;
extern OS_Q controlQueue;
extern struct _ControlStruct control;

#endif