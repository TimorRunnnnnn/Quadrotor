#ifndef _ANDROID_H_
#define _ANDROID_H_
#include "stm32f4xx_hal.h"
#include "stdlib.h"
#include "function.h"
#include "Global.h"

#define HEAD_FIRST (0xAA)
#define TAIL_SECOND (0xAA)
#define HEAD_SECOND (0xBB)
#define TAIL_FIRST (0xBB)



#define REQUEST_PARAMETER (0x01)
#define REQUEST_UPDATAPARAMETER (0x02)
#define RESPOND_PARAMETER (0x11)
#define RESPOND_UPDATAPARAMETER (0x12)


#define REQUEST_GYRO_CAL 0x03
#define RESPOND_GYRO_CAL 0x13
#define REQUEST_MAG_CAL 0x04
#define RESPOND_MAG_CAL 0x14

#define REQUEST_GYRO_AND_MAG_OFFSET 0x05
#define RESPOND_GYRO_AND_MAG_OFFSET 0x15

extern short parameterHasChanged;

void AddParameter(char name[], float *currentValue, float step);
void SendParameter(void);
void SendMagCalData(void);
void SendGyroCalData(void);
void SendMagCalProcess(void);

#endif