#ifndef _FUNCTION_H_
#define _FUNCTION_H_
#include "stm32f4xx_hal.h"
#include "global.h"
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
#define Byte16(Type, ByteH, ByteL)  ((Type)((((uint16_t)(ByteH))<<8) | ((uint16_t)(ByteL))))


uint8_t ReadKeyState(void);
uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);
void Float2Byte(float *target, unsigned char *buf, unsigned char beg);
void Byte2Float(float *target, unsigned char *buf, unsigned char beg);
void Int2Byte(int *target, unsigned char *buf, unsigned char beg);
void Byte2Int(int *target, unsigned char *buf, unsigned char beg);
void Short2Byte(int16_t *target, uint8_t *buf, int beg);
void Byte2Short(short *target, unsigned char *buf, unsigned char beg);
void AHRS_SendData(int16_t a_x, int16_t a_y, int16_t a_z,
	int16_t g_x, int16_t g_y, int16_t g_z,
	int16_t m_x, int16_t m_y, int16_t m_z,
	int bar,
	float rol, float pit, float yaw);


#endif