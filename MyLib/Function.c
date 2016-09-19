#include "Function.h"

/**
* @brief  Compares two buffers.
* @param  pBuffer1, pBuffer2: buffers to be compared.
* @param  BufferLength: buffer's length
* @retval 0  : pBuffer1 identical to pBuffer2
*         >0 : pBuffer1 differs from pBuffer2
*/
uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength) {
	while (BufferLength--) {
		if ((*pBuffer1) != *pBuffer2) {
			return BufferLength;
		}
		pBuffer1++;
		pBuffer2++;
	}

	return 0;
}

uint8_t ReadKeyState(void) {
	return HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);
}


#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
void AHRS_SendData(int16_t a_x, int16_t a_y, int16_t a_z,
	int16_t g_x, int16_t g_y, int16_t g_z,
	int16_t m_x, int16_t m_y, int16_t m_z,
	int bar,
	float rol, float pit, float yaw) {
	static uint8_t data_to_send[60];
	uint8_t _cnt = 0;
	int16_t _temp;

	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0x02;
	data_to_send[_cnt++] = 0;

	_temp = a_x;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = a_z;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);

	_temp = g_x;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = g_y;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = g_z;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);

	_temp = m_x;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = m_y;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = m_z;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);

	data_to_send[3] = _cnt - 4;

	uint8_t sum = 0;
	for (uint8_t i = 0; i < _cnt; i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;

	int len = 0;
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0x01;

	len = _cnt;
	data_to_send[_cnt++] = 0;
	//int16_t _temp;
	_temp = (int)(rol * 100);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (int)(pit * 100);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (int)(yaw * 100);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	//int _temp2 = 0;
	data_to_send[_cnt++] = 0;
	data_to_send[_cnt++] = 0;
	data_to_send[_cnt++] = 0;
	data_to_send[_cnt++] = 0;

	//	data_to_send[_cnt++] = 0xA0;
	data_to_send[_cnt++] = 0xA1;

	data_to_send[len] = 11;

	sum = 0;
	for (uint8_t i = 23; i < _cnt; i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;

	HAL_UART_Transmit_DMA(BL_Handler, data_to_send, _cnt);
}



void Float2Byte(float *target, unsigned char *buf, unsigned char beg) {
	unsigned char *point;
	point = (unsigned char*)target; //得到float的地址
	buf[beg] = point[0];
	buf[beg + 1] = point[1];
	buf[beg + 2] = point[2];
	buf[beg + 3] = point[3];
}
void Byte2Float(float *target, unsigned char *buf, unsigned char beg) {
	unsigned char *point;
	point = (unsigned char*)target; //得到float的地址
	point[0] = buf[beg];
	point[1] = buf[beg + 1];
	point[2] = buf[beg + 2];
	point[3] = buf[beg + 3];
}
void Byte2Int(int *target, unsigned char *buf, unsigned char beg) {
	unsigned char *point;
	point = (unsigned char*)target; //得到float的地址
	point[0] = buf[beg];
	point[1] = buf[beg + 1];
	point[2] = buf[beg + 2];
	point[3] = buf[beg + 3];
}

void Int2Byte(int *target, unsigned char *buf, unsigned char beg) {
	unsigned char *point;
	point = (unsigned char*)target; //得到float的地址
	buf[beg] = point[0];
	buf[beg + 1] = point[1];
	buf[beg + 2] = point[2];
	buf[beg + 3] = point[3];
}


void Short2Byte(int16_t *target, uint8_t *buf, int beg) {
	int8_t *point;
	point = (int8_t*)target;
	buf[beg] = point[0];
	buf[beg + 1] = point[1];
}

void Byte2Short(short *target, unsigned char *buf, unsigned char beg) {
	unsigned char *point;
	point = (unsigned char*)target; //得到float的地址
	point[0] = buf[beg];
	point[1] = buf[beg + 1];
}