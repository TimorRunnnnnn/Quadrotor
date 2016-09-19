/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * �ļ���  ��led.c
 * ����    ��led����Ӧ��
 * ʵ��ƽ̨��Air Nano���������
 * ��汾  ��ST3.5.0
 * ����    ��Air Nano Team
 * �Ա�    ��http://byd2.taobao.com
**********************************************************************************/
#include "Algorithm_math.h"

/*====================================================================================================*/
/*====================================================================================================*
**���� : Q_rsqrt
**���� : ���ټ��� 1/Sqrt(x)
**���� : number
**��� : ���
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
float Q_rsqrt(float number) {
	long i;
	float x2, y;
	const float threehalfs = 1.5F;

	x2 = number * 0.5F;
	y = number;
	i = *(long *)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float *)&i;
	y = y * (threehalfs - (x2 * y * y));   // 1st iteration ����һ��ţ�ٵ�����
	return y;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:    array_astrict_lower(int16_t *array,int16_t value)
*��������:    ��������������
���������    *array   Ŀ������ָ��
*             value
���������    ��
*******************************************************************************/
void array_astrict(int16_t *array, int16_t lower, int16_t upper) {
	int16_t length = sizeof(array);
	for (uint16_t i = 0; i < length; i++) {
		if (*(array + i) < lower)  *(array + i) = lower;
		else if (*(array + i) > upper)  *(array + i) = upper;
	}
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:    array_assign(int16_t *array,int16_t value)
*��������:    �����鸳ֵ
���������    *array   Ŀ������ָ��
*             value
���������    ��
*******************************************************************************/
void array_assign(uint16_t *array, int16_t value, uint16_t length) {
	for (uint16_t i = 0; i < length; i++) {
		*(array + i) = value;
	}
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:    data_limit(float data,flaot toplimit,float lowerlimit)
*��������:    �����޷�
���������    data       Ҫ����������
*             toplimit   ����
*             lowerlimit ����
���������    ��
*******************************************************************************/
float data_limit(float data, float toplimit, float lowerlimit) {
	if (data > toplimit)  data = toplimit;
	else if (data < lowerlimit) data = lowerlimit;
	return data;
}


/***********************************************
  * @brief  �ɱ���������Ӧ����
  * @param  None
  * @retval None
************************************************/
float VariableParameter(float error) {
	float  result = 0;

	if (error < 0) {
		error = -error;
	}
	if (error > 0.6f) {
		error = 0.6f;
	}
	result = 1 - 1.667 * error;
	if (result < 0) {
		result = 0;
	}
	return result;
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:    rad(double angle)
*��������:    �Ƕ�ת��Ϊ����
���������    �Ƕ�
���������    ����
*******************************************************************************/
float Rad(float angle) {
	return (angle * M_PI / 180.0);
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:    degree(double rad)
*��������:    ����ת��Ϊ�Ƕ�
���������    ����
���������    �Ƕ�
*******************************************************************************/
float Degree(float rad) {
	return (rad / M_PI * 180.0);
}

/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/
