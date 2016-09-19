/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * 文件名  ：led.c
 * 描述    ：led函数应用
 * 实验平台：Air Nano四轴飞行器
 * 库版本  ：ST3.5.0
 * 作者    ：Air Nano Team
 * 淘宝    ：http://byd2.taobao.com
**********************************************************************************/
#include "Algorithm_math.h"

/*====================================================================================================*/
/*====================================================================================================*
**函数 : Q_rsqrt
**功能 : 快速计算 1/Sqrt(x)
**输入 : number
**输出 : 结果
**备注 : None
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
	y = y * (threehalfs - (x2 * y * y));   // 1st iteration （第一次牛顿迭代）
	return y;
}

/**************************实现函数********************************************
*函数原型:    array_astrict_lower(int16_t *array,int16_t value)
*功　　能:    对数组下限限制
输入参数：    *array   目标数组指针
*             value
输出参数：    无
*******************************************************************************/
void array_astrict(int16_t *array, int16_t lower, int16_t upper) {
	int16_t length = sizeof(array);
	for (uint16_t i = 0; i < length; i++) {
		if (*(array + i) < lower)  *(array + i) = lower;
		else if (*(array + i) > upper)  *(array + i) = upper;
	}
}

/**************************实现函数********************************************
*函数原型:    array_assign(int16_t *array,int16_t value)
*功　　能:    对数组赋值
输入参数：    *array   目标数组指针
*             value
输出参数：    无
*******************************************************************************/
void array_assign(uint16_t *array, int16_t value, uint16_t length) {
	for (uint16_t i = 0; i < length; i++) {
		*(array + i) = value;
	}
}

/**************************实现函数********************************************
*函数原型:    data_limit(float data,flaot toplimit,float lowerlimit)
*功　　能:    数据限幅
输入参数：    data       要操作的数据
*             toplimit   上限
*             lowerlimit 下限
输出参数：    无
*******************************************************************************/
float data_limit(float data, float toplimit, float lowerlimit) {
	if (data > toplimit)  data = toplimit;
	else if (data < lowerlimit) data = lowerlimit;
	return data;
}


/***********************************************
  * @brief  可变增益自适应参数
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
/**************************实现函数********************************************
*函数原型:    rad(double angle)
*功　　能:    角度转化为弧度
输入参数：    角度
输出参数：    弧度
*******************************************************************************/
float Rad(float angle) {
	return (angle * M_PI / 180.0);
}
/**************************实现函数********************************************
*函数原型:    degree(double rad)
*功　　能:    弧度转化为角度
输入参数：    弧度
输出参数：    角度
*******************************************************************************/
float Degree(float rad) {
	return (rad / M_PI * 180.0);
}

/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/
