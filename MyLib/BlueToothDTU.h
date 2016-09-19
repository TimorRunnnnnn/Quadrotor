#ifndef _BLUETOOTHDTU_H_
#define _BLUETOOTHDTU_H_
#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef *BL_Handler;

void Bl_Init(UART_HandleTypeDef *huart);
uint8_t Bl_EnterAT(void);
uint8_t Bl_ParameterInit(void);
uint8_t BL_ReadBLState(void);


#endif