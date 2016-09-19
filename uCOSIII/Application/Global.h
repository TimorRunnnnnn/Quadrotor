#ifndef _GLOBAL_H_
#define _GLOBAL_H_
#include  <stdarg.h>
#include  <stdio.h>
#include  <math.h>
#include  <stm32f4xx_hal.h>

#include  <cpu.h>
#include  <lib_math.h>
#include  <lib_mem.h>
#include  <os.h>
#include  <os_app_hooks.h>

#include  <bsp.h>
#include "stm32f4xx_it.h"
#include "stm32f405xx.h"
#include "stdint.h"
#include "android.h"
#include "BlueToothDTU.h"
#include "AHRS.h"
#include "Init.h"
#include "Control.h"
#include "Algorithm_math.h"
#include "ms5611.h"

#include "bsp_driver_sd.h"
#include "fatfs.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h"
#include "minIni.h"
#include "filefunction.h"

#define MPU9250_ON  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET)
#define MPU9250_OFF HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET)

#define MS5611_ON HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET)
#define MS5611_OFF HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET)


#define MPU_DT (0.0002f)
#define TIMES (10)   //ÂË²¨´ÎÊý
#define INTEGRATION_TIME	(MPU_DT*TIMES)

#define ABS(x) ((x>0)?(x):(-x))

#define LED_RED_ON() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET)
#define LED_RED_OFF() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET)
#define LED_RED_TOGGLE() HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_0)
//#define LED_BLUE_ON() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET)
//#define LED_BLUE_OFF() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET)
#define LED_BLUE_ON()  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 99)
#define LED_BLUE_OFF()  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0)
#define LED_BLUE_TOGGLE() HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_1)



#endif