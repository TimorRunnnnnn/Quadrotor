

#include  <stdarg.h>
#include  <stdio.h>
#include  <math.h>
#include  <stm32f4xx_hal.h>

#include  <cpu.h>
#include  <lib_math.h>
#include  <lib_mem.h>
#include  <os.h>
#include  <os_app_hooks.h>

#include  <app_cfg.h>
#include  <bsp.h>
#include "init.h"
#include "global.h"
#if (APP_CFG_SERIAL_EN == DEF_ENABLED)
#include  <app_serial.h>
#endif

/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/

/* --------------- APPLICATION GLOBALS ---------------- */
static  OS_TCB       AppTaskStartTCB;
static  CPU_STK      AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE];

static OS_TCB LEDAndFile_TCB;
static CPU_STK LED_STK[APP_STK_SIZE_LED_AND_FILE];

static OS_TCB AHRSTask_TCB;
static CPU_STK AHRS_STK[APP_STK_SIZE_AHRSTASK];

static OS_TCB ControlTask_TCB;
static CPU_STK ControlTask_STK[APP_STK_SIZE_CONTROLTASK];


static  void  AppTaskStart(void  *p_arg);
void LEDAndFileTask(void *p_arg);
void Uart3_tx_rx(void *p_arg);


int main(void) {
	OS_ERR   err;
#if (CPU_CFG_NAME_EN == DEF_ENABLED)
	CPU_ERR  cpu_err;
#endif
	CPU_SR_ALLOC();
	BSP_Init();
	HAL_Init();                                                 /* See Note 1.                                          */

	Mem_Init();                                                 /* Initialize Memory Managment Module                   */
	Math_Init();                                                /* Initialize Mathematical Module                       */

#if (CPU_CFG_NAME_EN == DEF_ENABLED)
	CPU_NameSet((CPU_CHAR *)"STM32F405RG",
		(CPU_ERR  *)&cpu_err);
#endif

	BSP_IntDisAll();                                            /* Disable all Interrupts.                              */

	OSInit(&err);                                               /* Init uC/OS-III.                                      */
	App_OS_SetAllHooks();

	OS_CRITICAL_ENTER();
        control.flag_Lock=1;
	OSTaskCreate(&AppTaskStartTCB,                              /* Create the start task                                */
		"App Task Start",
		AppTaskStart,
		0u,
		APP_CFG_TASK_START_PRIO,
		&AppTaskStartStk[0u],
		AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE / 10u],
		APP_CFG_TASK_START_STK_SIZE,
		0u,
		0u,
		0u,
		(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
		&err);

	OSStart(&err);                                              /* Start multitasking (i.e. give control to uC/OS-III). */
	OS_CRITICAL_EXIT();
	while (DEF_ON) {                                            /* Should Never Get Here.                               */
		;
	}
}


/*
*********************************************************************************************************
*                                          STARTUP TASK
*
* Description : This is an example of a startup task.  As mentioned in the book's text, you MUST
*               initialize the ticker only once multitasking has started.
*
* Arguments   : p_arg   is the argument passed to 'AppTaskStart()' by 'OSTaskCreate()'.
*
* Returns     : none
*
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
********************************************************c*************************************************
*/

static  void  AppTaskStart(void *p_arg) {
	OS_ERR      err;
	CPU_SR_ALLOC();
	(void)p_arg;
	OS_CRITICAL_ENTER();


	/* Initialize BSP functions                            */
	CPU_Init();                                                 /* Initialize the uC/CPU services                       */

#if OS_CFG_STAT_TASK_EN > 0u
	OSStatTaskCPUUsageInit(&err);                               /* Compute CPU capacity with no task running            */
#endif

#ifdef CPU_CFG_INT_DIS_MEAS_EN
	CPU_IntDisMeasMaxCurReset();
#endif

	InitUser();

	OSTaskCreate(&LEDAndFile_TCB,                              /* Create the start task                                */
		"LEDAndFileTask",
		LEDAndFileTask,
		0u,
		APP_PRIO_LED_AND_FILE,
		&LED_STK[0u],
		LED_STK[APP_STK_SIZE_LED_AND_FILE / 10u],
		APP_STK_SIZE_LED_AND_FILE,
		0u,
		0u,
		0u,
		(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
		&err);

	OSTaskCreate(&AHRSTask_TCB,
		"AHRS_Task",
		AHRS_Task,
		0,
		APP_PRIO_AHRSTASK,
		&AHRS_STK[0],
		AHRS_STK[APP_STK_SIZE_AHRSTASK / 10],
		APP_STK_SIZE_AHRSTASK,
		0,
		0,
		0,
		(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR | OS_OPT_TASK_SAVE_FP),
		&err);

	OSTaskCreate(&ControlTask_TCB,
		"ControlTask",
		ControlTask,
		0,
		APP_PRIO_CONTROLTASK,
		&ControlTask_STK[0],
		ControlTask_STK[APP_STK_SIZE_CONTROLTASK / 10],
		APP_STK_SIZE_CONTROLTASK,
		0,
		0,
		0,
		(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR | OS_OPT_TASK_SAVE_FP),
		&err);
	OS_CRITICAL_EXIT();
	OSTaskDel(0, &err);
}

/*void Uart3_tx_rx(void *p_arg)
{
OS_ERR err;
static uint8_t txbuffer[] = { "Uart3_Dma_Test!" };

while (DEF_TRUE)
{
//HAL_UART_Transmit_DMA(&Uart3_Handle, (uint8_t*)txbuffer, (COUNTOF(txbuffer) - 1));
OSTimeDlyHMSM(0u, 0u, 1u, 0u,
OS_OPT_TIME_HMSM_STRICT,
&err);
}
}*/