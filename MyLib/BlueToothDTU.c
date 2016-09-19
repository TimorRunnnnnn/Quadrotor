#include "BlueToothDTU.h"
#include "Function.h"
#include "string.h"

#define BL_PSWD {'0','1','2','6'}
#define BL_NAME {'E','R','B','A','I'}

#define BL_KEY_PORTx GPIOC
#define BL_KEY_PINx GPIO_PIN_13
#define BL_STATE_PORTx GPIOC
#define BL_STATE_PINx GPIO_PIN_14
#define BL_ENCLK() __GPIOC_CLK_ENABLE()

#define BL_NORMAL_BAUD 115200
#define BL_ORGL_BAUD 38400

UART_HandleTypeDef *BL_Handler;


static void BlSimpleDelay(void)
{
	for (int i = 0; i < 168000; i++)
	{
		asm("nop");
	}
}

void Bl_Hander_Bind(UART_HandleTypeDef *huart)
{
	BL_Handler = huart;
}

void Bl_Init(UART_HandleTypeDef *huart)
{
	Bl_Hander_Bind(huart);

	GPIO_InitTypeDef GPIO_InitStruct;
	BL_ENCLK();

	GPIO_InitStruct.Pin = BL_KEY_PINx;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(BL_KEY_PORTx, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = BL_STATE_PINx;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(BL_STATE_PORTx, &GPIO_InitStruct);
        
    
}

uint8_t BL_ReadBLState(void)
{
	return HAL_GPIO_ReadPin(BL_STATE_PORTx, BL_STATE_PINx);
}

uint8_t Bl_EnterAT(void)
{
	uint8_t tx[] = { "AT\r\n" };
	uint8_t rx[4] = { 0, 0, 0, 0 };
	uint8_t ok[4] = { 'O', 'K', '\r', '\n' };
	int t = 4;

	HAL_GPIO_WritePin(BL_KEY_PORTx, BL_KEY_PINx, GPIO_PIN_SET);

	//初始化波特率
	BL_Handler->Init.BaudRate = BL_ORGL_BAUD;
	HAL_UART_Init(BL_Handler);
	BlSimpleDelay();
	while (t > 0)
	{
		t--;
		HAL_UART_Transmit(BL_Handler, tx, 4, 100);
		HAL_UART_Receive(BL_Handler, rx, 4, 100);
		BlSimpleDelay();
		__HAL_UART_FLUSH_DRREGISTER(BL_Handler);
		if (Buffercmp(rx, ok, 4) == 0)
		{
			return SUCCESS;
		}
	}
	//切换到正常波特率
        t = 4;
	BL_Handler->Init.BaudRate = BL_NORMAL_BAUD;
	HAL_UART_Init(BL_Handler);
	BlSimpleDelay();
	while (t > 0)
	{
		t--;
		HAL_UART_Transmit(BL_Handler, tx, 4, 100);
		HAL_UART_Receive(BL_Handler, rx, 4, 100);
		BlSimpleDelay();
		__HAL_UART_FLUSH_DRREGISTER(BL_Handler);
		if (Buffercmp(rx, ok, 4) == 0)
		{
			return SUCCESS;
		}
	}
	return ERROR;
}

uint8_t Bl_ParameterInit(void)
{
#define PARAM_NUM 5
	uint8_t param[PARAM_NUM][20] = {
		{ "AT+NAME=ErBai\r\n" },
		{ "AT+ROLE=0\r\n" },
		{ "AT+UART=115200,0,0\r\n" },
		{ "AT+CMODE=1\r\n" },
		{ "AT+PSWD=0126\r\n" }
	};
	uint8_t rx[4] = { 0, 0, 0, 0 };
	uint8_t ok[4] = { 'O', 'K', '\r', '\n' };

	if (Bl_EnterAT() != SUCCESS)
	{
		return ERROR;
	}
	for (int i = 0; i < PARAM_NUM; i++)
	{
		int timeout = 0;
		while (1)
		{
			timeout++;
			if (timeout>10)
			{
				return ERROR;
			}
			HAL_UART_Transmit(BL_Handler, param[i], (strlen((char*)param[i])), 100);//减掉最后的\0
			HAL_UART_Receive(BL_Handler, rx, 4, 100);
			BlSimpleDelay();
			__HAL_UART_FLUSH_DRREGISTER(BL_Handler);
			if (Buffercmp(rx, ok, 4) == 0)
			{
				break;
			}
			else
			{
				BlSimpleDelay();
			}
		}
	}
	return SUCCESS;
}