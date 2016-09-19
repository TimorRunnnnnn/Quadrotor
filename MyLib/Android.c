#include "Android.h"
#define BLUETOOTH_UART USART3

void SendRespond(uint8_t respond);

extern UART_HandleTypeDef *BL_Handler;
extern CRC_HandleTypeDef hcrc;

typedef struct {
	float *currentValue;
	float step;
	char name[20];
}ParameterStructDef;

typedef struct NODE {
	ParameterStructDef parameter;
	uint8_t address;
	struct NODE *next;
}NODE;


#define PARAMETER_MAXNUM 30
static NODE *head = NULL;
static NODE *nodeArr[PARAMETER_MAXNUM];

short parameterHasChanged = 0;


short IsAllowChangeParameter(void) {
	//在解锁的情况下不允许修改参数
	if (control.flag_Lock == 1) {
		return 1;
	} else {
		return 0;
	}
}

void AddParameter(char name[], float *currentValue, float step) {
	static uint8_t address_s = 0;
	NODE *node = (NODE*)malloc(sizeof(NODE));
	if (node != NULL) {
		for (int i = 0; i < 20; i++) {
			node->parameter.name[i] = name[i];
			if (name[i] == '\0')
				break;
		}

		node->address = address_s;
		address_s++;

		node->parameter.currentValue = currentValue;
		node->parameter.step = step;

		if (address_s <= PARAMETER_MAXNUM) {//如果大于了最大参数数量则放弃
			if (head == NULL) {
				head = node;
				head->next = NULL;
				nodeArr[address_s - 1] = head;
			} else {
				NODE *tem = head;
				while (tem->next != NULL) {
					tem = tem->next;
				}
				nodeArr[address_s - 1] = node;
				tem->next = node;
				node->next = NULL;
			}
		}
	}
}

void crcDelay(void) {
	for (int i = 0; i < 4; i++) {
		asm("nop");
	}
}

void SendParameter(void) {
	NODE *tem = head;
	int countOfParameter = 1;
	if (tem == NULL) {
		return;
	} else {
		while (tem->next != NULL) {
			tem = tem->next;
			countOfParameter++;
		}
	}
	uint8_t *sendBuffer = (uint8_t*)malloc(sizeof(uint8_t)*countOfParameter * 31 + 11);

	sendBuffer[0] = HEAD_FIRST;
	sendBuffer[1] = HEAD_SECOND;
	sendBuffer[4] = RESPOND_PARAMETER;

	int dataCount = 0;
	for (int i = 0; i < countOfParameter; i++) {
		sendBuffer[5 + dataCount] = '<';
		dataCount++;
		int cnt = 0;
		while (nodeArr[i]->parameter.name[cnt] != '\0') {
			sendBuffer[5 + dataCount] = nodeArr[i]->parameter.name[cnt];
			cnt++;
			dataCount++;
		}//复制了name
		sendBuffer[5 + dataCount] = '|';
		dataCount++;
		sendBuffer[5 + dataCount] = nodeArr[i]->address;
		dataCount++;
		Float2Byte(nodeArr[i]->parameter.currentValue, &sendBuffer[5 + dataCount], 0);
		dataCount += 4;
		Float2Byte(&nodeArr[i]->parameter.step, &sendBuffer[5 + dataCount], 0);
		dataCount += 4;
	}
	int r = 4 - ((dataCount) % 4);
	for (int i = 0; i < r; i++) {
		sendBuffer[5 + dataCount] = 0;
		dataCount++;
	}
	int frameCount = dataCount + 5;
	//这里生成4个字节的CRC校验;
	//crc调不通,换求和校验,
	int checkSum = 0;
	for (int i = 5; i < dataCount + 5; i++) {
		checkSum += (int8_t)sendBuffer[i];//为了配合java
	}
	Int2Byte(&checkSum, &sendBuffer[frameCount], 0);

	frameCount += 4;
	sendBuffer[frameCount] = TAIL_FIRST;
	frameCount++;
	sendBuffer[frameCount] = TAIL_SECOND;
	frameCount++;

	short len = frameCount;
	Short2Byte(&len, &sendBuffer[2], 0);
	//HAL_UART_Transmit(BL_Handler, sendBuffer, frameCount, 100);
	HAL_UART_Transmit_DMA(BL_Handler, sendBuffer, frameCount);
	//free(sendBuffer);
}

static uint8_t* CreatFrame(int len, uint8_t command) {
	//生成一个帧
	uint8_t *buffer = (uint8_t*)malloc((len)*sizeof(uint8_t));
	buffer[0] = HEAD_FIRST;
	buffer[1] = HEAD_SECOND;
	buffer[2] = len;
	buffer[3] = 0;
	buffer[4] = command;
	buffer[len - 2] = TAIL_FIRST;
	buffer[len - 1] = TAIL_SECOND;
	return buffer;
}
static void GenerateCheckSum(uint8_t *buffer, int start, int num) {
	//生成校验和并放到buffer[num+start]的后四个字节
	int checkSum = 0;
	for (int i = 0; i < num; i++) {
		checkSum += (int8_t)buffer[start + i];
	}
	Int2Byte(&checkSum, buffer, start + num);
}
void SendOffset(void) {
	//进入校准界面的时候请求的数据
	uint8_t *buffer = CreatFrame(11 + 4 * 10, RESPOND_GYRO_AND_MAG_OFFSET);
	for (int j = 0; j < 3; j++) {
		Float2Byte(&SensorStatue.offsetGyro[j], buffer, 5 + j * 4);
	}
	for (int i = 0; i < 3; i++) {
		Float2Byte(&SensorStatue.magMax[i], buffer, 5 + 12 + i * 4);
	}
	for (int i = 0; i < 3; i++) {
		Float2Byte(&SensorStatue.magMin[i], buffer, 5 + 12 + 12 + i * 4);
	}
	int tem = SensorStatue.gyroCalTime / 1000;
	Int2Byte(&tem, buffer, 5 + 36);
	// 	int checkSum = 0;
	// 	for (int i = 0; i < 40; i++) {
	// 		checkSum += (int8_t)buffer[5 + i];
	// 	}
	// 	Int2Byte(&checkSum, buffer, 5 + 40);
	GenerateCheckSum(buffer, 5, 40);
	HAL_UART_Transmit_DMA(BL_Handler, buffer, 51);
}
void SendGyroCalData(void) {
	//校准完成后发送校准完的数据
	uint8_t *buffer = CreatFrame(11 + 4 * 3, RESPOND_GYRO_CAL);
	for (int j = 0; j < 3; j++) {
		Float2Byte(&SensorStatue.offsetGyro[j], buffer, 5 + j * 4);
	}
	GenerateCheckSum(buffer, 5, 12);
	HAL_UART_Transmit_DMA(BL_Handler, buffer, 23);
}
void SendMagCalData(void) {
	//校准完成后发送校准完的数据
	uint8_t *buffer = CreatFrame(11 + 4 * 6, RESPOND_MAG_CAL);
	for (int j = 0; j < 3; j++) {
		Float2Byte(&SensorStatue.magMax[j], buffer, 5 + j * 4);
	}
	for (int j = 0; j < 3; j++) {
		Float2Byte(&SensorStatue.magMin[j], buffer, 5 + 12 + j * 4);
	}
	GenerateCheckSum(buffer, 5, 24);

	HAL_UART_Transmit_DMA(BL_Handler, buffer, 35);
}
void SendMagCalProcess(void) {
	uint8_t *buffer = CreatFrame(11 + 4 * 3, RESPOND_MAG_CAL);

	for (int j = 0; j < 3; j++) {
		Float2Byte(&SensorStatue.magCalAngle[j], buffer, 5 + j * 4);
	}
	GenerateCheckSum(buffer, 5, 12);
	HAL_UART_Transmit_DMA(BL_Handler, buffer, 23);
}

void SendRespond(uint8_t respond) {
	uint8_t *buffer = CreatFrame(15, respond);
	GenerateCheckSum(buffer, 5, 4);
	HAL_UART_Transmit_DMA(BL_Handler, buffer, 15);
}

static uint8_t reciveData[1000];
static int reciveSize = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		__HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
		//校验帧尾
		if (reciveData[reciveSize - 4 - 1] != TAIL_SECOND&&reciveData[reciveSize - 4 - 2] != TAIL_FIRST) {
			asm("nop");
			return;
		}
		//求和校验
		int checkSum = 0;
		for (int i = 1; i < reciveSize - 6 - 4; i++) {
			checkSum += (int8_t)reciveData[i];
		}
		int reciveCheckSum = 0;
		Byte2Int(&reciveCheckSum, reciveData, reciveSize - 6 - 4);
		if (reciveCheckSum != checkSum) {
			asm("nop");
			return;
		}
		if (IsAllowChangeParameter() == 1 || reciveData[0] == REQUEST_PARAMETER) {
			//允许请求参数不允许修改参数
			//下面根据命令字来处理一些事
			if (reciveData[0] == REQUEST_PARAMETER) {
				SendParameter();
			} else if (reciveData[0] == REQUEST_UPDATAPARAMETER) {
				//SendRespond(RESPOND_UPDATAPARAMETER);
				parameterHasChanged = 1;//参数已修改
				for (int i = 0; i < ((reciveSize - 11) / 5); i++) {
					Byte2Float(nodeArr[reciveData[1 + i * 5]]->parameter.currentValue, reciveData, i * 5 + 2);
				}
			} else if (reciveData[0] == REQUEST_GYRO_CAL) {
				SensorStatue.flagGyroCal = 1;
				SensorStatue.gyroCalTime = reciveData[1] * 1000;
			} else if (reciveData[0] == REQUEST_MAG_CAL) {
				Mem_Set(&SensorStatue.magMax, 0, 3 * sizeof(float));
				Mem_Set(&SensorStatue.magMin, 0, 3 * sizeof(float));
				SensorStatue.flagMagCal = 1;
			} else if (reciveData[0] == REQUEST_GYRO_AND_MAG_OFFSET) {
				SendOffset();
			}
		}
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		if (!(BL_Handler->pTxBuffPtr[0] == 0xAA && BL_Handler->pTxBuffPtr[1] == 0xAA)) {//如果是发往匿名地面站的数据包则不释放数据,尽量少用malloc!
			free(BL_Handler->pTxBuffPtr);
		}
	}
}
void USART3_IRQHandler(void) {
	/* USER CODE BEGIN USART3_IRQn 0 */
	static uint8_t reciveBuffer[4];
	static int count = 0;

	if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE) == SET) {
		HAL_UART_Receive(&huart3, &reciveBuffer[count], 1, 1);
		if (count == 0) {
			if (reciveBuffer[0] != HEAD_FIRST) {
				count = 0;
			} else {
				count++;
			}
		} else if (count == 1) {
			if (reciveBuffer[1] != HEAD_SECOND) {
				if (reciveBuffer[1] == HEAD_FIRST) {
					count = 1;
				} else {
					count = 0;
				}
			} else {
				count++;
			}
		} else if (count < 4) {
			count++;
			if (count == 4) {
				count = 0;
				short x = 0;
				Byte2Short(&x, &reciveBuffer[2], 0);
				if (!(reciveSize<0||reciveSize>500))
				{
					__HAL_UART_DISABLE_IT(&huart3, UART_IT_RXNE);
					reciveSize = x;
					HAL_UART_Receive_DMA(&huart3, reciveData, x - 4);
				}
			}
		}
	} else {
		HAL_UART_IRQHandler(&huart3);
	}
}
