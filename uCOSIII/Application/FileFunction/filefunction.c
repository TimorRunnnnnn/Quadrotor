#include "filefunction.h"
#include "stdlib.h"
#define SECTION_OFFSET "OFFSET"



#define KEY_GYRO_CAL_TIME "GYRO_CAL_TIME"

#define KEY_OFFSET_GYRO_X "GYRO_X"
#define KEY_OFFSET_GYRO_Y "GYRO_Y"
#define KEY_OFFSET_GYRO_Z "GYRO_Z"
#define KEY_OFFSET_MAG_MAX_X "MAG_MAX_X"
#define KEY_OFFSET_MAG_MAX_Y "MAG_MAX_Y"
#define KEY_OFFSET_MAG_MAX_Z "MAG_MAX_Z"
#define KEY_OFFSET_MAG_MIN_X "MAG_MIN_X"
#define KEY_OFFSET_MAG_MIN_Y "MAG_MIN_Y"
#define KEY_OFFSET_MAG_MIN_Z "MAG_MIN_Z"

#define FILE_NAME "parameter.ini"
//char fileParameter[] = "parameter.ini";

typedef struct _StorageParameterNode {
	float *currentValue;
	float oldValue;
	char keyWord[20];//¼ü
	char section[20];

	struct _StorageParameterNode *next;
}StorageParameterNode;

#define STORAGE_PARAMETER_MAXNUM 30
static StorageParameterNode *head;

void AddStorageParameter(char section[], char keyWord[], float *currentValue) {
	static short address_s = 0;
	StorageParameterNode *node = (StorageParameterNode*)malloc(sizeof(StorageParameterNode));
	if (node != NULL) {
		for (int i = 0; i < 20; i++) {
			node->keyWord[i] = keyWord[i];
			if (keyWord[i] == '\0') {
				break;
			}
		}
		for (int i = 0; i < 20; i++) {
			node->section[i] = section[i];
			if (section[i] == '\0') {
				break;
			}
		}
		node->currentValue = currentValue;
		address_s++;
		if (address_s <= STORAGE_PARAMETER_MAXNUM) {
			if (head == NULL) {
				head = node;
				head->next = NULL;
			} else {
				StorageParameterNode *tem = head;
				while (tem->next != NULL) {
					tem = tem->next;
				}
				tem->next = node;
				node->next = NULL;
			}
		}
	}
}



short LoadParameterFromFile(void) {
	StorageParameterNode *tem = head;
	char tempread[20];
	CPU_SR_ALLOC();
	OS_CRITICAL_ENTER();
	while (tem != NULL) {
		ini_gets(tem->section, tem->keyWord, "0", tempread, 20, FILE_NAME);
		*tem->currentValue = atof(tempread);
		tem->oldValue = *(tem->currentValue);
		tem = tem->next;
	}
	OS_CRITICAL_EXIT();
	return 1;//OK
}

short WriteParameterToFile(void) {
	CPU_SR_ALLOC();
	OS_CRITICAL_ENTER();
	StorageParameterNode *tem = head;
	char tempWrite[20];
	while (tem != NULL) {
		if (tem->oldValue != *(tem->currentValue)) {
			tem->oldValue = *(tem->currentValue);
			sprintf(tempWrite, "%.2f", *(tem->currentValue));
			while (!(ini_puts(tem->section, tem->keyWord, tempWrite, FILE_NAME))) {
			}

		}
		tem = tem->next;
	}
	OS_CRITICAL_EXIT();
	return 1;
}




void LoadOffsetFromFile(void) {
	char tempread[20];
	CPU_SR_ALLOC();
	OS_CRITICAL_ENTER();
	ini_gets(SECTION_OFFSET, KEY_GYRO_CAL_TIME, "0", tempread, 20, FILE_NAME);
	SensorStatue.gyroCalTime = (short)(atoi(tempread));
//	ini_gets(SECTION_OFFSET, KEY_OFFSET_ROLL, "0", tempread, 20, FILE_NAME);
//	SensorStatue.offsetRoll = atof(tempread);
//	ini_gets(SECTION_OFFSET, KEY_OFFSET_PITCH, "0", tempread, 20, FILE_NAME);
//	SensorStatue.offsetPitch = atof(tempread);

	ini_gets(SECTION_OFFSET, KEY_OFFSET_GYRO_X, "0", tempread, 20, FILE_NAME);
	SensorStatue.offsetGyro[0] = atof(tempread);
	ini_gets(SECTION_OFFSET, KEY_OFFSET_GYRO_Y, "0", tempread, 20, FILE_NAME);
	SensorStatue.offsetGyro[1] = atof(tempread);
	ini_gets(SECTION_OFFSET, KEY_OFFSET_GYRO_Z, "0", tempread, 20, FILE_NAME);
	SensorStatue.offsetGyro[2] = atof(tempread);

	ini_gets(SECTION_OFFSET, KEY_OFFSET_MAG_MAX_X, "0", tempread, 20, FILE_NAME);
	SensorStatue.magMax[0] = atof(tempread);
	ini_gets(SECTION_OFFSET, KEY_OFFSET_MAG_MAX_Y, "0", tempread, 20, FILE_NAME);
	SensorStatue.magMax[1] = atof(tempread);
	ini_gets(SECTION_OFFSET, KEY_OFFSET_MAG_MAX_Z, "0", tempread, 20, FILE_NAME);
	SensorStatue.magMax[2] = atof(tempread);

	ini_gets(SECTION_OFFSET, KEY_OFFSET_MAG_MIN_X, "0", tempread, 20, FILE_NAME);
	SensorStatue.magMin[0] = atof(tempread);
	ini_gets(SECTION_OFFSET, KEY_OFFSET_MAG_MIN_Y, "0", tempread, 20, FILE_NAME);
	SensorStatue.magMin[1] = atof(tempread);
	ini_gets(SECTION_OFFSET, KEY_OFFSET_MAG_MIN_Z, "0", tempread, 20, FILE_NAME);
	SensorStatue.magMin[2] = atof(tempread);

//	ini_gets(SECTION_OFFSET, KEY_OFFSET_PITCH, "0", tempread, 20, FILE_NAME);
//	SensorStatue.offsetPitch = atof(tempread);
//
//	ini_gets(SECTION_OFFSET, KEY_OFFSET_ROLL, "0", tempread, 20, FILE_NAME);
//	SensorStatue.offsetRoll = atof(tempread);
	OS_CRITICAL_EXIT();
}

void WriteOffsetToFile(void) {
	char tempwrite[20];
	CPU_SR_ALLOC();
	OS_CRITICAL_ENTER();
	sprintf(tempwrite, "%d", SensorStatue.gyroCalTime);
	ini_puts(SECTION_OFFSET, KEY_GYRO_CAL_TIME, tempwrite, FILE_NAME);
//	sprintf(tempwrite, "%.2f", SensorStatue.offsetRoll);
//	ini_puts(SECTION_OFFSET, KEY_OFFSET_ROLL, tempwrite, FILE_NAME);
//	sprintf(tempwrite, "%.2f", SensorStatue.offsetPitch);
//	ini_puts(SECTION_OFFSET, KEY_OFFSET_PITCH, tempwrite, FILE_NAME);

	sprintf(tempwrite, "%.2f", SensorStatue.offsetGyro[0]);
	ini_puts(SECTION_OFFSET, KEY_OFFSET_GYRO_X, tempwrite, FILE_NAME);
	sprintf(tempwrite, "%.2f", SensorStatue.offsetGyro[1]);
	ini_puts(SECTION_OFFSET, KEY_OFFSET_GYRO_Y, tempwrite, FILE_NAME);
	sprintf(tempwrite, "%.2f", SensorStatue.offsetGyro[2]);
	ini_puts(SECTION_OFFSET, KEY_OFFSET_GYRO_Z, tempwrite, FILE_NAME);

	sprintf(tempwrite, "%.2f", SensorStatue.magMax[0]);
	ini_puts(SECTION_OFFSET, KEY_OFFSET_MAG_MAX_X, tempwrite, FILE_NAME);
	sprintf(tempwrite, "%.2f", SensorStatue.magMax[1]);
	ini_puts(SECTION_OFFSET, KEY_OFFSET_MAG_MAX_Y, tempwrite, FILE_NAME);
	sprintf(tempwrite, "%.2f", SensorStatue.magMax[2]);
	ini_puts(SECTION_OFFSET, KEY_OFFSET_MAG_MAX_Z, tempwrite, FILE_NAME);

	sprintf(tempwrite, "%.2f", SensorStatue.magMin[0]);
	ini_puts(SECTION_OFFSET, KEY_OFFSET_MAG_MIN_X, tempwrite, FILE_NAME);
	sprintf(tempwrite, "%.2f", SensorStatue.magMin[1]);
	ini_puts(SECTION_OFFSET, KEY_OFFSET_MAG_MIN_Y, tempwrite, FILE_NAME);
	sprintf(tempwrite, "%.2f", SensorStatue.magMin[2]);
	ini_puts(SECTION_OFFSET, KEY_OFFSET_MAG_MIN_Z, tempwrite, FILE_NAME);


	OS_CRITICAL_EXIT();
}
void LoadDataFromFile(void) {
  CPU_SR_ALLOC();
  OS_CRITICAL_ENTER();
	LoadOffsetFromFile();
	LoadParameterFromFile();
        OS_CRITICAL_EXIT();
}

void LEDAndFileTask(void *p_arg) {
	OS_ERR err;
	short delayTime = 500;
	short redLedReserveTime = 0;
        
	if (ReadKeyState() == GPIO_PIN_RESET) {
		if (Bl_EnterAT() == SUCCESS) {
			if (Bl_ParameterInit() != SUCCESS) {
				while (1) {
				}
			}
		} else {
			while (1) {
			}
		}
	}
	LoadDataFromFile();
	magGainCalc(SensorStatue.magMax, SensorStatue.magMin, SensorStatue.gain);
	while (DEF_TRUE) {
		//LED_BLUE_TOGGLE();
		if (control.flag_Lock == 0) {
			delayTime = 50;
			if (__HAL_TIM_GET_COMPARE(&htim5, TIM_CHANNEL_2) > 50) {
				__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
			} else {
				__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 100);
			}
		} else {
			delayTime = 500;
			if (__HAL_TIM_GET_COMPARE(&htim5, TIM_CHANNEL_2) > 50) {
				__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
			} else {
				__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 100);
			}
		}
		if (parameterHasChanged == 1) {
			WriteParameterToFile();
			parameterHasChanged = 0;
			redLedReserveTime = 2000;
		}
		if (needLoadParameterFromFile == 1) {
			needLoadParameterFromFile = 0;
			LoadParameterFromFile();
		}
		if (offsetHasChanged == 1) {
			offsetHasChanged = 0;
			WriteOffsetToFile();
		}
		if (redLedReserveTime > 0) {
			LED_RED_ON();
			redLedReserveTime -= delayTime;
			if (redLedReserveTime <= 0) {
				LED_RED_OFF();
				redLedReserveTime = 0;
			}
		}
		OSTimeDlyHMSM(0u, 0u, 0u, delayTime,
			OS_OPT_TIME_HMSM_STRICT,
			&err);
	}
}