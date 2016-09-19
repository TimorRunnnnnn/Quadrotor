#include "ms5611.h"

SPI_HandleTypeDef *MS5611_Handler;
uint16_t MS5611_C1 = 0, MS5611_C2 = 0, MS5611_C3 = 0, MS5611_C4 = 0, MS5611_C5 = 0, MS5611_C6 = 0;

static void delay(void) {
	for (int i = 0; i < 168000; i++) {
		asm("nop");
	}
}

uint16_t MS5611_ReadWord(uint8_t addr) {
	uint8_t data[2] = { 0 };
	uint16_t value = 0;

	// Chip Select low 
	//Chip_Select(MS5611);
	MS5611_ON;
	//SPIx_SendByte(MS5611, addr);
	HAL_SPI_Transmit(MS5611_Handler, &addr, 1, 1000);

	uint8_t trdata = MS5611_READ;
	//HAL_SPI_Transmit(MS5611_Handler, &trdata, 1, 1000);
	HAL_SPI_TransmitReceive(MS5611_Handler,&trdata,&data[0],1,1000);
	HAL_SPI_TransmitReceive(MS5611_Handler, &trdata, &data[1], 1, 1000);
	// Chip Select high
	MS5611_OFF;

	value = data[0] << 8 | data[1];
	return value;
}

void MS5611_Start_P(void) {
	MS5611_ON;
	uint8_t data = 0x48;
	HAL_SPI_Transmit(MS5611_Handler, &data, 1, 10);
	MS5611_OFF;
}

void MS5611_Start_T(void) {
	MS5611_ON;
	uint8_t data = 0x58;
	HAL_SPI_Transmit(MS5611_Handler, &data, 1, 10);
	MS5611_OFF;
}

uint32_t MS5611_Read() {
	MS5611_ON;
	uint8_t data[3];
	uint8_t tx = MS5611_READ_ADC;
	HAL_SPI_Transmit(MS5611_Handler, &tx, 1, 10);
	HAL_SPI_Receive(MS5611_Handler, data, 3, 100);
	uint32_t value = data[0] << 16 | data[1] << 8 | data[2];
	MS5611_OFF;
	return value;
}


void MS5611_BaroAltCalculate(int *P,uint32_t D2,uint32_t D1){
	int32_t  dT;
       static int TEMP;
	int64_t OFF, SENS, OFF2 = 0, SENS2 = 0;
	int32_t lowTEMP, verylowTemp;

	dT = D2 - ((uint32_t)MS5611_C5 << 8);
	TEMP = 2000 + (((int64_t)dT * MS5611_C6) >> 23);
	OFF = ((uint32_t)MS5611_C2 << 16) + ((MS5611_C4 * (int64_t)dT) >> 7);
	SENS = ((uint32_t)MS5611_C1 << 15) + ((MS5611_C3 * (int64_t)dT) >> 8);
	//
	//*T = TEMP;
	//////////////////////////////////////////////////////////////////////////
	//second order temperature compensation
	if (TEMP < 2000) {
		//T2 = (int64_t)((int64_t)dT * (int64_t)dT) >> 31;
		lowTEMP = TEMP - 2000;
		lowTEMP *= lowTEMP;
		OFF2 = (5 * lowTEMP) >> 1;
		SENS2 = (5 * lowTEMP) >> 2;
		if (TEMP < -1500) {
			verylowTemp = TEMP + 1500;
			verylowTemp *= verylowTemp;
			OFF2 = OFF2 + 7 * verylowTemp;
			SENS2 = SENS2 + ((11 * verylowTemp) >> 1);
		}
		//
		OFF = OFF - OFF2;
		SENS = SENS - SENS2;
		//*T = TEMP - T2;
	}
	//////////////////////////////////////////////////////////////////////////
	*P = ((((int64_t)D1 * SENS) >> 21) - OFF) >> 15;

	static float oldAltitude = 0;
	SensorStatue.altitude = (int)((101000 - *P)); //分米
	LPF_1st(&oldAltitude, &SensorStatue.altitude, 0.3);
}
void MS5611_Update(void) {
	static uint8_t state = 0;
	static uint32_t temp, pressure;
	if (state)
	{//温度
		state = 0;
		temp=MS5611_Read();
		MS5611_Start_P();
		MS5611_BaroAltCalculate(&SensorStatue.pressure,temp,pressure);
	} else {
		//气压
		state = 1;
		pressure=MS5611_Read();
		MS5611_Start_T();
	}
}

void MS5611_ReadPROM(void) {
	// Read Calibration Data C1
	MS5611_C1 = MS5611_ReadWord(MS5611_READ_PROM_C1);
	delay();
	// Read Calibration Data C2
	MS5611_C2 = MS5611_ReadWord( MS5611_READ_PROM_C2);
	delay();
	// Read Calibration Data C3
	MS5611_C3 = MS5611_ReadWord( MS5611_READ_PROM_C3);
	delay();
	// Read Calibration Data C4
	MS5611_C4 = MS5611_ReadWord( MS5611_READ_PROM_C4);
	delay();
	// Read Calibration Data C5
	MS5611_C5 = MS5611_ReadWord( MS5611_READ_PROM_C5);
	delay();
	// Read Calibration Data C6
	MS5611_C6 = MS5611_ReadWord( MS5611_READ_PROM_C6);
	delay();
}

void MS5611_Reset() {
	// Chip Select low 
	MS5611_ON;
	uint8_t data = MS5611_RESET;
	HAL_SPI_Transmit(MS5611_Handler, &data,1,1000);
	delay();
	delay();
	MS5611_OFF;
}

void MS5611_Init(SPI_HandleTypeDef *hspi) {
	MS5611_Handler = hspi;
	MS5611_Reset();
	delay();
	MS5611_ReadPROM();
	delay();
}
