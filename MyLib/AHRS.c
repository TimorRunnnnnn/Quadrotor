#include "AHRS.h"
#include "Function.h"
//#include "ekf.h"
//#include "ukf.h"
#include "Quaternion.h"

#include "Algorithm_math.h"
#include "Algorithm_filter.h"
#include "Algorithm_quaternion.h"
//#include "miniAhrs.h"
//#include "ins_ekf.h"

#include "MadgwickAHRS.h"
// #include "UM6_states.h"

//#include "measureproc.h"

#define MAG_READ_DELAY 30

SPI_HandleTypeDef *MPU9250_Handler;
SensorStatusTypeDef SensorStatue;

short offsetHasChanged = 0;
static int16_t AK8963_ASA[3] = { 0 };

static void simpdelay(void) {
	for (int i = 0; i < 84000; i++) {
		asm("nop");
	}
}



uint8_t MPU9250_ReadReg(uint8_t ReadAddr) {
	MPU9250_ON;
	uint8_t ReadData = 0;
	uint8_t tx = ReadAddr | 0x80;
	HAL_SPI_Transmit(MPU9250_Handler, &tx, 1, 100);
	HAL_SPI_Receive(MPU9250_Handler, &ReadData, 1, 100);
	MPU9250_OFF;
	return ReadData;
}

void MPU9250_WriteReg(uint8_t WriteAddr, uint8_t WriteData) {
	MPU9250_ON;
	HAL_SPI_Transmit(MPU9250_Handler, &WriteAddr, 1, 100);
	HAL_SPI_Transmit(MPU9250_Handler, &WriteData, 1, 100);
	MPU9250_OFF;
}

void MPU9250_ReadRegs(uint8_t ReadAddr, uint8_t *ReadBuf, uint8_t Bytes) {
	MPU9250_ON;
	uint8_t tx = ReadAddr | 0x80;
	HAL_SPI_Transmit(MPU9250_Handler, &tx, 1, 100);
	HAL_SPI_Receive(MPU9250_Handler, ReadBuf, Bytes, 100);
	MPU9250_OFF;
}

void MPU9250_Mag_WriteReg(uint8_t writeAddr, uint8_t writeData) {
	uint8_t  status = 0;
	uint32_t timeout = MAG_READ_DELAY;

	MPU9250_WriteReg(MPU6500_I2C_SLV4_ADDR, AK8963_I2C_ADDR);
	simpdelay();
	MPU9250_WriteReg(MPU6500_I2C_SLV4_REG, writeAddr);
	simpdelay();
	MPU9250_WriteReg(MPU6500_I2C_SLV4_DO, writeData);
	simpdelay();
	MPU9250_WriteReg(MPU6500_I2C_SLV4_CTRL, MPU6500_I2C_SLVx_EN);
	simpdelay();

	do {
		status = MPU9250_ReadReg(MPU6500_I2C_MST_STATUS);
		simpdelay();
	} while (((status & MPU6500_I2C_SLV4_DONE) == 0) && (timeout--));
	simpdelay();
}

uint8_t MPU9250_Mag_ReadReg(uint8_t readAddr) {
	uint8_t status = 0;
	uint8_t readData = 0;
	uint32_t timeout = MAG_READ_DELAY;

	MPU9250_WriteReg(MPU6500_I2C_SLV4_ADDR, AK8963_I2C_ADDR | 0x80);
	simpdelay();
	MPU9250_WriteReg(MPU6500_I2C_SLV4_REG, readAddr);
	simpdelay();
	MPU9250_WriteReg(MPU6500_I2C_SLV4_CTRL, MPU6500_I2C_SLVx_EN);
	simpdelay();

	do {
		status = MPU9250_ReadReg(MPU6500_I2C_MST_STATUS);
		simpdelay();
	} while (((status & MPU6500_I2C_SLV4_DONE) == 0) && (timeout--));

	readData = MPU9250_ReadReg(MPU6500_I2C_SLV4_DI);
	simpdelay();
	return readData;
}


void MPU9250_Mag_ReadRegs(uint8_t readAddr, uint8_t *readData, uint8_t lens) {
	for (uint8_t i = 0; i < lens; i++) {
		readData[i] = MPU9250_Mag_ReadReg(readAddr + i);
		simpdelay();
	}
}




#define AK8963_CNTL1_Value 0x16
uint8_t MPU9250_Mag_Init(void) {
	uint8_t buf;
	while (1) {
		MPU9250_WriteReg(MPU6500_I2C_SLV0_ADDR, 0x0C);  //write I2C addr 
		simpdelay();
		MPU9250_WriteReg(MPU6500_I2C_SLV0_DO, AK8963_CNTL1_Value);
		MPU9250_WriteReg(MPU6500_I2C_SLV0_REG, AK8963_CNTL1);     // Set Write Reg
		MPU9250_WriteReg(MPU6500_I2C_SLV0_CTRL, 0x81);          // Start Write, 1 bytes
		// read back to check
		simpdelay();
		MPU9250_WriteReg(MPU6500_I2C_SLV0_ADDR, 0x8C);//read I2C addr   
		MPU9250_WriteReg(MPU6500_I2C_SLV0_REG, AK8963_CNTL1);     // Set Write Reg
		MPU9250_WriteReg(MPU6500_I2C_SLV0_CTRL, 0x81);          // Start Read, 6 bytes
		simpdelay();
		buf = MPU9250_ReadReg(MPU6500_EXT_SENS_DATA_00);   // Read Data
		if (buf != AK8963_CNTL1_Value) {
			asm("nop");
			//return ERROR;
		} else
			return SUCCESS;
	}
}





#define MPU9250_InitRegNum 10
void MPU9250_Init(SPI_HandleTypeDef *hspi) {
	MPU9250_Handler = hspi;
	uint8_t i = 0;
	/*
	uint8_t MPU6500_InitData[MPU9250_InitRegNum][2] =    {
	{ 0x80, MPU6500_PWR_MGMT_1 },     // Reset Device
	{ 0x04, MPU6500_PWR_MGMT_1 },     // Clock Source
	{ 0x10, MPU6500_INT_PIN_CFG },    // Set INT_ANYRD_2CLEAR
	{ 0x01, MPU6500_INT_ENABLE },     // Set RAW_RDY_EN
	{ 0x00, MPU6500_PWR_MGMT_2 },     // Enable Acc & Gyro
	{ 0x00, MPU6500_SMPLRT_DIV },     // Sample Rate Divider
	{ 0x13, MPU6500_GYRO_CONFIG },    // default : +-1000dps
	{ 0x08, MPU6500_ACCEL_CONFIG },   // default : +-4G
	{ 0x07, MPU6500_CONFIG },         // default : LPS_3600Hz
	{ 0x0B, MPU6500_ACCEL_CONFIG_2 }, // default : LPS_41Hz 1011
	{ 0x30, MPU6500_USER_CTRL },      // Set I2C_MST_EN, I2C_IF_DIS
	};*/
	uint8_t MPU6500_InitData[MPU9250_InitRegNum][2] = {
		//{ 0x80, MPU6500_PWR_MGMT_1 },     // Reset Device
		{ 0x04, MPU6500_PWR_MGMT_1 },     // Clock Source
		{ 0x10, MPU6500_INT_PIN_CFG },    // Set INT_ANYRD_2CLEAR
		{ 0x01, MPU6500_INT_ENABLE },     // Set RAW_RDY_EN
		{ 0x00, MPU6500_PWR_MGMT_2 },     // Enable Acc & Gyro
		{ 0x11, MPU6500_GYRO_CONFIG },    // default : +-1000dps
		{ 0x08, MPU6500_ACCEL_CONFIG },   // default : +-4G
		{ 0x07, MPU6500_CONFIG },         // default : LPS_20Hz
		{ 0x0E, MPU6500_ACCEL_CONFIG_2 }, // default : LPS_10Hz 1101
		{ 0x40, MPU6500_I2C_MST_CTRL },
		{ 0x35, MPU6500_USER_CTRL }, // Set I2C_MST_EN, I2C_IF_DIS
	};

	for (i = 0; i < MPU9250_InitRegNum; i++) {
		MPU9250_WriteReg(MPU6500_InitData[i][1], MPU6500_InitData[i][0]);
		simpdelay();
	}
	//check write
	// 	for (i = 1; i < MPU9250_InitRegNum;i++)
	// 	{
	// 		uint8_t temp = MPU9250_ReadReg(MPU6500_InitData[i][1]);
	// 		if (temp!=MPU6500_InitData[i][0])
	// 		{
	// 			while (1)
	// 			{
	// 				asm("nop");
	// 			}
	// 		}
	// 	}
}

uint8_t MPU9250_Check(void) {
	uint8_t DeviceID = 0x00;

	/* MPU6500 Check*/
	DeviceID = 0x00;
	DeviceID = MPU9250_ReadReg(MPU6500_WHO_AM_I);
	if (DeviceID != MPU6500_Device_ID)
		return ERROR;

	/* AK8975 Check */
	DeviceID = 0x00;
	DeviceID = MPU9250_Mag_ReadReg(AK8963_WIA);
	if (DeviceID != AK8963_Device_ID) {
		//return ERROR;
	}
	uint8_t tmpRead[3];

	MPU9250_Mag_WriteReg(AK8963_CNTL2, 0x01);       // Reset Device
	simpdelay();

	MPU9250_Mag_WriteReg(AK8963_CNTL1, 0x10);       // Power-down mode
	simpdelay();

	MPU9250_Mag_WriteReg(AK8963_CNTL1, 0x1F);       // Fuse ROM access mode

	MPU9250_Mag_ReadRegs(AK8963_ASAX, tmpRead, 3);  // Read sensitivity adjustment values
	simpdelay();
	MPU9250_Mag_WriteReg(AK8963_CNTL1, 0x10);       // Power-down mode
	simpdelay();

	if (tmpRead[0] == 0x00 || tmpRead[1] == 0x00 || tmpRead[2] == 0x00) {
		return ERROR;
	}
	AK8963_ASA[0] = (int16_t)(tmpRead[0]) + 128;
	AK8963_ASA[1] = (int16_t)(tmpRead[1]) + 128;
	AK8963_ASA[2] = (int16_t)(tmpRead[2]) + 128;


	simpdelay();

	MPU9250_Mag_WriteReg(AK8963_CNTL1, 0x16);       // 连续测量模式2
	simpdelay();
	MPU9250_WriteReg(MPU6500_I2C_SLV4_CTRL, 0x09); //关闭slv4,陀螺仪加速度计odr=1000,延迟9个周期,磁力计50Hz(磁力计ODR=8Hz)
	simpdelay();
	MPU9250_WriteReg(MPU6500_I2C_MST_DELAY_CTRL, 0x81); //开启延迟
	simpdelay();


	MPU9250_WriteReg(MPU6500_I2C_MST_CTRL, 0x5D);
	simpdelay();
	MPU9250_WriteReg(MPU6500_I2C_SLV0_ADDR, AK8963_I2C_ADDR | 0x80);
	simpdelay();
	MPU9250_WriteReg(MPU6500_I2C_SLV0_REG, AK8963_ST1);
	simpdelay();
	MPU9250_WriteReg(MPU6500_I2C_SLV0_CTRL, MPU6500_I2C_SLVx_EN | 8);//从st1开始读8个字节,中间六个为磁场数据,最后是st2
	simpdelay();


	return SUCCESS;
}


// void MPU9250_getData(int16_t *dataIMU) {
// 	static uint8_t tmpRead[23] = { 0 };
// 
// 	MPU9250_ReadRegs(MPU6500_INT_STATUS, tmpRead, 23);
// 
// 	dataIMU[0] = (Byte16(int16_t, tmpRead[6], tmpRead[7]));    // Temp
// 	dataIMU[1] = (Byte16(int16_t, tmpRead[0], tmpRead[1]));    // Acc.X
// 	dataIMU[2] = (Byte16(int16_t, tmpRead[2], tmpRead[3]));    // Acc.Y
// 	dataIMU[3] = (Byte16(int16_t, tmpRead[4], tmpRead[5]));    // Acc.Z
// 	dataIMU[4] = (Byte16(int16_t, tmpRead[8], tmpRead[9]));    // Gyr.X
// 	dataIMU[5] = (Byte16(int16_t, tmpRead[10], tmpRead[11]));   // Gyr.Y
// 	dataIMU[6] = (Byte16(int16_t, tmpRead[12], tmpRead[13]));   // Gyr.Z
// 
// 
// 	/*if (!(tmpRead[14] & AK8963_STATUS_DRDY) || (tmpRead[14] & AK8963_STATUS_DOR))
// 	return;*/
// 	if (tmpRead[21] & AK8963_STATUS_HOFL)
// 		return;
// 
// 	dataIMU[7] = (Byte16(int16_t, tmpRead[16], tmpRead[15]));   // Mag.X
// 	dataIMU[8] = (Byte16(int16_t, tmpRead[18], tmpRead[17]));   // Mag.Y
// 	dataIMU[9] = (Byte16(int16_t, tmpRead[20], tmpRead[19]));   // Mag.Z
// }
// 
// void MPUDataDeal(float *dataIMU, int16_t *orgdata) {
// 	dataIMU[0] = orgdata[0] * MPU9250T_85degC;    // Temp
// 	dataIMU[1] = orgdata[1] * MPU9250A_4g;    // Acc.X
// 	dataIMU[2] = orgdata[2] * MPU9250A_4g;     // Acc.Y
// 	dataIMU[3] = orgdata[3] * MPU9250A_4g;     // Acc.Z
// 	dataIMU[4] = orgdata[4] * MPU9250G_1000dps;     // Gyr.X
// 	dataIMU[5] = orgdata[5] * MPU9250G_1000dps;    // Gyr.Y
// 	dataIMU[6] = orgdata[6] * MPU9250G_1000dps;   // Gyr.Z
// 	dataIMU[7] = orgdata[7] * MPU9250M_4800uT;    // Mag.X
// 	dataIMU[8] = orgdata[8] * MPU9250M_4800uT;    // Mag.Y
// 	dataIMU[9] = orgdata[9] * MPU9250M_4800uT;    // Mag.Z
// }

//2ms中断服务函数
static int cnt_xxx;

static OS_Q AHRS_Q;
static OS_MEM AHRS_RAWDataPartition;
static uint8_t AHRS_RAWDataPartitionStorage[5][22];

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM6) {
          static short cnt=0;
#define BARO_TIMES (1.0f/(MPU_DT*100))
          cnt++;
          if(cnt>BARO_TIMES)
          {
            cnt=0;
            MS5611_Update();
          }
          
		MPU9250_ON;
		OS_ERR err = OS_ERR_NONE;
		uint8_t *buffer = OSMemGet(&AHRS_RAWDataPartition, &err);
		if (buffer != NULL) {
			uint8_t tx = MPU6500_ACCEL_XOUT_H | 0x80;
			HAL_StatusTypeDef status;
			status = HAL_SPI_Transmit(MPU9250_Handler, &tx, 1, 100);
			if (status == HAL_BUSY) {
				HAL_SPI_DMAStop(MPU9250_Handler);
			}
			if (status != HAL_OK) {
				OSMemPut(&AHRS_RAWDataPartition, buffer, &err);
				return;
			}
			status = HAL_SPI_Receive_DMA(MPU9250_Handler, buffer, 22);
			if (status != HAL_OK) {
				OSMemPut(&AHRS_RAWDataPartition, buffer, &err);
				return;
			}
			cnt_xxx += 1;
		}
	}
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == MPU9250_Handler->Instance) {
		OS_ERR err = OS_ERR_NONE;
		MPU9250_OFF;
		OSQPost(&AHRS_Q,
			(void*)hspi->pRxBuffPtr,
			22,
			OS_OPT_POST_FIFO,
			&err);
	}
}

void magGainCalc(float *max, float *min, float *gain) {
	if (((max[0] - min[0]) >= (max[1] - min[1])) && ((max[0] - min[0]) >= (max[2] - min[2]))) {
		gain[0] = 1.0;
		gain[1] = (max[0] - min[0]) / (max[1] - min[1]);
		gain[2] = (max[0] - min[0]) / (max[2] - min[2]);
		// 			MXoffset = -0.5 * (max[0] + min[0]);
		// 			MYoffset = -0.5 * gain[1] * (max[1] + min[1]);
		// 			MZoffset = -0.5 * gain[2] * (max[2] + min[2]);
	}
	if (((max[1] - min[1]) > (max[0] - min[0])) && ((max[1] - min[1]) >= (max[2] - min[2]))) {
		gain[0] = (max[1] - min[1]) / (max[0] - min[0]);
		gain[1] = 1.0;
		gain[2] = (max[1] - min[1]) / (max[2] - min[2]);
		// 			MXoffset = -0.5 * gain[0] * (max[0] + min[0]);
		// 			MYoffset = -0.5 * (max[1] + min[1]);
		// 			MZoffset = -0.5 * gain[2] * (max[2] + min[2]);
	}
	if (((max[2] - min[2]) > (max[0] - min[0])) && ((max[2] - min[2]) > (max[1] - min[1]))) {
		gain[0] = (max[2] - min[2]) / (max[0] - min[0]);
		gain[1] = (max[2] - min[2]) / (max[1] - min[1]);
		gain[2] = 1.0;
		// 			MXoffset = -0.5 * gain[0] * (max[0] + min[0]);
		// 			MYoffset = -0.5 * gain[1] * (max[1] + min[1]);
		// 			MZoffset = -0.5 * (max[2] + min[2]);
	}
}


uint8_t Mag_Calibration(float *gyro, float *mag, float *max, float *min) {
	static int ledCount = 0;
	//static float angle[3];
	static float lastMag[3];
	ledCount++;
	if (ledCount % 25 == 0) {
		LED_RED_TOGGLE();
	}
	for (int i = 0; i < 3; i++) {
		if (SensorStatue.magCalAngle[i] != 1024) {
			float tem = mag[i];
			mag[i] = mag[i] * 0.6 + lastMag[i] * 0.4;
			lastMag[i] = tem;
			if (max[i] < mag[i]) {
				max[i] = mag[i];
			}
			if (min[i] > mag[i]) {
				min[i] = mag[i];
			}
			SensorStatue.magCalAngle[i] += gyro[i] * MPU_DT;
			if (ABS(SensorStatue.magCalAngle[i])>360 && SensorStatue.magCalAngle[i] != 1024) {
				SensorStatue.magCalAngle[i] = 1024;//标记
			}
		}
	}


	if (SensorStatue.magCalAngle[0] == 1024 && SensorStatue.magCalAngle[1] == 1024 && SensorStatue.magCalAngle[2] == 1024) {
		if ((ledCount / 25) % 2 == 1) {
			LED_RED_TOGGLE();
		}
		magGainCalc(max, min, SensorStatue.gain);
		ledCount = 0;
		Mem_Set(SensorStatue.magCalAngle, 0, 3 * sizeof(int));
		return 1;
	}
	return 0;
}


float KpDef = 0.65f;
float  KiDef = 9.99999974E-6;


float GyrX, GyrY, GyrZ;
static Quaternion NumQ = { 1, 0, 0, 0 };
void AHRS_Geteuler(float *acc, float *gyro, float *mag, float *fRPY) {
#define  IIR_ORDER     4       
#define squa( Sq )        (((float)Sq)*((float)Sq))

#define SampleRateHalf (MPU_DT/(2.0f))  
	// 	static float b_IIR[IIR_ORDER + 1] = { 0.0008f, 0.0032f, 0.0048f, 0.0032f, 0.0008f };
	// 	static float a_IIR[IIR_ORDER + 1] = { 1.0000f, -3.0176f, 3.5072f, -1.8476f, 0.3708f };
	// 	static float InPut_IIR[3][IIR_ORDER + 1] = { 0 };
	// 	static float OutPut_IIR[3][IIR_ORDER + 1] = { 0 };
	float averag_x = acc[0], averag_y = acc[1], averag_z = acc[2];
	static float lastAcc[3];
	// 	averag_x = IIR_I_Filter(acc[0], InPut_IIR[0], OutPut_IIR[0], b_IIR, IIR_ORDER + 1, a_IIR, IIR_ORDER + 1);
	// 	averag_y = IIR_I_Filter(acc[1], InPut_IIR[1], OutPut_IIR[1], b_IIR, IIR_ORDER + 1, a_IIR, IIR_ORDER + 1);
	// 	averag_z = IIR_I_Filter(acc[2], InPut_IIR[2], OutPut_IIR[2], b_IIR, IIR_ORDER + 1, a_IIR, IIR_ORDER + 1);

#define ACC_LPF_FACTOR 0.1
	LPF_1st(&lastAcc[0], &averag_x, ACC_LPF_FACTOR);
	LPF_1st(&lastAcc[1], &averag_y, ACC_LPF_FACTOR);
	LPF_1st(&lastAcc[2], &averag_z, ACC_LPF_FACTOR);

	// 	averag_x = acc[0];
	// 	averag_y = acc[1];
	// 	averag_z = acc[2];


	float ErrX, ErrY, ErrZ;
	float AccX, AccY, AccZ;

	float Normalize;
	static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;
	Gravity V;


	static EulerAngle AngE = { 0 };

	Normalize = Q_rsqrt(squa(averag_x) + squa(averag_y) + squa(averag_z));
	AccX = averag_x*Normalize;
	AccY = averag_y*Normalize;
	AccZ = averag_z*Normalize;

	V = Quaternion_vectorGravity(&NumQ);


	ErrX = (AccY*V.z - AccZ*V.y);
	ErrY = (AccZ*V.x - AccX*V.z);
	ErrZ = (AccX*V.y - AccY*V.x);

	exInt = exInt + ErrX * KiDef * MPU_DT;
	eyInt = eyInt + ErrY * KiDef * MPU_DT;
	ezInt = ezInt + ErrZ * KiDef * MPU_DT;


	float kptemp;
	if (control.flag_Lock == 1) {
		kptemp = KpDef * 100;
	} else {
		kptemp = KpDef;
	}
	GyrX = Rad(gyro[0]) + kptemp /* VariableParameter(ErrX)*/ * ErrX + exInt;
	GyrY = Rad(gyro[1]) + kptemp /* VariableParameter(ErrY)*/ * ErrY + eyInt;
	GyrZ = Rad(gyro[2]) + kptemp /* VariableParameter(ErrZ)*/ * ErrZ + ezInt;
	//
	//	GyrX = Rad(gyro[0]) + KpDef * ErrX + exInt;
	//	GyrY = Rad(gyro[1]) + KpDef * ErrY + eyInt;
	//	GyrZ = Rad(gyro[2]) + KpDef * ErrZ + ezInt;
	Quaternion_RungeKutta(&NumQ, GyrX, GyrY, GyrZ, SampleRateHalf);

//	float w[4];
//	w[0] = 0;
//	w[1] = GyrX;
//	w[2] = GyrY;
//	w[3] = GyrZ;
//	float q[4];
//	q[0] = NumQ.q0;
//	q[1] = NumQ.q1;
//	q[2] = NumQ.q2;
//	q[3] = NumQ.q3;
//	Quaternion_RungeKutta4(q, w, SampleRateHalf * 2, 0);
//	NumQ.q0 = q[0];
//	NumQ.q1 = q[1];
//	NumQ.q2 = q[2];
//	NumQ.q3 = q[3];
	Quaternion_Normalize_(&NumQ);
	Quaternion_ToAngE(&NumQ, &AngE);


	static short flagFirst = 500;

	if (SensorStatue.magIssue == 0) {
		float sin_pitch, sin_roll, cos_roll, cos_pitch;
		sin_roll = arm_sin_f32(AngE.Roll);
		sin_pitch = arm_sin_f32(AngE.Pitch);
		cos_roll = arm_cos_f32(AngE.Roll);
		cos_pitch = arm_cos_f32(AngE.Pitch);

		float hx = mag[1] * cos_pitch + mag[0] * sin_pitch*sin_roll - mag[2] * cos_roll*sin_pitch;
		float hy = mag[0] * cos_roll + mag[2] * sin_roll;
		// 		float hx = mag[0];
		// 		float hy = mag[1];

		// 		float hx = mag[1] * cos_pitch + mag[0] * sin_pitch*sin_roll - mag[2] * cos_roll*sin_pitch;
		// 		float hy = mag[0] * cos_roll + mag[2] * sin_roll;


		float mag_yaw = Degree(atan2((float)hy, (float)hx));

		static float magFactor = 0.0001;

		AngE.Yaw += gyro[2] * 2 * SampleRateHalf; //这个必须保持,
		if ((mag_yaw > 90 && AngE.Yaw < -90) || (mag_yaw < -90 && AngE.Yaw>90))
			AngE.Yaw = -AngE.Yaw *(1 - magFactor) + mag_yaw * magFactor;
		else AngE.Yaw = AngE.Yaw * (1 - magFactor) + mag_yaw * magFactor;

		if (flagFirst > 0 && (mag[0] != 0 && mag[1] != 0 && mag[2] != 0)) {
			flagFirst--;
			AngE.Yaw = mag_yaw;
		}
		if (AngE.Yaw > 180) {
			AngE.Yaw = (AngE.Yaw - 180) - 180;
		} else if (AngE.Yaw < -180) {
			AngE.Yaw = AngE.Yaw + 180 + 180;
		}
	} else {
		AngE.Yaw = Degree(AngE.Yaw);
	}

	AngE.Roll = Degree(AngE.Roll);  // roll
	AngE.Pitch = Degree(AngE.Pitch); // pitch


	//交换roll和pitch
	fRPY[0] = AngE.Roll;
	fRPY[1] = AngE.Pitch;
	fRPY[2] = AngE.Yaw;
}
void AHRS_Task(void *parg) {
	int16_t RAWData[10];
	uint8_t *rawData;
	OS_ERR err = OS_ERR_NONE;
	OS_MSG_SIZE size;
	float fRPY[3] = { 0, 0, 0 };
	static float AHRSData[10];

	//static uint8_t magCalibrationFlag = 0;
	//OS_ERR err;
	OSQCreate((OS_Q     *)&AHRS_Q,
		(CPU_CHAR *)"AHRSDataQ",
		(OS_MSG_QTY)5,
		(OS_ERR   *)&err);
	OSMemCreate(&AHRS_RAWDataPartition,
		"AHRS_Partition",
		(void *)AHRS_RAWDataPartitionStorage,
		5,
		22,
		&err);

	MPU9250_Init(&hspi1);
	MS5611_Init(&hspi1);
	if (MPU9250_Check() != SUCCESS) {
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
		LED_RED_ON();
	}
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	HAL_SPI_Init(&hspi1);

	HAL_TIM_Base_Start_IT(&htim6);



	//	SensorStatue.magIssue = 1;//不要磁力计了..什么jb玩意儿,竟然不是圆形的

	while (DEF_TRUE) {
		static int ahrsCnt = 0;
		ahrsCnt++;
		rawData = (uint8_t*)OSQPend(&AHRS_Q,
			0,
			OS_OPT_PEND_BLOCKING,
			&size,
			NULL,
			&err);
		if (rawData != NULL) {

			RAWData[0] = (Byte16(int16_t, rawData[6], rawData[7]));    // Temp
			RAWData[1] = (Byte16(int16_t, rawData[0], rawData[1]));    // Acc.X
			RAWData[2] = (Byte16(int16_t, rawData[2], rawData[3]));    // Acc.Y
			RAWData[3] = (Byte16(int16_t, rawData[4], rawData[5]));    // Acc.Z
			RAWData[4] = (Byte16(int16_t, rawData[8], rawData[9]));    // Gyr.X
			RAWData[5] = (Byte16(int16_t, rawData[10], rawData[11]));   // Gyr.Y
			RAWData[6] = (Byte16(int16_t, rawData[12], rawData[13]));   // Gyr.Z


			static short oldxxx[3];
			static short sameCnt;
			if (oldxxx[0] == RAWData[4] && oldxxx[1] == RAWData[5] && oldxxx[2] == RAWData[6]) {
				sameCnt++;
			}

			/*if (!(rawData[14] & AK8963_STATUS_DRDY) || (rawData[14] & AK8963_STATUS_DOR))
			return;*/
			if (!(rawData[21] & AK8963_STATUS_HOFL)) {
				//Hadj = (s16)((ASA - 128.0f) / 256.0f + 1.0f) ;
				//magData_x=(magData_x * (ASA[0]+128))>>8;
				RAWData[7] = (Byte16(int16_t, rawData[16], rawData[15]));   // Mag.X
				RAWData[8] = (Byte16(int16_t, rawData[18], rawData[17]));   // Mag.Y
				RAWData[9] = (Byte16(int16_t, rawData[20], rawData[19]));   // Mag.Z

				RAWData[7] = ((long)RAWData[7] * AK8963_ASA[0]) >> 8;
				RAWData[8] = ((long)RAWData[8] * AK8963_ASA[1]) >> 8;
				RAWData[9] = ((long)RAWData[9] * AK8963_ASA[2]) >> 8;
			}
			OSMemPut(&AHRS_RAWDataPartition,
				(void*)rawData, &err);
			if (SensorStatue.flagGyroCal == 1) {
				static int countOffset = 0;
				static float x, y, z;
				countOffset += MPU_DT * 1000000;
				x += RAWData[4];
				y += RAWData[5];
				z += RAWData[6];
				if (countOffset / 1000 >= SensorStatue.gyroCalTime) {
					SensorStatue.offsetGyro[0] = x / ((float)SensorStatue.gyroCalTime / (MPU_DT * 1000));
					SensorStatue.offsetGyro[1] = y / ((float)SensorStatue.gyroCalTime / (MPU_DT * 1000));
					SensorStatue.offsetGyro[2] = z / ((float)SensorStatue.gyroCalTime / (MPU_DT * 1000));
					countOffset = 0;
					SensorStatue.flagGyroCal = 0;
					x = 0, y = 0, z = 0;
					//WriteOffsetToFile();
					offsetHasChanged = 1;
					SendGyroCalData();
				}
			}
			AHRSData[0] = RAWData[0] * MPU9250T_85degC;    // Temp
			AHRSData[1] = RAWData[1] * MPU9250A_4g;    // Acc.X
			AHRSData[2] = RAWData[2] * MPU9250A_4g;     // Acc.Y
			AHRSData[3] = RAWData[3] * MPU9250A_4g;     // Acc.Z

#define OFFSET_LPF_FACTOR 0.000001
			SensorStatue.offsetGyro[0] += AHRSData[4] * OFFSET_LPF_FACTOR;
			SensorStatue.offsetGyro[1] += AHRSData[5] * OFFSET_LPF_FACTOR;
			SensorStatue.offsetGyro[2] += AHRSData[6] * OFFSET_LPF_FACTOR;

			AHRSData[4] = ((float)RAWData[4] - SensorStatue.offsetGyro[0]) * MPU9250G_1000dps;     // Gyr.X
			AHRSData[5] = ((float)RAWData[5] - SensorStatue.offsetGyro[1]) * MPU9250G_1000dps;    // Gyr.Y
			AHRSData[6] = -((float)RAWData[6] - SensorStatue.offsetGyro[2]) * MPU9250G_1000dps;   // Gyr.Z
			static int16_t lastMagData[3];
			if (Mem_Cmp(lastMagData, &RAWData[7], 3 * sizeof(int16_t)) == DEF_NO) {
				// static float magMax[3], magMin[3];
				static float An[3] = { 0, 0, 0 };
				AHRSData[7] = RAWData[7] * MPU9250M_4800uT;    // Mag.X
				AHRSData[8] = RAWData[8] * MPU9250M_4800uT;    // Mag.Y
				AHRSData[9] = RAWData[9] * MPU9250M_4800uT;    // Mag.Z

				if (SensorStatue.flagMagCal == 1) {
					if (Mag_Calibration(&AHRSData[4], &AHRSData[7], SensorStatue.magMax, SensorStatue.magMin) == 1) {
						SensorStatue.flagMagCal = 0;
						//WriteOffsetToFile();
						offsetHasChanged = 1;
						SendMagCalData();//最好在文件写完以后再发送数据,写文件的时候会进入临界段
					}
				}

#define FILTER_TIMES 10.0f
				for (int i = 0; i < 3; i++) {
					An[i] -= An[i] / FILTER_TIMES;
					An[i] += AHRSData[7 + i];
					AHRSData[7 + i] = (An[i] / FILTER_TIMES)*SensorStatue.gain[i] - (SensorStatue.magMax[i] + SensorStatue.magMin[i]) / 2.0f;
				}
			}
// 			static EKF_Filter ekf;
// 
// 			static short first = 1;
// 			static float fRealQ[4], fRealGyro[3], fQ[4];
// 			fRealGyro[0] = DEGTORAD(AHRSData[4]);
// 			fRealGyro[1] = DEGTORAD(AHRSData[5]);
// 			fRealGyro[2] = DEGTORAD(-AHRSData[6]);
// 			float frpy2[3];
// 			AHRSData[7] = 1;
// 			AHRSData[8] = 0; AHRSData[9] = 0;
// 			if (first == 1) {
// 				EKF_New(&ekf);
// 				first = 0;
// 				Quaternion_From6AxisData(fRealQ, &AHRSData[1], &AHRSData[7]);
// 				EKF_Init(&ekf, fRealQ, fRealGyro);
// 			} else {
// 				Quaternion_From6AxisData(fRealQ, &AHRSData[1], &AHRSData[7]);
// 				EKF_Init(&ekf, fQ, fRealGyro);
// 				EFK_Update(&ekf, fRealQ, fRealGyro, &AHRSData[1], &AHRSData[7], MPU_DT);
// 
// 				EKF_GetAngle(&ekf, frpy2);
// 				EKF_GetQ(&ekf, fQ);
// 			}
			// 
			// 			Quaternion q;
			// 
			// 			MadgwickAHRSupdate(Rad(AHRSData[4]), Rad(AHRSData[5]), Rad(AHRSData[6]), AHRSData[1], AHRSData[2], AHRSData[3], AHRSData[8], AHRSData[7], AHRSData[9]);
			// 
			// 			static EulerAngle e;
			// 			q.q0 = q0, q.q1 = q1, q.q2 = q2, q.q3 = q3;
			// 			//Quaternion_Normalize_(&q);
			// 			Quaternion_ToAngE(&q, &e);

                        
            AHRS_Geteuler(&AHRSData[1], &AHRSData[4], &AHRSData[7], fRPY);
			uint8_t tmp1 = BL_Handler->State;
			static int send_cnt = 0;
			send_cnt++;
			if (send_cnt >= 2) {
				if ((tmp1 == HAL_UART_STATE_READY) || (tmp1 == HAL_UART_STATE_BUSY_RX)) {
					send_cnt = 0;
					AHRS_SendData((short)(fRPY[0] * 10), (short)(fRPY[1] * 10),
						(short)(SensorStatue.altitude), (short)(AHRSData[1] * 10), (short)(AHRSData[2] * 10), \
						(short)(AHRSData[3] * 10), (short)(AHRSData[3] * 10), (short)(AHRSData[4] * 10), \
						(short)(AHRSData[5] * 10), (short)(AHRSData[6] * 10), (short)(10), \
						0, (short)( 10));
				}
			}

			static float controlData_AHRS[7],times=0;

			//static float lastGyro[3];
			if (times<TIMES)
			{
				times++;
				controlData_AHRS[1] += fRPY[0];
				controlData_AHRS[2] += fRPY[1];
				controlData_AHRS[3] += fRPY[2];
				controlData_AHRS[4] += AHRSData[5];
				controlData_AHRS[5] += AHRSData[4];//roll和pitch的陀螺仪需要交换
				controlData_AHRS[6] += AHRSData[6];
			} else {
				times = 0;
				for (int i = 1; i < 7;i++)
				{
					controlData_AHRS[i] /= (float)TIMES;
				}
				if (*(int*)controlData_AHRS == 0)//表示数据已读取,可以更新数据
				{
					OSQPost(&controlQueue,
						(void *)controlData_AHRS,
						7,
						OS_OPT_POST_FIFO,
						&err
						);
					if (err == OS_ERR_NONE) {
						*(int*)controlData_AHRS = 0;
						//lastGyro[0]=controlData_AHRS[]
					}
				}
			}

		}
	}
}