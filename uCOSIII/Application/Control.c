#include "Control.h"
#include "Algorithm_filter.h"

OS_Q controlQueue;
struct _ControlStruct control;

#define RC_CYCLE		11    //ms

#define CH_THROTTLE		2
#define CH_YAW			3
#define CH_PITCH		0
#define CH_ROLL			1
#define CH_LOCK			4
#define CH_MAGISSUE     6


#define RC_FLIGHT_CHECK		(RC_MIN+50)
#define RC_MAX			1900
#define RC_MIN			1100
#define RC_MID			(RC_MAX+RC_MIN)/2
#define LOCK_SECOND		(10)
#define LOCK_TIMES		(LOCK_SECOND*1000/RC_CYCLE)
#define UNLOCK_MIN		(RC_MIN+50)
#define UNLOCK_MAX      (RC_MAX-50)
#define IDEL_CHANGE		50

#define RC_THRESHOLD_MIN (RC_MIN+100)
#define RC_THRESHOLD_MAX (RC_MAX-100)

#define MOTOR_1		0
#define MOTOR_2		2
#define MOTOR_3		3
#define MOTOR_4		1

#define MOTOR_MIN	1100
#define MOTOR_MAX	1900

#define IDEL_SPEED	(RC_MIN+80)

#define SECTION_PARAMETER					"PARAMETER"
#define KEY_PARAMETER_INNER_P				"Inner_P"	
#define KEY_PARAMETER_INNER_I				"Inner_I"
#define KEY_PARAMETER_INNER_D				"Inner_D"
#define KEY_PARAMETER_YAW_OUTER_P			"YAW_OUT_P"	
#define KEY_PARAMETER_YAW_OUTER_I			"YAW_OUT_I"	
#define KEY_PARAMETER_YAW_P					"YAW_P"	
#define KEY_PARAMETER_YAW_I					"YAW_I"
#define KEY_PARAMETER_YAW_D					"YAW_D"
#define KEY_PARAMETER_OUTER_P				"Outer_P"
#define KEY_PARAMETER_OUTER_I				"Outer_I"
#define KEY_PARAMETER_INNER_INTEGRAL_LIMIT	"Inner_Int_Limit"
#define KEY_PARAMETER_OUTER_INTEGRAL_LIMIT	"Outer_Int_Limit"
#define KEY_PARAMETER_YAW_INTEGRAL_LIMIT	"YAW_Int_Limit"
#define KEY_PARAMETER_ROLL_DEGREE_LIMIT		"Roll_Deg_Limit"
#define KEY_PARAMETER_PITCH_DEGREE_LIMIT	"Pitch_Deg_Limit"
#define KEY_PARAMETER_YAW_DEGREE_LIMIT		"Yaw_Deg_Limit"


#define KEY_OFFSET_ROLL "ROLL"
#define KEY_OFFSET_PITCH "PITCH"

#define YAW_MODE_LOCK	0
#define YAW_MODE_DPS	1

short needLoadParameterFromFile = 0;

void UpdatePWM(struct _ControlStruct *ctrl);
void UpdatePID(struct _ControlStruct *ctrl);
void UpdateTarget(struct _ControlStruct *ctrl);
void ResetIntegral(struct _ControlStruct *ctrl);

void ControlTask(void *parg) {
	OS_MSG_SIZE size = 0;
	OS_ERR err;
	float *controlData = NULL;

	OSQCreate((OS_Q     *)&controlQueue,
		(CPU_CHAR *)"ControlQueue",
		(OS_MSG_QTY)5,
		(OS_ERR   *)&err);



	MX_TIM1_Init();
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	//7个通道的输入捕获
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_4);

	control.flag_Lock = 1;//上锁
	control.limitPitchDegree = 20;
	control.limitRollDegree = 15;//度
	//control.limitPIDOut = 900;
	//control.MotorMAX = 1950;
	control.limitInnerIntegral = 100;
	control.limitOutterIntegral = 20;

	extern float KpDef;
	extern float KiDef;

	AddStorageParameter(SECTION_PARAMETER, KEY_PARAMETER_OUTER_P, &control.outer.pitch.kp);
	AddStorageParameter(SECTION_PARAMETER, KEY_PARAMETER_OUTER_I, &control.outer.pitch.ki);
	AddStorageParameter(SECTION_PARAMETER, KEY_PARAMETER_INNER_P, &control.inner.pitch.kp);
	AddStorageParameter(SECTION_PARAMETER, KEY_PARAMETER_INNER_I, &control.inner.pitch.ki);
	AddStorageParameter(SECTION_PARAMETER, KEY_PARAMETER_INNER_D, &control.inner.pitch.kd);
	AddStorageParameter(SECTION_PARAMETER, KEY_PARAMETER_INNER_INTEGRAL_LIMIT, &control.limitInnerIntegral);
	AddStorageParameter(SECTION_PARAMETER, KEY_PARAMETER_OUTER_INTEGRAL_LIMIT, &control.limitOutterIntegral);
	AddStorageParameter(SECTION_PARAMETER, KEY_PARAMETER_YAW_INTEGRAL_LIMIT, &control.limitYawIntegral);

	AddStorageParameter(SECTION_PARAMETER, KEY_PARAMETER_PITCH_DEGREE_LIMIT, &control.limitPitchDegree);
	AddStorageParameter(SECTION_PARAMETER, KEY_PARAMETER_ROLL_DEGREE_LIMIT, &control.limitRollDegree);
	AddStorageParameter(SECTION_PARAMETER, "GYRO_KP", &KpDef);
	AddStorageParameter(SECTION_PARAMETER, "GYRO_KI", &KiDef);
	AddStorageParameter(SECTION_PARAMETER, KEY_PARAMETER_YAW_DEGREE_LIMIT, &control.limitYawDPS);
	AddStorageParameter(SECTION_PARAMETER, KEY_PARAMETER_YAW_P, &control.inner.yaw.kp);
	AddStorageParameter(SECTION_PARAMETER, KEY_PARAMETER_YAW_I, &control.inner.yaw.ki);
	AddStorageParameter(SECTION_PARAMETER, KEY_PARAMETER_YAW_D, &control.inner.yaw.kd);
	AddStorageParameter(SECTION_PARAMETER, KEY_PARAMETER_YAW_OUTER_P, &control.outer.yaw.kp);
	AddStorageParameter(SECTION_PARAMETER, KEY_PARAMETER_YAW_OUTER_I, &control.outer.yaw.ki);
	AddStorageParameter("OFFSET", KEY_OFFSET_PITCH, &SensorStatue.offsetPitch);
	AddStorageParameter("OFFSET", KEY_OFFSET_ROLL, &SensorStatue.offsetRoll);

	needLoadParameterFromFile = 1;


	AddParameter("Offset_P", &SensorStatue.offsetPitch, 0.1);
	AddParameter("Offset_R", &SensorStatue.offsetRoll, 0.1);
	AddParameter("out_p", &control.outer.pitch.kp, 0.1);
	AddParameter("out_i", &control.outer.pitch.ki, 0.1);
	AddParameter("in_p", &control.inner.pitch.kp, 0.1);
	AddParameter("in_i", &control.inner.pitch.ki, 0.1);
	AddParameter("in_d", &control.inner.pitch.kd, 0.1);
	//AddParameter("Yaw_O_P",&control.outer.yaw.kp,0.1);
	//AddParameter("Yaw_O_I", &control.outer.yaw.ki, 0.1);
	AddParameter("Yaw_P", &control.inner.yaw.kp, 0.1);
	AddParameter("Yaw_I", &control.inner.yaw.ki, 0.1);
	AddParameter("Yaw_D", &control.inner.yaw.kd, 0.1);
	AddParameter("Yaw_I_Limit", &control.limitYawIntegral, 10);
	//AddParameter("PIDLimit", &control.limitPIDOut, 20);
	//AddParameter("M_Limit", &control.MotorMAX, 50);
	AddParameter("In_I_Limit", &control.limitInnerIntegral, 10);
	AddParameter("OUT_I_Limit", &control.limitOutterIntegral, 10);
	AddParameter("Limit_Roll", &control.limitRollDegree, 1);
	AddParameter("Limit_Pitch", &control.limitPitchDegree, 1);
	AddParameter("Limit_Yaw", &control.limitYawDPS, 1);
	AddParameter("CF_KP", &KpDef, 0.1);
	SendParameter();


	//	SensorStatue.magIssue = 1;
	while (DEF_TRUE) {
		static int contrlCnt = 0;
		contrlCnt++;
		controlData = (float*)OSQPend(&controlQueue,
			0,
			OS_OPT_PEND_BLOCKING,
			&size,
			NULL,
			&err
			);

		//static int errorcnt = 0;
		//static float rawGyro[3];
		if (controlData != NULL&&size == 7) {
			//姿态更新
			*(int*)controlData = 0;
			static float lastGyro[3];
			control.currentAttitude.roll = -controlData[1]+ SensorStatue.offsetRoll;
			control.currentAttitude.pitch = controlData[2]+SensorStatue.offsetPitch;
			control.currentAttitude.yaw = controlData[3];
			control.gyro.pitch = (controlData[4] );
			control.gyro.roll = -(controlData[5] );
			control.gyro.yaw = controlData[6];

#define lpfFactor 0.8
			LPF_1st(&lastGyro[0], &control.gyro.pitch, lpfFactor);
			LPF_1st(&lastGyro[1], &control.gyro.roll, lpfFactor);
			LPF_1st(&lastGyro[2], &control.gyro.yaw, lpfFactor);

			UpdatePID(&control);
			UpdatePWM(&control);
		} else if (controlData == NULL&&size == 4) {
			//遥控器更新

			static short lastCapture[7];
			static short flagFirstCapture = 5;

			if (flagFirstCapture <= 0) {
				for (int i = 0; i < 7; i++) {
					if (control.PWMCapture[i]<(RC_MIN - 50) || control.PWMCapture[i]>(RC_MAX + 50)) {
						//LED_RED_ON();
						control.PWMCapture[i] = lastCapture[i];
						//errorcnt++;
					} else {
						lastCapture[i] = control.PWMCapture[i];
					}
				}
			}
			if (flagFirstCapture > 0) {
				flagFirstCapture--;
			}


			static short /*unlockHasChecked = 0,*/ lockHasChecked = 0;

			if (control.PWMCapture[CH_LOCK] > RC_THRESHOLD_MAX) {
				lockHasChecked = 0;//如果拨回开关则清除上锁检查标志
				if (control.flag_Lock == 1 /*&& unlockHasChecked == 0*/) {
					// 					if (control.PWMCapture[CH_THROTTLE] < (RC_MIN + 50)) {
					// 						//要求解锁的时候油门最低

					if (control.PWMCapture[CH_THROTTLE] < UNLOCK_MIN\
						&&control.PWMCapture[CH_YAW] > UNLOCK_MAX\
						&&control.PWMCapture[CH_PITCH] < UNLOCK_MIN\
						&&control.PWMCapture[CH_ROLL] < UNLOCK_MIN) {
						if (ABS(control.currentAttitude.pitch)>15 || ABS(control.currentAttitude.roll)>15) {
							LED_RED_ON();
						} else {
							control.flag_Lock = 0;
						}
					} else {
						//如果油门未到最低,则需要重新拨解锁
						//unlockHasChecked = 1;
					}
				}
			} else if (control.PWMCapture[CH_LOCK] < RC_THRESHOLD_MIN) {
				//unlockHasChecked = 0;
				if (control.flag_Lock == 0 && lockHasChecked == 0) {
					//if (control.PWMCapture[CH_THROTTLE] < (RC_MIN + 50))//上锁时需要油门置零
					{
						control.flag_Lock = 1;
					}
					// 				else {
					// 						lockHasChecked = 1;
					// 					}
				}
			}
			//if (control.PWMCapture[CH_MAGISSUE] > RC_THRESHOLD_MIN) 
                        {
				SensorStatue.magIssue = 1;
			} 
                        //else if (control.PWMCapture[CH_MAGISSUE] < RC_THRESHOLD_MAX) {
			//	SensorStatue.magIssue = 0;
			//}
			UpdateTarget(&control);
			// 			uint8_t tmp1 = BL_Handler->State;
			// 			if ((tmp1 == HAL_UART_STATE_READY) || (tmp1 == HAL_UART_STATE_BUSY_RX)) {
			// 				AHRS_SendData((short)(control.PWMCapture[0]), (short)(control.PWMCapture[1]), (short)(control.PWMCapture[2]), (short)control.PWMCapture[3], (short)control.PWMCapture[4], (short)control.PWMCapture[5], (short)control.PWMCapture[6], 0, 0, 0,
			// 					(short)((gyroHistory[0][0] - gyroHistory[4][0])*0.6+lastDiff_pitch*0.4), (short)((gyroHistory[0][1] - gyroHistory[4][1])*0.6+lastDiff_roll*0.4), (short)(gyroHistory[0][2] - gyroHistory[4][2]));
			// 			}
		}
		if (control.flag_Lock == 0) {
			for (int i = 0; i < 4; i++) {
				control.PWMOutput[i] = control.motorOut[i] < IDEL_SPEED ? IDEL_SPEED : control.motorOut[i];//不能小于怠速速度
			}
		} else {
			control.PWMOutput[0] = 1000;
			control.PWMOutput[1] = 1000;
			control.PWMOutput[2] = 1000;
			control.PWMOutput[3] = 1000;
		}
		if (ABS(control.gyro.pitch) > 1000 || ABS(control.gyro.roll)>1000 || ABS(control.currentAttitude.pitch) > 360 || ABS(control.currentAttitude.roll) > 360) {
			LED_RED_ON();
		}
//		 		 		uint8_t tmp1 = BL_Handler->State;
//		 		 		static int send_cnt = 0;
//		 		 		send_cnt++;
//		 		 		if (send_cnt >= 2) {
//		 		 			if ((tmp1 == HAL_UART_STATE_READY) || (tmp1 == HAL_UART_STATE_BUSY_RX)) {
//		 		 				send_cnt = 0;
//		 		 				AHRS_SendData((short)(control.gyro.pitch * 10), (short)(control.gyro.roll * 10),
//		 		 					(short)(control.currentAttitude.pitch * 10), (short)(control.currentAttitude.roll * 10), (short)(control.currentAttitude.yaw * 100), (short)control.inner.pitch.pidOut, (short)control.inner.roll.pidOut, (short)control.outer.pitch.integral, (short)control.outer.roll.integral, (short)control.inner.pitch.integral, \
//		 	 					0, 0, 0);
//		 		 			}
//		 		 		}
						// 		//测试

		//                static int ch1,ch2,ch3,ch4;
		//                control.PWMOutput[0]=ch1;
		//                control.PWMOutput[1]=ch2;
		//                control.PWMOutput[2]=ch3;
		//                control.PWMOutput[3]=ch4;

		if (control.PWMOutput[0] < MOTOR_MIN)control.PWMOutput[0] = MOTOR_MIN;
		if (control.PWMOutput[1] < MOTOR_MIN)control.PWMOutput[1] = MOTOR_MIN;
		if (control.PWMOutput[2] < MOTOR_MIN)control.PWMOutput[2] = MOTOR_MIN;
		if (control.PWMOutput[3] < MOTOR_MIN)control.PWMOutput[3] = MOTOR_MIN;


		if (control.PWMOutput[0] > MOTOR_MAX)control.PWMOutput[0] = MOTOR_MAX;
		if (control.PWMOutput[1] > MOTOR_MAX)control.PWMOutput[1] = MOTOR_MAX;
		if (control.PWMOutput[2] > MOTOR_MAX)control.PWMOutput[2] = MOTOR_MAX;
		if (control.PWMOutput[3] > MOTOR_MAX)control.PWMOutput[3] = MOTOR_MAX;

		// 		if (control.PWMOutput[0] > MOTOR_MAX)control.PWMOutput[0] = MOTOR_MAX;
		// 		if (control.PWMOutput[1] > MOTOR_MAX)control.PWMOutput[1] = MOTOR_MAX;
		// 		if (control.PWMOutput[2] > MOTOR_MAX)control.PWMOutput[2] = MOTOR_MAX;
		// 		if (control.PWMOutput[3] > MOTOR_MAX)control.PWMOutput[3] = MOTOR_MAX;

		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, control.PWMOutput[0]);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, control.PWMOutput[1]);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, control.PWMOutput[2]);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, control.PWMOutput[3]);
	}
}

void UpdatePID(struct _ControlStruct *ctrl) {

	ctrl->outer.roll.kp = ctrl->outer.pitch.kp;
	ctrl->outer.roll.ki = ctrl->outer.pitch.ki;
	ctrl->inner.roll.kp = ctrl->inner.pitch.kp;
	ctrl->inner.roll.kd = ctrl->inner.pitch.kd;
	ctrl->inner.roll.ki = ctrl->inner.pitch.ki;

#define GYRO_FILTER_TIMES 3

	static float gyroHistory[GYRO_FILTER_TIMES][3];
	for (int i = 0; i < (GYRO_FILTER_TIMES - 1); i++) {
		for (int j = 0; j < 3; j++) {
			gyroHistory[i][j] = gyroHistory[i + 1][j];
		}
	}
	gyroHistory[GYRO_FILTER_TIMES - 1][0] = ctrl->gyro.pitch;
	gyroHistory[GYRO_FILTER_TIMES - 1][1] = ctrl->gyro.roll;
	gyroHistory[GYRO_FILTER_TIMES - 1][2] = ctrl->gyro.yaw;

	float deviation = ctrl->targetAttitude.pitch - ctrl->currentAttitude.pitch;
	ctrl->outer.pitch.integral += deviation*INTEGRATION_TIME;
	ctrl->outer.pitch.integral = data_limit(ctrl->outer.pitch.integral, ctrl->limitOutterIntegral, -ctrl->limitOutterIntegral);

	//假设如果目标是20度,现在是0度,error=20,kp为1,需要的角速度是20deg/s,所以不加负号
	ctrl->outer.pitch.pidOut = (ctrl->outer.pitch.kp*deviation + ctrl->outer.pitch.ki*ctrl->outer.pitch.integral);

	deviation = ctrl->targetAttitude.roll - ctrl->currentAttitude.roll;
	ctrl->outer.roll.integral += deviation*INTEGRATION_TIME;
	ctrl->outer.roll.integral = data_limit(ctrl->outer.roll.integral, ctrl->limitOutterIntegral, -ctrl->limitOutterIntegral);

	ctrl->outer.roll.pidOut = (ctrl->outer.roll.kp*deviation + ctrl->outer.roll.ki*ctrl->outer.roll.integral);

	//内环yaw用摇杆控制
	if (ctrl->yawMode == YAW_MODE_DPS) {
		ctrl->outer.yaw.pidOut = ctrl->targetAttitude.yaw;
		ctrl->outer.yaw.integral = 0;
	} else {
		deviation = ctrl->targetAttitude.yaw - ctrl->currentAttitude.yaw;
		if (deviation > 180) {
			deviation -= 360;
		} else if (deviation < -180) {
			deviation += 360;
		}
		ctrl->outer.yaw.integral += deviation*INTEGRATION_TIME;
		ctrl->outer.yaw.integral = data_limit(ctrl->outer.yaw.integral, ctrl->limitOutterIntegral, -ctrl->limitOutterIntegral);
		ctrl->outer.yaw.pidOut = ctrl->outer.yaw.kp * deviation + ctrl->outer.yaw.ki*ctrl->outer.yaw.integral;
	}

	ctrl->outer.yaw.pidOut = data_limit(ctrl->outer.yaw.pidOut, ctrl->limitYawDPS, -ctrl->limitYawDPS);
	//这里需要对外环的pid输出限幅
	//内环计算
	deviation = ctrl->outer.pitch.pidOut - ctrl->gyro.pitch;
	ctrl->inner.pitch.integral += deviation*INTEGRATION_TIME;
	ctrl->inner.pitch.integral = data_limit(ctrl->inner.pitch.integral, ctrl->limitInnerIntegral, -ctrl->limitInnerIntegral);


	/*static float lastDiff_pitch, lastDiff_roll;*/
	float diff = gyroHistory[0][0] - ctrl->gyro.pitch;
	ctrl->inner.pitch.pidOut = (ctrl->inner.pitch.kp*deviation\
		+ ctrl->inner.pitch.kd*data_limit(diff, 30, -30)\
		+ ctrl->inner.pitch.ki*ctrl->inner.pitch.integral);
	//ctrl->inner.pitch.pidOut = (ctrl->inner.pitch.pidOut > ctrl->limitPIDOut ? ctrl->limitPIDOut : ctrl->inner.pitch.pidOut);
	//ctrl->inner.pitch.pidOut = (ctrl->inner.pitch.pidOut < (-ctrl->limitPIDOut) ? (-ctrl->limitPIDOut) : ctrl->inner.pitch.pidOut);

	diff = gyroHistory[0][1] - ctrl->gyro.roll;
	deviation = ctrl->outer.roll.pidOut - ctrl->gyro.roll;
	ctrl->inner.roll.integral += deviation*INTEGRATION_TIME;
	ctrl->inner.roll.integral = data_limit(ctrl->inner.roll.integral, ctrl->limitInnerIntegral, -ctrl->limitInnerIntegral);

	ctrl->inner.roll.pidOut = (ctrl->inner.roll.kp*deviation\
		+ ctrl->inner.roll.kd*data_limit(diff, 30, -30)\
		+ ctrl->inner.roll.ki*ctrl->inner.roll.integral);
	//ctrl->inner.roll.pidOut = (ctrl->inner.roll.pidOut > ctrl->limitPIDOut ? ctrl->limitPIDOut : ctrl->inner.roll.pidOut);
	//ctrl->inner.roll.pidOut = (ctrl->inner.roll.pidOut < (-ctrl->limitPIDOut) ? ctrl->limitPIDOut : ctrl->inner.roll.pidOut);

	diff = gyroHistory[0][2] - ctrl->gyro.yaw;
	deviation = ctrl->outer.yaw.pidOut - ctrl->gyro.yaw;

	ctrl->inner.yaw.integral += deviation*INTEGRATION_TIME;
	ctrl->inner.yaw.integral = data_limit(ctrl->inner.yaw.integral, ctrl->limitInnerIntegral, -ctrl->limitInnerIntegral);
	ctrl->inner.yaw.pidOut = (ctrl->inner.yaw.kp*deviation\
		+ ctrl->inner.yaw.kd*(data_limit(diff, 30, -30))\
		+ ctrl->inner.yaw.ki*ctrl->inner.yaw.integral);

	// 	ctrl->lastGyro.pitch = ctrl->gyro.pitch;
	// 	ctrl->lastGyro.roll = ctrl->gyro.roll;
	// 	ctrl->lastGyro.yaw = ctrl->gyro.yaw;
}

void UpdatePWM(struct _ControlStruct *ctrl) {
	float roll = ctrl->inner.roll.pidOut;
	float pitch = ctrl->inner.pitch.pidOut;
	float yaw = ctrl->inner.yaw.pidOut;



	int throttle = (int)((ctrl->PWMCapture[CH_THROTTLE]) / (arm_cos_f32(ctrl->currentAttitude.pitch / 57.324841f)) / (arm_cos_f32(ctrl->currentAttitude.roll / 57.324841f)));

	//假设target是0,机头上扬为正,角速度为正,error为负,p为正,pidOut为负,1,2电机应该减速,所以12电机的pitch为+
	//架设target是0,向右翻滚为正,角速度为正,error为负,p为正,pidOut为负,2,3电机应该加速,所以23电机的roll为-  //这个变反了,不知道为什么
	//从上往下看顺时针转角速度为正,目标为0,error为负,p为正,pidout为负,2,4电机加速,13电机减速(如果电机转向估算没错)
	//roll的p为正,d为正,i为正
	ctrl->motorOut[MOTOR_1] = (short)(throttle + pitch + roll + yaw);
	ctrl->motorOut[MOTOR_2] = (short)(throttle + pitch - roll - yaw);
	ctrl->motorOut[MOTOR_3] = (short)(throttle - pitch - roll + yaw);
	ctrl->motorOut[MOTOR_4] = (short)(throttle - pitch + roll - yaw);


}
void UpdateTarget(struct _ControlStruct *ctrl) {
	static short flagYawHasConfirmed = 0;
	if (ctrl->flag_Lock == 0) {
		if (ctrl->PWMCapture[CH_THROTTLE] < RC_FLIGHT_CHECK) {
			//如果小于检查值,则说明不想起飞,不计算目标姿态
			//ctrl->targetAttitude.pitch = 0;
			//ctrl->targetAttitude.roll = 0;
			ctrl->targetAttitude.pitch = ctrl->currentAttitude.pitch;
			ctrl->targetAttitude.roll = ctrl->currentAttitude.roll;
			ctrl->yawMode = YAW_MODE_DPS;
			ctrl->targetAttitude.yaw = 0;
			flagYawHasConfirmed = 0;
		} else {
			if (ctrl->PWMCapture[CH_THROTTLE] < (RC_MID)) {
				//油门在小于起飞油门的时候清零积分制
				ResetIntegral(&control);
			}
			ctrl->targetAttitude.pitch = -ctrl->limitPitchDegree / (RC_MAX - RC_MID)*(ctrl->PWMCapture[CH_PITCH] - RC_MID);
			ctrl->targetAttitude.roll = ctrl->limitRollDegree / (RC_MAX - RC_MID)*(ctrl->PWMCapture[CH_ROLL] - RC_MID);

			static short isConfirm = 0;
			short temp = ctrl->PWMCapture[CH_YAW] - RC_MID;
			if (ABS(temp) > 50) {
				isConfirm = 0;//有舵量的时候就按照给定的角速度来旋转,没有舵量的时候锁定
				ctrl->targetAttitude.yaw = ctrl->limitYawDPS / (RC_MAX - RC_MID)*(ctrl->PWMCapture[CH_YAW] - RC_MID);
				ctrl->yawMode = YAW_MODE_DPS;
			} else	if (flagYawHasConfirmed == 0) {
				isConfirm = 1;
				flagYawHasConfirmed = 1;
				ctrl->yawMode = YAW_MODE_LOCK;
				ctrl->targetAttitude.yaw = ctrl->currentAttitude.yaw;
			} else if (isConfirm == 0) {
				isConfirm = 1;
				ctrl->yawMode = YAW_MODE_LOCK;
				ctrl->targetAttitude.yaw = ctrl->currentAttitude.yaw;
			}
			//ctrl->targetAttitude.yaw = ctrl->limitYawDPS / (RC_MAX - RC_MID)*(ctrl->PWMCapture[CH_YAW] - RC_MID);
		}
	}
}
void ResetIntegral(struct _ControlStruct *ctrl) {
	ctrl->outer.pitch.integral = 0;
	ctrl->outer.roll.integral = 0;
	ctrl->outer.yaw.integral = 0;
	ctrl->inner.pitch.integral = 0;
	ctrl->inner.roll.integral = 0;
	ctrl->inner.yaw.integral = 0;
}
/*
Ch1 --- TIM2_Ch2 --- PB3
Ch2 --- TIM3_Ch1 --- PB4
Ch3 --- TIM3_Ch2 --- PB5
Ch4 --- TIM4_Ch1 --- PB6
Ch5 --- TIM4_Ch2 --- PB7
Ch6 --- TIM4_Ch3 --- PB8
Ch7 --- TIM4_Ch4 --- PB9
*/

/*
油门:上推增大
yaw:右推增大
pitch:上推增大
roll:右推增大
*/

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	static int capture[7][2];
	static uint8_t flag[7];
	if (htim->Instance == TIM2) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_SET) {
				capture[0][0] = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
				flag[0] = 0;
			} else {
				if (flag[0] == 0) {
					flag[0] = 1;
					capture[0][1] = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
					if (capture[0][1] > capture[0][0]) {
						control.PWMCapture[0] = capture[0][1] - capture[0][0];
					} else {
						control.PWMCapture[0] = 0xFFFF - capture[0][0] + capture[0][1];
					}
				}
			}
		}
	} else if (htim->Instance == TIM3) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_SET) {
				capture[1][0] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
				flag[1] = 0;
			} else {
				if (flag[1] == 0) {
					flag[1] = 1;
					capture[1][1] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
					if (capture[1][1] > capture[1][0]) {
						control.PWMCapture[1] = capture[1][1] - capture[1][0];
					} else {
						control.PWMCapture[1] = 0xFFFF - capture[1][0] + capture[1][1];
					}
				}
			}
		}
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET) {
				capture[2][0] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);
				flag[2] = 0;
			} else {
				if (flag[2] == 0) {
					flag[2] = 1;
					capture[2][1] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);
					if (capture[2][1] > capture[2][0]) {
						control.PWMCapture[2] = capture[2][1] - capture[2][0];
					} else {
						control.PWMCapture[2] = 0xFFFF - capture[2][0] + capture[2][1];
					}
				}
			}
		}
	} else if (htim->Instance == TIM4) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET) {
				capture[3][0] = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1);
				flag[3] = 0;
			} else {
				if (flag[3] == 0) {
					flag[3] = 1;
					capture[3][1] = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1);
					if (capture[3][1] > capture[3][0]) {
						control.PWMCapture[3] = capture[3][1] - capture[3][0];
					} else {
						control.PWMCapture[3] = 0xFFFF - capture[3][0] + capture[3][1];
					}
				}
			}
		}
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET) {
				capture[4][0] = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_2);
				flag[4] = 0;
			} else {
				if (flag[4] == 0) {
					flag[4] = 1;
					capture[4][1] = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_2);
					if (capture[4][1] > capture[4][0]) {
						control.PWMCapture[4] = capture[4][1] - capture[4][0];
					} else {
						control.PWMCapture[4] = 0xFFFF - capture[4][0] + capture[4][1];
					}
				}
			}
		}
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET) {
				capture[5][0] = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_3);
				flag[5] = 0;
			} else {
				if (flag[5] == 0) {
					flag[5] = 1;
					capture[5][1] = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_3);
					if (capture[5][1] > capture[5][0]) {
						control.PWMCapture[5] = capture[5][1] - capture[5][0];
					} else {
						control.PWMCapture[5] = 0xFFFF - capture[5][0] + capture[5][1];
					}
				}
			}
		}
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET) {
				capture[6][0] = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_4);
				flag[6] = 0;
			} else {
				if (flag[6] == 0) {
					flag[6] = 1;
					capture[6][1] = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_4);
					if (capture[6][1] > capture[6][0]) {
						control.PWMCapture[6] = capture[6][1] - capture[6][0];
					} else {
						control.PWMCapture[6] = 0xFFFF - capture[6][0] + capture[6][1];
					}
				}
			}
		}
	}
	if (flag[0] == 1 && flag[1] == 1 && flag[2] == 1 && flag[3] == 1 && flag[4] == 1 && flag[5] == 1 && flag[6] == 1) {
		//所有通道都捕获到了
		OS_ERR err;
		//Post一个空消息
		OSQPost((OS_Q *)&controlQueue,
			(void *)NULL,
			(OS_MSG_SIZE)4,
			OS_OPT_POST_FIFO,
			&err);
		for (int i = 0; i < 7; i++) {
			flag[i] = 0;
		}
	}
}