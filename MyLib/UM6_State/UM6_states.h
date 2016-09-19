/* ------------------------------------------------------------------------------
  File: chr6dm_states.c
  Author: CH Robotics
  Version: 1.0
  
  Description: Function declarations for CHR-6dm state estimation.
------------------------------------------------------------------------------ */ 

//#include "stm32f10x.h"

#ifndef _CHR6DM_STATES__
#define _CHR6DM_STATES__

#include "CHR_quat.h"
#include "CHR_matrix.h"
#include "global.h"
// Structure for holding raw sensor data
typedef struct __RawSensorData {
	 float gyro_x;
	 float gyro_y;
	 float gyro_z;
	 
	 float accel_x;
	 float accel_y;
	 float accel_z;

	 float mag_x;
	 float mag_y;
	 float mag_z;
} RawSensorData;


// Structure for storing AHRS states and other data related to state computation
// This structure is, in a way, redundant because all this data is also stored in the
// UM6_config or UM6_data structures.  However, in the config and data strucutres, the 
// data is packaged as UInt32 entries into an array for convenience with communication.
// To use the data as floats, special formatting is required.  This structure provides
// a place to store that data in the expected format, which makes accessing it easier.
typedef struct __AHRS_state_data {
	 
	 // Orientation states
	 union {
		  float heading;
		  float yaw;
		  float psi;
	 };
	 union {
		  float pitch;
		  float theta;
	 };
	 union {
		  float roll;
		  float phi;
	 };
	 
	 // Orientation rate states
	 union {
		  float heading_rate;
		  float yaw_rate;
		  float psi_dot;
	 };
	 
	 union {
		  float pitch_rate;
		  float theta_dot;
	 };
	 
	 union {
		  float roll_rate;
		  float phi_dot;
	 };
	 
	 // Quaternion states "qib" = Quaternion from Inertial to Body
	 quat qib;
	 
	 // Gyro biases
	 short beta_p,beta_q,beta_r;
	 
	 // Accelerometer biases
	 short beta_acc_x, beta_acc_y, beta_acc_z;
	 
	 // Magnetometer biases
	 short beta_mag_x, beta_mag_y, beta_mag_z;
	 
	 // Process noise matrix
	 fMatrix R;
	 
	 // Accelerometer alignment matrix
	 fMatrix accel_cal;
	 
	 // Gyro alignment matrix
	 fMatrix gyro_cal;
	 
	 // Magnetometer calibration matrix
	 fMatrix mag_cal;
	 	 
	 // EKF covariance
	 fMatrix Sigma;
	 
	 // Magnetic field reference vector
	 float mag_ref_x;
	 float mag_ref_y;
	 float mag_ref_z;
	 
	 // Accelerometer	reference vector
	 float accel_ref_x;
	 float accel_ref_y;
	 float accel_ref_z;
	 
	 // accelerometer measurement variance
	 float accel_var;
	 
	 // Magnetometer variance
	 float mag_var;
	 
	 // Process variance
	 float process_var;
	 
	 // Entries for storing processed sensor data
	 float gyro_x;
	 float gyro_y;
	 float gyro_z;
	 
	 float accel_x;
	 float accel_y;
	 float accel_z;
	 	 
	 float mag_x;
	 float mag_y;
	 float mag_z;
} AHRS_state_data;

extern RawSensorData gSensorData;
extern AHRS_state_data gStateData;

extern unsigned char gEKF_mode;

// Function declarations
//void EKF_Init( AHRS_state_data* estimated_states );
void EKF_InitFromSensors( AHRS_state_data* estimated_states, RawSensorData* sensor_data );
void EKF_EstimateStates( AHRS_state_data* estimated_states, RawSensorData* sensor_data );
void EKF_Predict( AHRS_state_data* estimated_states, RawSensorData* sensor_data );
void EKF_Update( AHRS_state_data* estimated_states, RawSensorData* sensor_data );
void EKF_Correction( fMatrix* C, float sensor_data, float sensor_hat, float sensor_covariance, AHRS_state_data* estimated_states, int sensor_type );

void ConvertRawSensorData( AHRS_state_data* estimated_states, RawSensorData* sensor_data );

#define		MAG_UPDATE			0
#define		ACCEL_UPDATE		1

// EKF "mode" stored in the global variable gEKF_mode
//#define		EKF_MODE_QUAT		0
#define		EKF_MODE_EULER		1


void compute_euler_angles( AHRS_state_data* estimated_states );

void unroll_states( AHRS_state_data* states );

#endif
