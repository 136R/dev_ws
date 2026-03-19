#ifndef __APP_CONFIG_H_
#define __APP_CONFIG_H_
#include "main.h"

#define DEBUG_MODE	1
#ifdef DEBUG_MODE
	#define DEBUG_PERIOD	100
#endif

//#define DMA_PRINT 1

#define VERSION_INFO
#ifdef VERSION_INFO
#define SOFTWARE_MAIN_VERSION	1
#define SOFTWARE_SUBVERSION   8
#define SOFTWARE_PATCHLEVEL   2

#define HARDWARE_MAIN_VERSION	1
#define HARDWARE_SUBVERSION   6
#define HARDWARE_PATCHLEVEL   0
#endif

#define BASE_4WD	4
#define BASE_2WD	2
#define BASE_ACKERMAN 1

#define DIFF_2WD	1
#define DIFF_4WD	2
#define ACKERMAN  3
#define MECANUM 	4

#define _25GA370	1
#define	_37GB520  2

#define ENCODER_CW   1
#define ENCODER_CCW  -1

#define BATTERY_VOLTAGE_CHECK_PERIOD	100
#define BATTERY_CURRENT_CHECK_PERIOD	5
#define MOTOR_CONTROL_PERIOD	10
#define PS2_CHECK_PERIOID			20
#define BATTERY_LOW_VOLTAGE		9.6f
#define DISCONNECT_WATI_TIME	1000
#define VELOCITY_LAST_TIME		800
#define SONAR_CHECK_PERIOD	  50
#define INDICATE_PERIOD				10

#define K_P    0.25f // P constant
#define K_I    0.15f // I constant
#define K_D    0.0f // D constant

#define VOLTAGE_ADC_CHANNEL ADC_CHANNEL_1
#define CURRENT_ADC_CHANNEL	ADC_CHANNEL_12

#define VOLTAGE_R1	47.0f
#define VOLTAGE_R2	10.0f
#define VOLTAGE_ERROR 0.30f
#define VOLTAGE_STATE_CHANGE_GAP 0.2f

#define CURRENT_RES	0.01f
#define CURRENT_SCALE	20

#define SONAR_CHANNEL_NUM 1

//#define BASE_TYPE	BASE_4WD

//GA370 Motor Param
#define MOTOR_TYPE_1 			_25GA370
#define MOTOR_MAX_RPM_1 	6000
#define ENCODER_PPR_1     11
#define RATIO_1						20.0f	// reduction ratio param 1
#define ENCODER_DIR_1			ENCODER_CCW

//GB520 Motor Param
#define MOTOR_TYPE_2 			_37GB520
#define MOTOR_MAX_RPM_2 	10000
#define ENCODER_PPR_2     11
#define RATIO_2						30.00f	// reduction ratio param 1
#define ENCODER_DIR_2			ENCODER_CW

//NanoRobot Chassis Param(2WD Diff)
#define WHEEL_TREAD_1			0.171f
#define WHEEL_BASE_1			0.150f
#define WHEEL_DIAMETER_1  0.065f //wheel's diameter in meters
#define MAX_X_VELOCITY_1	MOTOR_MAX_RPM_1/60.0/RATIO_1*WHEEL_DIAMETER_1*3.14159f*0.8f
#define MAX_Y_VELOCITY_1  MOTOR_MAX_RPM_1/60.0/RATIO_1*WHEEL_DIAMETER_1*3.14159f*0.8f
#define MAX_Z_ANGLE_1			MAX_X_VELOCITY_1/(WHEEL_TREAD_1/2.0f)
#define KINETICS_TYPE_1   DIFF_2WD 

//NanoCar Chassis Param(Ackerman Steering)
#define WHEEL_TREAD_2			0.158f
#define WHEEL_BASE_2			0.1445f
#define WHEEL_DIAMETER_2  0.064f //wheel's diameter in meters
#define MAX_X_VELOCITY_2	MOTOR_MAX_RPM_2/60.0/RATIO_2*WHEEL_DIAMETER_2*3.14159f*0.8f
#define MAX_Y_VELOCITY_2  MOTOR_MAX_RPM_2/60.0/RATIO_2*WHEEL_DIAMETER_2*3.14159f*0.8f
#define MAX_Z_ANGLE_2			MAX_X_VELOCITY_2/(WHEEL_TREAD_2/2.0f)
#define KINETICS_TYPE_2   ACKERMAN 

//NanoCar Chassis Param(Ackerman Steering in Diff Mode)
#define WHEEL_TREAD_3			0.158f
#define WHEEL_BASE_3			0.1445f
#define WHEEL_DIAMETER_3  0.064f //wheel's diameter in meters
#define MAX_X_VELOCITY_3	MOTOR_MAX_RPM_2/60.0/RATIO_2*WHEEL_DIAMETER_3*3.14159*0.8f
#define MAX_Y_VELOCITY_3  MOTOR_MAX_RPM_2/60.0/RATIO_2*WHEEL_DIAMETER_3*3.14159*0.8f
#define MAX_Z_ANGLE_3			MAX_X_VELOCITY_3/(WHEEL_TREAD_3/2.0f)
#define KINETICS_TYPE_3   DIFF_2WD

////NEW 4WD Chassis Param(MECANUM)
//#define WHEEL_TREAD_3			0.225f
//#define WHEEL_BASE_3			0.214f
//#define WHEEL_DIAMETER_3  0.080f //wheel's diameter in meters
//#define MAX_X_VELOCITY_3	MOTOR_MAX_RPM_2/60.0/RATIO_2*WHEEL_DIAMETER_3*3.14159*0.8f
//#define MAX_Y_VELOCITY_3  MOTOR_MAX_RPM_2/60.0/RATIO_2*WHEEL_DIAMETER_3*3.14159*0.8f
//#define MAX_Z_ANGLE_3			MAX_X_VELOCITY_3/(WHEEL_TREAD_3/2.0f)
//#define KINETICS_TYPE_3   MECANUM

////NEW 4WD Chassis Param(DIFF_4WD)
//#define WHEEL_TREAD_4			0.225f
//#define WHEEL_BASE_4			0.214f
//#define WHEEL_DIAMETER_4  0.080f //wheel's diameter in meters
//#define MAX_X_VELOCITY_4	MOTOR_MAX_RPM_2/60.0/RATIO_2*WHEEL_DIAMETER_4*3.14159*0.8f
//#define MAX_Y_VELOCITY_4  MOTOR_MAX_RPM_2/60.0/RATIO_2*WHEEL_DIAMETER_4*3.14159*0.8f
//#define MAX_Z_ANGLE_4			MAX_X_VELOCITY_4/(WHEEL_TREAD_4/2.0f)
//#define KINETICS_TYPE_4   DIFF_4WD

#endif

