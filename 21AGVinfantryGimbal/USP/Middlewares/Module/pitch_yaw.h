#ifndef PITCH_YAW_H
#define PITCH_YAW_H

#include "FreeRTOS.h"
#include "PID.h"
#include <main.h>
#include "motor.h"
#include "mpu6050_config.h"
#include "dr16.h"
#include "PCvision.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern myPID pitchSpeed,pitchAngle,yawSpeed,yawAngle;
extern MPUData_Typedef MPUData;

extern float IMUPitchOffset;

enum E_pitchYawPidType
{
	PITCH_SPEED = 0,
	PITCH_ANGLE = 1,
	YAW_SPEED   = 2,
	YAW_ANGLE = 3,
	
};
enum E_pitchYawMode
{
	REMOTE_CTRL_P = 0,
	KEY_BOARD_P   = 1,
	MINI_PC_P     = 2,
	LOST_P 				= 3,
};

class C_Pitch_Yaw
{
	public:
		void IMU_cala(float);
		void pitch_cal();
		void pid_init(E_pitchYawPidType type,float kp,float ki,float kd, float ki_max,float out_max);
		
		void Control(E_pitchYawMode mode, 
								 float pitch_data, 
								 float yaw_data, 
								 float motor_pitch_angle,
								 float IMU_pitch_speed, 
								 float IMU_yaw_angle,
								 float IMU_yaw_speed);
		void Reset();
		void targetUpdate();
		void pidCalc();
		void pithchAngleLimit();
		float feedforward(float target);
		myPID pitchSpeed,yawSpeed,pitchAngle,yawAngle;
		float lastPitchAngleOut,trgAngle,totalYawAngle,lastYawAngle,recPitchData,recYawData,pitchScale,yawScale,lastTotalYawAngle,lastPitchAngle;
		float lastPitchAngleTarget,lastYawAngleTarget;
		int16_t cnt;
		E_pitchYawMode Mode;
		uint8_t snipermode;
		int8_t feedforward_stste = 1;
		int8_t isLandMode = 1;
};



#endif