#ifndef INFANTRY_CONGIF_H
#define INFANTRY_CONGIF_H
//#include "shoot.h"
#include "shoot.h"
#include "pitch_yaw.h"
#include "chassis.h"
#include "Indicator.h"
#include "System_Config.h"
#include "Service_Communication.h"
struct S_Infantry{
	E_pitchYawMode 		pitchYawCtrlMode;
	E_ShootMode 			shootMode;
	E_ShootCtrlMode 	shootCtrlMode;
	E_FriWheelState		friWheelState;
	E_LaserState 			laserState;
	E_BulletBayState 	bulletBayState;
	E_FireFlag 				fireFlag;
	E_ChassisState 		chassisMode;
	E_MoveMode				moveMode;
	E_CtrlMode  			chassisCtrlMode;
	uint8_t						bulletMaxSpeed;   
	uint16_t					coolingRate;
	uint16_t					coolingLimit;
	uint16_t					shooterHeat;
	uint8_t						robotID;
	uint8_t						isLandFlag;
	uint8_t						mode;
	uint8_t						snipermode;
	uint8_t						finalMode;
	uint8_t						power_off;
	uint8_t						fuckMode;//不是闲的别用这玩意!!!!!!!
	uint8_t						GGMode = 0;
	int8_t						states_change;
	uint8_t						maxSpin = 0;
	float							y_pitchYaw;
	float							x_pitchYaw;
	float							y_data;
	float							x_data;
	float							r_data;
	float							y_back;
	float							x_back;
};


extern C_Shoot Shoot;
extern C_Chassis Chassis;
extern C_Pitch_Yaw Pitch_Yaw;
extern S_Infantry Infantry;
extern PackFromVisionUnionDef PackFromVisionUnion;
extern Indicator_Classdef state_light;

void Shoot_Init(void);
void Infantry_Config_Init(void);
void Pitch_Yaw_Init();


	

#endif