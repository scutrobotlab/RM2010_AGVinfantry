#include "Infantry_Config.h"
#include "PID.h"
S_Infantry Infantry;

void Infantry_Config_Init()
{
	Shoot_Init();
	Pitch_Yaw_Init();
}

//发射pid参数设置
void Shoot_Init()
{
	Shoot.pid_init(LEFT_FRI_SPEED,10.0f,0.5f,0.0f,0,30000.0f);
	Shoot.pid_init(RIGHT_FRI_SPEED,10.0f,0.5f,0.0f,0,30000.0f);
	Shoot.pid_init(TURNPLATE_SPEED,20.0f,0.0f,0.0f,7000.0,8000.0f);
	Shoot.pid_init(TURNPLATE_ANGLE,10.0f,0.0f,0.05f,0,8000.0f);
	Shoot.turnPlateSpeed.I_SeparThresh  = 4000;
}

//云台pid参数设置
void Pitch_Yaw_Init()
{
	Pitch_Yaw.pid_init(PITCH_SPEED,14.0f,180.0f,0,9000.0f,30000.0f);
	Pitch_Yaw.pid_init(PITCH_ANGLE,12.0f,200.0f,0.0f,0.0f,10000.0f);
	Pitch_Yaw.pid_init(YAW_SPEED,25.0f,850.0f,0.0f,9000.0f,30000.0f);
	Pitch_Yaw.pid_init(YAW_ANGLE,200.0f,100.0f,0.0f,0.0f,15000.0f);
	Pitch_Yaw.pitchAngle.Target = 8769;
	Pitch_Yaw.pitchSpeed.I_SeparThresh = 15000;
	Pitch_Yaw.yawSpeed.I_SeparThresh = 15000;
	Pitch_Yaw.yawAngle.Target = 0;
	Chassis.pid_init(10.0f,0,0.0f,0,1000.0f);
	Infantry.chassisMode = NORMAL_C;
	Infantry.chassisCtrlMode = REMOTE_CTRL_C;	
	Infantry.isLandFlag = 1;
}
