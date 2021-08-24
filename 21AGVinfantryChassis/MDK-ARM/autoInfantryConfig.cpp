#include "autoInfantryConfig.h"
#include "Source_manage.h"
#include "PID.h"
#include "board_com.h"
#include "Service_Communication.h"
uint8_t a=0,abab=0;
//C_Chassis Chassis;
C_BoardCom BoardCom;
referee_Classdef Referee;
PowerCtrl_ClassDef PowerCtrl( 4, REAL_POWER_LOOP, __ENABLE, 1);	
myPID powerCtrl;
myPID capCtrl;


C_SourceManage_Classdef SourceManage(CAP);
//C_SourceManage_Classdef SourceManage(BAT);
S_Source_Typedef source;

uint32_t Get_RefereeTime()
{
	return xTaskGetTickCount()*1000;
}

void AutoInfantry_Config_Init()	//充电、裁判系统、功率控制初始化
{
	SourceManage.SourceManage_Init();
	Referee.Init(&huart6, Get_RefereeTime);	
	
	PowerCtrl.Load_motorLimitController(MotorLimitController);
	PowerCtrl.Load_capChargeController(CapChargeController);
}

float MotorLimitController(const float current, const float target)
{
	powerCtrl.Target = target;
	powerCtrl.Current = current;
	powerCtrl.Adjust();
	return powerCtrl.Out;		
}

float CapChargeController(const float current, const float target)
{
	capCtrl.Target = target;
	capCtrl.Current = current;
	capCtrl.Adjust();
	return capCtrl.Out;		
}

void Chassis_Init()		//pid参数初始化
{
	auto_infantry.movingMotor[0].SetPIDParam(20,0,0,0,16000);
	auto_infantry.movingMotor[1].SetPIDParam(20,0,0,0,16000);
	auto_infantry.movingMotor[2].SetPIDParam(20,0,0,0,16000);
	auto_infantry.movingMotor[3].SetPIDParam(20,0,0,0,16000);
	
	capCtrl.SetPIDParam(1.0f, 0, 0, 200, 1000); //5.0f
	powerCtrl.SetPIDParam(70.0f,800.0f,0,12000,16000);
}