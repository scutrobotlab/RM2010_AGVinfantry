#ifndef AUTOINFANTRYCONFIG_H
#define AUTOINFANTRYCONFIG_H


#include "System_Config.h"
//#include "Service_Communication.h"
#include "power_ctrl.h"
#include "Source_manage.h"
#include "referee.h"
#include "AGV_chassis.h"
extern referee_Classdef Referee;
extern C_SourceManage_Classdef SourceManage;
extern PowerCtrl_ClassDef PowerCtrl;
struct S_Source_Typedef{
	int motor_power_max;/*<! ?????? */
	int remain_energy_max; 	/*<! ????? */
	int source_power_max;/*<! ??????? */
	int source_power; /*<! ???????? */
	int	motor_power; /*<! ???????? */
	int remain_energy;/*<! ??????? */
};
void AutoInfantry_Config_Init(void);
void Chassis_Init();
float MotorLimitController(const float current, const float target);
float CapChargeController(const float current, const float target);



//test below
extern float speed_test_error[3];
//test above


#endif
