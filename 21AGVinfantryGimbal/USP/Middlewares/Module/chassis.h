#ifndef CHASSIS_H
#define CHASSIS_H

//#include "Infantry_Funtion.h"
//#include "main.h"
#include "FreeRTOS.h"
#include "PID.h"
#include <main.h>
#include "drv_can.h"
#include "motor.h"
#include "arm_math.h"
#include "math.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern Motor_GM6020 pitchyawMotor[2];

enum E_ChassisState
{	
	LOST_C				= 0,
	NORMAL_C			= 1,
	UNLIMITED_C		=	2,
	
};

enum E_MoveMode
{
	RUN_C				= 0,		
	ROTATE_C			= 1,
};

enum E_CtrlMode
{
	REMOTE_CTRL_C				= 0,		
	KEY_BOARD_C			= 1,
};

class C_Chassis
{
	public:
		void Control(E_ChassisState chassis_stste,
			           E_MoveMode move_mode,
		             E_CtrlMode ctrl_mode,
		             float y_data,
								 float x_data,
								 float r_data,
								 float y_back=0 ,
								 float x_back=0 );  //使用遥控器控制只需要一组x y参数，键鼠需要两组
		void Reset();
		void maxSpeedUpdate();
		void msgSend();
		void setMaxSpeed(int16_t max_Y, int16_t max_X, int16_t max_R);
		void rSpeedLimit();
		void targetUpdate();
		void angleCnt();
		void pid_init(float kp,float ki,float kd, float ki_max,float out_max);
		int16_t speed_Y,speed_X,speed_R;
		int16_t speed_Y_last = 0,speed_X_last = 0,speed_R_last = 0;
		int16_t	maxSpeed_Y,maxSpeed_X,maxSpeed_R,sourcePowerMax;
		int16_t	velocity[4];        //键盘速度临时数据
		int16_t rotateCnt;          //记录云台旋转圈数
		E_ChassisState chassisMode;
		E_MoveMode	moveMode;
		E_CtrlMode  ctrlMode;
		bool rewrite_UI;
		bool magazinemode;
		bool isLandMode;					
		myPID chassisYawAngle;
		uint8_t rad = 180;
		uint8_t chassis_states = 0; 
		float temp, pi=3.1415926;
		float rec_Y, rec_X, rec_R, rec_Y_Key, rec_X_Key;
		uint8_t snipermode = 0;
		uint8_t finalMode = 0;
		uint8_t fuckMode = 0;
		uint8_t GGMode = 0;
		int8_t rodirection = 1;
		uint8_t maxSpin = 0;
};

extern C_Chassis Chassis;

#endif