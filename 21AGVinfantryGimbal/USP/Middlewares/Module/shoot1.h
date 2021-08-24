#ifndef SHOOT_H
#define SHOOT_H

#include "FreeRTOS.h"
#include "PID.h"
#include <main.h>
#include "motor.h"
#include "pitch_yaw.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern TIM_HandleTypeDef htim2;
extern C_Pitch_Yaw Pitch_Yaw;
extern myPID leftFriSpeed,rightFriSpeed,turnplateSpeede;

enum E_FireFlag
{
	STOP_F = 0,
	FIRE_F = 1
};

enum E_ShootMode
{
	RUN = 0,
	ONCE = 1,
};

enum E_ShootCtrlMode
{
	REMOTE_CTRL_S = 0,
	KEY_BOARD_S = 1,
	MINI_CTRL_S = 2,
};


enum E_FriWheelState
{
	CLOSE_F = 0,
	OPEN_F = 1,
};

enum E_LaserState
{
	CLOSE_L = 0,
	OPEN_L = 1,
};

enum E_BulletBayState
{
	CLOSE_B = 0,
	OPEN_B = 1,
};

enum E_ShootPidType
{
	LEFT_FRI_SPEED 	= 0,
	RIGHT_FRI_SPEED = 1,
	TURNPLATE_SPEED = 2,
	TURNPLATE_ANGLE	= 3,
};



class C_Shoot
{
	public:
		
		void Control(E_ShootMode 	    shoot_mode,
			           E_ShootCtrlMode 	ctrl_mode,
								 E_FriWheelState 	fri_wheel_stste,
		             E_LaserState    	laser_state,
		             E_BulletBayState bullet_bay_state,
		             E_FireFlag				fire_flag,
								 uint8_t 					max_speed,
								 uint16_t 				cooling_rate,
								 uint16_t 				cooling_limit,
								 uint16_t					shooter_heat);
		void laserCtrl();							//¼¤¹â
		void friWheelCtrl();
		void bulletBayCtrl();         //µ¯²Õ
		void shootCtrl();
		void heatCalc();
		void Reset();
		float abs(float x);
		void pid_init(E_ShootPidType,float kp,float ki,float kd, float ki_max,float out_max);
		myPID leftFriSpeed,rightFriSpeed,turnPlateSpeed,turnPlateAngle;
	  float friSpeed_15 = 3920  ,friSpeed_18 = 4360,friSpeed_30 = 7300;		//15_3960			6500	4300/17	//Ä¦²ÁÂÖ×ªË
		float bulletSpeed ,lastBulletSpeed,heat,heatRf;
		int16_t cnt,delayCnt,friWheelDelay,bulletBayDelay; 
		int16_t period = 100;				//Éä»÷ÖÜÆÚ
		int16_t coolingRate = 100,heatLimit = 120,shooterHeat,index;
		uint8_t maxSpeed = 15;						//×î¸ß³õËÙ¶È
	
		E_ShootMode 			shootMode;
		E_ShootCtrlMode 	ctrlMode;
		E_FriWheelState		friWheelState;
		E_LaserState 			laserState;
		E_BulletBayState 	bulletBayState;
		E_FireFlag 				fireFlag,fireFlag2;
		
};



#endif