#ifndef AGV_CHASSIS_H
#define AGV_CHASSIS_H

#include "FreeRTOS.h"
#include "PID.h"
#include <main.h>
#include "motor.h"
#include "math.h"
#include "autoInfantryConfig.h"



extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern Motor_GM6020 SteerMotor[4];
extern Motor_C620 Wheel[4];

class Chassis_AGV
{
	public:
		//函数
		//初始化
		Chassis_AGV(); 
		//设置与控制部分
		void setMaxSpeed(int16_t);                    												//设置最大速度
		void setMaxPower(int16_t);                    												//设置最大功率
		void speedChange();                           												//速度解算
		void speedCalculate();																								//速度处理
	  void AGVControl();                            												//底盘控制
		//功能函数部分
	  float abs(float);                             												//取绝对值
		void writeMovingTarget(uint8_t*);																		  //获取目标速度
		void writeResetMessage(uint8_t*);																			//获取重启标志
		void sendMassageClear(uint8_t*);																			//can发送数据清零
		void sendMassageWrite(uint8_t*, int16_t, int16_t, int16_t, int16_t);	//can发送数据填充
		void movingContralCalculate(float, int8_t);
		void writeRefereemsg(referee_Classdef *, uint8_t *);
//		void checkLink();
		//变量
		//板间通信获取的变量
		uint8_t write_state1[8];//can包第7位(receive[6])得到的标志位
		uint8_t write_state2[8];//can包第8位(receive[7])得到的标志位
		uint8_t sendRefereeMsg[8];
		uint8_t state1,state2,state_all;//can包数据总大小
		float receiveMovingTarget[3];
		float movingTarget[3];
		uint8_t linkState;
		//轮控制
		myPID movingMotor[4];//轮速pid
		float movingSpeedTarget[4];
		float powerSpeedScale;
		uint8_t ctrl_3508_message[8];		//can发送轮数据存储
		uint8_t ctrl_can_message[8];		//can发送舵数据存储
		float pi;
		float w_scale;
		//功率控制
		float motorPowerMax;
		float sourcePowerMax; 
		float maxSpeed[3];
		int motorOutput[4];
		float power_scale;
		uint8_t chassis_cap_states;
		uint8_t resetFlag;
//		/*待添加私有成员*/
};

extern Chassis_AGV auto_infantry;

#endif

