#ifndef CHASSIS_AGV_H
#define CHASSIS_AGV_H

#include "FreeRTOS.h"
#include "PID.h"
#include <main.h>
#include "motor.h"
#include "math.h"
#include "referee.h"
#define angleCtrl 0
#define speedCtrl 1


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/*pitchyaw_test below*/
extern Motor_GM6020 SteerMotor[4];
extern Motor_C620 Wheel[4];


/*pitchyaw_test above*/
extern referee_Classdef Referee;



class Chassis_AGV
{
	public:
		Chassis_AGV();//初始化
		void calculateRoundCnt(Motor_GM6020*, int8_t, int8_t, uint8_t);//更新舵轮圈数
		void calculateTargetRoundCnt(int8_t motorNum , int8_t status , uint8_t opposite);
		float getSteeringAngle(int8_t);
		void writeMovingTarget(uint8_t can_rx_data[]);
		void writeResetMessage(uint8_t*);																			//获取重启标志
		void writeMovingLimitScale(uint8_t can_rx_data[]);
		float getMovingSpeedTarget(int8_t);
		float getSteeringAngleTarget(int8_t);
		void AGVTargetTransmit();
		myPID movingMotor[4];
//		void streeingContralCalculate(float*, float*);
		void movingContralCalculate(float*);
	  
		void sendMassageClear(uint8_t*);//can发送数据清零
		void sendMassageWrite(uint8_t*, int16_t, int16_t, int16_t, int16_t);//can发送数据填充
		
		void streeingContralCalculate(Motor_GM6020* stmotor, int8_t motor_num);
		
		
		void speedChange();
		void motor_6020_control();
		void AGVControl();
		float abs(float);
		//板间通信获取的变量
		uint8_t write_state1[8];//can包第7位(receive[6])得到的标志位
		uint8_t write_state2[8];//can包第8位(receive[7])得到的标志位
		uint8_t state1,state2,state_all;//can包数据总大小
		float receiveMovingTarget[3];
		float movingTarget[3];
		
		//舵控制
		myPID steeringMotor[4][2];//轮速pid
		float steeringAngleTarget[4];
		float lastSteeringAnglearget[4];
		float steeringAngleOffset[4];
		float steeringAngle[4];
		int32_t motor_count[4];
		int32_t motor_target_count[4];
		int32_t turnFlag[4];
		uint8_t ctrl_6020_message[8];		//can发送舵数据存储
		uint8_t ctrl_duoji[1];
		float pi;
		uint8_t resetFlag;
		/*待添加私有成员*/
};

extern Chassis_AGV auto_infantry;

#endif

