//前往"Chassis_AGV.h"了解更多
#include "Chassis_AGV.h"




Motor_GM6020 SteerMotor[4] = {Motor_GM6020(1),Motor_GM6020(2),Motor_GM6020(3),Motor_GM6020(4)};
Motor_C620 Wheel[4] = {Motor_C620(1),Motor_C620(2),Motor_C620(3),Motor_C620(4)};

float Radius = 0.212;	//轮中心距






Chassis_AGV::Chassis_AGV()//修改初始化参数
{
	steeringAngleTarget[0] = 0;
	steeringAngleTarget[1] = 0;
	steeringAngleTarget[2] = 0;
	steeringAngleTarget[3] = 0;
	
	
///*****************************************/
///*****************************************/
////有悬挂舵轮舵校准数据
//	steeringAngleOffset[0] = 92;//92
//	steeringAngleOffset[1] = 31.29;//31.29
//	steeringAngleOffset[2] = -89;//-89
//	steeringAngleOffset[3] = 91.45;//91.45
//	
//	steeringAngle[0] = 94.48;
//	steeringAngle[1] = 31.29;
//	steeringAngle[2] = -86.66;
//	steeringAngle[3] = 91.45;
///*****************************************/
///*****************************************/
	
	
/*****************************************/
/*****************************************/
//无悬挂舵轮舵校准数据
	steeringAngleOffset[0] = 17.09;
	steeringAngleOffset[1] = 46.76;
	steeringAngleOffset[2] = 90-15.42;
	steeringAngleOffset[3] = 90-44.82;
	
	steeringAngle[0] = 17.09;
	steeringAngle[1] = 46.76;
	steeringAngle[2] = 90-15.42;
	steeringAngle[3] = 90-44.82;
/*****************************************/
/*****************************************/
	
	
	
	
	
	motor_count[0] = 0;
	motor_count[1] = 0;
	motor_count[2] = 0;
	motor_count[3] = 0;
	
	motor_target_count[0] = 0;
	motor_target_count[1] = 0;
	motor_target_count[2] = 0;
	motor_target_count[3] = 0;
	
	turnFlag[0] = 0;
	turnFlag[1] = 0;
	turnFlag[2] = 0;
	turnFlag[3] = 0;
	
	movingTarget[0] = 0;
	movingTarget[1] = 0;
	movingTarget[2] = 0;
	
	write_state1[0] = false;
	write_state1[1] = false;
	write_state1[2] = false;
	write_state1[3] = false;
	write_state1[4] = false;
	write_state1[5] = false;
	write_state1[6] = false;
	write_state1[7] = false;
	
	write_state2[0] = false;
	write_state2[1] = false;
	write_state2[2] = false;
	write_state2[3] = false;
	write_state2[4] = false;
	write_state2[5] = false;
	write_state2[6] = false;
	write_state2[7] = false;
	
	
	pi = 3.14159265;
}



void Chassis_AGV::AGVControl()
{
	
	
	speedChange();	//三轴速度解算舵角度
	
	//计算带圈数的角度目标值
	calculateTargetRoundCnt(0,1,0);
	calculateTargetRoundCnt(1,1,0);
	calculateTargetRoundCnt(2,1,0);
	calculateTargetRoundCnt(3,1,0);
	//计算带圈数的角度真实值
	calculateRoundCnt(SteerMotor + 0,0,1,0);
	calculateRoundCnt(SteerMotor + 1,1,1,0);
	calculateRoundCnt(SteerMotor + 2,2,1,0);
	calculateRoundCnt(SteerMotor + 3,3,1,0);
	//控制角度
	streeingContralCalculate(SteerMotor + 0,0);
	streeingContralCalculate(SteerMotor + 1,1);
	streeingContralCalculate(SteerMotor + 2,2);
	streeingContralCalculate(SteerMotor + 3,3);
	//控制信息写入
	sendMassageWrite(
	ctrl_6020_message,
	(int16_t)steeringMotor[0][1].Out,
	(int16_t)steeringMotor[1][1].Out,
	(int16_t)steeringMotor[2][1].Out,
	(int16_t)steeringMotor[3][1].Out);
	
	CANx_SendData(&hcan2,0x1ff,ctrl_6020_message,8);
}





//计算带圈数的角度真实值
void Chassis_AGV::calculateRoundCnt(Motor_GM6020* motor_steering , int8_t motorNum , int8_t status , uint8_t opposite)
{
	float now_encoder = motor_steering->getencoder()/8192*360;
	static float last_encoder[4] = {0};
	if(now_encoder - last_encoder[motorNum] > 180) motor_count[motorNum] --;
	else if(now_encoder - last_encoder[motorNum] < -180) motor_count[motorNum] ++;
	last_encoder[motorNum] = now_encoder;
	if(status == 0)motor_count[motorNum] = 0;//圈数重置
	if(opposite == 0)
	{
		steeringAngle[motorNum] = now_encoder + motor_count[motorNum]*360;
	}
	else if(opposite == 1)
	{
		steeringAngle[motorNum] = -now_encoder - motor_count[motorNum]*360;
	}
}


//计算带圈数的角度目标值
void Chassis_AGV::calculateTargetRoundCnt(int8_t motorNum , int8_t status , uint8_t opposite)
{
	float now_target = steeringAngleTarget[motorNum];
	static float last_target[4] = {steeringAngleOffset[0], steeringAngleOffset[1], steeringAngleOffset[2], steeringAngleOffset[3]};
	static float lastAngleTarget[4] = {steeringAngleOffset[0], steeringAngleOffset[1], steeringAngleOffset[2], steeringAngleOffset[3]};
	if(now_target - last_target[motorNum] > 180) motor_target_count[motorNum] --;
	else if(now_target - last_target[motorNum]  < -180) motor_target_count[motorNum] ++;
	last_target[motorNum] = now_target;
	if(status == 0)motor_target_count[motorNum] = 0;//圈数重置
	if(opposite == 0)
	{

		steeringAngleTarget[motorNum] = now_target + motor_target_count[motorNum]*360 + steeringAngleOffset[motorNum] + turnFlag[motorNum] * 180;
	}
	else if(opposite == 1)
	{
		steeringAngleTarget[motorNum] = -now_target - motor_target_count[motorNum]*360 - steeringAngleOffset[motorNum] - turnFlag[motorNum] * 180;
	}
	//
	lastAngleTarget[motorNum] = steeringAngleTarget[motorNum];
}


//舵pid输出计算
void Chassis_AGV::streeingContralCalculate(Motor_GM6020* stmotor, int8_t motor_num)
{
	steeringMotor[motor_num][0].Target = steeringAngleTarget[motor_num];
	steeringMotor[motor_num][0].Current = steeringAngle[motor_num];
	steeringMotor[motor_num][0].Adjust();
	steeringMotor[motor_num][1].Target = steeringMotor[motor_num][0].Out;
	steeringMotor[motor_num][1].Current = stmotor->getSpeed();
	steeringMotor[motor_num][1].Adjust();
}




void Chassis_AGV::writeMovingTarget(uint8_t can_rx_data[])
{
	
	movingTarget[0] = (int16_t)((can_rx_data[0]<<8)+can_rx_data[1]);//前进速度
	movingTarget[1] = (int16_t)((can_rx_data[2]<<8)+can_rx_data[3]);//向右平移速度
	movingTarget[2] = (int16_t)((can_rx_data[4]<<8)+can_rx_data[5]);//旋转速度
	
	write_state1[0] = 1&(can_rx_data[6]>>7);
	write_state1[1] = 1&(can_rx_data[6]>>6);
	write_state1[2] = 1&(can_rx_data[6]>>5);
	write_state1[3] = 1&(can_rx_data[6]>>4);
	write_state1[4] = 1&(can_rx_data[6]>>3);
	write_state1[5] = 1&(can_rx_data[6]>>2);
	write_state1[6] = 1&(can_rx_data[6]>>1);
	write_state1[7] = 1&(can_rx_data[6]>>0);
	
	state1 = can_rx_data[6];

	write_state2[0] = 1&(can_rx_data[7]>>7);
	write_state2[1] = 1&(can_rx_data[7]>>6);
	write_state2[2] = 1&(can_rx_data[7]>>5);
	write_state2[3] = 1&(can_rx_data[7]>>4);
	write_state2[4] = 1&(can_rx_data[7]>>3);
	write_state2[5] = 1&(can_rx_data[7]>>2);
	write_state2[6] = 1&(can_rx_data[7]>>1);
	write_state2[7] = 1&(can_rx_data[7]>>0);
	
	state2 = can_rx_data[7];
	
	state_all = (can_rx_data[6] << 8) + can_rx_data[7];
}




void Chassis_AGV::writeResetMessage(uint8_t can_rx_data[])
{
	resetFlag = 1;
	receiveMovingTarget[0] = 0;//前进速度
	receiveMovingTarget[1] = 0;//向右平移速度
	receiveMovingTarget[2] = 0;//旋转速度
}




//解算函数
void Chassis_AGV::speedChange()
{
	float theta = atan(1.0/1.0);
	
	if((movingTarget[0] == 0)&&(movingTarget[1] == 0)&&(movingTarget[2] == 0))
	{
		steeringAngleTarget[0] = lastSteeringAnglearget[0];
																																																																																			
		steeringAngleTarget[1] = lastSteeringAnglearget[1];    
																																																																																	
		steeringAngleTarget[2] = lastSteeringAnglearget[2];          
																																																																																		
		steeringAngleTarget[3] = lastSteeringAnglearget[3]; 
	}
	else
	{
		steeringAngleTarget[0] = 
			atan2(movingTarget[1] - movingTarget[2]*(Radius*sin(theta)),
						movingTarget[0] - movingTarget[2]*Radius*cos(theta));//舵角，弧度制         
																																																																																			
		steeringAngleTarget[1] = 
			atan2(movingTarget[1] + movingTarget[2]*(Radius*sin(theta)),
						movingTarget[0] - movingTarget[2]*Radius*cos(theta));//舵角，弧度制          
																																																																																	
		steeringAngleTarget[2] = 
			atan2(movingTarget[1] + movingTarget[2]*(Radius*sin(theta)),
						movingTarget[0] + movingTarget[2]*Radius*cos(theta));//舵角，弧度制          
																																																																																		
		steeringAngleTarget[3] = 
			atan2(movingTarget[1] - movingTarget[2]*(Radius*sin(theta)),
						movingTarget[0] + movingTarget[2]*Radius*cos(theta));//舵角，弧度制 
	}
	

	lastSteeringAnglearget[0] = steeringAngleTarget[0];
	lastSteeringAnglearget[1] = steeringAngleTarget[1];
	lastSteeringAnglearget[2] = steeringAngleTarget[2];
	lastSteeringAnglearget[3] = steeringAngleTarget[3];
	
	
	steeringAngleTarget[0] = steeringAngleTarget[0] * 180 / pi;
	steeringAngleTarget[1] = steeringAngleTarget[1] * 180 / pi;
	steeringAngleTarget[2] = steeringAngleTarget[2] * 180 / pi;
	steeringAngleTarget[3] = steeringAngleTarget[3] * 180 / pi;
	
}






//发送数据清零
void Chassis_AGV::sendMassageClear(uint8_t* sendMessage)
{
	*sendMessage = 0;
	*(sendMessage + 1) = 0;
	*(sendMessage + 2) = 0;
	*(sendMessage + 3) = 0;
	*(sendMessage + 4) = 0;
	*(sendMessage + 5) = 0;
	*(sendMessage + 6) = 0;
	*(sendMessage + 7) = 0;
}



//发送数据填充
void Chassis_AGV::sendMassageWrite(
			uint8_t* sendMessage,
			int16_t current_control1,
			int16_t current_control2,
			int16_t current_control3,
			int16_t current_control4)
{
	*sendMessage = (int8_t)(current_control1 >> 8);
	*(sendMessage + 1) = current_control1;
	*(sendMessage + 2) = (int8_t)(current_control2 >> 8);;
	*(sendMessage + 3) = current_control2;
	*(sendMessage + 4) = (int8_t)(current_control3 >> 8);;
	*(sendMessage + 5) = current_control3;
	*(sendMessage + 6) = (int8_t)(current_control4 >> 8);;
	*(sendMessage + 7) = current_control4;
}




//
float Chassis_AGV::abs(float a)
{
	return a > 0 ? a : -a;
}