
#include "board_com.h"

void C_BoardCom :: Send()
{
	Update(
		Referee.GameRobotState.classis_power_limit,
		Referee.GameRobotState.shooter_id1_17mm_speed_limit,
		Referee.GameRobotState.shooter_id1_17mm_cooling_rate,
		Referee.GameRobotState.shooter_id1_17mm_cooling_limit,
		Referee.PowerHeatData.shooter_id1_17mm_cooling_heat,
		Referee.ShootData.bullet_speed
	);
	CANx_SendData(&hcan1,0x221,data,8);
	CANx_SendData(&hcan1,0x220,dataFloat,8);
	
}

void C_BoardCom :: Update(
			uint8_t 	source_power_max,	
			uint8_t 	shooter_speed_limit,
			uint16_t 	cooling_rate,
			uint16_t 	cooling_limit,
			uint16_t 	shooter_heat,
			float 		bullet_speed)
{
	uint8_t * temp;
	float wtemp = bullet_speed;
	data[0] = source_power_max;
	data[1] = shooter_speed_limit;
	data[2] = cooling_rate>>8;
	data[3] = cooling_rate;
	data[4] = cooling_limit>>8;
	data[5]	= cooling_limit;
	data[6] = shooter_heat>>8;
	data[7] = shooter_heat;
	temp = (uint8_t*)(&wtemp);
	dataFloat[0] = temp[0];
	dataFloat[1] = temp[1];
	dataFloat[2] = temp[2];
	dataFloat[3] = temp[3];
	
}


