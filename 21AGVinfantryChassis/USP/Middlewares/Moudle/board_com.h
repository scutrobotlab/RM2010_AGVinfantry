#ifndef BOARD_COM_H
#define BOARD_COM_H

#include "FreeRTOS.h"
#include "PID.h"
#include <main.h>
#include "drv_can.h"
#include "referee.h"
//#include "Infantry_Config.h"

extern CAN_HandleTypeDef hcan1;
extern referee_Classdef Referee;

class C_BoardCom
{
	public:
		void Update(
			uint8_t sourcePowerMax,	
			uint8_t shooterSpeedLimit,
			uint16_t coolingRate,
			uint16_t coolingLimit,
			uint16_t shooterHeat,
			float 		bullet_speed);
		void Send();
		uint8_t data[8];
		uint8_t dataFloat[8];
		
			
};



#endif