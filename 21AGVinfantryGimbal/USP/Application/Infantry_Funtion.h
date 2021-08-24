#ifndef INFANTRY_FUNCTION_H
#define INFANTRY_FUNCTION_H

#include "shoot.h"
#include "pitch_yaw.h"
#include "System_Config.h"
#include "chassis.h"
#include "Service_Communication.h"
#include "System_DataPool.h"
#include "PCvision.h"
#include "Infantry_Config.h"
#endif


uint8_t logicJudge(uint8_t last_res,uint8_t now,uint8_t* last);
void IMU_update(void);
void CheakState();
void shootState();
void chassisState();

void DataUpdate();
void pitchYawUpdate();
void chassisUpdate();
float repeatJudge(float now, float *last);
//extern C_Chassis Chassis;
