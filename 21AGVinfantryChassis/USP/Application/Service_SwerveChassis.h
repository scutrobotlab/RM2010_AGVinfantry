#ifndef _SWERVE_CHASSIS_H
#define _SWERVE_CHASSIS_H

#include "System_DataPool.h"

extern TaskHandle_t TestSwerveChassis_Handle;
extern CChassis  AlphaTest;
void Task_SwerveChassis(void *arg);
uint32_t DR16_call_back(uint8_t *buf, uint16_t len);
#endif

