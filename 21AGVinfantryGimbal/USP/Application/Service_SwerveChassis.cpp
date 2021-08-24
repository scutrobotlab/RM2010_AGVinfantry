#include "Service_SwerveChassis.h"
#include "Drivers/Devices/motor.h"


int32_t* speed_controller(const int16_t* current, const int16_t* target);
float debug_w = 2.0f;
float debug_term1,debug_term2;
Motor_GM6020 Test(1);



int32_t wheel_Out[4];
int32_t* speed_controller(const int16_t* current, const int16_t* target)
{
  
  return wheel_Out;
}

uint32_t DR16_call_back(uint8_t *buf, uint16_t len)
{
  DR16.DataCapture((DR16_DataPack_Typedef*)buf);
  
  return 0;
}


