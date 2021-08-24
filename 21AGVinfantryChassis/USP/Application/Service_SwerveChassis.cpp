#include "Service_SwerveChassis.h"
#include "Drivers/Devices/motor.h"
CChassis  AlphaTest(1, 1, 1, 16000 ,500 ,0 , 0.01f);
TaskHandle_t TestSwerveChassis_Handle;
int32_t* speed_controller(const int16_t* current, const int16_t* target);
float debug_w = 2.0f;
float debug_term1,debug_term2;
Motor_GM6020 Test(1);

void Task_SwerveChassis(void *arg)
{
  /* Cache for Task */
  float now_time;
  /* Pre-Load for task */
  AlphaTest.Load_SpeedController(speed_controller);
  /* Infinite loop */
  TickType_t xLastWakeTime_t;
  xLastWakeTime_t = xTaskGetTickCount();
  for(;;)
  {
    /*Generate input */
    now_time = HAL_GetTick()/1000.0f;
    AlphaTest.Set_Target(1.0f*cosf(debug_w*now_time),1.0f*sinf(debug_w*now_time),0.8f);
    //AlphaTest.Set_Target(DR16.Get_LX_Norm(),DR16.Get_LY_Norm(),DR16.Get_RX_Norm());
    AlphaTest.Chassis_Control();

    /* Pass control to the next task */
    vTaskDelayUntil(&xLastWakeTime_t,1);
  }
}

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


