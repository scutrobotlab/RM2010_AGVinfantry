/**
  ******************************************************************************
  * @file   Service_Devices.cpp
  * @brief  Devices service running file.
  ******************************************************************************
  * @note
  *  - Before running your devices, just do what you want ~ !
  *  - More devices or using other classification is decided by yourself ~ !
  ===============================================================================
                                    Task List
  ===============================================================================
  * <table>
  * <tr><th>Task Name     <th>Priority          <th>Frequency/Hz    <th>Stack/Byte
  * <tr><td>              <td>                  <td>                <td>    
  * </table>
  *
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "Service_Devices.h"
#include "pitchyaw.h"
#include "Chassis_AGV.h"
#include "SourceManage.h"
#include "referee.h"

SourceManage_ClassDef ChassisSource;
referee_Classdef Referee;


/* Private define ------------------------------------------------------------*/
TaskHandle_t DeviceActuators_Handle;
TaskHandle_t DeviceDR16_Handle;
TaskHandle_t DeviceSensors_Handle;
TaskHandle_t DeviceControl_Handle;
TaskHandle_t RecvReferee_Handle;

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern DAC_HandleTypeDef hdac;
extern DMA_HandleTypeDef hdma_dac1;

/* Private variables ---------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
void Device_Actuators(void *arg);
void Device_Sensors(void *arg);
void Device_DR16(void *arg);
void Device_Control(void *arg);
void Recv_Referee(void *arg);



/* Exported devices ----------------------------------------------------------*/
/* Motor & ESC & Other actuators*/
//Motor_AK80_9  Test_Motor(1, 0, 0);
/* Remote control */

/* IMU & NUC & Other sensors */

/* Other boards */






//底盘
Chassis_AGV auto_infantry;




/* Function prototypes -------------------------------------------------------*/
/**
* @brief  Initialization of device management service
* @param  None.
* @return None.
*/
void Service_Devices_Init(void)
{
//  xTaskCreate(Device_DR16,      "Dev.DR16"     , Tiny_Stack_Size,    NULL, PriorityHigh,        &DeviceDR16_Handle);
//  xTaskCreate(Device_Sensors,   "Dev.Sensors"  , Large_Stack_Size,    NULL, PriorityHigh,        &DeviceSensors_Handle);
	xTaskCreate(Device_Control,   "Dev.Control"  , 	Large_Stack_Size,    NULL, PrioritySuperHigh,        &DeviceControl_Handle);
	xTaskCreate(Recv_Referee,    "Rx.Referee"   , Large_Stack_Size,    NULL, PriorityRealtime,  &RecvReferee_Handle);
}










void Recv_Referee(void *arg)
{
  /* Cache for Task */
	
  /* Pre-Load for task */
//	static USART_COB* referee_pack;
  /* Infinite loop */
  TickType_t xLastWakeTime_t;
  xLastWakeTime_t = xTaskGetTickCount();
  for(;;)
  {
		vTaskDelayUntil(&xLastWakeTime_t,1);
//		Sent_Contorl(&huart1);
    /* Read IMU Message */
//		if(xTaskNotifyWait(0x00000000, 0xFFFFFFFF, (uint32_t *) &referee_pack, 0) == pdTRUE)
//		{
//			Referee.unPackDataFromRF((uint8_t*)referee_pack->address, referee_pack->len);
//		}
    /* Exchange NUC Meaasge */
    
    /* Read Other board Message */

    
  /* Pass control to the next task ------------------------------------------*/
    
  }
}

void Device_Control(void *arg)
{
	TickType_t xLastWakeTime_t;
  xLastWakeTime_t = xTaskGetTickCount();
	
	//舵双环参数设置
	auto_infantry.steeringMotor[0][0].SetPIDParam(290, 100, 0, 1000, 2000);
	auto_infantry.steeringMotor[0][0].I_SeparThresh = 20;
	auto_infantry.steeringMotor[0][1].SetPIDParam(10, 0, 0, 0, 20000);
	auto_infantry.steeringMotor[1][0].SetPIDParam(290, 100, 0, 1000, 2000);
	auto_infantry.steeringMotor[1][0].I_SeparThresh = 20;
	auto_infantry.steeringMotor[1][1].SetPIDParam(10, 0, 0, 0, 20000);
	auto_infantry.steeringMotor[2][0].SetPIDParam(290, 100, 0, 1000, 2000);
	auto_infantry.steeringMotor[2][0].I_SeparThresh = 20;
	auto_infantry.steeringMotor[2][1].SetPIDParam(10, 0, 0, 0, 20000);
	auto_infantry.steeringMotor[3][0].SetPIDParam(290, 100, 0, 1000, 2000);
	auto_infantry.steeringMotor[3][0].I_SeparThresh = 20;
	auto_infantry.steeringMotor[3][1].SetPIDParam(10, 0, 0, 0, 20000);

	float pi = 3.1415926;
		for(;;)
	{
		vTaskDelayUntil(&xLastWakeTime_t, 1);
		auto_infantry.AGVControl();		//舵控制函数
	}
}

/* User Code End Here ---------------------------------*/

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
