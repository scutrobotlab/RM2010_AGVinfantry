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
#include "Service_Devices.h"
//#include "Infantry_Congif.h"
/* Private define ------------------------------------------------------------*/
TaskHandle_t DeviceDR16_Handle;
TaskHandle_t DeviceSensors_Handle;
TaskHandle_t DeviceTest_Handle;
TaskHandle_t TaskVision_Handle;

C_Shoot Shoot;
extern C_Chassis Chassis;
C_Pitch_Yaw Pitch_Yaw;
Indicator_Classdef state_light(&htim3,&hdma_tim3_ch1_trig,TIM_CHANNEL_1);

float time2;

float yaw_timestamp[21] = {0};

/* Private variables ---------------------------------------------------------*/

LinkageStatus_Typedef DR16STATUS;
float IMUa[10],IMUSpeedReal,IMUSpeedError;
float yaw_angle,pitch_angle;
float IMUp[10],IMUSpeedRealPitch,IMUSpeedErrorPitch;
float IMUPitchOffset;

E_pitchYawMode pitchYawMode;
E_ChassisState chassisMode;
E_MoveMode	moveMode;
extern S_Infantry Infantry;
/* Private function declarations ---------------------------------------------*/
void Device_Sensors(void *arg);
void Device_DR16(void *arg);
void Device_Test(void *arg);
void Task_Vision(void *arg);

/* Function prototypes -------------------------------------------------------*/
/**
* @brief  Initialization of device management service
* @param  None.
* @return None.
*/
void Service_Devices_Init(void)
{
  //xTaskCreate(Device_Actuators, "Dev.Actuator" , Tiny_Stack_Size,    NULL, PrioritySuperHigh,   &DeviceActuators_Handle);
  xTaskCreate(Device_DR16,      "Dev.DR16"     , Tiny_Stack_Size,    NULL, PriorityHigh,        &DeviceDR16_Handle);
  xTaskCreate(Device_Sensors,   "Dev.Sensors"  , Large_Stack_Size,    NULL, PriorityHigh,        &DeviceSensors_Handle);
	xTaskCreate(Device_Test,   "Dev.Test"  , Huge_Stack_Size,    NULL, PriorityHigh,        &DeviceTest_Handle);
	xTaskCreate(Task_Vision,   "Task.Vision"  , Huge_Stack_Size,    NULL, PrioritySuperHigh,        &TaskVision_Handle);
}

void Device_Test(void *arg)
{
  /* Cache for Task */ 

  /* Pre-Load for task */

  /* Infinite loop */
  static TickType_t xLastWakeTime_t = xTaskGetTickCount();
  static LinkageStatus_Typedef dr16_status,last_dr16_status;
	static uint8_t i=0,j=0,k=0;
	static int16_t count = 0;
	
	state_light.Change_Singal_RGB(LED_1, 255, 0, 0, 255);
	state_light.Change_Singal_RGB(LED_2, 255, 0, 0, 255);
	state_light.Change_Singal_RGB(LED_3, 255, 0, 0, 255);
	state_light.Change_Singal_RGB(LED_4, 255, 0, 0, 255);
	state_light.Change_Singal_RGB(LED_5, 255, 0, 0, 255);
	state_light.Change_Singal_RGB(LED_6, 255, 0, 0, 255);
	state_light.Update();
	
	HAL_Delay(50);
	
	for(;;)
  {
		vTaskDelayUntil(&xLastWakeTime_t,1);
		
		DR16.Check_Link(xTaskGetTickCount());
		dr16_status = DR16.GetStatus();
		
		if((dr16_status == Connection_Lost))//遥控断连后断开控制
		{
			if(i==10)
			{
				i=0;
			}
			IMUa[i++] =  MPUData.gz;		
			IMUSpeedError = 0;
			for(uint8_t m=0;m<10;m++)
			{
				IMUSpeedError += MPUData.gz*0.1;
			}
			IMUSpeedReal = MPUData.gz - IMUSpeedError;
			
			if(j==10)
			{
				j=0;
			}
			IMUp[j++] =  MPUData.gy;
			IMUSpeedErrorPitch = 0;
			for(uint8_t n=0;n<10;n++)
			{
				IMUSpeedErrorPitch += MPUData.gy*0.1;
			}
			IMUSpeedRealPitch = MPUData.gy - IMUSpeedErrorPitch;
				
			Pitch_Yaw.yawAngle.Current = MPUData.yaw;
			Pitch_Yaw.yawAngle.Target = MPUData.yaw;
			//Pitch_Yaw.pitchAngle.Current = MPUData.roll;
			//Pitch_Yaw.pitchAngle.Target = MPUData.roll;
			Pitch_Yaw.pitchAngle.Current =pitchyawMotor[0].encoder;
			Pitch_Yaw.pitchAngle.Target =pitchyawMotor[0].encoder;
			if(Pitch_Yaw.pitchAngle.Current < 4096)
			{
				Pitch_Yaw.pitchAngle.Current = Pitch_Yaw.pitchAngle.Current + 8192;
			}
			if(Pitch_Yaw.pitchAngle.Target < 4096)
			{
				Pitch_Yaw.pitchAngle.Target = Pitch_Yaw.pitchAngle.Target + 8192;
			}
			Pitch_Yaw.cnt = 0;
			Pitch_Yaw.totalYawAngle = MPUData.yaw;
			IMUPitchOffset = MPUData.roll;
			Pitch_Yaw.yawSpeed.I_Term = 0;
			Pitch_Yaw.Reset();
			Chassis.Reset();
			Shoot.Reset();
		}
		else		//遥控在线后控制
		{
			time2+=0.1;
			
			IMUSpeedReal = MPUData.gz - IMUSpeedError;
			IMUSpeedRealPitch = MPUData.gy - IMUSpeedErrorPitch;
			
			CheakState();
			DataUpdate();
			
			//云台控制
			Pitch_Yaw.Control(Infantry.pitchYawCtrlMode,
												Infantry.y_pitchYaw,						
												Infantry.x_pitchYaw,
												pitchyawMotor[0].encoder,
												-MPUData.gy,
												MPUData.yaw,
												MPUData.gz);

			//底盘控制
			Chassis.Control(Infantry.chassisMode,
											Infantry.moveMode,
											Infantry.chassisCtrlMode,
											Infantry.y_data,
											Infantry.x_data,
											pitchyawMotor[1].getAngle(),
											Infantry.y_back,
											Infantry.x_back);
			
			//发射控制
			Shoot.Control(Infantry.shootMode,
										Infantry.shootCtrlMode,
										Infantry.friWheelState,
										Infantry.laserState,
										Infantry.bulletBayState,
										Infantry.fireFlag,
										Infantry.bulletMaxSpeed,
										Infantry.coolingRate,
										Infantry.coolingLimit,
										Infantry.shooterHeat);								
		}
		
		//led 更新
//********************************************************************		
//********************************************************************	
//********************************************************************	
//********************************************************************	

		
		if((dr16_status == Connection_Lost))
		{
			state_light.Change_Singal_RGB(LED_1, 255, 0, 0, 255);
		}
		else
		{
			if(Infantry.pitchYawCtrlMode == REMOTE_CTRL_P)
			{
				state_light.Change_Singal_RGB(LED_1, 0, 0, 255, 255);
			}
			else
			{
				state_light.Change_Singal_RGB(LED_1, 0, 255, 0, 255);
			}
		}
		
		if(PCvisionStatus == Connected)
		{
			if(Infantry.mode == 2)
			{
				state_light.Change_Singal_RGB(LED_2, 0, 0, 255, 255);
			}
			else if(Infantry.mode == 3)
			{
				state_light.Change_Singal_RGB(LED_2, 0, 255, 255, 255);
			}
			else if(Infantry.mode == 1)
			{
					state_light.Change_Singal_RGB(LED_2, 255, 255, 255, 255);
			}
			else if(Infantry.mode == 0)
			{
				state_light.Change_Singal_RGB(LED_2, 0, 255, 0, 255);
			}
		}
		else
		{
			if(Infantry.mode == 0)
			{
				state_light.Change_Singal_RGB(LED_2, 255, 255, 0, 255);
			}
			else if(Infantry.mode == 1)
			{
				state_light.Change_Singal_RGB(LED_2, 255, 0, 255, 255);
			}
			else
			{
				state_light.Change_Singal_RGB(LED_2, 255, 0, 0, 255);
			}
			
		}
		if(Infantry.bulletBayState == OPEN_B)
		{
			state_light.Change_Singal_RGB(LED_3, 255, 0, 0, 255);
		}
		else
		{
			state_light.Change_Singal_RGB(LED_3, 0, 255, 0, 255);
		}

		
		switch(Chassis.chassis_states&0x07)
		{
			case 0:state_light.Change_Singal_RGB(LED_4, 0, 0, 0, 255);
				break;
			case 1:state_light.Change_Singal_RGB(LED_4, 255, 0, 0, 255);
				break;
			case 2:state_light.Change_Singal_RGB(LED_4, 255, 255, 0, 255);
				break;
			case 3:state_light.Change_Singal_RGB(LED_4, 0, 255, 0, 255);
				break;
			case 4:state_light.Change_Singal_RGB(LED_4, 255, 255, 255, 255);
				break;
		}
		
		if(Infantry.chassisMode == UNLIMITED_C)
		{
			state_light.Change_Singal_RGB(LED_5, 255, 255, 255, 255);
		}
		else
		{
			state_light.Change_Singal_RGB(LED_5, 0, 255, 0, 255);
		}
		if(Infantry.fuckMode == 1)
		{
			state_light.Change_Singal_RGB(LED_5, 0, 0, 255, 255);
		}
		else
		{
		}
		
		if((Infantry.finalMode == 1)&&(Infantry.maxSpin == 1))
		{
			state_light.Change_Singal_RGB(LED_6, 255, 255, 255, 255);
		}
		else if(Infantry.finalMode == 1)
		{
			state_light.Change_Singal_RGB(LED_6, 0, 255, 255, 255);
		}
		else if(Infantry.maxSpin == 1)
		{
			state_light.Change_Singal_RGB(LED_6, 0, 0, 255, 255);
		}
		else
		{
			state_light.Change_Singal_RGB(LED_6, 0, 255, 0, 255);
		}
		count ++;
		if(count >= 200)
		{
			state_light.Update();
			count = 0;
		}
//********************************************************************		
//********************************************************************	
//********************************************************************	
//********************************************************************	
		
		//DR16.Exce_Click_Fun();
//		Sent_Contorl(&huart6);
	
	
    /* Pass control to the next task */
    
  }
}



//遥控函数
void Device_DR16(void *arg)
{
  /* Cache for Task */
	static DR16_DataPack_Typedef* dr16_pack;
  static TickType_t xLastWakeTime_t = xTaskGetTickCount();
	/* Pre-Load for task */
//	HAL_IWDG_Init(&hiwdg);
  /* Infinite loop */
  for(;;)
  {
		if(xTaskNotifyWait(0x00000000, 0xFFFFFFFF, (uint32_t *) &dr16_pack, 0) == pdTRUE)
		{
			/* Remote control data unpacking */
			DR16.DataCapture(dr16_pack);
		}
		static int count_DOG = 0,dogFlag = 0;
		if(Infantry.GGMode == 1)
		{
			static uint8_t date[1]={0};
			while(CANx_SendData(&hcan2,0x101,date,1) == CAN_SUCCESS)
			{
				count_DOG = 1000;
			}
			__set_FAULTMASK(1);
			NVIC_SystemReset();
			dogFlag = 1;
		}
		else if(Infantry.GGMode == 0)
		{
			dogFlag = 0;
		}
		if(dogFlag == 0)
		{
			count_DOG++;
			if(count_DOG>100)
			{
//				HAL_IWDG_Refresh(&hiwdg);
				count_DOG = 0;
			}
		}
		/* Pass control to the next task */
    vTaskDelayUntil(&xLastWakeTime_t,1);
  }
}




//陀螺仪数据获取
void Device_Sensors(void *arg)
{
  /* Cache for Task */
  
  /* Pre-Load for task */
	static uint8_t i=0;
	
  /* Infinite loop */
  TickType_t xLastWakeTime_t;
  xLastWakeTime_t = xTaskGetTickCount();
  for(;;)
  {
    /* Read IMU Message */
		MPU_Get_Gyroscope(&MPU6050_IIC_PIN, &MPUData.gx, &MPUData.gy, &MPUData.gz);//
		{
			MPUData.gx -= MPUData.gxoffset;    	//自行测算，每块板子的陀螺仪不一样
			MPUData.gy -= MPUData.gyoffset;
			MPUData.gz -= MPUData.gzoffset;
		}
		MPU_Get_Accelerometer(&MPU6050_IIC_PIN, &MPUData.ax, &MPUData.ay, &MPUData.az);//
    mpu_dmp_get_data(&MPUData.roll,&MPUData.pitch,&MPUData.yaw);
		/* Exchange NUC Meaasge */

    /* Read Other board Message */

    
  /* Pass control to the next task ------------------------------------------*/
    vTaskDelayUntil(&xLastWakeTime_t,1);
  }
}




//与视觉通信检查
void Task_Vision(void *arg)
{
  /* Cache for Task */
  static int16_t k = 0;
  /* Pre-Load for task */
	
  /* Infinite loop */
  TickType_t xLastWakeTime_t;
  xLastWakeTime_t = xTaskGetTickCount();
  for(;;)
  {
		SendGimbleStatus(&PackToVisionUnion);
		k++;
		if(k >= 100)
		{
			PC_CheckLink();
			k = 0;
		}
		
  /* Pass control to the next task ------------------------------------------*/
    vTaskDelayUntil(&xLastWakeTime_t,2);
  }
}

/* User Code End Here ---------------------------------*/

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
