/**
  ******************************************************************************
  * @file   Service_Devices.h
  * @brief  Devices service running file.
  ******************************************************************************
  * @note
  *  - Before running your devices, just do what you want ~ !
  *  - More devices or using other classification is decided by yourself ~ !
 */
#ifndef  _SERVICE_DEVICES_H_
#define  _SERVICE_DEVICES_H_
/* Includes ------------------------------------------------------------------*/
#include "System_DataPool.h"
#include "System_Config.h"
#include "Infantry_Funtion.h"
#include "Infantry_Config.h"
#include "shoot.h"
#include "pitch_yaw.h"
#include "chassis.h"
#include "Motor.h"
#include "task.h"
#include "PCvision.h"
#include "Indicator.h"
/* Private macros ------------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
//extern  Motor_C620 FriMotor[2];
/*extern C_Shoot Shoot;
extern C_Chassis Chassis;
extern C_Pitch_Yaw Pitch_Yaw;*/

extern float yaw_timestamp[21];

#ifdef  __cplusplus

#endif

#ifdef  __cplusplus
extern "C"{
#endif
extern TaskHandle_t DeviceActuators_Handle;
extern TaskHandle_t DeviceDR16_Handle;
extern TaskHandle_t DeviceSensors_Handle;
	
//extern C_Chassis Chassis;
/* Exported function declarations --------------------------------------------*/
void Service_Devices_Init(void);
  
#ifdef  __cplusplus
}
#endif

#endif  

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/

