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
/* Private macros ------------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
#ifdef  __cplusplus

#endif

#ifdef  __cplusplus
extern "C"{
#endif
extern TaskHandle_t DeviceActuators_Handle;
extern TaskHandle_t DeviceDR16_Handle;
extern TaskHandle_t DeviceSensors_Handle;
extern TaskHandle_t DeviceControl_Handle;
extern TaskHandle_t RecvReferee_Handle;
/* Exported function declarations --------------------------------------------*/
void Service_Devices_Init(void);
  
#ifdef  __cplusplus
}
#endif

#endif  

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/

