/**
  **********************************************************************************
  * @file   : Service_Debug.h
  * @brief  : Debug support file.This file provides access ports to debug.
  **********************************************************************************
  *  
**/
#ifndef  _SERVICE_DEBUG_H_
#define  _SERVICE_DEBUG_H_
/* Includes ------------------------------------------------------------------*/
#include "System_DataPool.h"
#ifdef  __cplusplus
extern "C"{
#endif
/* Private macros ------------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern TaskHandle_t Debug_Handle;

/* Exported function declarations --------------------------------------------*/
void Service_Debug_Init(void);

#ifdef  __cplusplus
}
#endif
#endif 
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
