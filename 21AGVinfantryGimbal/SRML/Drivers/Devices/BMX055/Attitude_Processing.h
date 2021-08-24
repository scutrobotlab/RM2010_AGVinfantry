/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    Attitude_Processing.h
  * @author  YDX 2244907035@qq.com
  * @brief   Code for BMX055 attitude processing.
  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have many 
  * bugs, update the version Number, write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding 
  * through your new brief.
  *
  * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
  */
#ifndef _ATTITUDE_PROCESSING_H_
#define _ATTITUDE_PROCESSING_H_

#ifdef __cplusplus
extern "C" {
#endif
	
/* Includes ------------------------------------------------------------------*/ 	
#include <math.h>                         /* ffabs */	
#include "Attitude_Calculation.h"

/* Private macros ------------------------------------------------------------*/   
/* Private type --------------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern uint8_t IsGyroOffsetReset;  	
/** 
  @brief 角度 
*/
extern EulerAngleTypedef SystemAttitude;    

/** 
  @brief 角速度 
*/
extern EulerAngleTypedef SystemAttitudeRate;  
/* Exported function declarations --------------------------------------------*/  
void BMX055_solve_data(void);

#ifdef __cplusplus
}
#endif

#endif
/************************ COPYRIGHT SCUT-ROBOTLAB *****END OF FILE*************/
