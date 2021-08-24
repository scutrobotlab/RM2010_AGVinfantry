/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    Attitude_Processing.h
  * @author  YDX 2244907035@qq.com
  * @brief   Code for attitude processing using BMX055.
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
#ifndef _BMX055_PROCESSING_H_
#define _BMX055_PROCESSING_H_

#ifdef __cplusplus
extern "C" {
#endif
	
/* Includes ------------------------------------------------------------------*/ 	
#include <math.h>                         /* ffabs */	
#include "BMX055_Config.h"
	
/* Private macros ------------------------------------------------------------*/   
/* Private type --------------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
void Gyro_Offset_Init(void);
   	
/* Exported function declarations --------------------------------------------*/ 
void BMX055_Solve_Data(void);	

#ifdef __cplusplus
}
#endif

#endif
/************************ COPYRIGHT SCUT-ROBOTLAB *****END OF FILE*************/
