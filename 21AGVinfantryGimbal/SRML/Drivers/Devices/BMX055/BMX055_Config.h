/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    BMX055_Config.h
  * @author  YDX 2244907035@qq.com
  * @brief   Code for BMX055 config.
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
#ifndef _BMX055_CONFIG_H_
#define _BMX055_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif
	
/* Includes ------------------------------------------------------------------*/  
#include "BMX055.h"
#include "BMX055_Processing.h"
	
/* Private macros ------------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
extern IIC_PIN_Typedef BMX055_IIC_PIN;	

/* Exported function declarations --------------------------------------------*/
uint8_t BMX055_Init(GPIO_TypeDef* gpiox,uint32_t scl_pinx,uint32_t sda_pinx);

#ifdef __cplusplus
}
#endif

#endif

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
