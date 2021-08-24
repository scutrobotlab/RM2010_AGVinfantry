/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    mpu6050_config.h
  * @author  YDX 2244907035@qq.com
  * @brief   Code for MPU6050 config.
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

#ifndef __MPU6050_CONFIG_H
#define __MPU6050_CONFIG_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/  
#include "mpu6050.h"
#include "inv_mpu.h"
#
	 
/* Private macros ------------------------------------------------------------*/
	 
/* Private type --------------------------------------------------------------*/	 
typedef struct _MPUData
{
	short ax;
	short ay;
	short az;
	short gx;
	short gy;
	short gz;
	float roll;
	float pitch;
	float yaw;
	float gxoffset;
  float gyoffset;
  float gzoffset;
}MPUData_Typedef;


/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern IIC_PIN_Typedef MPU6050_IIC_PIN;
extern MPUData_Typedef MPUData;

/* Exported function declarations --------------------------------------------*/
uint8_t MPU6050_Init(GPIO_TypeDef *gpiox,uint32_t scl_pinx,uint32_t sda_pinx);
#ifdef __cplusplus
}
#endif

#endif
/************************ COPYRIGHT SCUT-ROBOTLAB *****END OF FILE*************/
