/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    BMX055.h
  * @author  YDX 2244907035@qq.com
  * @brief   Code for BMX055.
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
#ifndef _BMX055_H_
#define _BMX055_H_

#ifdef __cplusplus
extern "C" {
#endif
	
/* Includes ------------------------------------------------------------------*/ 
#include "Drivers/Components/drv_i2c.h"
#include "BMX055_Processing.h"
#include "Mahony_AHRS.h"
	
/* Private macros ------------------------------------------------------------*/       
#define IIC_BMX055_ACC_ADR    0x18
#define IIC_BMX055_GYRO_ADR   0x68
#define IIC_BMX055_MAG_ADR    0x10   

#define BMX055_ACC_XDATALSB   0x02
#define BMX055_ACC_ID         0x00
#define BMX055_ACC_PMURANGE   0x0F
#define BMX055_ACC_PMUBW      0x10
#define BMX055_ACC_PMULPM     0x11


#define BMX055_GYRO_XDATALSB  0x02
#define BMX055_GYRO_ID        0x00
#define BMX055_GYRO_RANGE     0x0F
#define BMX055_GYRO_BW        0x10
#define BMX055_GYRO_LPM       0x11
#define BMX055_GYRO_RATEHBW   0x13

#define BMX055_MAG_XDATALSB   0x42
#define BMX055_MAG_ID         0x40
#define BMX055_MAG_POM        0x4B
#define BMX055_MAG_DATARATE   0x4C
#define BMX055_MAG_INTEN      0x4E

/* Private type --------------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct
{
  float gx;
  float gy;
  float gz;
  float ax;
  float ay;
  float az;
  float mx;
  float my;
  float mz;
  float gx_offset;
  float gy_offset;
  float gz_offset;
}BMX055Datatypedef;

/* Exported variables --------------------------------------------------------*/
extern BMX055Datatypedef BMX055;
extern struct ahrs_sensor BMX055_Sensor;
extern struct attitude Attitude;

/* Exported function declarations --------------------------------------------*/
uint8_t BMX_Conf(IIC_PIN_Typedef *iic_pin);
uint8_t BMX055_DataRead(IIC_PIN_Typedef *iic_pin, BMX055Datatypedef *bmxdata, uint8_t type);
uint8_t BMX055_Get_Gyro(IIC_PIN_Typedef *iic_pin, float *gx, float *gy, float *gz);
uint8_t BMX055_Get_Acc(IIC_PIN_Typedef *iic_pin, float *ax, float *ay, float *az);
uint8_t BMX055_Get_Angle(float *roll, float *pitch, float *yaw);

#ifdef __cplusplus
}
#endif

#endif
/************************ COPYRIGHT SCUT-ROBOTLAB *****END OF FILE*************/

