/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    BMX055.c
  * @author  YDX 2244907035@qq.com
  * @brief   Code for BMX055.
  * @date    2019-11-21
  * @version 1.0
  * @par Change Log:
  * <table>
  * <tr><th>Date        <th>Version  <th>Author  <th>Description
  * <tr><td>2019-11-21  <td> 1.0     <td>YDX     <td>Creator
  * </table>
  ******************************************************************************
   @note
      -# 使用说明在`BMX055_Config.c`。
	  
   @warning	
	  
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
/* Includes ------------------------------------------------------------------*/
#include "BMX055.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
BMX055Datatypedef BMX055;
struct ahrs_sensor BMX055_Sensor = {0};
struct attitude Attitude;
	
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  BMX055 Config
  * @param  hiic: handle of iic
  * @retval 0,successful
  *         1,fail        
  */
uint8_t BMX_Conf(IIC_PIN_Typedef *iic_pin)
{
  /* accelerometer config */
  uint8_t ErrCount = 0;
	
  /* initialize iic bus */
  IIC_Init(iic_pin);
	
  /* confirm accelerometer chip ID */
  while(IIC_Device_Read_Byte(iic_pin, IIC_BMX055_ACC_ADR, BMX055_ACC_ID) != 0xFA)   
  {
    ErrCount++;
    if(ErrCount > 5)
      return 1;
  }
  
  /* accelerometer range config */
  if(IIC_Device_Write_Byte(iic_pin, IIC_BMX055_ACC_ADR, BMX055_ACC_PMURANGE, 0x05) == 1)return 1;   //4G
  HAL_Delay(10);
  
  /* accelerometer output rate config */
  if(IIC_Device_Write_Byte(iic_pin, IIC_BMX055_ACC_ADR, BMX055_ACC_PMUBW, 0x0F) == 1)return 1;     //1000HZ      
  HAL_Delay(10);
  
  /* accelerometer mode config */
  if(IIC_Device_Write_Byte(iic_pin, IIC_BMX055_ACC_ADR, BMX055_ACC_PMULPM, 0x00) == 1)return 1;   //Normal MODE      
  HAL_Delay(10);
  
  /* gyroscope config */
  ErrCount = 0;
  
  /* confirm gyroscope chip ID */
  while(IIC_Device_Read_Byte(iic_pin, IIC_BMX055_GYRO_ADR, BMX055_GYRO_ID) != 0x0F)   
  {
    ErrCount++;
    if(ErrCount > 5)
      return 1;
  }
  
  /* gyroscope range config */
  if(IIC_Device_Write_Byte(iic_pin, IIC_BMX055_GYRO_ADR, BMX055_GYRO_RANGE, 0x00) == 1)return 1;   //+-2000dps      
  HAL_Delay(10);
  
  /* gyroscope output rate config */
  if(IIC_Device_Write_Byte(iic_pin, IIC_BMX055_GYRO_ADR, BMX055_GYRO_BW, 0x02) == 1)return 1;     //1000HZ      
  HAL_Delay(10);
  
  /* gyroscope mode config */
  if(IIC_Device_Write_Byte(iic_pin, IIC_BMX055_GYRO_ADR, BMX055_GYRO_LPM, 0x00) == 1)return 1;   //Normal MODE      
  HAL_Delay(10);
  
  /* gyroscope high-pass filter config */  
  if(IIC_Device_Write_Byte(iic_pin,IIC_BMX055_GYRO_ADR, BMX055_GYRO_RATEHBW, 0x08) == 1)return 1;   //not necessary 
  HAL_Delay(10);
  
  /* magnetometer config */
  ErrCount = 0;
  
  /* awake magnetometer */   
  IIC_Device_Write_Byte(iic_pin, IIC_BMX055_MAG_ADR, BMX055_MAG_POM, 0x81);
  HAL_Delay(10);
  
  /* confirm magnetometer chip ID */  
  while(IIC_Device_Read_Byte(iic_pin, IIC_BMX055_MAG_ADR, BMX055_MAG_ID) != 0x32)   
  {
    ErrCount++;
    if(ErrCount > 5)
      return 1;
  }

  /* magnetometer output rate config */  
  if(IIC_Device_Write_Byte(iic_pin, IIC_BMX055_MAG_ADR, BMX055_MAG_DATARATE, 0x38) == 1)return 1;   //max speed 30HZ       
  HAL_Delay(10);
  
  /* magnetometer interrupt mode config */  
  if(IIC_Device_Write_Byte(iic_pin, IIC_BMX055_MAG_ADR, BMX055_MAG_INTEN, 0x00) == 1)return 1;      //disable interrupt    
  HAL_Delay(10);
  return 0;
}

/**
  * @brief  read BMX055 data
  * @param  hiic: handle of iic
  * @param  bmxdata: struct for storing bmx055 data 
  * @param  type: 0 read acc and gyro data
  *               1 read acc,gyro and mag data
  * @retval 0,success
  *         1,fail    
  * @note   In F4 platform,it takes approximately 75us to read data form an axis.
  */
uint8_t BMX055_DataRead(IIC_PIN_Typedef *iic_pin, BMX055Datatypedef *bmxdata, uint8_t type)
{
  uint8_t datatemp[6] = {0};
  
  /* read gyroscope data*/
  if(IIC_Device_Read_Len(iic_pin, IIC_BMX055_GYRO_ADR, BMX055_GYRO_XDATALSB, 6, datatemp) == 1)return 1;
  bmxdata->gx = (float)((int16_t)((datatemp[1] << 8) | datatemp[0]));
  bmxdata->gy = (float)((int16_t)((datatemp[3] << 8) | datatemp[2]));
  bmxdata->gz = (float)((int16_t)((datatemp[5] << 8) | datatemp[4]));
  
  /* read accelerometer data*/  
  if(IIC_Device_Read_Len(iic_pin, IIC_BMX055_ACC_ADR, BMX055_ACC_XDATALSB, 6, datatemp) == 1)return 1;
  bmxdata->ax = (float)((int16_t)((datatemp[1] << 8) | datatemp[0]) >> 4);
  bmxdata->ay = (float)((int16_t)((datatemp[3] << 8) | datatemp[2]) >> 4);
  bmxdata->az = (float)((int16_t)((datatemp[5] << 8) | datatemp[4]) >> 4);

  /* read magnetometer data*/
  if(type)
  {
    if(IIC_Device_Read_Len(iic_pin, IIC_BMX055_MAG_ADR, BMX055_MAG_XDATALSB, 6, datatemp) == 1)return 1;
    bmxdata->mx = (float)((int16_t)((datatemp[1] << 8) | datatemp[0]) >> 3);
    bmxdata->my = (float)((int16_t)((datatemp[3] << 8) | datatemp[2]) >> 3);
    bmxdata->mz = (float)((int16_t)((datatemp[5] << 8) | datatemp[4]) >> 1);
  }
  return 0;
}

/**
 * @brief  Get the raw data of gyroscope from BMX055. 
 * @param  hiic: handle of iic
 * @param  *gx, *gy, *gz: the raw data of gyroscope 
 * @retval 0,success
 *         1,fail
 */
uint8_t BMX055_Get_Gyro(IIC_PIN_Typedef *iic_pin, float *gx, float *gy, float *gz)
{
  uint8_t datatemp[6] = {0};
  
  /* read gyroscope data*/
  if(IIC_Device_Read_Len(iic_pin, IIC_BMX055_GYRO_ADR, BMX055_GYRO_XDATALSB, 6, datatemp) == 1)return 1;
  *gx = (float)((int16_t)((datatemp[1] << 8) | datatemp[0]));
  *gy = (float)((int16_t)((datatemp[3] << 8) | datatemp[2]));
  *gz = (float)((int16_t)((datatemp[5] << 8) | datatemp[4]));
  return 0;
}

/**
 * @brief  Get the raw data of accelerometer from BMX055. 
 * @param  hiic: handle of iic
 * @param  *ax, *ay, *az: the raw data of accelerometer 
 * @retval 0,success
 *         1,fail
 */
uint8_t BMX055_Get_Acc(IIC_PIN_Typedef *iic_pin, float *ax, float *ay, float *az)
{
  uint8_t datatemp[6] = {0};
	  
  /* read accelerometer data*/  
  if(IIC_Device_Read_Len(iic_pin, IIC_BMX055_ACC_ADR, BMX055_ACC_XDATALSB, 6, datatemp) == 1)return 1;
  *ax = (float)((int16_t)((datatemp[1] << 8) | datatemp[0]) >> 4);
  *ay = (float)((int16_t)((datatemp[3] << 8) | datatemp[2]) >> 4);
  *az = (float)((int16_t)((datatemp[5] << 8) | datatemp[4]) >> 4);
  return 0;
}

/**
 * @brief  Get the euler angle from BMX055. 
 * @param  *roll, *pitch, *yaw: the Euler angle 
 * @retval default 0
 * @note   roll range:  -180.0°<---> +180.0°
           pitch range: -90.0° <---> +90.0°
           yaw range:   -180.0°<---> +180.0°
 */
uint8_t BMX055_Get_Angle(float *roll, float *pitch, float *yaw)
{
  *roll = Attitude.roll;	
  *pitch = Attitude.pitch;		
  *yaw = Attitude.yaw;	
  return 0;
}

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
