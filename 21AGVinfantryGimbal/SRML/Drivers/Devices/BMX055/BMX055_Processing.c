/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    Attitude_Processing.c
  * @author  YDX 2244907035@qq.com
  * @brief   Code for attitude processing using BMX055.
  * @date    2020-04-02
  * @version 1.0
  * @par Change Log:
  * <table>
  * <tr><th>Date        <th>Version  <th>Author  <th>Description
  * <tr><td>2019-11-21  <td> 1.0     <td>YDX     <td>Creator
  * <tr><td>2020-04-02  <td> 1.1     <td>YDX     <td>Change the algorithm of solving data.
  * </table>
  *
  ==============================================================================
                     ##### How to use this driver #####
  ==============================================================================
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
#include "BMX055_Processing.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  Gyroscope offset initialization   
  * @param  void
  * @retval void     
  */
void Gyro_Offset_Init(void)      
{
	uint16_t i=0, cnt=0;
	for(i = 0;i < 1024 ;i++)
	{
		BMX055_DataRead(&BMX055_IIC_PIN, &BMX055, 0);
		if(i>300)
		{
            BMX055.gx_offset += BMX055.gx;
            BMX055.gy_offset += BMX055.gy;
            BMX055.gz_offset += BMX055.gz;
		    cnt++;
			HAL_Delay(2);
		}
	}
	BMX055.gx_offset /= (float)cnt;
    BMX055.gy_offset /= (float)cnt;
	BMX055.gz_offset /= (float)cnt;
}

/**
  * @brief  BMX055 solve data 
  * @param  void
  * @retval void              
  */
void BMX055_Solve_Data(void)
{ 
    BMX055_DataRead(&BMX055_IIC_PIN, &BMX055, 0);
	
    /* 2000dps */	
    BMX055.gx = (BMX055.gx - BMX055.gx_offset) * 2000 / 32768;   
    BMX055.gy = (BMX055.gy - BMX055.gy_offset) * 2000 / 32768;
    BMX055.gz = (BMX055.gz - BMX055.gz_offset) * 2000 / 32768;
	
    /* 4g */
    BMX055.ax = BMX055.ax * 4 / 2048;    
    BMX055.ay = BMX055.ay * 4 / 2048;
    BMX055.az = BMX055.az * 4 / 2048;
  
    /* tranfer the bmx055 data to attitude calculation module,you can put a filter here */
	
    /* dps -> rad/s */
    BMX055_Sensor.wx = BMX055.gx / 57.3f;
    BMX055_Sensor.wy = BMX055.gy / 57.3f;
    BMX055_Sensor.wz = BMX055.gz / 57.3f;
	
    BMX055_Sensor.ax = BMX055.ax;
    BMX055_Sensor.ay = BMX055.ay;
    BMX055_Sensor.az = BMX055.az;
	
	mahony_ahrs_update(&BMX055_Sensor, &Attitude);
}
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
