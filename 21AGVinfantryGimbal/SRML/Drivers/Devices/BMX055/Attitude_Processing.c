/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    Attitude_Processing.c
  * @author  YDX 2244907035@qq.com
  * @brief   Code for BMX055 attitude processing.
  * @date    2019-11-21
  * @version 1.0
  * @par Change Log:
  * <table>
  * <tr><th>Date        <th>Version  <th>Author  <th>Description
  * <tr><td>2019-11-21  <td> 1.0     <td>YDX     <td>Creator
  * </table>
  *
  ==============================================================================
                     ##### How to use this driver #####
  ==============================================================================
    @note
      -# 本文件中定义了储存姿态的两个结构体SystemAttitude和SystemAttitudeRate，
         Get_Attitude函数会自动更新这两个结构体里面的内容，可以直接使用这两个
         结构体中的数据。
      -# 用户可以更改陀螺仪零偏函数的执行次数，当前次数是1000。
      -# 使用说明在`BMX055_Config.C`
    @warning	
      -# 这里一堆算法，不要改！！！！！！！！！！！！！！！
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
#include "Attitude_Processing.h"
#include "BMX055_Config.h"
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t IsGyroOffsetReset = 1;    

/* 角度 */
EulerAngleTypedef SystemAttitude;    

/* 角速度 */
EulerAngleTypedef SystemAttitudeRate;        
AttitudeDatatypedef GyroOffset;

/* Private type --------------------------------------------------------------*/
static void Gyro_Offset_Init(void); 

/* Private function declarations ---------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  Gyroscope offset initialization   
  * @param  void
  * @retval void     
  * @note   1000 equal interval samples
  */
void Gyro_Offset_Init(void)      
{
  static uint16_t Gyro_Count = 0;  
  if(Gyro_Count == 0)
  {
    GyroOffset.Xdata = 0;
    GyroOffset.Ydata = 0;
    GyroOffset.Zdata = 0;
  }
  
  if(Gyro_Count == 1000)
  {
    GyroOffset.Xdata /= 1000;
    GyroOffset.Ydata /= 1000;
    GyroOffset.Zdata /= 1000;
    IsGyroOffsetReset = 0;
    Gyro_Count = 0;
  }
  else
  {
    BMX055_DataRead(&BMX055_IIC, &BMX055_data, 0);
    GyroOffset.Xdata += BMX055_data.GYROXdata;
    GyroOffset.Ydata += BMX055_data.GYROYdata;
    GyroOffset.Zdata += BMX055_data.GYROZdata;
    Gyro_Count++;
  }
}

/**
  * @brief  BMX055 data reading and attitude calculation
  * @param  void
  * @retval void        
  * @note   In F4 platform,it takes approximately 75us to read data form an axis,so 6 axis need approximately 450us to read data,
  *         the calculation needs nearly 50us.
  */
void BMX055_solve_data(void)
{
  static uint8_t IsAttitudeinit = 1;
  float AccZAngle = 0;
  float AccZ, AccZAdjust;   
  if(IsGyroOffsetReset)
  {
    /* keep gyroscope static when initializing gyroscope offset */
    Gyro_Offset_Init();     
    return;
  }
  BMX055_DataRead(&BMX055_IIC, &BMX055_data, 0);
  BMX055_data.GYROXdata = (BMX055_data.GYROXdata - GyroOffset.Xdata) * 2000 / 32768;   
  BMX055_data.GYROYdata = (BMX055_data.GYROYdata - GyroOffset.Ydata) * 2000 / 32768;
  BMX055_data.GYROZdata = (BMX055_data.GYROZdata - GyroOffset.Zdata) * 2000 / 32768;
  BMX055_data.ACCXdata = BMX055_data.ACCXdata * 4 / 2048;    
  BMX055_data.ACCYdata = BMX055_data.ACCYdata * 4 / 2048;
  BMX055_data.ACCZdata = BMX055_data.ACCZdata * 4 / 2048;
  
  /* tranfer the bmx055 data to attitude calculation module,you can put a filter in it */
  Acc.Xdata = BMX055_data.ACCXdata;
  Acc.Ydata = BMX055_data.ACCYdata;
  Acc.Zdata = BMX055_data.ACCZdata;
  Gyro.Xdata = BMX055_data.GYROXdata;
  Gyro.Ydata = BMX055_data.GYROYdata;
  Gyro.Zdata = BMX055_data.GYROZdata;

  if(IsAttitudeinit == 1)
  {
    /* attitude calculation initialization */    
    Quaternion_init();                        
    IsAttitudeinit = 0;
  }
  else
  {
    /* Quick update */    
    Attitude_UpdateGyro();               
      
    /* Deep fusion update */  
    Attitude_UpdateAcc();                     
	  
    /* calculate all angles and angular speeds */
    SystemAttitude.Pitch     = -EulerAngle.Roll / PI * 180;        
    SystemAttitude.Roll      = EulerAngle.Pitch / PI * 180;        
    SystemAttitude.Yaw       = EulerAngle.Yaw / PI * 180;          
    SystemAttitudeRate.Pitch = -EulerAngleRate.Roll / PI * 180;    
    SystemAttitudeRate.Roll  = EulerAngleRate.Pitch / PI * 180;    	
    SystemAttitudeRate.Yaw   = EulerAngleRate.Yaw / PI * 180;      
		
    /* pitch complementary filtering */
    AccZ = -Acc.Zdata;
    if (AccZ > 1)
	    AccZ = 1;
    if (AccZ < -1)
	    AccZ = -1;            
    AccZAngle = asinf(AccZ) * 180 / PI;
    AccZAdjust = (AccZAngle - SystemAttitude.Pitch);
    SystemAttitude.Pitch += (-Gyro.Xdata + AccZAdjust) * PERIODS;
  }
}
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
