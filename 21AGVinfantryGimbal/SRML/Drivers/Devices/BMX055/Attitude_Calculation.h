/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    Attitude_Calculation.h
  * @author  YDX 2244907035@qq.com
  * @brief   Code for bmx055 attitude calculation.
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
#ifndef  _ATTITUDE_CALCULATION_H_
#define  _ATTITUDE_CALCULATION_H_

#ifdef __cplusplus
extern "C" {
#endif
	
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <math.h>                        // ffabs 
#include <string.h>                      //memcpy                    	
	
/* Private macros ------------------------------------------------------------*/  
#define  PI            3.1415926f

/* Private type --------------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct{
  float W;
  float X;
  float Y;
  float Z;
}QuaternionTypedef;

typedef struct{ 
  float Pitch;  //!<俯仰角
  float Yaw;    //!<偏航角
  float Roll;   //!<翻滚角
}EulerAngleTypedef;

typedef struct{
  float Xdata;
  float Ydata;
  float Zdata;
}AttitudeDatatypedef;

/* Exported variables --------------------------------------------------------*/
extern EulerAngleTypedef      SystemAttitude;            
extern EulerAngleTypedef      SystemAttitudeRate;       
extern AttitudeDatatypedef    GyroOffset;

extern AttitudeDatatypedef         Acc;
extern AttitudeDatatypedef         Gyro;

extern QuaternionTypedef    Quaternion;   
extern EulerAngleTypedef    EulerAngle;   
extern QuaternionTypedef    AxisAngle;    
extern EulerAngleTypedef    EulerAngleRate;

extern QuaternionTypedef    MeaQuaternion;
extern EulerAngleTypedef    MeaEulerAngle;
extern QuaternionTypedef    MeaAxisAngle;

extern QuaternionTypedef    ErrQuaternion;
extern EulerAngleTypedef    ErrEulerAngle;
extern QuaternionTypedef    ErrAxisAngle;

extern float PERIODHZ;     
extern float PERIODS;   
/* Exported function declarations --------------------------------------------*/ 
extern void Quaternion_init(void);
extern void Attitude_UpdateGyro(void);
extern void Attitude_UpdateAcc(void);
extern void Calculate_Acc_Earth(QuaternionTypedef *q);
extern void Calculate_Vel_Earth(void);
extern void Calculate_Disp_Earth(void);
extern void Calculate_Acc_Earth_Offset(QuaternionTypedef *q);
	
#ifdef __cplusplus
}
#endif

#endif
/************************ COPYRIGHT SCUT-ROBOTLAB *****END OF FILE*************/
