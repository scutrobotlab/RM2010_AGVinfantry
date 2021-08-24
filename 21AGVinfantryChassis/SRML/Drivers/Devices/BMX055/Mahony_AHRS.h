/**
  ******************************************************************************
  * @file 		Mahony_AHRS.h
  * @author  	charlie 602894526@qq.com
  * @brief 		Code for mahony ahrs algorithm.
  * @version 	1.0
  * @date		02/04/2020
  * @editby 	charlie

  ==============================================================================
                     ##### How to use this conf #####
  ==============================================================================
	
  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have many 
  * bugs, update the version NO., write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding 
  * through your new brief.
  ******************************************************************************
  */
#ifndef __MAHONY_AHRS_H__
#define __MAHONY_AHRS_H__

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private type --------------------------------------------------------------*/
struct ahrs_sensor
{
  float ax;
  float ay;
  float az;

  float wx;
  float wy;
  float wz;

  float mx;
  float my;
  float mz;
};

struct attitude
{
  float roll;
  float pitch;
  float yaw;
};
/* Private function declarations ---------------------------------------------*/
float invSqrt(float x);

void mahony_ahrs_update(struct ahrs_sensor *sensor, struct attitude *atti);
void mahony_ahrs_updateIMU(struct ahrs_sensor *sensor, struct attitude *atti);

/* Private function prototypes -----------------------------------------------*/


#ifdef __cplusplus
}
#endif
#endif
/************************ COPYRIGHT SCUT-ROBOTLAB *****END OF FILE*************/
