/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
  * @file    led.h
  * @author  LJY 2250017028@qq.com
  * @brief   Code for LED.
  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have many 
  * bugs, update the version Number, write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding 
  * through your new brief.
  *
  * <h2><center>&copy; Copyright (c) 2021 - ~, SCUT-RobotLab Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
  */ 

#ifndef __LED_H
#define __LED_H	 

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/  
#include "main.h"
#include <algorithm>

/* Private define ------------------------------------------------------------*/
#define RGB_NUM 	24		//一个RGB灯需要的PWM脉冲数目
#define RESET_NUM	250		//复位灯带需要的PWM脉冲数目
#define LED_LOW 	34		//0信号的占空比
#define LED_HIGH 	68		//1信号的占空比									

/* Private include -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/			

/* Private type --------------------------------------------------------------*/

/* 颜色块结构体 */
typedef struct RGB
{	
	uint8_t red;
	uint8_t green;
	uint8_t blue;
}color;

/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
template<uint16_t NUM>
class LED
{
public:
	uint32_t Temp_Buffer[NUM][RGB_NUM];							//供32位的定时器使用的缓存区
	uint32_t Reset_RGB[RESET_NUM];								//250个脉冲使得输出清零信号>280us，能把整条灯带清零

	LED(TIM_HandleTypeDef *htimer, DMA_HandleTypeDef *_hdma, uint32_t _channel)
	: ledPwmTimer(htimer), hdma(_hdma), Channel(_channel){}
		
	/**@brief 亮灯
	*@param[in] data 颜色数组
	*@return 0 success
			 1 fail
	*/
	uint8_t ON(uint32_t data[NUM][RGB_NUM])
	{	
		uint16_t i, j;
		uint8_t res;
		
		/* 由于定时器2的寄存器为32位,所以分开讨论 */
		if(ledPwmTimer->Instance == TIM2)
		{
			for(i = 0; i < NUM; i ++)
			{
				for(j = 0; j < RGB_NUM; j ++) Temp_Buffer[i][j] = data[i][j];
			}
			res = Set((uint32_t *)Temp_Buffer, NUM * RGB_NUM);
		}
		else 
		{
			res = Set((uint32_t *)data, NUM * RGB_NUM);
		}
		/* 按照灯带时序，每次写入数据后需发送复位信号 */
		Reset();
		return res;
	}

private:
	TIM_HandleTypeDef* ledPwmTimer;									//用于输出PWM的TIM句柄
	DMA_HandleTypeDef* hdma;										//TIM对应的DMA
	uint32_t Channel;												//TIM输出通道
	
	/**@brief 发送颜色数据
		*@param void
		*@return 0 success
				 1 fail
	*/
	uint8_t Set(uint32_t* data, uint32_t num)
	{
		if(!Data_Transmit(data, num)) return 0;
		else return 1;	
	}

	/**@brief 发送重置数据
		*@param void
		*@return 0 success
				 1 fail
	*/
	uint8_t Reset(void)
	{
		if(!Data_Transmit(Reset_RGB,RESET_NUM)) return 0;
		else return 1;
	}

	/**@brief 数据传输
		*@param[in]	buffer 	数组缓存
		*@param[in] num	 传输数目
		*@return 0 success
				 1 fail
	*/
	uint8_t Data_Transmit(uint32_t* buffer, uint32_t num)
	{	
		/* 开始传输数据 */
		while(HAL_TIM_PWM_Start_DMA(ledPwmTimer, Channel, buffer, num) != HAL_OK);
		
		/* 等待传输完成 */
		while(HAL_DMA_GetState(hdma) != HAL_DMA_STATE_READY);
		
		/* 多通道的DMA要关闭 */
		while(HAL_TIM_PWM_Stop_DMA(ledPwmTimer, Channel) != HAL_OK);
		
		return 0;
	}	
};
/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/
#endif	
#endif
