/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    drv_timer.h
  * @author  Mentos_Seetoo 1356046979@qq.com
  * @brief   Code for Timer Management in STM32 series MCU, supported packaged:
  *          - STM32Cube_FW_F4_V1.24.0.
  *          - STM32Cube_FW_F1_V1.8.0.
  *          - STM32Cube_FW_H7_V1.5.0.
  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have any 
  * bugs, update the version Number, write dowm your name and the date. The most
  * important thing is make sure the users will have clear and definite under-
  * standing through your new brief.
  *
  * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
  */
#ifndef  _DRV_TIMER_H
#define  _DRV_TIMER_H

#ifdef  __cplusplus
extern "C"{
#endif

/* Includes ------------------------------------------------------------------*/
#if defined(USE_HAL_DRIVER)
  #if defined(STM32F405xx) || defined(STM32F407xx)
    #include <stm32f4xx_hal.h>
  #endif
  #if defined(STM32F103xx)
    #include <stm32f1xx_hal.h>
  #endif
  #if defined(STM32H750xx)
    #include <stm32h7xx_hal.h>
  #endif	
#endif
/* Private macros ------------------------------------------------------------*/
#define microsecond()    Get_SystemTimer()

/* Private type --------------------------------------------------------------*/
typedef struct{
  uint32_t last_time;	/*!< Last recorded real time from systick*/
  float dt;				/*!< Differentiation of real time*/
}TimeStamp;

/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef enum 
{
	USE_MODULE_DELAY = 1,	/*!< Use module function to implement delay_ms_nos()*/
	USE_HAL_DELAY			/*!< Use HAL_Delay() for delay_ms_nos()*/
}EDelay_src;

/* Exported variables ---------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/
void Timer_Init(TIM_HandleTypeDef* htim, EDelay_src src);
void Update_SystemTick(void);
uint32_t Get_SystemTimer(void);
void delay_ms_nos(uint32_t cnt);
void delay_us_nos(uint32_t cnt);


#ifdef  __cplusplus
}
#endif

#endif 
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
