/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    drv_timer.c
  * @author  Mentos_Seetoo 1356046979@qq.com
  * @brief   Code for Timer Management in STM32 series MCU, supported packaged:
  *          - STM32Cube_FW_F4_V1.24.0.
  *          - STM32Cube_FW_F1_V1.8.0.
  *          - STM32Cube_FW_H7_V1.5.0.
  * @date    2019-06-12
  * @version 2.0
  * @par Change Log：
  * <table>
  * <tr><th>Date        <th>Version  <th>Author         <th>Description
  * <tr><td>2019-06-12  <td> 1.0     <td>Mentos Seetoo  <td>Creator
  * <tr><td>2019-10-28  <td> 2.0     <td>Mentos Seetoo  <td>Add Timer manage object.
  * </table>
  *
  ==============================================================================
                            How to use this driver  
  ==============================================================================
    @note
      -# 使用`Timer_Init()`设置延时定时器`TIM_X`,设置`delay_ms()`函数使用HAL库的实现
          `HAL_Delay()`或使用模块内方法实现。		
      -# 配置`TIM_X`自增时间为1us，在对应中断函数中加入:`Update_SystemTick();`。
      -# 在需要延时的地方使用`delay_us_nos()`或`delay_ms_noe()`。
		
    @warning
      -# 本模块的所有延时函数均为堵塞式延时。
      -# 使用前必须进行初始化！
      -# 添加预编译宏`USE_FULL_ASSERT`可以启用断言检查。
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
/* Includes ------------------------------------------------------------------*/
#include "drv_timer.h"

/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
volatile uint32_t SystemTimerCnt;

struct timer_manage_obj_t
{
	TIM_HandleTypeDef*	htim_x;
	EDelay_src	delay_ms_src;
}Timer_Manager;
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
static void Error_Handler(void);

/* function prototypes -------------------------------------------------------*/
/**
* @brief  Initialize Timer
* @param  htim_x : HAL Handler of timer x.
* @param  src : Choose the src for delay_ms().
* @retval None
*/
void Timer_Init(TIM_HandleTypeDef* htim, EDelay_src src)
{
	/* Check the parameters */
	assert_param(htim != NULL);
	
	Timer_Manager.htim_x = htim;
	Timer_Manager.delay_ms_src = src;
    
  if(HAL_TIM_Base_Start_IT(Timer_Manager.htim_x)!=HAL_OK)
      Error_Handler();
  if(HAL_TIM_Base_Start(Timer_Manager.htim_x)!= HAL_OK)
      Error_Handler();
}


/**
* @brief  Get the system tick from timer.
* @param  None
* @retval current tick.
*/
uint32_t Get_SystemTimer(void)
{
	return Timer_Manager.htim_x->Instance->CNT + SystemTimerCnt * 0xffff;
}

/**
* @brief  Update system tick that had run in Timer Interupt.
* @not    Add this function into Timer interupt function.
* @param  None
* @retval None
*/
void Update_SystemTick(void)
{
	SystemTimerCnt++;
}

/**
* @brief  Delay microsecond.
* @param  cnt : microsecond to delay 
* @retval None
*/
void delay_us_nos(uint32_t cnt)
{
	uint32_t temp = cnt  + microsecond();

	while(temp >= microsecond());
}

/**
* @brief  Delay millisecond.
* @param  cnt : millisecond to delay
* @retval None
*/
void delay_ms_nos(uint32_t cnt)
{
	if(Timer_Manager.htim_x != NULL && Timer_Manager.delay_ms_src == USE_MODULE_DELAY)
	{
		uint32_t temp = cnt * 1000 + microsecond();
		while(temp >= microsecond());
	}
	else
		HAL_Delay(cnt);
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
static void Error_Handler(void)
{
  /* Nromally the program would never run here. */
  while(1){}
}
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
