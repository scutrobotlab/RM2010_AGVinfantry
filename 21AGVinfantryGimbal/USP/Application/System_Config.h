/**
  ******************************************************************************
  * @file   : System_config.h
  * @brief  : Header for System_config.c
  * @author : Mentos Seetoo
  ****************************************************************************** 
**/

#ifndef SYS_CONFIG_H
#define SYS_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Exported function declarations --------------------------------------------*/
#ifdef  __cplusplus
extern "C"{
#endif
void System_Tasks_Init(void);                                       
void System_Resource_Init(void);   
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern DMA_HandleTypeDef hdma_tim3_ch1_trig;	
#ifdef  __cplusplus
}
#endif                                


#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
