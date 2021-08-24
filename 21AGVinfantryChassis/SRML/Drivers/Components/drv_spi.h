/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    drv_spi.h
  * @author  Anthracene 
  * @brief   Code for spi driver in STM32 series MCU, supported packaged:
  *          - STM32Cube_FW_F4_V1.24.1.
  *          - STM32Cube_FW_F1_V1.8.0.
  *          - STM32Cube_FW_H7_V1.5.0.
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
#ifndef _DRV_SPI_H
#define _DRV_SPI_H

#ifdef __cplusplus
extern "C" {
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
/** 
* @brief  spi control operation
*/
typedef struct{
	SPI_HandleTypeDef * hspix;
	GPIO_TypeDef * cs_port;
	uint16_t cs_pin;
	uint32_t time_out;
	uint8_t * tx_buff;
	uint8_t * rx_buff;
}HardwareSPI_HandleTypeDef;
	
/* Private type --------------------------------------------------------------*/


/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function declarations --------------------------------------------*/	
void SPI_Init(SPI_HandleTypeDef* hspi,GPIO_TypeDef * cs_port,uint16_t cs_pin,uint32_t time_out);
void SPI_WriteRead(SPI_HandleTypeDef* hspi,uint8_t * tx_data,uint8_t * rx_buff,uint16_t tx_size,uint16_t rx_size);
void SPI_Write1ReadN(SPI_HandleTypeDef* hspi,uint8_t tx_data,uint8_t * rx_buff,uint16_t rx_size);

#ifdef __cplusplus
}
#endif

#endif

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
