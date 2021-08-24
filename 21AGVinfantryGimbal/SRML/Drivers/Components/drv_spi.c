/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    drv_spi.c
  * @author  Anthracene a742674806@163.com
  * @brief   Code for spi driver in STM32 series MCU, supported packaged:
  *          - STM32Cube_FW_F4_V1.24.0.
  *          - STM32Cube_FW_F1_V1.8.0.
  *          - STM32Cube_FW_H7_V1.5.0.
  * @date    2019-12-28
  * @version 1.1
  * @par Change Log:
  * <table>
  * <tr><th>Date        <th>Version  <th>Author         <th>Description
  * <tr><td>2019-12-28  <td> 1.0     <td>Anthracene     <td>Creator
  * </table>
  *
  ==============================================================================
                            How to use this driver
  ==============================================================================
    @note
      -# 本文件为硬件spi驱动程序，请确保硬件spi配置正确
	
    @warning
      -# 
		
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
#include "drv_spi.h"
#include "drv_timer.h"

/* Private define ------------------------------------------------------------*/
void __SPI_Write_Read(HardwareSPI_HandleTypeDef spix,uint16_t tx_size,uint16_t rx_size);
	
/* Private variables ---------------------------------------------------------*/
static HardwareSPI_HandleTypeDef hspi1_handle;
static HardwareSPI_HandleTypeDef hspi2_handle;
static HardwareSPI_HandleTypeDef hspi3_handle;

/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* function prototypes -------------------------------------------------------*/

/**
* @brief  Initialize SPI
* @param  hspi : HAL Handler of spi x.
* @param  cs_port : cs gpio port of spi x.
* @param  cs_pin : cs gpio pin of spi x.
* @retval None
*/
void SPI_Init(SPI_HandleTypeDef* hspi,GPIO_TypeDef * cs_port,uint16_t cs_pin,uint32_t time_out)
{
	if(hspi->Instance == SPI1)
	{
		hspi1_handle.hspix = hspi;
		hspi1_handle.cs_port = cs_port;
		hspi1_handle.cs_pin = cs_pin;
		hspi1_handle.time_out = time_out;
	}
	if(hspi->Instance == SPI2)
	{
		hspi2_handle.hspix = hspi;
		hspi2_handle.cs_port = cs_port;
		hspi2_handle.cs_pin = cs_pin;
		hspi2_handle.time_out = time_out;
	}
	if(hspi->Instance == SPI3)
	{
		hspi3_handle.hspix = hspi;
		hspi3_handle.cs_port = cs_port;
		hspi3_handle.cs_pin = cs_pin;
		hspi3_handle.time_out = time_out;
	}
}

/**
* @brief  SPI Write & Read function
* @param  hspi : HAL Handler of spi x.
* @param  tx_data : pointer of tx data
* @param  rx_buff : pointer of rx data buffer
* @param  tx_size : size of tx data.
* @param  rx_size : size of rx data.
* @retval None
*/
void SPI_WriteRead(SPI_HandleTypeDef* hspi,uint8_t * tx_data,uint8_t * rx_buff,uint16_t tx_size,uint16_t rx_size)
{
	if(hspi->Instance == SPI1)
	{
		hspi1_handle.tx_buff = tx_data;
		hspi1_handle.rx_buff = rx_buff;
		__SPI_Write_Read(hspi1_handle,tx_size,rx_size);
	}
	if(hspi->Instance == SPI2)
	{
		hspi2_handle.tx_buff = tx_data;
		hspi2_handle.rx_buff = rx_buff;
		__SPI_Write_Read(hspi2_handle,tx_size,rx_size);
	}
	if(hspi->Instance == SPI3)
	{
		hspi3_handle.tx_buff = tx_data;
		hspi3_handle.rx_buff = rx_buff;
		__SPI_Write_Read(hspi3_handle,tx_size,rx_size);
	}
}

/**
* @brief  SPI Write & Read function, write 1 Byte and Read N Bytes
* @param  hspi : HAL Handler of spi x.
* @param  tx_data : tx data, 1 Byte
* @param  rx_buff : pointer of rx data buffer
* @param  tx_size : size of tx data.
* @param  rx_size : size of rx data.
* @retval None
*/
void SPI_Write1ReadN(SPI_HandleTypeDef* hspi,uint8_t tx_data,uint8_t * rx_buff,uint16_t rx_size)
{
	if(hspi->Instance == SPI1)
	{
		hspi1_handle.tx_buff = &tx_data;
		hspi1_handle.rx_buff = rx_buff;
		__SPI_Write_Read(hspi1_handle,1,rx_size);
	}
	if(hspi->Instance == SPI2)
	{
		hspi2_handle.tx_buff = &tx_data;
		hspi2_handle.rx_buff = rx_buff;
		__SPI_Write_Read(hspi2_handle,1,rx_size);
	}
	if(hspi->Instance == SPI3)
	{
		hspi3_handle.tx_buff = &tx_data;
		hspi3_handle.rx_buff = rx_buff;
		__SPI_Write_Read(hspi3_handle,1,rx_size);
	}
}

inline void __SPI_Write_Read(HardwareSPI_HandleTypeDef spix,uint16_t tx_size,uint16_t rx_size)
{
	/* 选中片选 */
	HAL_GPIO_WritePin(spix.cs_port,spix.cs_pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(spix.hspix,spix.tx_buff,tx_size,spix.time_out);
	HAL_SPI_Receive(spix.hspix,spix.rx_buff,rx_size,spix.time_out);
	HAL_GPIO_WritePin(spix.cs_port,spix.cs_pin,GPIO_PIN_SET);
	delay_us_nos(10);
}

