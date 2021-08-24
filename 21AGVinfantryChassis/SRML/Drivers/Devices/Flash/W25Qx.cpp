/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    W25Qx.cpp
  * @author  Anthracene a742674806@163.com
  * @brief   Code for driver of SPI flash chip W25Qx
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
      -# 本文件为W25QxFlash芯片驱动程序，需要使用硬件spi，请确保硬件SPI配置正确
			-# 依赖于硬件SPI的驱动文件drv_spi.c
			-# 依赖于定时器的驱动文件drv_timer.c
	
    @warning
      -# 如果需要使用默认W25Qx设备及c风格接口函数，请在srml_config.h内定义
				 USE_SRML_FATFS = 1
			-# FATFS文件系统依赖于此文件
		
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
#include "W25Qx.h"

/* Private define ------------------------------------------------------------*/
	
/* Private variables ---------------------------------------------------------*/
#if USE_SRML_FATFS
W25Qx_Classdef default_w25qx;
#endif

/* Private type --------------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/
uint8_t W25Qx_Classdef::init()
{
	SPI_Write1ReadN(hspix,0x9F,read_buffer,3);
	switch(model)
	{
		case(W25Q80):
			if(read_buffer[2]+read_buffer[1]*256 != DEVICE_ID_80)
				return 1;
			break;
		case(W25Q16):
			if(read_buffer[2]+read_buffer[1]*256 != DEVICE_ID_16)
				return 1;
			break;
		case(W25Q32):
			if(read_buffer[2]+read_buffer[1]*256 != DEVICE_ID_32)
				return 1;
			break;
		case(W25Q64):
			if(read_buffer[2]+read_buffer[1]*256 != DEVICE_ID_64)
				return 1;
			break;
		case(W25Q128):
			if(read_buffer[2]+read_buffer[1]*256 != DEVICE_ID_128)
				return 1;
			break;
		default:
			break;
	}
	return 0;
}

uint8_t W25Qx_Classdef::init(SPI_HandleTypeDef * _hspi,W25QxModel_Enumdef _model)
{
	hspix = _hspi;
	model = _model;
	blockSize = 16 * model;
	sectorSize = 16;
	pageSize = 16;
	byteSize = 256;
	
	return init();
}

uint8_t W25Qx_Classdef::readByte(uint8_t block,uint8_t sector,uint8_t page,uint8_t byte)
{
	assert_param(block < blockSize);
	assert_param(sector < sectorSize);
	assert_param(page < pageSize);
	assert_param(byte < byteSize);
		
	uint8_t tx_data[4] = {READ_DATA,block,uint8_t((sector<<4)|page),byte};
	SPI_WriteRead(hspix,tx_data,read_buffer,4,2);
		
	return read_buffer[0];
}

uint8_t W25Qx_Classdef::writeByte(uint8_t block,uint8_t sector,uint8_t page,uint8_t byte,uint8_t data)
{
	assert_param(block < blockSize);
	assert_param(sector < sectorSize);
	assert_param(page < pageSize);
	assert_param(byte < byteSize);
	
	writeEnable();
	//getStatus();
	
	uint8_t tx_data[5] = {PAGE_PROGRAM,block,uint8_t((sector<<4)|page),byte,data};
	SPI_WriteRead(hspix,tx_data,read_buffer,5,0);
	
	return 0;
}

uint8_t W25Qx_Classdef::continueRead(uint8_t block,uint8_t sector,uint8_t page,uint8_t byte,uint8_t * data,uint16_t length)
{
	assert_param(block < blockSize);
	assert_param(sector < sectorSize);
	assert_param(page < pageSize);
	assert_param(byte < byteSize);
	assert_param(length < 256);
	
	uint8_t tx_data[4] = {READ_DATA,block,uint8_t((sector<<4)|page),byte};
	SPI_WriteRead(hspix,tx_data,data,4,length);
		
	return 0;
}

/**
* @brief  W25Qx flash chip continue write function, do not warite more than 4096 Bytes(1 sector)
* @param  block : block num
* @param  sector : address offset in a block
* @param  page : address offset in a sector
* @param  byte : address offset in a page (page*256+byte+length<=4096)
* @param  data : data to write.
* @param	length : write length (length<=4096)
* @retval None
*/
uint8_t W25Qx_Classdef::continueWrite(uint8_t block,uint8_t sector,uint8_t page,uint8_t byte,uint8_t * data,uint16_t length)
{
	assert_param(block < blockSize);
	assert_param(sector < sectorSize);
	assert_param(page < pageSize);
	assert_param(byte < byteSize);
	assert_param(page*256+byte+length < FLASH_PAGE_SIZE+1);
	
	uint8_t head[4] = {PAGE_PROGRAM,block,uint8_t((sector<<4)|page),byte};
		
	if(length > FLASH_PAGE_SIZE){
		/* if write bytes is lager than FLASH_PAGE_SIZE
			 the addressing will wrap to the beginning of
			 the page and overwrite previously sent data.
		 */
		do{
			/* compare the size of FLASH_PAGE_SIZE and length
				 prevent from memory overflow.
			 */
			uint16_t cpy_size = (FLASH_PAGE_SIZE>length)?length:FLASH_PAGE_SIZE;
			memcpy(&write_buffer[0],head,4);
			memcpy(&write_buffer[4],data,cpy_size);
			
			writeEnable();
			
			SPI_WriteRead(hspix,write_buffer,read_buffer,cpy_size+4,0);
			/* wait untill flash write done */
			waitBusy();
			
			length -= FLASH_PAGE_SIZE;
			data += FLASH_PAGE_SIZE;
			head[2] += 1;
			
		}while(length > 0);
	}
	else{
		memcpy(&write_buffer[0],head,4);
		memcpy(&write_buffer[4],data,length);
		
		writeEnable();
		
		SPI_WriteRead(hspix,write_buffer,read_buffer,length+4,0);
	}
	
	return 0;
}

uint16_t W25Qx_Classdef::getStatus()
{
	SPI_Write1ReadN(hspix,READ_STATUS1,read_buffer,1);
	chipStatus = read_buffer[0];
	SPI_Write1ReadN(hspix,READ_STATUS2,read_buffer,1);
	chipStatus |= (read_buffer[0]<<8);
	return chipStatus;
}

uint8_t W25Qx_Classdef::eraseSector(uint8_t block,uint8_t sector)
{
	writeEnable();
	
	waitBusy();
	uint8_t tx_data[4] = {SECTOR_ERASE,block,uint8_t(sector<<4),0x00};
	SPI_WriteRead(hspix,tx_data,read_buffer,4,0);
	waitBusy();
	
	return 0;
}

uint8_t W25Qx_Classdef::eraseBlock(uint8_t block)
{
	writeEnable();
	
	uint8_t tx_data[4] = {BLOCK_ERASE,block,0x00,0x00};
	SPI_WriteRead(hspix,tx_data,read_buffer,4,0);
	
	return 0;
}

uint8_t W25Qx_Classdef::eraseChip()
{
	writeEnable();
	
	SPI_Write1ReadN(hspix,CHIP_ERASE,read_buffer,0);
	
	return 0;
}

void W25Qx_Classdef::waitBusy()
{
	while(getStatus()&0x01);
}

uint8_t W25Qx_Classdef::writeEnable()
{
	uint8_t tx_data[1] = {WRITE_ENABLE};
	SPI_WriteRead(hspix,tx_data,read_buffer,1,0);
	
	return 0;
}

#if USE_SRML_FATFS
/**
* @brief  init the default w25qx handle
* @param  SPI_HandleTypeDef: used hal spi handle
* @param  W25QxModel_Enumdef: w25qx model specification
* @retval init result
*/
uint8_t W25Qx_Init(SPI_HandleTypeDef * _hspi,W25QxModel_Enumdef _model)
{
	return default_w25qx.init(_hspi,_model);
}

/**
* @brief  write selected sector
* @param  buff: write data buff
* @param  sector: selected sector, from 0 to max(sector_size*block_size)
* @param  sector_count: number of sectors to write
* @retval 0
*/
uint8_t W25Qx_Write_Sector(uint8_t * buff,uint16_t sector,uint8_t sector_count)
{
	for(size_t i = 0;i < sector_count;i++)
	{
		default_w25qx.eraseSector(sector/16,sector%16);
		default_w25qx.continueWrite(sector/16,sector%16,0,0,buff,FLASH_SECTOR_SIZE);
		sector++;
		buff+=FLASH_SECTOR_SIZE;
	}
	buff-=FLASH_SECTOR_SIZE;
	return 0;
}

/**
* @brief  read selected sector
* @param  buff: read data buff
* @param  sector: selected sector, from 0 to max(sector_size*block_size)
* @param  sector_count: number of sectors to write
* @retval 0
*/
uint8_t W25Qx_Read_Sector(uint8_t * buff,uint16_t sector,uint8_t sector_count)
{
	default_w25qx.continueRead(sector/16,sector%16,0,0,buff,FLASH_SECTOR_SIZE*sector_count);
	return 0;
}

#endif
