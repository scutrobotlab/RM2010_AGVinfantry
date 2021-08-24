/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    W25Qx.h
  * @author  Anthracene 
  * @brief   Code for driver of SPI flash chip W25Qx
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
#ifndef _W25QX_H
#define _W25QX_H

/* Includes ------------------------------------------------------------------*/
#include "../../Components/drv_spi.h"
#include "../../Components/drv_timer.h"
#include "srml_config.h"
#include <string.h>
	
/* Private macros ------------------------------------------------------------*/
#define MANUFACTURER_ID	0xEF
#define DEVICE_ID_80		0x4014
#define DEVICE_ID_16		0x4015
#define DEVICE_ID_32		0x4016
#define DEVICE_ID_64		0x4017
#define DEVICE_ID_128		0x4018

/* Private type --------------------------------------------------------------*/
#define FLASH_SECTOR_SIZE	4096 /*4KBytes*/
#define FLASH_PAGE_SIZE		256	 /*256Bytes*/

/* Exported macros -----------------------------------------------------------*/
#define WRITE_ENABLE	0x06
#define WRITE_DISABLE	0x04
#define WRITE_STATUS	0x01
#define PAGE_PROGRAM	0x02
#define SECTOR_ERASE	0x20
#define BLOCK_ERASE		0xd8
#define CHIP_ERASE		0xc7
#define READ_STATUS1	0x05
#define READ_STATUS2	0x35
#define READ_DATA			0x03
#define FAST_READ			0x0b

/* Exported types ------------------------------------------------------------*/
enum W25QxModel_Enumdef
{
	W25Q80 = 1,
	W25Q16 = 2,
	W25Q32 = 4,
	W25Q64 = 8,
	W25Q128 = 16,
};

#ifdef __cplusplus

class W25Qx_Classdef
{
public:
	W25Qx_Classdef(){}
	W25Qx_Classdef(SPI_HandleTypeDef * _hspi,W25QxModel_Enumdef _model){
		hspix = _hspi;
		model = _model;
		blockSize = 16 * model;
		sectorSize = 16;
		pageSize = 16;
		byteSize = 256;
	}
	uint8_t init();
	uint8_t init(SPI_HandleTypeDef * _hspi,W25QxModel_Enumdef _model);
	uint8_t readByte(uint8_t block,uint8_t sector,uint8_t page,uint8_t byte);
	uint8_t writeByte(uint8_t block,uint8_t sector,uint8_t page,uint8_t byte,uint8_t data);
	uint8_t continueRead(uint8_t block,uint8_t sector,uint8_t page,uint8_t byte,uint8_t * data,uint16_t length);
	uint8_t continueWrite(uint8_t block,uint8_t sector,uint8_t page,uint8_t byte,uint8_t * data,uint16_t length);
	uint16_t getStatus();
	uint8_t eraseSector(uint8_t block,uint8_t sector);
	uint8_t eraseBlock(uint8_t block);
	uint8_t eraseChip();
	void waitBusy();
private:
	uint8_t writeEnable();

	SPI_HandleTypeDef * hspix;
	W25QxModel_Enumdef model;
	uint8_t write_buffer[300];
	uint8_t read_buffer[256];
	uint16_t blockSize;
	uint16_t sectorSize;
	uint16_t pageSize;
	uint16_t byteSize;

	uint16_t chipStatus;
};

#if USE_SRML_FATFS
extern W25Qx_Classdef default_w25qx;
#endif

#endif
/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/
#if USE_SRML_FATFS
#ifdef __cplusplus
extern "C"{
#endif
	
	uint8_t W25Qx_Init(SPI_HandleTypeDef * _hspi,W25QxModel_Enumdef _model);
	uint8_t W25Qx_Write_Sector(uint8_t * buff,uint16_t sector,uint8_t sector_count);
	uint8_t W25Qx_Read_Sector(uint8_t * buff,uint16_t sector,uint8_t sector_count);

#ifdef __cplusplus
}
#endif
#endif

#endif

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
