/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    drv_flash.h
  * @author  chenpeiqi 
  * @brief   Code for flash driver(on chip) in STM32F4 series MCU.
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
#ifndef _FLASH_H_
#define _FLASH_H_

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

#include "string.h"

#ifdef __cplusplus
extern "C" {
#endif
/* Private macros ------------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/	
/* Base address of the Flash sectors */
#define ADDR_FLASH_SECTOR_0 ((uint32_t)0x08000000)  /* Base address of Sector 0, 16 Kbytes   */
#define ADDR_FLASH_SECTOR_1 ((uint32_t)0x08004000)  /* Base address of Sector 1, 16 Kbytes   */
#define ADDR_FLASH_SECTOR_2 ((uint32_t)0x08008000)  /* Base address of Sector 2, 16 Kbytes   */
#define ADDR_FLASH_SECTOR_3 ((uint32_t)0x0800C000)  /* Base address of Sector 3, 16 Kbytes   */
#define ADDR_FLASH_SECTOR_4 ((uint32_t)0x08010000)  /* Base address of Sector 4, 64 Kbytes   */
#define ADDR_FLASH_SECTOR_5 ((uint32_t)0x08020000)  /* Base address of Sector 5, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_6 ((uint32_t)0x08040000)  /* Base address of Sector 6, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_7 ((uint32_t)0x08060000)  /* Base address of Sector 7, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_8 ((uint32_t)0x08080000)  /* Base address of Sector 8, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_9 ((uint32_t)0x080A0000)  /* Base address of Sector 9, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_10 ((uint32_t)0x080C0000) /* Base address of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11 ((uint32_t)0x080E0000) /* Base address of Sector 11, 128 Kbytes */
#define FLASH_END_ADDR       ((uint32_t)0x08100000)

/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function declarations --------------------------------------------*/
extern void flash_erase_address(uint32_t address, uint16_t len);
extern int8_t flash_write_single_address(uint32_t start_address, uint32_t *buf, uint32_t len);
extern int8_t flash_write_muli_address(uint32_t start_address, uint32_t end_address, uint32_t *buf, uint32_t len);
extern void flash_read(uint32_t address, uint32_t *buf, uint32_t len);
extern uint32_t get_sector(uint32_t address);
extern uint32_t get_next_flash_address(uint32_t address);

#ifdef __cplusplus
}
#endif

#endif

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
