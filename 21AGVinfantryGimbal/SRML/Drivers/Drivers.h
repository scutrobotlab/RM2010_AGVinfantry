/**
  ******************************************************************************
  * @file    Drivers.h
  * @brief   Header to include all Drivers.
  * @version 0.0.1
  ******************************************************************************
  * MOST IMPORTANTLY, this library is not open source for developers from other
  * schools' robot team currently. Plz make sure your code is under your supervision.
  * 
  * Thank for @mannychen @HuanpengLu and other pioneers who have put forward such 
  * architechure, and the endeavors of all developers.
  * 
  * By downloading, copying, installing or using the software you agree to this license.
  * If you do not agree to this license, do not download, install,
  * copy or use the software.
  * 
  *                          License Agreement
  *                For SCUT RobotLab Middilware Layer Library
  * 
  * Copyright (c) 2019 - ~, SCUT RobotLab Development Team, all rights reserved.
  * 
  * This file includes all of the headers of SRML.
  * 
  * Before using this library, plz make sure that u have read the README document
  * carefully,  
  *    @note
  *     - Plz do not modifiy this file(Except for developer).
  *     - Plz remember to update the version number.
  */
#pragma once
/** @addtogroup Drivers
  * @{
  */
#include <srml_config.h>
/* Devices header begin */
#if USE_SRML_REFEREE
//#include "Devices/referee.h"
#endif
#if USE_SRML_DJI_MOTOR
#include "Devices/motor.h"
#endif
#if USE_SRML_OTHER_MOTOR
#include "Devices/motor_AK80.h"
#endif
#if USE_SRML_DR16
#include "Devices/dr16.h"
#endif
#if USE_SRML_BMX055
#include "Devices/BMX055/BMX055_config.h"
#endif
#if USE_SRML_MPU6050
#include "Devices/MPU6050/mpu6050_config.h"
#endif
#if USE_SRML_W25Qx
#include "Devices/Flash/W25Qx.h"
#endif
#if USE_SRML_FATFS
#include "Devices/Flash/FATFS/diskio.h"
#endif
/* Devices header end */

/* Components header begin */
#if USE_SRML_I2C
#include "Components/drv_i2c.h"
#endif
#if USE_SRML_SPI
#include "Components/drv_spi.h"
#endif
#if USE_SRML_CAN
#include "Components/drv_can.h"
#endif
#if USE_SRML_UART
#include "Components/drv_uart.h"
#endif
#if USE_SRML_TIMER
#include "Components/drv_timer.h"
#endif
#if USE_SRML_FLASH
#include "Components/drv_flash.h"
#endif
/* Components header end */

/**
  * @}
  */

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
