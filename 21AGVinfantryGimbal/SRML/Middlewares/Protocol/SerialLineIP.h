/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    SerialLineIP.h
  * @author  BigeYoung  SCUT.BigeYoung@gmail.com
  * @brief   SerialLineIP is an easy serial protocol for Data-Link-Layer.This file
  *          provide functions for framing and transparent transmission.
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
#ifndef _SERIAL_LINE_IP_H
#define _SERIAL_LINE_IP_H
#ifdef  __cplusplus
/* Includes ------------------------------------------------------------------*/
#include <stdint.h> 
#include <vector>

/* Private macros ------------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
namespace SerialLineIP
{
/** @addtogroup SLIP_Special_Character
  * @{
  */
const uint8_t END = 0xC0;     /*!< indicates end of packet */
const uint8_t ESC = 0xDB;     /*!< indicates byte stuffing */
const uint8_t ESC_END = 0xDC; /*!< ESC ESC_END means END data byte */
const uint8_t ESC_ESC = 0xDD; /*!< ESC ESC_ESC means ESC data byte */
/**
  * @}
  */
	
/* Exported function declarations --------------------------------------------*/	
/** @addtogroup SLIP_Functions
  * @{
  */
std::vector<uint8_t> Pack(const void *const p_PDU, int PDU_len);
std::vector<uint8_t> Unpack(const void *const p_SDU, int SDU_len);
/**
  * @}
  */

} 
#endif
#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
