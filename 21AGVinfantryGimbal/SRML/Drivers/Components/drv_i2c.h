/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    drv_i2c.h
  * @author  YDX 
  * @brief   Code for iic driver in STM32 series MCU, supported packaged:
  *          - STM32Cube_FW_F4_V1.24.0.
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
#ifndef _DRV_I2C_H
#define _DRV_I2C_H

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
* @brief  subject to the hardware circuit, user should set the communication line of I2C as small as possible, 
*         user can decrease this time to decrease the time of reading data 
*/
#define IIC_Delay_Time 20	
	
/* Private type --------------------------------------------------------------*/
typedef struct 
{
	GPIO_TypeDef * IIC_GPIO_PORT;
	uint32_t IIC_SCL_PIN;
	uint32_t IIC_SDA_PIN;
	uint32_t IIC_SCL_PIN_NUM;
	uint32_t IIC_SDA_PIN_NUM;	
}IIC_PIN_Typedef;

/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function declarations --------------------------------------------*/	
void IIC_Delay(uint8_t m);
void IIC_Init(IIC_PIN_Typedef *iic_pin);                                           
void IIC_Start(IIC_PIN_Typedef *iic_pin);				                  
void IIC_Stop(IIC_PIN_Typedef *iic_pin);	  			                  
void IIC_Send_Byte(IIC_PIN_Typedef *iic_pin,unsigned char txd);			   
unsigned char IIC_Read_Byte(IIC_PIN_Typedef *iic_pin,unsigned char ack); 
unsigned char IIC_Wait_Ack(IIC_PIN_Typedef *iic_pin); 				    
void IIC_Ack(IIC_PIN_Typedef *iic_pin);					                  
void IIC_NAck(IIC_PIN_Typedef *iic_pin);				                 
unsigned char IIC_Device_Write_Byte(IIC_PIN_Typedef *iic_pin, unsigned char addr, unsigned char reg, unsigned char data);
unsigned char IIC_Device_Read_Byte(IIC_PIN_Typedef *iic_pin,unsigned char addr,unsigned char reg);
unsigned char IIC_Device_Write_Len(IIC_PIN_Typedef *iic_pin, unsigned char addr,unsigned char reg,unsigned char len,unsigned char *buf);
unsigned char IIC_Device_Read_Len(IIC_PIN_Typedef *iic_pin, unsigned char addr,unsigned char reg,unsigned char len,unsigned char *buf);

#ifdef __cplusplus
}
#endif

#endif

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
