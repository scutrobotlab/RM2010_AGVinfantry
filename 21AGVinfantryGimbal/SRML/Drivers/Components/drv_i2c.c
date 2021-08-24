/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    drv_i2c.c
  * @author  YDX 2244907035@qq.com
  * @brief   Code for iic driver in STM32 series MCU, supported packaged:
  *          - STM32Cube_FW_F4_V1.24.0.
  *          - STM32Cube_FW_F1_V1.8.0.
  *          - STM32Cube_FW_H7_V1.5.0.
  * @date    2019-11-21
  * @version 1.1
  * @par Change Log:
  * <table>
  * <tr><th>Date        <th>Version  <th>Author         <th>Description
  * <tr><td>2019-06-12  <td> 1.0     <td>mannychen      <td>Creator
  * <tr><td>2019-11-21  <td> 1.1     <td>YDX            <td>add iic device operations.
  * </table>
  *
  ==============================================================================
                            How to use this driver
  ==============================================================================
    @note
      -# 陀螺仪的文件中已经调用了本文件的接口，如果使用陀螺仪固件库的话，不需要
          更改此文件。
      -# 如果使用软件iic读取其它设备的数据，请阅读注释，调用相应外部接口。   
	
    @warning
      -# 本文件的延时函数的延时时间比较难计算，在Sysclk=168Mhz，IIC_Delay_Time=20时，
         IIC_Delay几乎相当于delay_us，如果用户的时间资源比较紧张，可以将IIC_Delay_Time
         减小到15（测试通过），甚至更小，以减小iic读取时间。
		
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
#include "drv_i2c.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* function prototypes -------------------------------------------------------*/

/*<! SDA input config */
void xSDA_IN(IIC_PIN_Typedef *iic_pin)              		    
{
	iic_pin->IIC_GPIO_PORT->MODER&=~(3<<(iic_pin->IIC_SDA_PIN_NUM*2));
	iic_pin->IIC_GPIO_PORT->MODER|= (0<<(iic_pin->IIC_SDA_PIN_NUM*2));	    
}

/*<! SDA output config */
void xSDA_OUT(IIC_PIN_Typedef *iic_pin)             		    
{
	iic_pin->IIC_GPIO_PORT->MODER&=~(3<<(iic_pin->IIC_SDA_PIN_NUM*2));
	iic_pin->IIC_GPIO_PORT->MODER|= (1<<(iic_pin->IIC_SDA_PIN_NUM*2)); 		
}

/*<! SCL output level */
void xIIC_SCL(IIC_PIN_Typedef *iic_pin,const char x)        
{
	(x!=0)?HAL_GPIO_WritePin(iic_pin->IIC_GPIO_PORT,iic_pin->IIC_SCL_PIN,GPIO_PIN_SET)\
				:HAL_GPIO_WritePin(iic_pin->IIC_GPIO_PORT,iic_pin->IIC_SCL_PIN,GPIO_PIN_RESET);
}

/*<! SDA output level */
void xIIC_SDA(IIC_PIN_Typedef *iic_pin,const char x)        
{
  (x!=0)?HAL_GPIO_WritePin(iic_pin->IIC_GPIO_PORT,iic_pin->IIC_SDA_PIN,GPIO_PIN_SET)\
				:HAL_GPIO_WritePin(iic_pin->IIC_GPIO_PORT,iic_pin->IIC_SDA_PIN,GPIO_PIN_RESET);	
}

/*<! read SDA */
GPIO_PinState xREAD_SDA(IIC_PIN_Typedef *iic_pin)
{
    GPIO_PinState bitstatus;
	
    if((iic_pin->IIC_GPIO_PORT->IDR & iic_pin->IIC_SDA_PIN) != (uint32_t)GPIO_PIN_RESET)
    {
        bitstatus = GPIO_PIN_SET;
    }
    else
    {
        bitstatus = GPIO_PIN_RESET;
    }
    return bitstatus;	
}

/**
  * @brief  iic delay
  * @param  m: time for delay, 1 represent nearly 1us
  * @retval void
  */
void IIC_Delay(uint8_t m)
{
  uint8_t i = 0;    
  while(m--)
  {
    i = IIC_Delay_Time;
    while(i--);
  }
}

/**
  * @brief  iic initialization
  * @param  hiic: iic handler
  * @retval void
  */
void IIC_Init(IIC_PIN_Typedef *iic_pin)
{ 
	/* SCL High */
	xIIC_SCL(iic_pin,1);

	/* SDA High */  
	xIIC_SDA(iic_pin,1);
}

/**
  * @brief  iic start signal
  * @param  hiic: iic handler
  * @retval void
  */
void IIC_Start(IIC_PIN_Typedef *iic_pin)
{  
	/* SDA Output */
	xSDA_OUT(iic_pin);   
	xIIC_SDA(iic_pin, 1);  
	xIIC_SCL(iic_pin, 1);
	IIC_Delay(1);
	
	/* START: when CLK is high,DATA change form high to low */
	xIIC_SDA(iic_pin, 0);  
	IIC_Delay(1);
	
	/* Clamp I2C bus, prepare to send or receive data */
	xIIC_SCL(iic_pin, 0);  
}

/**
  * @brief  iic stop signal
  * @param  hiic: iic handler
  * @retval void
  */
void IIC_Stop(IIC_PIN_Typedef *iic_pin)
{ 
	/* SDA Output */
	xSDA_OUT(iic_pin);   
	xIIC_SCL(iic_pin, 0);  
	
	/* STOP: when CLK is high DATA change form low to high */
	xIIC_SDA(iic_pin, 0);  
	IIC_Delay(1);
	xIIC_SCL(iic_pin, 1);  
	IIC_Delay(1);
	
	/* Send I2C bus stop signal */
	xIIC_SDA(iic_pin, 1);  
}

/**
  * @brief  iic wait ack signal
  * @param  hiic: iic handler
  * @retval 1,fail
  *         0,success
  */
unsigned char IIC_Wait_Ack(IIC_PIN_Typedef *iic_pin)
{
	unsigned char ucErrTime=0;
	
  /* SDA Input */
  xSDA_IN(iic_pin);   
  xIIC_SDA(iic_pin, 1);

  IIC_Delay(1);
  xIIC_SCL(iic_pin, 1);
  IIC_Delay(1);

  while(xREAD_SDA(iic_pin))
  {
    ucErrTime++;
    if(ucErrTime>250)
    {
      IIC_Stop(iic_pin);
      return 1;
    }
  }
  xIIC_SCL(iic_pin, 0);
  return 0;
}

/**
  * @brief  iic produce ack signal
  * @param  hiic: iic handler
  * @retval void
  */
void IIC_Ack(IIC_PIN_Typedef *iic_pin)
{
	xIIC_SCL(iic_pin, 0); 
	xSDA_OUT(iic_pin);   
	xIIC_SDA(iic_pin, 0);  
	IIC_Delay(1);
	xIIC_SCL(iic_pin, 1);  
	IIC_Delay(1);
	xIIC_SCL(iic_pin, 0);  
}

/**
  * @brief  iic produce no ack signal
  * @param  hiic: iic handler
  * @retval void
  */
void IIC_NAck(IIC_PIN_Typedef *iic_pin)
{  
	xIIC_SCL(iic_pin, 0); 
	xSDA_OUT(iic_pin);   
	xIIC_SDA(iic_pin, 1);  
	IIC_Delay(1);
	xIIC_SCL(iic_pin, 1);  
	IIC_Delay(1);
	xIIC_SCL(iic_pin, 0);  
}

/**
  * @brief  iic send a byte
  * @param  hiic: iic handler
  * @param  txd: the byte to be sent 
  * @retval void
  */
void IIC_Send_Byte(IIC_PIN_Typedef *iic_pin,unsigned char txd)
{
  unsigned char t;
	xSDA_OUT(iic_pin);  
	xIIC_SCL(iic_pin, 0);
	for(t=0;t<8;t++)
	{
		xIIC_SDA(iic_pin, (txd&0x80)>>7);
		txd<<=1;
		
		/* All three delays are necessary for TEA5767 */
		IIC_Delay(1);   
		xIIC_SCL(iic_pin, 1);
		IIC_Delay(1);
		xIIC_SCL(iic_pin, 0);
		IIC_Delay(1);
	}
}

/**
  * @brief  iic read a byte
  * @param  hiic: iic handler
  * @param  ack: 0,send no ack signal
  *              1,send ack signal
  * @retval void
  */
unsigned char IIC_Read_Byte(IIC_PIN_Typedef *iic_pin,unsigned char ack)
{
	unsigned char i,receive=0;
    
	/* SDA Input */
	xSDA_IN(iic_pin);
	
	for(i=0;i<8;i++ )
	{
		xIIC_SCL(iic_pin, 0);
		IIC_Delay(1);
		xIIC_SCL(iic_pin, 1);
		receive<<=1;
		if(xREAD_SDA(iic_pin))
			receive++;
		IIC_Delay(1);
	}
	if(!ack)
	{
		/* send no ack signal */
		IIC_NAck(iic_pin);
	}
	else 
	{
		/* send ack signal */
		IIC_Ack(iic_pin);
	}
	/* Return the byte received */
	return receive;	
}

/**
  * @brief  iic write a byte to the register of the device
  * @param  hiic: iic handler
  * @param  addr: the address of the device
  * @param  reg: the address of the register(offset address)
  * @param  data: the byte to be sent 
  * @retval 0,success
  *         1,fail
  */
unsigned char IIC_Device_Write_Byte(IIC_PIN_Typedef *iic_pin, unsigned char addr, unsigned char reg, unsigned char data)
{
  IIC_Start(iic_pin);
	
	/* send the address of the device and the write command */
	IIC_Send_Byte(iic_pin, (addr<<1)|0);
	if(IIC_Wait_Ack(iic_pin))	    
	{
		IIC_Stop(iic_pin);
		return 1;
	}
	/* send the address of the register */
  IIC_Send_Byte(iic_pin,reg);	
  IIC_Wait_Ack(iic_pin);		
	
  /* send the data to the register */    
	IIC_Send_Byte(iic_pin,data);   
	if(IIC_Wait_Ack(iic_pin))	    
	{
		IIC_Stop(iic_pin);
		return 1;
	}
    IIC_Stop(iic_pin);
	return 0;
}

/**
  * @brief  iic read a byte of the register of the device
  * @param  hiic: iic handler
  * @param  addr: the address of the device
  * @param  reg: the address of the register(offset address)
  * @retval the data read        
  */
unsigned char IIC_Device_Read_Byte(IIC_PIN_Typedef *iic_pin,unsigned char addr,unsigned char reg)
{
	unsigned char res;
  IIC_Start(iic_pin);
	
	/* send the address of the device and the write command */
	IIC_Send_Byte(iic_pin,(addr<<1)|0);
	IIC_Wait_Ack(iic_pin);		
  
	/* send the address of the register */
  IIC_Send_Byte(iic_pin,reg);	
  IIC_Wait_Ack(iic_pin);		
  IIC_Start(iic_pin);
	
	/* send the address of the device and the read command */
	IIC_Send_Byte(iic_pin,(addr<<1)|1);
    IIC_Wait_Ack(iic_pin);		
  
	/* read data and send no ack signal */
	res = IIC_Read_Byte(iic_pin,0);
	
	/* send a stop signal */
  IIC_Stop(iic_pin);			
	return res;
}

/**
  * @brief  iic continuous write to the register of the device 
  * @param  hiic: iic handler
  * @param  addr: the address of the device
  * @param  reg: the address of the register(offset address)
  * @param  len: write length
  * @param  buf: data buffer
  * @retval 0,success
  *         1,fail
  */
unsigned char IIC_Device_Write_Len(IIC_PIN_Typedef *iic_pin, unsigned char addr,unsigned char reg,unsigned char len,unsigned char *buf)
{
	unsigned char i;
    IIC_Start(iic_pin);
	
	/* send the address of the device and the write command */
	IIC_Send_Byte(iic_pin,(addr<<1)|0);
	if(IIC_Wait_Ack(iic_pin))	
	{
		IIC_Stop(iic_pin);
		return 1;
	}
	
	/* send the address of the register */
    IIC_Send_Byte(iic_pin,reg);	
    IIC_Wait_Ack(iic_pin);		
	for(i=0;i<len;i++)
	{
		/* send the data to the register */
		IIC_Send_Byte(iic_pin,buf[i]);	
		if(IIC_Wait_Ack(iic_pin))		
		{
			IIC_Stop(iic_pin);
			return 1;
		}
	}
    IIC_Stop(iic_pin);
	return 0;
}

/**
  * @brief  iic continuous read of the register of the device
  * @param  hiic: iic handler
  * @param  addr: the address of the device
  * @param  reg: the address of the register(offset address)
  * @param  len: read length
  * @param  buf: data buffer
  * @retval 0,success
  *         1,fail
  */
unsigned char IIC_Device_Read_Len(IIC_PIN_Typedef *iic_pin, unsigned char addr,unsigned char reg,unsigned char len,unsigned char *buf)
{
 	IIC_Start(iic_pin);
	
	/* send the address of the device and the write command */
	IIC_Send_Byte(iic_pin,(addr<<1)|0);
	if(IIC_Wait_Ack(iic_pin))	
	{
		IIC_Stop(iic_pin);
		return 1;
	}
	
	/* send the address of the register */
  IIC_Send_Byte(iic_pin,reg);	
  IIC_Wait_Ack(iic_pin);		
  IIC_Start(iic_pin);
	
	/* send the address of the device and the read command */
	IIC_Send_Byte(iic_pin,(addr<<1)|1);
  IIC_Wait_Ack(iic_pin);		
	while(len)
	{
		/* end of the reading, send no ack signal */
		if(len==1)*buf=IIC_Read_Byte(iic_pin,0);
		else *buf=IIC_Read_Byte(iic_pin,1);		
		len--;
		buf++;
	}
	/* produce a stop signal */
  IIC_Stop(iic_pin);	
	return 0;
}

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
