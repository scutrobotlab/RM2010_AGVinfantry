/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    mpu6050.cpp
  * @author  YDX 2244907035@qq.com
  * @brief   Code for MPU6050.
  * @date    2019-11-21
  * @version 1.1
  * @par Change Log:
  * <table>
  * <tr><th>Date        <th>Version  <th>Author         <th>Description
  * <tr><td>2018-10-18  <td> 1.0     <td>mannychen      <td>Creator
  * <tr><td>2019-11-21  <td> 1.1     <td>YDX            <td>Remove iic functions \n    
  *                                                         add notes.   
  * </table>
  *
  ==============================================================================
                          How to use this driver  
  ==============================================================================
    @note      
	  -# 使用说明在`mpu6050_config.c`。
		
	@warning
	
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
#include "mpu6050.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  初始化MPU6050
  * @param  void
  * @retval 0,成功 
  *         其他,错误代码
  */
unsigned char MPU_Init(IIC_PIN_Typedef *iic_pin)
{
	unsigned char res;
	
	/* 初始化IIC总线 */
	IIC_Init(iic_pin);
	
	IIC_Device_Write_Byte(iic_pin,MPU_ADDR,MPU_PWR_MGMT1_REG,0X80);			//复位MPU6050
    HAL_Delay(100);
	
	IIC_Device_Write_Byte(iic_pin,MPU_ADDR,MPU_PWR_MGMT1_REG,0X00);			//唤醒MPU6050
	MPU_Set_Gyro_Fsr(iic_pin,3);													                //陀螺仪传感器,±2000dps
	MPU_Set_Accel_Fsr(iic_pin,80);												                //加速度传感器,±2g
	MPU_Set_Rate(iic_pin,1000);													                //设置采样率1000Hz
  
	IIC_Device_Write_Byte(iic_pin,MPU_ADDR,MPU_INT_EN_REG,0X00);					//关闭所有中断
	IIC_Device_Write_Byte(iic_pin,MPU_ADDR,MPU_USER_CTRL_REG,0X00);			//I2C主模式关闭
	IIC_Device_Write_Byte(iic_pin,MPU_ADDR,MPU_FIFO_EN_REG,0X00);				//关闭FIFO
	IIC_Device_Write_Byte(iic_pin,MPU_ADDR,MPU_INTBP_CFG_REG,0X80);			//INT引脚低电平有效
	res = IIC_Device_Read_Byte(iic_pin,MPU_ADDR,MPU_DEVICE_ID_REG);
	if(res == MPU_ADDR)                                               //器件ID正确
	{
		IIC_Device_Write_Byte(iic_pin,MPU_ADDR,MPU_PWR_MGMT1_REG,0X01);		//设置CLKSEL,PLL X轴为参考
		IIC_Device_Write_Byte(iic_pin,MPU_ADDR,MPU_PWR_MGMT2_REG,0X00);		//加速度与陀螺仪都工作
		MPU_Set_Rate(iic_pin,1000);												                //设置采样率为50Hz
 	}else return 1;
	return 0;
}


/**
  * @brief  设置MPU6050陀螺仪传感器满量程范围
  * @param  fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
  * @retval 0,success
  *         1,fail
  */
unsigned char MPU_Set_Gyro_Fsr(IIC_PIN_Typedef *iic_pin,unsigned char fsr)
{
	return IIC_Device_Write_Byte(iic_pin,MPU_ADDR,MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围
}

/**
  * @brief  设置MPU6050加速度传感器满量程范围
  * @param  fsr:0,±2g;1,±4g;2,±8g;3,±16g
  * @retval 0,success
  *         1,fail
  */
unsigned char MPU_Set_Accel_Fsr(IIC_PIN_Typedef *iic_pin,unsigned char fsr)
{
	return IIC_Device_Write_Byte(iic_pin,MPU_ADDR,MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围
}

/**
  * @brief  设置MPU6050的数字低通滤波器
  * @param  lpf:数字低通滤波频率(Hz)
  * @retval 0,success
  *         1,fail
  */
unsigned char MPU_Set_LPF(IIC_PIN_Typedef *iic_pin,unsigned short int lpf)
{
	unsigned char data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6;
	return IIC_Device_Write_Byte(iic_pin,MPU_ADDR,MPU_CFG_REG,data);//设置数字低通滤波器
}

/**
  * @brief  设置MPU6050的采样率(假定Fs=1KHz)
  * @param  rate:4-8000(Hz)
  * @retval 0,success
  *         1,fail
  */
unsigned char MPU_Set_Rate(IIC_PIN_Typedef *iic_pin,unsigned short int rate)
{
	unsigned char data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=IIC_Device_Write_Byte(iic_pin,MPU_ADDR,MPU_SAMPLE_RATE_REG,data);	//设置采样频率分频器
 	return MPU_Set_LPF(iic_pin,rate/2);	//自动设置LPF为采样率的一半
}


/**
  * @brief  获取温度值IIC_Device_Read_Len
  * @param  void
  * @retval 温度值(扩大了100倍)
  *         
  */
short MPU_Get_Temperature(IIC_PIN_Typedef *iic_pin)
{
  unsigned char buf[2];
  short raw;
  float temp;
  IIC_Device_Read_Len(iic_pin,MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf);
  raw=((unsigned short int)buf[0]<<8)|buf[1];
  temp=36.53+((double)raw)/340;
  return temp*100;;
}

/**
  * @brief  得到陀螺仪值(原始值)，即角速度值
  * @param  gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
  * @retval 0,success
  *         1,fail
  */
unsigned char MPU_Get_Gyroscope(IIC_PIN_Typedef * iic_pin,short *gx,short *gy,short *gz)
{
  unsigned char buf[6],res;
	
	res=IIC_Device_Read_Len(iic_pin,MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=(short)(((unsigned short int)buf[0]<<8)|buf[1]);
		*gy=(short)(((unsigned short int)buf[2]<<8)|buf[3]);
		*gz=(short)(((unsigned short int)buf[4]<<8)|buf[5]);
	}
  return res;
}


void MPU_Get_Gyroscope_Init(IIC_PIN_Typedef * iic_pin,float *gxoffset, float *gyoffset, float *gzoffset)
{
	unsigned char buf[6];
	short gx,gy,gz=0;
	int i=0,cnt=0,sum_x=0,sum_y=0,sum_z=0;
	
	for(i = 0;i < 1024 ;i++)
	{
		if(IIC_Device_Read_Len(iic_pin,MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf)==0)
		{
      gx=(short)(((unsigned short int)buf[0]<<8)|buf[1]);
      gy=(short)(((unsigned short int)buf[2]<<8)|buf[3]);
      gz=(short)(((unsigned short int)buf[4]<<8)|buf[5]);
			if(i>300)// 前300次数据不用
			{
        sum_x += gx;
        sum_y += gy;
				sum_z += gz;
				cnt ++;
			}
		}
	}
	*gxoffset = ((float)sum_x)/((float)cnt);
  *gyoffset = ((float)sum_y)/((float)cnt);
  *gzoffset = ((float)sum_z)/((float)cnt);
}


unsigned char MPU_Get_Accelerometer(IIC_PIN_Typedef *iic_pin, short *ax,short *ay,short *az)
{
  unsigned char buf[6],res;
	res=IIC_Device_Read_Len(iic_pin,MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((unsigned short int)buf[0]<<8)|buf[1];
		*ay=((unsigned short int)buf[2]<<8)|buf[3];
		*az=((unsigned short int)buf[4]<<8)|buf[5];
	}
    return res;;
}

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
