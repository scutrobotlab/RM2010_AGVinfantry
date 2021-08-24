/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    mpu6050_config.cpp
  * @author  YDX 2244907035@qq.com
  * @brief   Code for MPU6050 config.
  * @date    2020-04-02
  * @version 1.1
  * @par Change Log:
  * <table>
  * <tr><th>Date        <th>Version  <th>Author     <th>Description
  * <tr><td>2018-10-18  <td> 1.0     <td>mannychen  <td>Creator
  * <tr><td>2019-11-21  <td> 1.1     <td>YDX        <td>remove config macro.
  * <tr><td>2020-04-02  <td> 1.1     <td>YDX        <td>Reduce parameters in initialization function and add notes. 
  * </table>
  *
  ==============================================================================
                              How to use this driver  
  ==============================================================================
    @note
      -# 初始化
	     调用`MPU6050_Init()`进行初始化
         e.g:
         MPU6050_Init(GPIOB, GPIO_PIN_6, GPIO_PIN_7);
         
      -# 获取角速度和加速度原始数据
	     调用`MPU_Get_Gyroscope()`获取角速度
         e.g:
         if(!MPU_Get_Gyroscope(&MPU6050_IIC_PIN, &MPUData.gx, &MPUData.gy, &MPUData.gz))
         {
            MPUData.gx -= MPUData.gxoffset;
            MPUData.gy -= MPUData.gyoffset;
            MPUData.gz -= MPUData.gzoffset;
         }
		 
		 调用`MPU_Get_Accelerometer()`获取加速度
         e.g:
         MPU_Get_Accelerometer(&MPU6050_IIC_PIN,&MPUData.ax,&MPUData.ay,&MPUData.az);

      -# 获取角度
	     调用`mpu_dmp_get_data()`获取角度	 
         e.g:
         mpu_dmp_get_data(&MPUData.roll,&MPUData.pitch,&MPUData.yaw);
		 
    @warning	
      -# 添加预编译宏`USE_FULL_ASSERT`可以启用断言检查。
      -# 需要`SRML`的`drv_i2c`支持。
      -# 软件IIC的IO口需要配置成：超高速开漏上拉模式。
	  
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
#include "mpu6050_config.h"
#include "stdio.h"
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
IIC_PIN_Typedef MPU6050_IIC_PIN;
MPUData_Typedef MPUData;

/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  MPU6050 initialization
  * @param  gpiox: MPU6050 iic gpio port
  * @param  scl_pinx: MPU6050 iic scl pin
  * @param  sda_pinx: MPU6050 iic sda pin
  * @retval 0:success
  *         1:fail   
  */
uint8_t MPU6050_Init(GPIO_TypeDef *gpiox, uint32_t scl_pinx, uint32_t sda_pinx)
{
    /* MPU6050_IIC initialization */
    MPU6050_IIC_PIN.IIC_GPIO_PORT = gpiox;
    MPU6050_IIC_PIN.IIC_SCL_PIN = scl_pinx;
    MPU6050_IIC_PIN.IIC_SDA_PIN = sda_pinx;

    if (scl_pinx == GPIO_PIN_0)
        MPU6050_IIC_PIN.IIC_SCL_PIN_NUM = 0;
    else if (scl_pinx == GPIO_PIN_1)
        MPU6050_IIC_PIN.IIC_SCL_PIN_NUM = 1;
    else if (scl_pinx == GPIO_PIN_2)
        MPU6050_IIC_PIN.IIC_SCL_PIN_NUM = 2;
    else if (scl_pinx == GPIO_PIN_3)
        MPU6050_IIC_PIN.IIC_SCL_PIN_NUM = 3;
    else if (scl_pinx == GPIO_PIN_4)
        MPU6050_IIC_PIN.IIC_SCL_PIN_NUM = 4;
    else if (scl_pinx == GPIO_PIN_5)
        MPU6050_IIC_PIN.IIC_SCL_PIN_NUM = 5;
    else if (scl_pinx == GPIO_PIN_6)
        MPU6050_IIC_PIN.IIC_SCL_PIN_NUM = 6;
    else if (scl_pinx == GPIO_PIN_7)
        MPU6050_IIC_PIN.IIC_SCL_PIN_NUM = 7;
    else if (scl_pinx == GPIO_PIN_8)
        MPU6050_IIC_PIN.IIC_SCL_PIN_NUM = 8;
    else if (scl_pinx == GPIO_PIN_9)
        MPU6050_IIC_PIN.IIC_SCL_PIN_NUM = 9;
    else if (scl_pinx == GPIO_PIN_10)
        MPU6050_IIC_PIN.IIC_SCL_PIN_NUM = 10;
    else if (scl_pinx == GPIO_PIN_11)
        MPU6050_IIC_PIN.IIC_SCL_PIN_NUM = 11;
    else if (scl_pinx == GPIO_PIN_12)
        MPU6050_IIC_PIN.IIC_SCL_PIN_NUM = 12;
    else if (scl_pinx == GPIO_PIN_13)
        MPU6050_IIC_PIN.IIC_SCL_PIN_NUM = 13;
    else if (scl_pinx == GPIO_PIN_14)
        MPU6050_IIC_PIN.IIC_SCL_PIN_NUM = 14;
    else if (scl_pinx == GPIO_PIN_15)
        MPU6050_IIC_PIN.IIC_SCL_PIN_NUM = 15;
    else
        return 1;

    if (sda_pinx == GPIO_PIN_0)
        MPU6050_IIC_PIN.IIC_SDA_PIN_NUM = 0;
    else if (sda_pinx == GPIO_PIN_1)
        MPU6050_IIC_PIN.IIC_SDA_PIN_NUM = 1;
    else if (sda_pinx == GPIO_PIN_2)
        MPU6050_IIC_PIN.IIC_SDA_PIN_NUM = 2;
    else if (sda_pinx == GPIO_PIN_3)
        MPU6050_IIC_PIN.IIC_SDA_PIN_NUM = 3;
    else if (sda_pinx == GPIO_PIN_4)
        MPU6050_IIC_PIN.IIC_SDA_PIN_NUM = 4;
    else if (sda_pinx == GPIO_PIN_5)
        MPU6050_IIC_PIN.IIC_SDA_PIN_NUM = 5;
    else if (sda_pinx == GPIO_PIN_6)
        MPU6050_IIC_PIN.IIC_SDA_PIN_NUM = 6;
    else if (sda_pinx == GPIO_PIN_7)
        MPU6050_IIC_PIN.IIC_SDA_PIN_NUM = 7;
    else if (sda_pinx == GPIO_PIN_8)
        MPU6050_IIC_PIN.IIC_SDA_PIN_NUM = 8;
    else if (sda_pinx == GPIO_PIN_9)
        MPU6050_IIC_PIN.IIC_SDA_PIN_NUM = 9;
    else if (sda_pinx == GPIO_PIN_10)
        MPU6050_IIC_PIN.IIC_SDA_PIN_NUM = 10;
    else if (sda_pinx == GPIO_PIN_11)
        MPU6050_IIC_PIN.IIC_SDA_PIN_NUM = 11;
    else if (sda_pinx == GPIO_PIN_12)
        MPU6050_IIC_PIN.IIC_SDA_PIN_NUM = 12;
    else if (sda_pinx == GPIO_PIN_13)
        MPU6050_IIC_PIN.IIC_SDA_PIN_NUM = 13;
    else if (sda_pinx == GPIO_PIN_14)
        MPU6050_IIC_PIN.IIC_SDA_PIN_NUM = 14;
    else if (sda_pinx == GPIO_PIN_15)
        MPU6050_IIC_PIN.IIC_SDA_PIN_NUM = 15;
    else
        return 1;

    /* MPU6050 initialization */
    MPU_Init(&MPU6050_IIC_PIN);
    if (mpu_dmp_init() != 0)
    {
        HAL_Delay(100);
        __set_FAULTMASK(1); //reset
        NVIC_SystemReset();
    }
    MPU_Get_Gyroscope_Init(&MPU6050_IIC_PIN, &MPUData.gxoffset, &MPUData.gyoffset, &MPUData.gzoffset);
    return 0;
}

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
