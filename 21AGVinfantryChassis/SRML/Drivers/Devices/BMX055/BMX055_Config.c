/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    BMX055_Config.c
  * @author  YDX 2244907035@qq.com
  * @brief   Code for BMX055 config.
  * @date    2020-04-02
  * @version 1.1
  * @par Change Log:
  * <table>
  * <tr><th>Date        <th>Version  <th>Author  <th>Description
  * <tr><td>2019-11-21  <td> 1.0     <td>YDX     <td>Use the codes in mpu6050_config.cpp.                                              
  * <tr><td>2020-04-02  <td> 1.1     <td>YDX     <td>Change the external interface functions and add notes.\n
                                                     Reduce parameters in initialization function.
  * </table>
  *
  ==============================================================================
                            How to use this driver  
  ==============================================================================
    @note
      -# 初始化：
	     调用`BMX055_Init()`进行初始化。
	     e.g: 
		 BMX055_Init(GPIOB, GPIO_PIN_10, GPIO_PIN_11);
		
      -# 获取角速度和加速度原始数据：		
	     调用`BMX055_Get_Gyro()`获取角速度。
		 e.g: 
		 BMX055_Get_Gyro(&BMX055_IIC_PIN, &gx, &gy, &gz);	
		 gx -= BMX055.gx_offset;
		 gy -= BMX055.gy_offset;		
		 gz -= BMX055.gz_offset;	
		 
	     调用`BMX055_Get_Acc()`获取加速度。
		 e.g: 
		 BMX055_Get_Acc(&BMX055_IIC_PIN, &ax, &ay, &az);	
		 
		 角速度和加速度的更新频率为1kHz。
		 
	  -# 获取角度： 
         先调用`BMX055_Solve_Data()`解算角度，再调用`BMX055_Get_Angle()`获取角度。
		 e.g:         
		 BMX055_Solve_Data();
		 BMX055_Get_Angle(&roll, &pitch, &yaw);
		 
	     解算角度时，需要以2ms为周期调用`BMX055_Solve_Data()`，对应解算频率为500Hz。
	     若想自定义解算频率，请自行修改Mahony_AHRS.c中的sampleFreq与自定义解算频率对应。
		 可以自行修改Mahony_AHRS.c中的twoKpDef和twoKiDef两个参数以调整解算角度的效果。
		 
    @warning	
      -# 添加预编译宏`USE_FULL_ASSERT`可以启用断言检查。
      -# 需要`SRML`的`drv_i2c`支持。
	  
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
#include "BMX055_Config.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
IIC_PIN_Typedef BMX055_IIC_PIN;

/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/	

/**
  * @brief  BMX055 initialization
  * @param  gpiox: BMX055 iic gpio port
  * @param  scl_pinx: BMX055 iic scl pin
  * @param  sda_pinx: BMX055 iic sda pin
  * @retval 0:success
  *         1:fail
  */
uint8_t BMX055_Init(GPIO_TypeDef* gpiox,uint32_t scl_pinx,uint32_t sda_pinx)
{
    /* BMX055_IIC initialization */
    BMX055_IIC_PIN.IIC_GPIO_PORT = gpiox;
    BMX055_IIC_PIN.IIC_SCL_PIN = scl_pinx;
    BMX055_IIC_PIN.IIC_SDA_PIN = sda_pinx;
	
	if(scl_pinx == GPIO_PIN_0)
        BMX055_IIC_PIN.IIC_SCL_PIN_NUM = 0;
	else if(scl_pinx == GPIO_PIN_1)
	    BMX055_IIC_PIN.IIC_SCL_PIN_NUM = 1;
	else if(scl_pinx == GPIO_PIN_2)
	    BMX055_IIC_PIN.IIC_SCL_PIN_NUM = 2;	
	else if(scl_pinx == GPIO_PIN_3)
	    BMX055_IIC_PIN.IIC_SCL_PIN_NUM = 3;	
	else if(scl_pinx == GPIO_PIN_4)
	    BMX055_IIC_PIN.IIC_SCL_PIN_NUM = 4;	
	else if(scl_pinx == GPIO_PIN_5)
	    BMX055_IIC_PIN.IIC_SCL_PIN_NUM = 5;		
	else if(scl_pinx == GPIO_PIN_6)
	    BMX055_IIC_PIN.IIC_SCL_PIN_NUM = 6;	
	else if(scl_pinx == GPIO_PIN_7)
	    BMX055_IIC_PIN.IIC_SCL_PIN_NUM = 7;		
	else if(scl_pinx == GPIO_PIN_8)
	    BMX055_IIC_PIN.IIC_SCL_PIN_NUM = 8;		
	else if(scl_pinx == GPIO_PIN_9)
	    BMX055_IIC_PIN.IIC_SCL_PIN_NUM = 9;		
	else if(scl_pinx == GPIO_PIN_10)
	    BMX055_IIC_PIN.IIC_SCL_PIN_NUM = 10;		
	else if(scl_pinx == GPIO_PIN_11)
	    BMX055_IIC_PIN.IIC_SCL_PIN_NUM = 11;	
	else if(scl_pinx == GPIO_PIN_12)
	    BMX055_IIC_PIN.IIC_SCL_PIN_NUM = 12;		
	else if(scl_pinx == GPIO_PIN_13)
	    BMX055_IIC_PIN.IIC_SCL_PIN_NUM = 13;		
	else if(scl_pinx == GPIO_PIN_14)
	    BMX055_IIC_PIN.IIC_SCL_PIN_NUM = 14;	
	else if(scl_pinx == GPIO_PIN_15)
	    BMX055_IIC_PIN.IIC_SCL_PIN_NUM = 15;	
    else
        return 1;
	
	if(sda_pinx == GPIO_PIN_0)
        BMX055_IIC_PIN.IIC_SDA_PIN_NUM = 0;
	else if(sda_pinx == GPIO_PIN_1)
        BMX055_IIC_PIN.IIC_SDA_PIN_NUM = 1;	
	else if(sda_pinx == GPIO_PIN_2)
        BMX055_IIC_PIN.IIC_SDA_PIN_NUM = 2;	
	else if(sda_pinx == GPIO_PIN_3)
        BMX055_IIC_PIN.IIC_SDA_PIN_NUM = 3;	
	else if(sda_pinx == GPIO_PIN_4)
        BMX055_IIC_PIN.IIC_SDA_PIN_NUM = 4;	
	else if(sda_pinx == GPIO_PIN_5)
        BMX055_IIC_PIN.IIC_SDA_PIN_NUM = 5;	
	else if(sda_pinx == GPIO_PIN_6)
        BMX055_IIC_PIN.IIC_SDA_PIN_NUM = 6;	
	else if(sda_pinx == GPIO_PIN_7)
        BMX055_IIC_PIN.IIC_SDA_PIN_NUM = 7;	
	else if(sda_pinx == GPIO_PIN_8)
        BMX055_IIC_PIN.IIC_SDA_PIN_NUM = 8;	
	else if(sda_pinx == GPIO_PIN_9)
        BMX055_IIC_PIN.IIC_SDA_PIN_NUM = 9;	
	else if(sda_pinx == GPIO_PIN_10)
        BMX055_IIC_PIN.IIC_SDA_PIN_NUM = 10;	
	else if(sda_pinx == GPIO_PIN_11)
        BMX055_IIC_PIN.IIC_SDA_PIN_NUM = 11;	
	else if(sda_pinx == GPIO_PIN_12)
        BMX055_IIC_PIN.IIC_SDA_PIN_NUM = 12;	
	else if(sda_pinx == GPIO_PIN_13)
        BMX055_IIC_PIN.IIC_SDA_PIN_NUM = 13;	
	else if(sda_pinx == GPIO_PIN_14)
        BMX055_IIC_PIN.IIC_SDA_PIN_NUM = 14;	
	else if(sda_pinx == GPIO_PIN_15)
        BMX055_IIC_PIN.IIC_SDA_PIN_NUM = 15;	
	else
		return 1;
	
	/* BMX055 initialization */
	BMX_Conf(&BMX055_IIC_PIN);
	Gyro_Offset_Init();
	return 0;

}
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
