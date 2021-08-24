/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    Indicator.cpp
  * @author  Lingzi_Xie 1357657340@qq.com
  * @brief   State Indicaotr in Robomaster 2019
  * @date    2021-04-30
  * @version 1.0
  * @par Change Log：
  * <table>
  * <tr><th>Date        	<th>Version  <th>Author    		<th>Description
  * <tr><td>2021-05-28   	<td> 1.0     <td>Lingzi_Xie    	<td> Create Module
  * </table>
  *
  ==============================================================================
                            How to use this Module  
  ==============================================================================
    @note 
		-# 2021年的状态指示灯采用PWM驱动，可以考虑占用云台主控一个小4pin UART口，且设置为TIM的PWM输出
		
			-# Cube MX配置好TIM，PWM输出模式。
			（1）实测对普通定时器，可以取预分频系数为0，ARR初值为104（使PWM频率为800KHz）
			（2）使能TIM的DMA发送，注意DMA传输方向设置为内存 -> 外设，且数据宽度设置为字！
			
			-# 新建Indicator_Classdef实例，并使用构造函数初始化（传入TIM和对应通道DMA的句柄）
			
			-# 调用Change_Singal_RGB()设置指定LED的亮度、颜色；指定LED编号为1~6
			  示例
			  		indicator.Change_Singal_RGB(LED_1|LED_2, 0xff, 0, 0xff, 150);
			
			-# 设置完LED的颜色后，调用Update()对指示灯状态进行更新！
			
		-# 状态指示灯的引脚接线：
			G：地
			V：5V电源
			U：PWM信号

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

#include "Indicator.h"

/* 指定RGB的颜色和亮度 */
void Indicator_Classdef::Change_Singal_RGB(uint8_t led_num, uint8_t r, uint8_t g, uint8_t b, uint8_t strength)
{
	uint32_t rgb_code;
	uint8_t i, num_mask = 0x01;
	
	rgb_code = RGB2BIN(r, g, b, strength);
	
	for(i = 1; i <= 6; i++)
	{
		if((led_num & num_mask) == 1)
			BIN2CODE(i, rgb_code);
		led_num = led_num >> 1;
	}
}

/* 更新所有LED的状态 */
void Indicator_Classdef::Update()
{
	Indicator_LED.ON(buffer);
}

/* 关闭状态指示灯 */
void Indicator_Classdef::Reset()
{
	uint16_t count_i, count_j;
	
	for(count_i = 0;count_i < LED_NUM;count_i++)									//缓存区设置为低电平对应编码
		for(count_j = 0;count_j < RGB_NUM;count_j++)
			buffer[count_i][count_j] = LED_LOW;
	
	Indicator_LED.ON(buffer);
}

/* 生成单个LED的驱动数组 */
uint32_t Indicator_Classdef::RGB2BIN(uint8_t r, uint8_t g, uint8_t b, uint8_t strength)
{
	if(strength >= 1 && strength <= 255)
	{
		r = r / strength;
		b = b / strength;
		g = g / strength;
		return ((g << 16) | (r << 8) | b);		
	}
	
	else return 0;
}

/* 生成单个LED的PWM CCR值，并写到缓存区对应位置 */
void Indicator_Classdef::BIN2CODE(uint8_t led_num, uint32_t rgb_bin)
{
	uint16_t count_i;
	
	for(count_i = 0;count_i < RGB_NUM;count_i++)
	{
		buffer[led_num - 1][count_i] = ((rgb_bin << count_i) & 0x800000) ? LED_HIGH : LED_LOW;
	}
}

/* 电容状态指示 */
void Indicator_Classdef::Cap_Energy_Indicator(float current_volt, float max_volt, float high_volt, float low_volt, uint8_t led_num, uint8_t strenght)
{
	static uint8_t volt_proportion, cap_status, indicator_tick;

	//输入检测
	if(max_volt < high_volt || high_volt < low_volt)
		return;
	
	//电容状态检测
	if(current_volt < low_volt)
		cap_status = CAP_CHARGE;
	if(current_volt > high_volt)
		cap_status = CAP_READY;
	
	//灯珠控制
	if((current_volt*current_volt - low_volt*low_volt)/(max_volt*max_volt - low_volt*low_volt)<0.3 && cap_status != CAP_CHARGE)
	{
		//能量小于30%时，灯变红
		Change_Singal_RGB(led_num, 0xff, 0, 0, strenght);
	}
	else if(cap_status == CAP_CHARGE)
	{
		//正在充电时，红色闪烁
		if(indicator_tick < 50)
			Change_Singal_RGB(led_num, 0xff, 0, 0, strenght);
		else
			Change_Singal_RGB(led_num, 0, 0, 0, strenght);
		
		indicator_tick++;
		indicator_tick %= 100;
	}
	else
	{
		//电容电压高时，灯变绿
		Change_Singal_RGB(led_num, 0, 0xff, 0, strenght);
	}
}
