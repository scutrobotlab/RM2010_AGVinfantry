/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    PowerCtrl.cpp
  * @author  Kainan.Su 15013073869
  * @brief   Power control for chassis in Robomaster.
  * @date    2019-11-12
  * @version 1.0
  * @par Change Log：
  * <table>
  * <tr><th>Date        	<th>Version  <th>Author    		<th>Description
  * <tr><td>2019-10-29   	<td> 1.0     <td>Kainan    		<td>实现充电开关，以及电机的限幅输出
  * <tr><td>2019-11-24   	<td> 2.0     <td>Kainan    		<td>增加构造函数，控制器外部实现，RF供电和CAP供电模式	\n
																														加入功率限制开关和电容充电计算开关
  * <tr><td>2019-12-22   	<td> 2.1     <td>Kainan    		<td>增加RF供电模式下电机和充电功率的计算	\n
  * <tr><td>2019-02-25   	<td> 2.2     <td>Kainan    		<td>更改接口Set_PE_Max为Set_PE_Target以及一些注释	\n
  * </table>
  *
  ==============================================================================
                            How to use this Module  
  ==============================================================================
    @note 
			-# 只有两种情况：
					1.!底盘!有电流采样板(英雄和步兵)，使用电流采样板的采样功率进行电机功率控制
					2.!底盘!无电容与电流采样板(哨兵)，使用裁判系统供电，使用剩余能量进行电机功率控制
			
			-# 新建"powerCtrl"对象，构造时设置初始参数，选择是否进行限幅和充电计算. \n
			
			-#	加载外部的控制器Load_xxxxController().  
			
      -# 按照配置的运行频率调用 Control().通过Set_xx()设置参数、目标
			
      -# 调用Get_LimScale()获得限幅比例,调用Get_ChargePower()获得允许充电功率
			
		@attention 
			-# 请仔细阅读ReadMe.md
			
			-# 如果一些测试机构不需要用到功率控制，不需要改变其他的内容， \n 
				 只需要在构造函数中设置DISABLE就可以了，此时当读取电机限幅比例会一直都是1.0，相当于不不限幅。
      	 	
    @example
			

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
#include "power_ctrl.h"
#include <stddef.h>	/* 使用NULL */

/* Private define ------------------------------------------------------------*/
#define myabs(x) ((x)>0? (x):(-(x)))

/* Private variables ---------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief Load external controller
 * @note 
 * @param None
 * @retval None
 * @author kainan
 */
void PowerCtrl_ClassDef::Load_capChargeController(float(*pFunc)(const float current, const float target))
{
	capChargeController = pFunc;
}

void PowerCtrl_ClassDef::Load_motorLimitController(float(*pFunc)(const float current, const float target))
{
	motorLimitController = pFunc;
}

/**
 * @brief  功率控制的主函数
 * @param  _RF_power:裁判系统功率
 * @param  _motor_power:电机实时功率
 * @param  _remain_energy:裁判系统剩余能量
 * @param  motor_out_raw:速度环输出值，也即电流值
 * @author kainan
 */ 
void PowerCtrl_ClassDef::Control(float _RF_power,float _motor_power, float _remain_energy, int *motor_out_raw)
{
	/* 更新功率数据 */
	Update(_RF_power,_motor_power, _remain_energy);
	
	/* 限幅值计算 */
	Calc_motorLimit(motor_out_raw);
	
	/* 充电功率计算 */
	if(cap_charge_enable == __ENABLE)
		Calc_capChargePower();
	else
		cap_charge_power = 0;
}

/**
 * @brief  得到电容充电的允许功率
 * @param  None
 * @retval cap_charge_power：电容充电的允许功率
 * @author kainan
 */
float PowerCtrl_ClassDef::Get_capChargePower(void)
{
	return cap_charge_power;
}

/**
 * @brief  得到电机输出限幅比例
 * @param  None
 * @retval lim_scale：限幅比例，每个电机输出应当乘以这个值
 * @author kainan
 */
float PowerCtrl_ClassDef::Get_limScale(void)
{
	return lim_scale;
}


/**
 * @brief  计算电机是否功率限制，并在内部得到限幅值
 * @param  motor_out_raw：电机速度环的输出值，即电流值
 * @retval 电机是否收到功率限制
 * @author kainan
 */

void PowerCtrl_ClassDef::Calc_motorLimit(int *motor_out_raw)
{
	int limit_current_total = 0;/* 最大总电流 */
	
	/* 底盘使用电容供电(英雄和步兵)，使用电路采样板的采样功率进行电机功率控制 */
	if(control_loop == REAL_POWER_LOOP)
	{	
		/* motor_power_target * 10相当于叠加一个前馈量，PID会好调一些 */
		if(motorLimitController != NULL)
			limit_current_total = motor_power_target * 10 + motorLimitController(motor_power, motor_power_target);
	}
	/* 无电流采样板，底盘使用裁判系统供电(哨兵)，使用剩余能量进行电机功率控制 */
	else
	{				
		if(motorLimitController != NULL)		
			limit_current_total = RF_power_target * 10 + motorLimitController(remain_energy, remain_energy_target);			
	}
	
	/* 如果不加限幅，当超功率时，limitation负大进行功率限制，scale也负大 */
	limit_current_total = _PowerCtrl_Constrain(limit_current_total, 300, max_current_out);
	
	float scale = 1.0f;
	float current_sum = 0.0f;
	float motor_current_max = 0.0f;
	
	/* 输出总电流 */
	for(uint8_t i = 0; i < motor_num; i++)
		current_sum += myabs(motor_out_raw[i]);
	
	/* 总电流超过限制值 */
	if(current_sum > limit_current_total)
	{
		scale = limit_current_total / current_sum;	
	}else{
		scale = 1.0f;
	}
	
	lim_scale = scale;
}


/**
 * @brief  计算允许电容充电功率
 * @param  None
 * @retval None
 * @author kainan
 */
void PowerCtrl_ClassDef::Calc_capChargePower(void)
{	
	if(capChargeController != NULL)		
	{
		cap_charge_power = RF_power_target - capChargeController(remain_energy, remain_energy_target); 	
		if(cap_charge_power < 0)
			cap_charge_power = 0;
		else{}		
	}
}

/**
 * @brief  更新功率数据的值
 * @param  电池功率，电机功率，RF剩余能量
 * @retval None
 * @author kainan
 */
void PowerCtrl_ClassDef::Update(float _RF_power,float _motor_power, float _remain_energy)
{
	static float last_remian_energy = 0;
	RF_power = _RF_power;
	motor_power = _motor_power;
	
	/* 裁判系统的数据是否更新 */
	if(_remain_energy != last_remian_energy)
	{		
		remain_energy = _remain_energy;
		last_remian_energy = remain_energy;
	}
	else
		remain_energy += (RF_power_target - RF_power) * ctrl_period / 1000.0f;
	if(remain_energy >= 250.0f)	
		remain_energy = 250.0f;
	else{}
}

/**
 * @brief  设置裁判系统功率，电机功率，剩余能量的目标值
 * @param  _RF_power_target:裁判系统功率目标值
 * @param  _motor_power_target:电机功率目标值
 * @param  _remain_energy_target:剩余能量目标值
 * @retval None
 * @author kainan
 */
void PowerCtrl_ClassDef::Set_PE_Target(float _RF_power_target, float _motor_power_target, float _remain_energy_target)
{
	RF_power_target = _RF_power_target;
	motor_power_target = _motor_power_target;
	remain_energy_target = _remain_energy_target;
}

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
