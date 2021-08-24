/**
******************************************************************************
* Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
* @file    power_ctrl.h
* @author  Kainan.Su 15013073869
* @brief   Header file of power_ctrl.
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

#ifndef __POWERCTRL_H_
#define __POWERCTRL_H_
#ifdef __cplusplus
/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Private macros ------------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
enum E_ENABLE{
	__DISABLE = 0,
	__ENABLE	= 1
};

enum E_LOOP{
	REAL_POWER_LOOP  = 0,		/* !底盘!有电流采样板(英雄和步兵)，使用电流采样板的采样功率进行电机功率控制 */
	REMAIN_ENERGY_LOOP = 1	/* !底盘!无电容与电流采样板(哨兵)，使用裁判系统供电，使用剩余能量进行电机功率控制 */
};

template<typename Type>
Type _PowerCtrl_Constrain(Type input,Type min,Type max){
  if (input <= min)
    return min;
  else if(input >= max)
    return max;
  else return input;
}

/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
class PowerCtrl_ClassDef
{
private:
	uint8_t motor_num; 					/*<! 底盘电机数量，舵轮底盘 = 8，麦轮底盘 = 4  */
	E_ENABLE cap_charge_enable; /*<! 电容充电开关 */
	E_LOOP control_loop;				/*<! 实时功率环/剩余能量环 */
	uint8_t ctrl_period;				/*<! 控制周期,功率预测用 单位ms */
	int max_current_out;			  /*<! 最大电流输出值，建议设置和电调最大电流值一致，如C620为16384 */
  
	float cap_charge_power;		  /*<! 电容允许充电功率 */	
	float lim_scale;						/*<! 限幅比例，每个电机输出应当乘以这个值 */
	
	float(*capChargeController)(const float current, const float target);
	float(*motorLimitController)(const float current, const float target);
	
	void Update(float _RF_power,float _motor_power, float _remain_energy);
	void Calc_motorLimit(int *motot_out_raw);	
	void Calc_capChargePower();

  public: 
	PowerCtrl_ClassDef(uint8_t _motor_num, E_LOOP _control_loop, E_ENABLE _cap_charge_enable, uint8_t _ctrl_period, float _max_current_out = 16384){	
		motor_num = _motor_num;
		control_loop = _control_loop;
		cap_charge_enable = _cap_charge_enable;
		ctrl_period = _ctrl_period;
		max_current_out = _max_current_out;			
	}
		
	/*<! 功率相关变量 为了方便调试 放在public */
	float RF_power;
	float motor_power;
	float remain_energy;
	float motor_power_target;
	float RF_power_target;
	float remain_energy_target;	
	
	void Load_capChargeController(float(*pFunc)(const float current, const float target));
	void Load_motorLimitController(float(*pFunc)(const float current, const float target));		

	/* 主函数 */
	void Control(float _RF_power,float _motor_power, float _remain_energy, int *motor_out_raw);
	
	/* 设定目标参数 */
	void Set_PE_Target(float _RF_power_target, float _motor_power_target, float _remain_energy_target);

	/* 获得电容充电供电 */    
	float Get_capChargePower(void);
	
	/* 获得功率控制值 */
	float Get_limScale(void);

};	
#endif
#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
