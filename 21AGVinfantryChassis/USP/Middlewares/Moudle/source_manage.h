/**
******************************************************************************
* Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
* @file    SourceManage.h
* @author  苏锴南 15013073869
* @brief   Header file of SourceManage.
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

#ifndef __SOURCE_MANAGE_H
#define __SOURCE_MANAGE_H
/*Include------------------------------------------------------------------------*/
#include "main.h"//使用到gpio.h  还没想好放在哪里
#include "stdint.h"
#include  <string.h>

#ifdef __cplusplus

#define LITTLE_CAP	1		/* 超级电容为2000J或者10000J，对应的充电电压等是不同的 */
	
#if LITTLE_CAP
#define CAPVOL_LOW 			13.0f	//电容电压低
#define CAPVOL_HIGH 		23.8f	//电容电压高
#define BOOSTCHARGE_dV 	4.0f	//开启升压板的压差
#define LOWCURRENT_VOL 	20.5f	//开启小电流充电的电压
#define LOWCURRENT	 		4.5f	//小电流的值
#else
#define CAPVOL_LOW 			12.0f	//电容电压低
#define CAPVOL_HIGH 		19.0f	//电容电压高
#define BOOSTCHARGE_dV 	5.0f	//开启升压板的压差
#define LOWCURRENT_VOL 	18.0f	//开启小电流充电的电压
#define LOWCURRENT	 		2.0f	//小电流的值
#endif
	
	
#define SWITCH_ON GPIO_PIN_SET //合上开关
#define SWITCH_OFF GPIO_PIN_RESET //断开开关
	 
#define G1_SWITCH_PIN GPIO_PIN_0
#define G2_SWITCH_PIN GPIO_PIN_1
#define G3_SWITCH_PIN GPIO_PIN_3
#define G4_SWITCH_PIN GPIO_PIN_2
#define G5_SWITCH_PIN GPIO_PIN_4
#define G_SWITCH_PORT GPIOC
	 
#define CHARGE_CTRL_Pin GPIO_PIN_0
#define CHARGE_CTRL_GPIO_Port GPIOB	 

/* ADC_DMA读取到数组的位置 */
#define VOL_IN	3
#define CUR_IN	4
#define VOL_CAP	0
#define VOL_OUT	5
#define CUR_OUT	2	/* 20年开始全部使用大量程，不再使用小量程，所以命名不再区分 */

#define Convert_To_RealVol(ADC_Val) ADC_Val*(3.3f/4096.0f)*11		/* 11为电阻比 */
#define Convert_To_RealCur_IN(ADC_Val) ADC_Val*(3.3f/4096.0f)*(1/2e-3)/100.0		/* I = U/R 150.5为差分放大器倍数 */
#define Convert_To_RealCur_Out(ADC_Val) ADC_Val*(3.3f/4096.0f)*(1/2e-3)/50.0
#define Calc_RealPower(voltage, current) voltage*current

typedef enum  
{
	Block_LOW = 0,
	Block_MID,
	Block_HIGH,
}_3_Block_e;//状态检测建

typedef struct 
{
	uint8_t statusCnt;
	_3_Block_e statusFlag;
}statusCheck_t;//状态检测建

typedef enum  
{
	LOW = 0,
	MIDD,
	FULL,
}capStatus_e;//电容状态：放电/充电

typedef enum 
{
	CLOSE = 0,
	OPEN
}boostBoardStatus_e;

typedef struct 
{
	float vol_In;
	float vol_Cap;
	float vol_Out;
}voltage_t;

typedef struct 
{
	float cur_In;
	float cur_Out;
}current_t;

typedef struct 
{
	float pow_In;
	float pow_motor;
	float pow_Charge;
}power_t;

/* 电容对象分为放电(电压)和充电两个部分
	应用层通过读取这些变量使用相应策略 */
typedef struct 
{
	float Voltage;
	float chargePower_Now;
	float chargePower_Set;
	float charge_DAC_Value;
	capStatus_e volStatus;	
	capStatus_e chargeStatus;
	statusCheck_t volStatusCheck;
	statusCheck_t chargeStatusCheck;	
}capObj_t;

typedef enum 
{
	BAT,
	CAP
}DischargeMode;

typedef struct 
{
	uint16_t ID;
	uint8_t msg_send[8];
}canMsg_t;

class C_SourceManage_Classdef
{
private:
	voltage_t voltage; 
	current_t current;
	DischargeMode Mode;

	void switchCtrl(uint8_t _switchNumber,GPIO_PinState switchStatus);

	void ADC_To_Real(uint32_t *_ADCReadBuff);
	void Calc_Power();
	
	/* 电容电压检测 */
	void Check_CapStatus();	
	void Block_Check(float min,float max,float real, statusCheck_t* p_statusCheck, int32_t cnt);	
	/* 电容充电 	*/
	void Set_ChargeCurrent();
	void StopCharge();
	void StartCharge();

public:
	C_SourceManage_Classdef(DischargeMode _mode){
		Mode = _mode;
	}
	boostBoardStatus_e BAT_BOOST; 	/* 电池升压板 */
	boostBoardStatus_e ESC_BOOST; 	/* 电调升压板 */
	capObj_t capObj;								/* 电容对象 */
	canMsg_t ESC_Boost_Board_Msg;
	canMsg_t BAT_Boost_Board_Msg;

	void SourceManage_Init();
	/* ADC数据更新 */
	void Update(uint32_t *_ADCReadBuff);
	void BATToCAP();	
	void ALLCLOSE();
	void CAPToBAT();
	void Set_Mode(DischargeMode _mode);
	/* 电容管理 --电容电压检测、充电状态检测 */
	void Cap_Manage(void);
	void Cap_Boot_Manage();/* 电容启动管理 */
	/* 设定充电功率 */
	void Set_ChargePower(float inputPower);
	
	/* 升压板管理 */
	void BAT_Boost_Manage();
	void ESC_Boost_Manage();
	uint8_t ESC_Boot_Manage();/* 电调升压板启动管理 */

	/* 底盘电源管理主函数 */
	void Manage(void);

	power_t power;

	
};
#endif




#endif
