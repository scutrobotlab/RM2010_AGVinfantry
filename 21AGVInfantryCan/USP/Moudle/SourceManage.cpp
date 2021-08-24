/**
******************************************************************************
* Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
* @file    SourceManage.cpp
* @author  苏锴南 15013073869
* @brief   底盘电源管理
* @date    2019-11-12
* @version 2.0
* @par Change Log：
* <table>
* <tr><th>Date        	<th>Version  <th>Author    		<th>Description
* <tr><td>2019-11-8   	<td> 1.0     <td>苏锴南        <td>
* </table>
*
==============================================================================
										##### How to use this driver #####
==============================================================================
	@note
		-# SourceManage_Init
		-# Update(ADC采集数组)  如果采集顺序变化  要更改.h里面相应的数组位置宏定义
		-# Set_ChargePower(设定充电功率)
		-# Cap_Manage  BAT_Boost_Manage  ESC_Boost_Manage
		
		example：
			void SourceManage_Task(void const * argument)
			{
				TickType_t xLastWakeTime;
				const TickType_t xFrequency = 2;
				xLastWakeTime = xTaskGetTickCount();
				
				static uint32_t ADCReadBuff[8];
				HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADCReadBuff, 7);
				HAL_DAC_Start(&hdac, DAC_CHANNEL_1);

				ChassisSource.SourceManage_Init();	

				for(;;)
				{
					vTaskDelayUntil(&xLastWakeTime,xFrequency);
					更新数据
					SourceManage.Update(ADCReadBuff);
					SourceManage.Set_ChargePower(PowerCtrl.Get_ChargePower());
					
					电容  升压板控制
					SourceManage.Manage();	
				}
			}
			
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


#include "SourceManage.h"
#include "string.h"
/**
 * @brief 更新电压电流等参数
 * @note 
 * @param ADC采集数组 顺序表在.h文件中
 * @retval None
 * @author 苏锴南
 */
void SourceManage_ClassDef::Update(uint32_t *_ADCReadBuff)
{
	ADC_To_Real(_ADCReadBuff);
	Calc_Power();
	capObj.Voltage = voltage.vol_Cap;
	capObj.chargePower_Now = power.pow_Charge;
}

/**
 * @brief 把ADC值转换成电压电流真实值
 * @note 
 * @param ADC采集数组
 * @retval None
 * @author 苏锴南
 */
void SourceManage_ClassDef::ADC_To_Real(uint32_t *_ADCReadBuff)
{	
	/* 计算电压真值 */
	voltage.vol_In = Convert_To_RealVol(_ADCReadBuff[VOL_IN]);
	voltage.vol_Cap = Convert_To_RealVol(_ADCReadBuff[VOL_CAP]);
	voltage.vol_Out = Convert_To_RealVol(_ADCReadBuff[VOL_OUT]);
	
	/* 计算电流真值 */	
	current.cur_In = Convert_To_RealCur_IN(_ADCReadBuff[CUR_IN]);
	current.cur_Out = Convert_To_RealCur_Out(_ADCReadBuff[CUR_OUT]);//大量程
	if(current.cur_Out < 0)	{current.cur_Out = 0;}
	
}

/**
 * @brief 计算电池输入功率、输出功率、电容功率
 * @note 
 * @param None
 * @retval None
 * @author 苏锴南
 */
void SourceManage_ClassDef::Calc_Power(void)
{

	power.pow_In = Calc_RealPower(voltage.vol_In, current.cur_In);
	power.pow_motor = Calc_RealPower(voltage.vol_Out, current.cur_Out);
		
	power.pow_Charge = power.pow_In - power.pow_motor;
	if(power.pow_Charge < 0)		{power.pow_Charge = 0;}
}


uint8_t switchNumber[6]={0,G1_SWITCH_PIN,G2_SWITCH_PIN,G3_SWITCH_PIN,G4_SWITCH_PIN,G5_SWITCH_PIN};
/**
  * @brief 控制开关的通断  
  * @param  _switchNumber [选择的开关数：1-5];
						switchStatus[开关状态：SWITCH_ON 打开开关 SWITCH_OFF 断开开关]
  * @return none
  * @author Joanna
  */
/**
 * @brief 控制开关的通断  
 * @note 
 * @param _switchNumber [选择的开关数：1-5];
						switchStatus[开关状态：SWITCH_ON 打开开关 SWITCH_OFF 断开开关]
 * @retval None
 * @author Joanna
 */
void SourceManage_ClassDef::switchCtrl(uint8_t _switchNumber,GPIO_PinState switchStatus)
{
 	HAL_GPIO_WritePin(G_SWITCH_PORT,switchNumber[_switchNumber], switchStatus);
}
void SourceManage_ClassDef::StopCharge()
{
	HAL_GPIO_WritePin(CHARGE_CTRL_GPIO_Port,CHARGE_CTRL_Pin,GPIO_PIN_RESET);
}
void SourceManage_ClassDef::StartCharge()
{
	HAL_GPIO_WritePin(CHARGE_CTRL_GPIO_Port,CHARGE_CTRL_Pin,GPIO_PIN_SET);
}

/**
 * @brief 由电池供电转到电容供电  
 * @note 
 * @param None
 * @retval None
 * @author 苏锴南
 */
void SourceManage_ClassDef::BATToCAP()
{
	switchCtrl(1,SWITCH_OFF); 
	HAL_Delay(1);
	switchCtrl(4,SWITCH_ON); 
	HAL_Delay(1);
	switchCtrl(2,SWITCH_OFF); 
	HAL_Delay(1);
	switchCtrl(3,SWITCH_ON); 
	HAL_Delay(1);
	switchCtrl(5,SWITCH_ON); 
}

/**
 * @brief 由电容供电转到电源供电  
 * @note 
 * @param None
 * @retval None
 * @author 苏锴南
 */
void SourceManage_ClassDef::CAPToBAT()
{
	switchCtrl(3,SWITCH_OFF); 
	HAL_Delay(1);
	switchCtrl(2,SWITCH_ON); 
	HAL_Delay(1);
	switchCtrl(4,SWITCH_OFF); 
	HAL_Delay(1);
	switchCtrl(1,SWITCH_ON); 
	HAL_Delay(1);
	switchCtrl(5,SWITCH_OFF); 
			
}

/**
 * @brief 24V输出全部关闭
 * @note 
 * @param None
 * @retval None
 * @author 苏锴南
 */
void SourceManage_ClassDef::ALLCLOSE()
{
	switchCtrl(3,SWITCH_OFF); 
	switchCtrl(2,SWITCH_OFF); 
	switchCtrl(4,SWITCH_OFF); 
	switchCtrl(1,SWITCH_OFF); 
	switchCtrl(5,SWITCH_OFF); 		
}

/** 
 * @brief 三区间检测 
 * @note 	1.带一定的min max滞后 
					2.cnt不能超过255
					3.min = max时，只会返回LOW和HIGH
 * @param None
 * @retval None
 * @author 苏锴南
 */
void SourceManage_ClassDef::Block_Check(float min,float max,float real, statusCheck_t* p_statusCheck, int32_t cnt)
{
	if(real < min)
	{
		p_statusCheck->statusCnt++;
		if(p_statusCheck->statusCnt >= cnt)
		{
			p_statusCheck->statusCnt = 0;
			p_statusCheck->statusFlag = Block_LOW;
		}
	}
	else if(real >= max)
	{
		p_statusCheck->statusCnt++;
		if(p_statusCheck->statusCnt >= cnt)
		{
			p_statusCheck->statusCnt = 0;
			p_statusCheck->statusFlag = Block_HIGH;
		}
	}
	else
	{
		p_statusCheck->statusCnt = 0;		
		p_statusCheck->statusFlag = Block_MID;	
	}
}

/** 
 * @brief 电容管理
 * @note 电容启动管理 电容状态检测 计算并设定电容充电电流
 * @param None
 * @retval None
 * @author 苏锴南
 */
void SourceManage_ClassDef::Cap_Manage(void)
{
	Check_CapStatus();
	Set_ChargeCurrent();
}

/**
 * @brief 检查电容状态 包括充电状态和放电状态
 * @note 
 * @param None
 * @retval None
 * @author 苏锴南
 */
void SourceManage_ClassDef::Check_CapStatus(void)
{
	/* 电容电量检测 */		
	Block_Check(CAPVOL_LOW, CAPVOL_HIGH, capObj.Voltage, &capObj.volStatusCheck, 50);
	if(capObj.volStatusCheck.statusFlag == Block_LOW)
	{
		capObj.volStatus = LOW;		/* 电容电压过低 */		
	}
	else if(capObj.volStatusCheck.statusFlag == Block_HIGH)
	{
		capObj.volStatus = FULL;	/* 电容充满 */		
	}
	else
	{
		capObj.volStatus = MID2;		
	}
	
	/* 电容充电检测 */		
	Block_Check(voltage.vol_In-BOOSTCHARGE_dV, voltage.vol_In-BOOSTCHARGE_dV, capObj.Voltage, &capObj.chargeStatusCheck, 50);
	if(capObj.chargeStatusCheck.statusFlag == Block_LOW)
	{
		capObj.chargeStatus = LOW;
	}
	else
	{
		capObj.chargeStatus = MID2;	/* 需要开启升压充电 */
	}	
}

/**
 * @brief 设定电容充电功率
 * @note 
 * @param None
 * @retval None
 * @author 苏锴南
 */
void SourceManage_ClassDef::Set_ChargePower(float inputPower)
{
	capObj.chargePower_Set = inputPower;
}

/**
 * @brief 设定电容充电电流
 * @note 
 * @param None
 * @retval None
 * @author 苏锴南
 */
void SourceManage_ClassDef::Set_ChargeCurrent(void)
{
	float output = 0;
	float chargeCurrent = capObj.chargePower_Set/capObj.Voltage;
	
	/* 判断电容是否充满 */
	if(capObj.volStatus == FULL)
	{
		chargeCurrent = 0;
	}
	else
	{
		/* 判断是否需要小电流充电 */
		if(capObj.Voltage >= LOWCURRENT_VOL)
			if(chargeCurrent >= LOWCURRENT)
				chargeCurrent = LOWCURRENT;
			
		if(chargeCurrent >= 10.0f)
			chargeCurrent = 10.0f;		
	}
	
	output = (uint32_t)(chargeCurrent / 5 / (3.3f/4096.0f));
	capObj.charge_DAC_Value = output;
}

/** 
 * @brief 电池升压板管理
 * @note 
 * @param None
 * @retval None
 * @author 苏锴南
 */
void SourceManage_ClassDef::BAT_Boost_Manage()
{
	if(capObj.chargeStatus == MID2)/* 需要开启升压充电 */
		BAT_BOOST = OPEN;
	else
		BAT_BOOST = CLOSE;
	
//	uint8_t msg_send[8] = {0};
//	msg_send[0] = 0x55;
//	if(BAT_BOOST == OPEN)//开启电池升压板
//		msg_send[1] = 0x33;
//	else
//		msg_send[1] = 0x44;
	
//  BAT_Boost_Board_Msg.ID = 0x200;
//	memcpy(BAT_Boost_Board_Msg.msg_send, msg_send, 8);/* 不能是200  等待改升压板程序 */
	
}

/** 
 * @brief 电调升压板管理
 * @note 
 * @param None
 * @retval None
 * @author 苏锴南
 */
void SourceManage_ClassDef::ESC_Boost_Manage()
{
	uint8_t msg_send[8] = {0};	
	
	ESC_BOOST=OPEN;

	msg_send[0]=4;//24V升压输出
	
	ESC_Boost_Board_Msg.ID = 0x303;
	memcpy(ESC_Boost_Board_Msg.msg_send, msg_send, 8);
}

/**
 * @brief 电源管理初始化
 * @note 
 * @param None
 * @retval None
 * @author 苏锴南
 */
void SourceManage_ClassDef::SourceManage_Init()
{	 
	/* 采样板缓启动 */
	ALLCLOSE();
	HAL_Delay(300);
  StartCharge();
	HAL_Delay(300);
	
	Set_ChargePower(0);
		
	ESC_BOOST=CLOSE;
	BAT_BOOST=CLOSE;
}

/**
 * @brief 电容启动管理
 * @note 初始化时只有当电容大于15V时才打开电容输出
 * @param None
 * @retval None
 * @author 苏锴南
 */
void SourceManage_ClassDef::Cap_Boot_Manage(void)
{
	static uint8_t bootFlag = 0;
	/* 等待充完电再开CAP供电 */
	if(bootFlag == 0)
	{
		if(capObj.Voltage>=15)
		{
			BATToCAP();	
			bootFlag=1;		
		}
		else
			ALLCLOSE();
	}
}

void  SourceManage_ClassDef::Set_Mode(DischargeMode _mode)
{
	Mode = _mode;
}


void SourceManage_ClassDef::Manage(void)
{
	/* 电容  升压板控制 */
	if(Mode == CAP)
		Cap_Boot_Manage();	/* 只有启动完成电容才会输出 */
	else
		CAPToBAT();

	Cap_Manage();
	BAT_Boost_Manage();
	ESC_Boost_Manage();
}
