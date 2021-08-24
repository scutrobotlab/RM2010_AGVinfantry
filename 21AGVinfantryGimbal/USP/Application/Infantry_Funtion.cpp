#include "Infantry_Funtion.h"
#include "Infantry_Config.h"
#include "chassis.h"
#include "pitch_yaw.h"
#include "PID.h"
#include "math.h"


//操作手按键判断


void CheakState()
{
	static uint8_t temp;
	chassisState();
	shootState();
//-----------------------------------------------------------
//比赛逻辑
	if(DR16.GetS1() == 1)//上
	{
		Infantry.pitchYawCtrlMode = REMOTE_CTRL_P;
		Infantry.shootCtrlMode = REMOTE_CTRL_S;
		Infantry.chassisCtrlMode = REMOTE_CTRL_C;
		if(DR16.GetS2() == 1)
		{
			Infantry.moveMode = ROTATE_C;
			Infantry.chassisMode = NORMAL_C;
		}
//		else
//		{
//			Infantry.moveMode = RUN_C;
//		}
		
		else if(DR16.GetS2() == 2)
		{
			Infantry.chassisMode = UNLIMITED_C;
			Infantry.moveMode = ROTATE_C;
		}
		else
		{
			Infantry.chassisMode = NORMAL_C;
			Infantry.moveMode = RUN_C;
		}
	}
	else if(DR16.GetS1() == 2)//下
	{
		Infantry.pitchYawCtrlMode = REMOTE_CTRL_P;
		Infantry.shootCtrlMode = REMOTE_CTRL_S;
		Infantry.chassisCtrlMode = REMOTE_CTRL_C;
		static uint8_t fire_fl1 = 0;
		static uint8_t open_fl1 = 0;
		static uint8_t fire_fl2 = 0;
		static uint8_t open_fl2 = 0;
		if((DR16.GetS2() == 1)&&(fire_fl2 == 1))
		{
			fire_fl1 += 1;
			fire_fl1 = fire_fl1%2;
			fire_fl2 = 0;
		}
		else if((DR16.GetS2() == 2)&&(open_fl2 == 1))
		{
			open_fl1 += 1;
			open_fl1 = open_fl1%2;
			open_fl2 = 0;
		}
		else if(DR16.GetS2() == 3)
		{
			fire_fl2 = 1;
			open_fl2 = 1;
			
		}
		else
		{}
		if(fire_fl1 == 1)
		{
			Infantry.fireFlag = FIRE_F;
//			fire_fl = 0;
		}
		else if(DR16.GetS2() == 3)
		{
			Infantry.fireFlag = STOP_F;
//			fire_fl = 1;
			
		}
		else
		{
			Infantry.fireFlag = STOP_F;
		}
		if(open_fl1 == 1)
		{
			Infantry.friWheelState = OPEN_F;
			Infantry.laserState = OPEN_L;
		}
		else
		{
			Infantry.friWheelState = CLOSE_F;
			Infantry.laserState = CLOSE_L;
			if((DR16.GetS2() == 1))
			{
				Infantry.bulletBayState = OPEN_B;
			}
			else
			{
				Infantry.bulletBayState = CLOSE_B;
			}
		}		
	}
	else//中
	{
			if(Infantry.pitchYawCtrlMode == REMOTE_CTRL_P)
			Infantry.pitchYawCtrlMode = KEY_BOARD_P;
			//Infantry.pitchYawCtrlMode = (E_pitchYawMode)(logicJudge(Infantry.pitchYawCtrlMode-1,DR16.IsKeyPress(_R),&temp)+1);
			Infantry.pitchYawCtrlMode = (E_pitchYawMode)(DR16.IsKeyPress(_Mouse_R)+1);
			Infantry.shootCtrlMode = KEY_BOARD_S;
			Infantry.chassisCtrlMode = KEY_BOARD_C;
		
	}

}

void DataUpdate()
{
	pitchYawUpdate();
	chassisUpdate();
}

void pitchYawUpdate()
{
	switch(Infantry.pitchYawCtrlMode)
	{
		case REMOTE_CTRL_P:
				Infantry.y_pitchYaw = -DR16.Get_RY_Norm();
				Infantry.x_pitchYaw = -DR16.Get_RX_Norm();
			break;
		case KEY_BOARD_P:
				Infantry.y_pitchYaw = DR16.Get_MouseY_Norm();
				Infantry.x_pitchYaw = -DR16.Get_MouseX_Norm();
				static int16_t turn_mode = 0 , turn_count = 0;
				if((DR16.IsKeyPress(_R))&&(turn_mode == 1))
				{
					Infantry.x_pitchYaw = Infantry.x_pitchYaw - 90;
					turn_mode = 0;
				}
				else if(DR16.IsKeyPress(_R))
				{}
				else
				{
					if(turn_mode == 0)
					{
						turn_count ++;
						turn_count %= 600;
						if(turn_count == 0)
						{
							turn_mode = 1;
						}
					}
					else
					{
						
					}
					
				}
//				if((turn_mode == 0)&&(DR16.IsKeyPress(_R)))
//				{
//					Infantry.x_pitchYaw = Infantry.x_pitchYaw - 170;
//					turn_mode ++;
//				}					
//				else if(turn_mode == 0)
//				{
//					
//				}
//				else
//				{
//					turn_mode ++;
//					turn_mode %= 600;
//				}
			break;
		case MINI_PC_P:
			if(PackFromVisionUnion.PackFromVision.flag == 37)
			{
				Infantry.y_pitchYaw = DR16.Get_MouseY_Norm();
				Infantry.x_pitchYaw = -DR16.Get_MouseX_Norm();
				static int16_t turn_mode = 0;
				if((DR16.IsKeyPress(_R))&&(turn_mode == 1))
				{
					Infantry.x_pitchYaw = Infantry.x_pitchYaw - 90;
					turn_mode = 0;
				}
				else if(DR16.IsKeyPress(_R))
				{}
				else
				{
					if(turn_mode == 0)
					{
						turn_count ++;
						turn_count %= 600;
						if(turn_count == 0)
						{
							turn_mode = 1;
						}
					}
					else
					{
						
					}
				}
			}
			else
			{
//				Infantry.y_pitchYaw = 0;
//				Infantry.x_pitchYaw = 0;
			}
			break;
	}
}

void chassisUpdate()
{
	switch(Infantry.chassisCtrlMode)
	{
		case KEY_BOARD_C:
			Infantry.y_data = DR16.IsKeyPress(_W);
			Infantry.x_data = DR16.IsKeyPress(_A);
			Infantry.y_back = DR16.IsKeyPress(_S);
			Infantry.x_back = DR16.IsKeyPress(_D);
		break;
		case REMOTE_CTRL_C:
			Infantry.y_data = DR16.Get_LY_Norm();
			Infantry.x_data = -DR16.Get_LX_Norm();
			Infantry.y_back = 0;
			Infantry.x_back = 0;
		break;
	}
}

void pitchYawState()
{
	
}

void chassisState()
{
	//暂时定义 shift小陀螺 C超功率V刷新UI
	static uint8_t temp,temp2,temp3,temp4,temp5,temp6;
	if(DR16.IsKeyPress(_SHIFT))
	{
		Infantry.moveMode = ROTATE_C;
	}
	else
	{
		Infantry.moveMode = RUN_C;
	}
	
	
	
//	Infantry.mode = logicJudge(Infantry.mode,DR16.IsKeyPress(_X),&temp4);
	
	
	Infantry.power_off = 0;
	if(DR16.IsKeyPress(_CTRL))
	{
		if(DR16.IsKeyPress(_Q))
		{
			Chassis.rodirection = 1;
		}
		else if(DR16.IsKeyPress(_E))
		{
			Chassis.rodirection = -1;
		}
		else if(DR16.IsKeyPress(_V))
		{
//			Infantry.power_off = 1;
		}
		else
		{
			Infantry.snipermode = true;
			Chassis.snipermode = true;
			Pitch_Yaw.snipermode = true;
		}
		
	}
	else if(DR16.IsKeyPress(_Q))
	{
		Infantry.mode = 0;
		Chassis.isLandMode = 1;
		Pitch_Yaw.isLandMode = 1;
	}
	else if(DR16.IsKeyPress(_E))
	{
		Infantry.mode = 1;
		Chassis.isLandMode = 0;
		Pitch_Yaw.isLandMode = 0;
	}
	else if(DR16.IsKeyPress(_V))
	{
		Chassis.rewrite_UI = true;
	}
	else
	{
		Chassis.rewrite_UI = false;
		Chassis.finalMode = Infantry.finalMode;
		Infantry.snipermode = false;
		Chassis.snipermode = false;
		Pitch_Yaw.snipermode = false;
	}
	
	
	if(Infantry.bulletBayState == OPEN_B)
	{
		Infantry.snipermode = true;
		Chassis.snipermode = true;
		Pitch_Yaw.snipermode = true;
	}
	if(DR16.IsKeyPress(_C)&&DR16.IsKeyPress(_CTRL))
	{
		Infantry.mode = 2;
	}
	else
	{
		Infantry.chassisMode = (E_ChassisState)(logicJudge(Infantry.chassisMode-1,DR16.IsKeyPress(_C),&temp2)+1);
	}
	
	if(DR16.IsKeyPress(_Z)&&DR16.IsKeyPress(_CTRL))
	{
		Infantry.finalMode = 1;
	}
	else
	{
		Infantry.maxSpin = logicJudge(Infantry.maxSpin,DR16.IsKeyPress(_Z),&temp3);;
		Chassis.maxSpin = Infantry.maxSpin;
	}
	
	if(DR16.IsKeyPress(_X)&&DR16.IsKeyPress(_CTRL))
	{
		Infantry.finalMode = 0;
	}
	else if(DR16.IsKeyPress(_X))
	{
		Infantry.fuckMode = 1;
		Chassis.fuckMode = 1;
	}
	else
	{
		Infantry.fuckMode = 0;
		Chassis.fuckMode = 0;
	}
	
}

void shootState()
{
	//鼠标左键发射 F开摩擦轮和激光 G关摩擦轮和激光
	static uint8_t temp,temp2,temp3;
	if(Infantry.pitchYawCtrlMode == MINI_PC_P)
	{
//		Infantry.fireFlag = (E_FireFlag)(((PackFromVisionUnion.PackFromVision.shootMode == 76)?1:0)&&DR16.IsKeyPress(_Mouse_L));
		if(Infantry.mode == 1)
		{
			Shoot.period = 65;
//			Infantry.fireFlag = (E_FireFlag)(DR16.IsKeyPress(_Mouse_L));
			Infantry.fireFlag = (E_FireFlag)(DR16.IsKeyPress(_Mouse_L));
			if(!(DR16.IsKeyPress(_Mouse_L)))
			{
				Shoot.delayCnt = 0; 
			}
		}
		else if(Infantry.mode == 2)
		{
			Shoot.period = 600;
//			Infantry.fireFlag = (E_FireFlag)(DR16.IsKeyPress(_Mouse_L));
			
			Infantry.fireFlag = (E_FireFlag)(((PackFromVisionUnion.PackFromVision.shootMode == 76)?1:0)&&DR16.IsKeyPress(_Mouse_L));
			if(!(DR16.IsKeyPress(_Mouse_L)))
			{
				Shoot.delayCnt = 0; 
			}
		}
		else
		{
			Shoot.period = 65;
//			Infantry.fireFlag = (E_FireFlag)(DR16.IsKeyPress(_Mouse_L));
			Infantry.fireFlag = (E_FireFlag)(((PackFromVisionUnion.PackFromVision.shootMode == 76)?1:0)&&DR16.IsKeyPress(_Mouse_L));
			if(!(DR16.IsKeyPress(_Mouse_L)))
			{
				Shoot.delayCnt = 0; 
			}
		}
	}
	else
	{
		Infantry.fireFlag = (E_FireFlag)DR16.IsKeyPress(_Mouse_L);
		Shoot.period = 65;
	}
	if(DR16.IsKeyPress(_F))
	{
		
		if(DR16.IsKeyPress(_CTRL))
		{
			Infantry.mode = 3;
		}
		else
		{
			Infantry.friWheelState = OPEN_F;
			Infantry.laserState = OPEN_L;
		}
	}
	
	if(DR16.IsKeyPress(_G))
	{
		Infantry.friWheelState = CLOSE_F;
		Infantry.laserState = CLOSE_L;
		if(DR16.IsKeyPress(_CTRL))
		{
			Infantry.GGMode = 1;
			Chassis.GGMode = 1;
		}
		else
		{
			Infantry.GGMode = 0;
			Chassis.GGMode = 0;
		}
	}
	else
	{
	}
	
	
	
//	Infantry.bulletBayState = (E_BulletBayState)logicJudge(Infantry.bulletBayState,DR16.IsKeyPress(_B),&temp3);
	if(DR16.IsKeyPress(_SHIFT))
	{
		Infantry.friWheelState = OPEN_F;
		Infantry.laserState = OPEN_L;
		if(Infantry.bulletBayState == OPEN_B)
		{
			Infantry.bulletBayState = (E_BulletBayState)logicJudge(Infantry.bulletBayState,true,&temp3);
		}
		else
		{}
	}
	else
	{
		Infantry.bulletBayState = (E_BulletBayState)logicJudge(Infantry.bulletBayState,DR16.IsKeyPress(_B),&temp3);
	}	
	
	
	if(Infantry.bulletBayState == OPEN_B)
	{
		Chassis.magazinemode = true;
	}
	else
	{
		Chassis.magazinemode = false;
	}
}

uint8_t logicJudge(uint8_t last_res,uint8_t now,uint8_t* last)
{
	if(*last==0 && now==1)
	{
		*last = 1;
		if(last_res == 1)
			return 0;
		if(last_res == 0)
			return 1;
	}
	if(now == 0)
	{ 
		*last = 0;
	}
	return last_res;
}
	
float repeatJudge(float now, float *last)
{
	if(now ==*last)
		return 0;
	else
		return now;
}	
	
	
	





	




