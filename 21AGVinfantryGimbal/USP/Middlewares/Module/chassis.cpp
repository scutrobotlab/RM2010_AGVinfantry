#include "chassis.h"
C_Chassis Chassis;
float aa,bb,cc;
int breakSpeed = 20;
int breakSpeed_w = 10;
float accScale = 0.02;
void C_Chassis :: Control(E_ChassisState chassis_stste,
													E_MoveMode move_mode,
													E_CtrlMode ctrl_mode,
													float y_data,
													float x_data,
													float r_data,
													float y_key,
													float x_key)
{
	pitchyawMotor[1].encoder_offset = 4716;//6920  4538  4528 3180 3188 4259
	chassisMode = chassis_stste;
	moveMode = move_mode;
	ctrlMode = ctrl_mode;
	rec_Y = y_data;
	rec_X = x_data;
	rec_R = r_data;
	rec_Y_Key = y_key;
	rec_X_Key = x_key;
	
	
	angleCnt();
	maxSpeedUpdate();
	targetUpdate();
	msgSend();
}

void C_Chassis :: Reset()
{
	speed_Y = 0;
	speed_X = 0;
	speed_R = 0;
	chassisMode = LOST_C;
	msgSend();
}


//板间通信函数
void C_Chassis :: msgSend()
{
	uint8_t data[8];
	
	if(Chassis.snipermode == true)
	{
		data[0] = (int16_t)((float)speed_Y / (float)maxSpeed_Y * 500.0)>>8;
		data[1] = (int16_t)((float)speed_Y / (float)maxSpeed_Y * 500.0);
		
		data[2] = (int16_t)((float)(-speed_X) / (float)maxSpeed_X * 500.0)>>8;
		data[3] = (int16_t)((float)(-speed_X) / (float)maxSpeed_X * 500.0); 
	}
	else
	{
		data[0] = (int16_t)((float)speed_Y / (float)maxSpeed_Y * 10000.0)>>8;
		data[1] = (int16_t)((float)speed_Y / (float)maxSpeed_Y * 10000.0);
		
		data[2] = (int16_t)((float)(-speed_X) / (float)maxSpeed_X * 10000.0)>>8;
		data[3] = (int16_t)((float)(-speed_X) / (float)maxSpeed_X * 10000.0); 
	}
	
	
	data[4] = (int16_t)((float)(-speed_R) / (float)maxSpeed_R * 10000.0)>>8;
	data[5] = (int16_t)((float)(-speed_R) / (float)maxSpeed_R * 10000.0);
	
//	
	data[6] = 0;
	data[7] = 0;
	switch(chassisMode)
	{
		case UNLIMITED_C:data[7] |= 1<<6;
			break;
		case NORMAL_C:data[7] |= 0;
			break;
		case LOST_C:data[7] |= 1<<7;
			break;
		default:
			break;
	}
	switch(moveMode)
	{
		case ROTATE_C:data[7] |= 1<<5;
			break;
		case RUN_C:data[7] |= 0;
			break;
		default:
			break;
	}
	if(magazinemode)
	{
		data[7] |= 1<<4;
	}
	else
	{
		data[7] |= 0;
	}
	if(rewrite_UI)
	{
		data[7] |= 1<<3;
	}
	else
	{
		data[7] |= 0;
	}
	if(isLandMode)
	{
		data[7] |= 1<<2;
	}
	else
	{
		data[7] |= 0;
	}
	if(finalMode)
	{
		data[7] |= 1<<1;
	}
	else
	{
		data[7] |= 0;
	}
	if(fuckMode)
	{
		data[7] |= 1<<0;
	}
	else
	{
		data[7] |= 0;
	}
	CANx_SendData(&hcan2,0x111,data,8);
}


//速度目标值更新函数
void C_Chassis :: targetUpdate()
{
//	static float speed_sum;
	switch(ctrlMode)
	{
		case KEY_BOARD_C:
			//目标值更新

			
			if(rec_Y > 0.5)
			{
				speed_Y = maxSpeed_Y;
			}
			else
			{
				if(rec_Y_Key > 0.5)
				{
					speed_Y = -maxSpeed_Y;
				}
				else
				{
					speed_Y = 0;
				}
			}
			if(rec_X > 0.5)
			{
				speed_X = maxSpeed_X;
			}
			else
			{
				if(rec_X_Key > 0.5)
				{
					speed_X = -maxSpeed_X;
				}
				else
				{
					speed_X = 0;
				}
			}
			
		break;
		case REMOTE_CTRL_C:
			//目标值更新
//			speed_Y = rec_Y * maxSpeed_Y * arm_cos_f32(pitchyawMotor[1].getAngle()/rad*pi)
//			- rec_X * maxSpeed_X * arm_sin_f32(pitchyawMotor[1].getAngle()/rad*pi);
//		
//			speed_X = rec_X * maxSpeed_X * arm_cos_f32(pitchyawMotor[1].getAngle()/rad*pi)
//			+ rec_Y * maxSpeed_Y * arm_sin_f32(pitchyawMotor[1].getAngle()/rad*pi);
		
			speed_Y = rec_Y * maxSpeed_Y;
			speed_X = rec_X * maxSpeed_X;
		break;
	}
	if((sqrtf(speed_X * speed_X + speed_Y * speed_Y) >= maxSpeed_X))
	{
		speed_X = maxSpeed_X * (speed_X/sqrtf(speed_X * speed_X + speed_Y * speed_Y));
		speed_Y = maxSpeed_Y * (speed_Y/sqrtf(speed_X * speed_X + speed_Y * speed_Y));
	}
	switch(moveMode)
	{
		case RUN_C:	
			//目标值更新
			if(rec_R - rotateCnt * 360 > 180)
			{
				chassisYawAngle.Current = rec_R - rotateCnt*360 - 360;
			}
			else if(rec_R - rotateCnt * 360 < -180)
			{
				chassisYawAngle.Current = rec_R - rotateCnt*360 + 360;
			}
			else
			{
				chassisYawAngle.Current = rec_R - rotateCnt*360;
			}
			chassisYawAngle.Target = 0;
			chassisYawAngle.Adjust();
			speed_R = chassisYawAngle.Out;
			
			if(speed_R - speed_R_last > 4)
			{
				speed_R = speed_R_last + 4;
			}
			else if(speed_R - speed_R_last < -4)
			{
				speed_R = speed_R_last - 4;
			}
			else
			{
				speed_R = chassisYawAngle.Out;
			}
			
			

		break;
		case ROTATE_C:
			//目标值更新
			static float R_scale = 1;
			static float speedChangeFlag = 0;
			
			if(rec_R - rotateCnt * 360 > 180)
			{
				speedChangeFlag = rec_R - rotateCnt*360 - 360;
			}
			else if(rec_R - rotateCnt * 360 < -180)
			{
				speedChangeFlag = rec_R - rotateCnt*360 + 360;
			}
			else
			{
				speedChangeFlag = rec_R - rotateCnt*360;
			}
			
			if((isLandMode == 1)&&(maxSpin == 0))
			{
				if(rodirection < 0)
				{
					speed_R = 1250;
				}
				else
				{
					speed_R = -1250;
				}
				speed_Y *= 1;
				speed_X *= 1;
				if((sqrtf(speed_X * speed_X + speed_Y * speed_Y + speed_R * speed_R * 6) >= maxSpeed_X))
				{
					if((maxSpeed_X * maxSpeed_X - speed_R * speed_R * 6) > 0)
					{
						speed_X = 0.7 * sqrtf(maxSpeed_X * maxSpeed_X - speed_R * speed_R * 6) * (speed_X/sqrtf(speed_X * speed_X + speed_Y * speed_Y));
						speed_Y = 0.7 * sqrtf(maxSpeed_Y * maxSpeed_Y - speed_R * speed_R * 6) * (speed_Y/sqrtf(speed_X * speed_X + speed_Y * speed_Y));
					}
					else
					{
						speed_Y = 1000;
						speed_X = 1000;
					}
				}
			}
			else
			{
				if(rodirection < 0)
				{
					speed_R = maxSpeed_R;
				}
				else
				{
					speed_R = -maxSpeed_R;
				}
				speed_Y *= 1;
				speed_X *= 1;
				if((sqrtf(speed_X * speed_X + speed_Y * speed_Y + speed_R * speed_R * 6) >= maxSpeed_X))
				{
					speed_X = 0.75 * maxSpeed_X * (speed_X/sqrtf(speed_X * speed_X + speed_Y * speed_Y + speed_R * speed_R * 6));
					speed_Y = 0.75 * maxSpeed_Y * (speed_Y/sqrtf(speed_X * speed_X + speed_Y * speed_Y + speed_R * speed_R * 6));
					speed_R =  1.3 * maxSpeed_R * (speed_R * 2.5/sqrtf(speed_X * speed_X + speed_Y * speed_Y + speed_R * speed_R * 6));
				}
			}
			
			if(rodirection < 0)
			{
				(speed_R_last + accScale * maxSpeed_R) < speed_R ?
				speed_R = speed_R_last + accScale * maxSpeed_R :
				speed_R = speed_R;
			}
			else
			{
				(speed_R_last - accScale * maxSpeed_R) > speed_R ?
				speed_R = speed_R_last - accScale * maxSpeed_R :
				speed_R = speed_R;
			}
			
			
			
		break;
		default:
			if(rodirection < 0)
			{
				speed_R-10>0?speed_R-=10:speed_R=0;
			}
			else
			{
				speed_R+10<0?speed_R+=10:speed_R=0;
			}
		break;
	}

		if(speed_Y_last + breakSpeed < speed_Y)
		{
			speed_Y = speed_Y_last + breakSpeed;
		}
		else if(speed_Y_last - breakSpeed > speed_Y)
		{
			speed_Y = speed_Y_last - breakSpeed;
		}
		else
		{
			
		}
		
		if(speed_X_last + breakSpeed < speed_X)
		{
			speed_X = speed_X_last + breakSpeed;
		}
		else if(speed_X_last - breakSpeed > speed_X)
		{
			speed_X = speed_X_last - breakSpeed;
		}
		else
		{
			
		}
		
		speed_Y_last = speed_Y;
		speed_X_last = speed_X;
		speed_R_last = speed_R;
		
		speed_Y = speed_Y_last*arm_cos_f32(pitchyawMotor[1].getAngle()/rad*pi)
		- speed_X_last*arm_sin_f32(pitchyawMotor[1].getAngle()/rad*pi);
		speed_X = speed_X_last*arm_cos_f32(pitchyawMotor[1].getAngle()/rad*pi)
		+ speed_Y_last*arm_sin_f32(pitchyawMotor[1].getAngle()/rad*pi);
		

	
}


void C_Chassis :: angleCnt()
{
	if(rec_R-rotateCnt*360>360)
	{
		rotateCnt++;
	}
	if(rec_R-rotateCnt *360<-360)
	{
		rotateCnt--;
	}
}

void C_Chassis :: maxSpeedUpdate()
{
	maxSpeed_Y = sourcePowerMax * 35 + 2500;
	maxSpeed_X = sourcePowerMax * 35 + 2500;
	maxSpeed_R = (sourcePowerMax * 35 + 2500)/3;
}

void C_Chassis :: setMaxSpeed(int16_t max_Y, int16_t max_X, int16_t max_R)
{
	maxSpeed_Y = max_Y;
	maxSpeed_X = max_X;
	maxSpeed_R = max_R;
}

void C_Chassis :: rSpeedLimit()
{

}

void C_Chassis :: pid_init(float kp,float ki,float kd, float ki_max,float out_max)
{
	chassisYawAngle.SetPIDParam(kp, ki, kd, ki_max, out_max);
}










