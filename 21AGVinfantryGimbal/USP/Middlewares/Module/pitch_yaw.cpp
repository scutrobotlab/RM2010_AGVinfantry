#include "pitch_yaw.h"
#include "Infantry_Config.h"

Motor_GM6020 pitchyawMotor[2] = {Motor_GM6020(1),Motor_GM6020(2)};//1为205pitch电机，2为206yaw电机

void C_Pitch_Yaw :: Control(
		E_pitchYawMode mode, 
		float pitch_data, 
		float yaw_data, 
		float motor_pitch_angle,
		float IMU_pitch_speed, 
		float IMU_yaw_angle,
		float IMU_yaw_speed)
{
	Mode = mode;
	recPitchData = pitch_data;
	recYawData = yaw_data;
	
	if(motor_pitch_angle < 4096)
	{
		motor_pitch_angle = motor_pitch_angle + 8192;
	}
	
	pitchAngle.Current =(motor_pitch_angle+lastPitchAngle)/2;
	
	pitchSpeed.Current = IMU_pitch_speed;
//	pitch_cal();
	IMU_cala(MPUData.yaw);
	
	yawAngle.Current = totalYawAngle;
	//yawAngle.Current = (totalYawAngle+lastTotalYawAngle)/2;
	yawSpeed.Current = IMU_yaw_speed;
	
	targetUpdate();
	pidCalc();
	lastPitchAngle = motor_pitch_angle;
	lastPitchAngleTarget = pitchAngle.Target;
	lastYawAngleTarget = yawAngle.Target;
	MotorMsgSend(&hcan1,pitchyawMotor);	//5号步兵为云台都为can1
	MotorMsgSend(&hcan2,pitchyawMotor); 
}


//云台目标值更新
void C_Pitch_Yaw :: targetUpdate()
{
	switch(Mode)
	{
		case REMOTE_CTRL_P:
			pitchAngle.Target += recPitchData * 9.0;
			yawAngle.Target += recYawData * 0.35;
		break;
		case KEY_BOARD_P:
			if(Pitch_Yaw.snipermode == true)
			{
					pitchAngle.Target += recPitchData * 10;
					yawAngle.Target += recYawData * 0.6;
			}
			else
			{
				pitchAngle.Target += recPitchData * 35;
				yawAngle.Target += recYawData * 1.8;
			}
		break;
		case MINI_PC_P:
			if(PackFromVisionUnion.PackFromVision.flag == 37)
			{
				pitchAngle.Target += recPitchData * 35;
				yawAngle.Target += recYawData * 1.8;
			}
			else
			{
//				Infantry.y_pitchYaw = 0;
//				Infantry.x_pitchYaw = 0;
			}
		break;
	}
	pithchAngleLimit();
}



//云台pitch轴目标值限定
void C_Pitch_Yaw :: pithchAngleLimit()
{
	if(pitchAngle.Target <7900)
	{
		pitchAngle.Target = 7900;
		
	}	
	if(pitchAngle.Target > 9072)
	{
		pitchAngle.Target = 9072;
	}

	if((isLandMode == 0)&&(PCvisionStatus == Unconnected))
	{
		
	}	
}

void C_Pitch_Yaw :: pid_init(E_pitchYawPidType type,float kp,float ki,float kd, float ki_max,float out_max)
{
	switch(type)
	{
		case PITCH_SPEED:
			pitchSpeed.SetPIDParam(kp, ki, kd, ki_max, out_max);
			break;
		case PITCH_ANGLE:
			pitchAngle.SetPIDParam(kp, ki, kd, ki_max, out_max);
			break;
		case YAW_SPEED:
			yawSpeed.SetPIDParam(kp, ki, kd, ki_max, out_max);
			break;
		case YAW_ANGLE:
			yawAngle.SetPIDParam(kp, ki, kd, ki_max, out_max);	
			break;
	}
	
}


//pid运算（带前馈）
void C_Pitch_Yaw :: pidCalc()
{
	pitchSpeed.Target = (pitchAngle.Adjust() + lastPitchAngleOut)/2;
	yawSpeed.Target = yawAngle.Adjust();
	if(abs(pitchAngle.Target - pitchAngle.Current) < 100)
	{
		pitchyawMotor[0].Out = pitchSpeed.Adjust() + feedforward(pitchAngle.Target);//+f(angle) = 5x - 12200;
	}
	else
	{
		pitchyawMotor[0].Out = pitchSpeed.Adjust() + feedforward(pitchAngle.Current);
	}
	if(pitchyawMotor[0].Out > pitchSpeed.Out_Max)
	{
		pitchyawMotor[0].Out = pitchSpeed.Out_Max;
	}
	else if(pitchyawMotor[0].Out < -pitchSpeed.Out_Max)
	{
		pitchyawMotor[0].Out = -pitchSpeed.Out_Max;
	}
	else
	{}
	pitchyawMotor[1].Out = yawSpeed.Adjust();//
	lastPitchAngleOut = pitchAngle.Adjust();
}



//陀螺仪当前值更新
void C_Pitch_Yaw :: IMU_cala(float nowImuAngle)
{
	lastTotalYawAngle = totalYawAngle;
	if(nowImuAngle-lastYawAngle>180)
	{
		cnt--;
	}
	else if(nowImuAngle-lastYawAngle<-180)
	{	
		cnt++;
	}
	totalYawAngle = nowImuAngle+cnt*360;
	lastYawAngle = nowImuAngle;
}

void C_Pitch_Yaw :: pitch_cal()
{
	if(pitchAngle.Current < 4096)
	{
		pitchAngle.Current = pitchAngle.Current + 8192;
	}
}

void C_Pitch_Yaw ::Reset()
{
	pitchyawMotor[0].Out = pitchyawMotor[1].Out = 0;
}


//前馈函数
float C_Pitch_Yaw ::feedforward(float target)
{
	return -0.7*feedforward_stste*(5 * target - 12200);
}