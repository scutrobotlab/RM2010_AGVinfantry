/**
 * @brief 用于执行视觉视觉对接相关操作,包括 接收数据 和 返回数据
 * @file  PCvision.cpp
**/
#include "Service_Devices.h"
#include "PCvision.h"
#include "Infantry_Config.h"
#include "Shoot.h"
int16_t time;
PCvisionStatusDef  PCvisionStatus;
PackFromVisionUnionDef PackFromVisionUnion;
PackToVisionUnionDef PackToVisionUnion;
uint8_t PCCheckLinkFlag = 0;
int vision_flag = 0;
float a=-25.0,b=0.8,yaw,pitch;//2.8
uint8_t index2;
extern float tampPitchAngle,tampYawAngle;
extern C_Shoot Shoot;

int test_num = 0;
/**
 * @brief  50ms执行检测一次，查看是否在线
 * @param  NULL
 * @return NULL
 */
void PC_CheckLink(void)
{
	if(PCCheckLinkFlag == 1)
	{
		PCvisionStatus = Connected;
		PCCheckLinkFlag = 0;
	}
	else
		PCvisionStatus = Unconnected;
}
void Vision_Init()
{
}

uint32_t GetViaionData(uint8_t* Recv_Data, uint16_t ReceiveLen)		//ReceiveLen实际数据长度
{
	PCCheckLinkFlag = 1; 
	time++;
	if(Infantry.pitchYawCtrlMode == MINI_PC_P)
	//SetVisionOrdor
	{
		if(PackFromVisionUnion.PackFromVision.flag == 37)
		{
//			Pitch_Yaw.pitchAngle.Target = Pitch_Yaw.pitchAngle.Target;
//			Pitch_Yaw.yawAngle.Target = Pitch_Yaw.yawAngle.Target;
		}
		else if((isnan(PackFromVisionUnion.PackFromVision.pitchData) || isnan(PackFromVisionUnion.PackFromVision.yawData)) == 0)
		{
			Pitch_Yaw.pitchAngle.Target = Pitch_Yaw.pitchAngle.Current - PackFromVisionUnion.PackFromVision.pitchData* a;
			Pitch_Yaw.yawAngle.Target = Pitch_Yaw.yawAngle.Current - PackFromVisionUnion.PackFromVision.yawData * b;
//			Pitch_Yaw.pitchAngle.Target = -PackFromVisionUnion.PackFromVision.pitchData * 8192.0 / 360.0 + 3500;
//			Pitch_Yaw.yawAngle.Target = +PackFromVisionUnion.PackFromVision.yawData;
		}
		else
		{}
		
	}
  return 0;
}
void SendGimbleStatus(PackToVisionUnionDef* _PackToVisionUnion)
{
		_PackToVisionUnion->PackToVision.pitchAngle = -((float)Pitch_Yaw.pitchAngle.Current-8769.0)/8192.0*360;
		
	
		_PackToVisionUnion->PackToVision.yawAngle		=	(float)Pitch_Yaw.yawAngle.Current;
//		_PackToVisionUnion->PackToVision.yawAngle		=	(float)MPUData.yaw;
		_PackToVisionUnion->PackToVision.color = (Infantry.robotID>100)?1:0;
		_PackToVisionUnion->PackToVision.bulletSpeed = Shoot.maxSpeed==255?30:Shoot.maxSpeed;
		
//		_PackToVisionUnion->PackToVision.UpPitchAngle		= Pitch_Yaw.pitchAngle.Current;
//		_PackToVisionUnion->PackToVision.UpPitchSpeed 	= Pitch_Yaw.pitchSpeed.Current;
//	
//		_PackToVisionUnion->PackToVision.UpYawAngle			= Pitch_Yaw.yawAngle.Current;
//		_PackToVisionUnion->PackToVision.UpYawSpeed			= Pitch_Yaw.yawSpeed.Current;
		_PackToVisionUnion->PackToVision.head = 0x44;
		_PackToVisionUnion->PackToVision.end = 0x55;
		_PackToVisionUnion->PackToVision.mode = Infantry.mode;
		HAL_UART_Transmit_DMA(&huart1 , _PackToVisionUnion->UsartData, sizeof(_PackToVisionUnion->PackToVision));

}

float BLEndianFloat(float fValue)
{
       Float_Conv			d1, d2;
 
       d1.f = fValue;
       d2.c[0] = d1.c[3];
       d2.c[1] = d1.c[2];
       d2.c[2] = d1.c[1];
       d2.c[3] = d1.c[0];
 
       return d2.f;
}
