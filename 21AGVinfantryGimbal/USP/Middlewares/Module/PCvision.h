#ifndef __PCvision_H__
#define __PCvision_H__


#include <stdint.h> 

#ifdef __cplusplus

/**************************************************************************
* @brief	视觉传过来的协商一致的数据包
* @note		Null
**************************************************************************/
#pragma pack(1)
typedef struct 
{
	uint8_t flag;
	uint8_t shootMode;
	float yawData;
	float pitchData;
	uint8_t End;
}PackFromVisionDef;
#pragma pack()

/**************************************************************************
* @brief	通过共用内存将视觉传过来的数据自动同步到PackFromVision
* @note		使用UsartData接收视觉的数据，PackFromVision中对应的参数将被修改
**************************************************************************/
typedef union 
{
	uint8_t UsartData[11];
	PackFromVisionDef PackFromVision;
}PackFromVisionUnionDef;

/**************************************************************************
* @brief	返回给视觉的协商一致的数据包
* @note		Null
**************************************************************************/
#pragma pack(1)
typedef struct
{
	uint8_t head;
	
	float	pitchAngle;
	float	yawAngle;
	uint8_t color;
	uint8_t bulletSpeed;
	uint8_t mode;
	uint8_t end;
	
}PackToVision_Def;
#pragma pack()

/**************************************************************************
* @brief	通过共用内存将反馈的数据自动同步到PackToVision
* @note		通过修改PackToVision中的参数，再将UsartData数组发送出即可
**************************************************************************/
typedef union 
{
	uint8_t UsartData[12];
	PackToVision_Def PackToVision;
}PackToVisionUnionDef;

/**************************************************************************
* @brief	通过共用内存及函数修改float类型的大小端
* @note		
**************************************************************************/
typedef union 
{
        float				f;
        char				c[4];
}Float_Conv;

enum PCvisionStatusDef
{
	Connected = 1,
	Unconnected = 0,
};

extern PackFromVisionUnionDef PackFromVisionUnion;
extern PackToVisionUnionDef PackToVisionUnion;
extern PCvisionStatusDef PCvisionStatus;
extern uint8_t PCCheckLinkFlag;

void PC_CheckLink(void);
uint32_t GetViaionData(uint8_t* Recv_Data, uint16_t ReceiveLen);
void Vision_Init();
void SendGimbleStatus(PackToVisionUnionDef* _PackToVisionUnion);
float BLEndianFloat(float fValue);
extern "C"{
#endif
	
	
#ifdef  __cplusplus
}
#endif

#endif

