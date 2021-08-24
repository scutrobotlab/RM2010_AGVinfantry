/**
  **********************************************************************************
  * @file   ：Service_Communication.h
  * @brief  ：Header of Service_Communication.cpp
  **********************************************************************************
**/
#ifndef  _SERVICE_COMMUNICATE_H_
#define  _SERVICE_COMMUNICATE_H_

#include "System_DataPool.h"
#include "shoot.h"
#include "pitch_yaw.h"
#include "motor.h"
#include "task.h"
#include "Infantry_Config.h"
#ifdef  __cplusplus
extern "C"{
#endif
/*------------------------------ System Handlers ------------------------------*/
extern TaskHandle_t CAN1SendPort_Handle;
extern TaskHandle_t CAN2SendPort_Handle;
extern TaskHandle_t CANReceivePort_Handle;
extern TaskHandle_t UartTransmitPort_Handle;
	
extern Motor_C620 FriMotor[3];
extern Motor_GM6020 pitchyawMotor[2];
extern Motor_C610	Turnplate[1];


/*------------------------------Function prototypes ---------------------------*/
uint32_t User_UART2_RxCpltCallback(uint8_t* Recv_Data, uint32_t ReceiveLen);
uint32_t User_UART3_RxCpltCallback(uint8_t* Recv_Data, uint32_t ReceiveLen);
uint32_t User_UART4_RxCpltCallback(uint8_t* Recv_Data, uint32_t ReceiveLen);
uint32_t User_UART5_RxCpltCallback(uint8_t* Recv_Data, uint32_t ReceiveLen);
void Service_Communication_Init(void);
	
void User_CAN1_RxCpltCallback(CAN_RxBuffer *CAN_RxMessage);
void User_CAN2_RxCpltCallback(CAN_RxBuffer *CAN_RxMessage);

#ifdef  __cplusplus
}
#endif
#endif  

/************************* COPYRIGHT SCUT-ROBOTLAB *****END OF FILE****************/

