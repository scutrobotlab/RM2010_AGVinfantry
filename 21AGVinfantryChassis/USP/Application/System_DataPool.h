/**
  ******************************************************************************
  * @file   System_DataPool.h
  * @brief  All used resources are contained in this file.
  ******************************************************************************
  * @note
  *  - User can define datas including variables ,structs ,and arrays in
  *    this file, which are used in deffrient tasks or services.
**/
#ifndef _DATA_POOL_H_
#define _DATA_POOL_H_

/* Includes ------------------------------------------------------------------*/
/* Middlewares & Drivers Support */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <stm32f4xx.h>
#include <SRML.h>

/* Macro Definitions ---------------------------------------------------------*/
#define Tiny_Stack_Size       64
#define Small_Stack_Size      128
#define Normal_Stack_Size     256
#define Large_Stack_Size      512
#define Huge_Stack_Size       1024
#define PriorityVeryLow       1
#define PriorityLow           2
#define PriorityBelowNormal   3
#define PriorityNormal        4
#define PriorityAboveNormal   5
#define PriorityHigh          6
#define PrioritySuperHigh     7
#define PriorityRealtime      8

/* HAL Handlers --------------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern TIM_HandleTypeDef htim4;

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
//extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

extern ADC_HandleTypeDef  hadc1;
extern DAC_HandleTypeDef  hdac;

//extern SPI_HandleTypeDef hspi1;
/* RTOS Resources ------------------------------------------------------------*/
/* Queues */
extern QueueHandle_t  USART_RxPort;
extern QueueHandle_t  USART_TxPort;
extern QueueHandle_t  CAN1_TxPort;
extern QueueHandle_t  CAN2_TxPort;

extern QueueHandle_t  DR16_QueueHandle;
extern QueueHandle_t  RMMotor_QueueHandle;
extern QueueHandle_t  SuperRelay_QueueHandle;
extern QueueHandle_t  IMU_QueueHandle;
extern QueueHandle_t  NUC_QueueHandle;
extern QueueHandle_t  Referee_QueueHandle;

/* Semaphores */
/* Mutexes */
/* Notifications */
/* Other Resources -----------------------------------------------------------*/
#define USART1_RX_BUFFER_SIZE 32
#define USART2_RX_BUFFER_SIZE 64
#define USART3_RX_BUFFER_SIZE 128
#define USART4_RX_BUFFER_SIZE 256
#define USART5_RX_BUFFER_SIZE 512
#define USART6_RX_BUFFER_SIZE 1024

extern uint8_t Uart1_Rx_Buff[USART1_RX_BUFFER_SIZE];
extern uint8_t Uart2_Rx_Buff[USART2_RX_BUFFER_SIZE];
extern uint8_t Uart3_Rx_Buff[USART3_RX_BUFFER_SIZE];
extern uint8_t Uart4_Rx_Buff[USART4_RX_BUFFER_SIZE];
extern uint8_t Uart5_Rx_Buff[USART5_RX_BUFFER_SIZE];
extern uint8_t Uart6_Rx_Buff[USART6_RX_BUFFER_SIZE];

extern DR16_Classdef DR16;
//extern W25Qx_Classdef Flash;
extern CLogger SysLog;
extern LogFilter_t Filter_List[2];
extern CAnalyzer Analyzer;

extern uint32_t ADCReadBuff[8];
//暂时被遗忘在这,统一标准接口制定后再集中回收
#ifndef __UUCOBTypeDef_DEFINED
#define __UUCOBTypeDef_DEFINED
typedef struct
{
  uint8_t  port_num;
  int16_t  len;
  void*    address;
}USART_COB;
#endif

#ifndef __CCOBTypeDef_DEFINED
#define __CCOBTypeDef_DEFINED
/* CAN message data type(Communication Object/标准数据帧) */
typedef struct{
  uint16_t  ID;
  uint8_t   DLC;
  uint8_t   Data[8];
}COB_TypeDef;
#endif

#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
