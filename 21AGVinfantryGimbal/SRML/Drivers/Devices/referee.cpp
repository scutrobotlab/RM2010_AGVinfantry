/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file   : referee.cpp
  * @author : charlie 602894526@qq.com
  * @brief  : Code for communicating with Referee system of Robomaster 2019.
  * @date   : 2019-03-01
  * @par Change Log：
  *  Date           Author   Version    Notes
  *  2019-03-01     charlie   2.0.0     Creator
	*  2019-12-28     kainan		3.0.0     增加绘画类
	*  2020-05-26 		kainan  	4.0.0			适应20年规则
  ==============================================================================
                          How to use this driver  
  ==============================================================================
	Init()初始化模块
	
	裁判系统数据接收与外部读取
	1.使用unPackDataFromRF()解包裁判系统串口数据
	2.如果需要用到裁判系统提供的各种数据（具体有些什么数据请查看手册），读取相应结构体即可
	
  机器人车间通信
  1. 发送端调用CV_ToOtherRobot()发送数据
	2. 接收端轮询机器人ID对应的robot_com_data[]数据情况，如工程发送过来的数据为robot_com_data[1]

	操作手界面UI
	1.Set_DrawingLayer()设置图层，0-9
	2.对应的drawin()，clean()
	3.组合图形:标尺UI_ruler()、Draw_FitingGraph()
	4.注意：发数据给裁判系统。务必注意等待上电稳定之后才发送，否则会丢包 \n
					每个图形的name必须不同，建议name[3]不同即可 \n
	
	注意：
	串口DMA接收缓存数组大小建议设置等于256，过小容易造成解包过程数据被覆盖。
	发送需要和串口接收不同任务，由于速率控制内部有大量的vTaskDelay
	要等待一段时间(等串口、裁判系统稳定)，再发送clean、数据、UI等 
	特别注意要用最新的裁判系统客户端，旧版有点问题
	
	
  目前参考的是裁判系统串口协议附录V1.0-2020-02-25
  如有问题，请参考《RM2020裁判系统用户接口协议附录》
  2019年度裁判系统资料大全：
  https://bbs.robomaster.com/thread-7099-1-1.html
  ******************************************************************************
  * @attention:
  * 
  * if you had modified this file, please make sure your code does not have many 
  * bugs, update the version NO., write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding 
  * through your new brief.
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "referee.h"
#include "FreeRTOS.h"
#include "task.h"//需要用到taskDelay

/* Private define ------------------------------------------------------------*/
/* Private function declarations --------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static const unsigned char CRC8_TAB[256] = {
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3,
    0xfd, 0x1f, 0x41, 0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01,
    0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe,
    0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62, 0xbe, 0xe0, 0x02, 0x5c,
    0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff, 0x46,
    0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb,
    0x59, 0x07, 0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5,
    0xfb, 0x78, 0x26, 0xc4, 0x9a, 0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6,
    0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24, 0xf8, 0xa6, 0x44, 0x1a, 0x99,
    0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9, 0x8c, 0xd2,
    0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93,
    0xcd, 0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31,
    0xb2, 0xec, 0x0e, 0x50, 0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d,
    0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee, 0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d,
    0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73, 0xca, 0x94, 0x76,
    0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4,
    0xaa, 0x48, 0x16, 0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75,
    0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8, 0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9,
    0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};

static const uint16_t wCRC_Table[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e,
    0xf8f7, 0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e, 0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64,
    0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e,
    0xfae7, 0xc87c, 0xd9f5, 0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50,
    0xfbef, 0xea66, 0xd8fd, 0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb, 0xce4c, 0xdfc5, 0xed5e,
    0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44,
    0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72, 0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e,
    0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e,
    0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff, 0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324,
    0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e,
    0xf2a7, 0xc03c, 0xd1b5, 0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710,
    0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c, 0xc60c, 0xd785, 0xe51e,
    0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb, 0xd68d, 0xc704,
    0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232, 0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 0xe70e,
    0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1,
    0x0f78
};
/* Private type --------------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief 
 * @note use in SendDrawData() and character_drawing()
 * @param 
 * @retval 
 */
template<typename Type>
Type _referee_Constrain(Type input,Type min,Type max){
  if (input <= min)
    return min;
  else if(input >= max)
    return max;
  else return input;
}

/**
  * @brief   
  * @param   *_huart, handle of HAL_uart
  * @param   *getTick_fun, handle of get microtick fun
  * @retval  
  */
void referee_Classdef::Init(UART_HandleTypeDef *_huart, uint32_t (*getTick_fun)(void))
{
	refereeUart = _huart;
	
	if(getTick_fun != NULL)
    Get_SystemTick = getTick_fun;

}


/**
  * @brief   CRC8 data check.
  * @param   *pchMessage:Data to be processed
              dwLength:Length of check data
              ucCRC8:Data after processing
  * @retval  	Gets the CRC8 checksum
  */
unsigned char referee_Classdef::Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8)
{
    unsigned char ucIndex;
    while (dwLength--) {
        ucIndex = ucCRC8^(*pchMessage++);
        ucCRC8 = CRC8_TAB[ucIndex];
    }
    return(ucCRC8);
}
/**
  * @brief   CRC16 data check.
  * @param   *pchMessage:Data to be processed
             dwLength:Length of check data
             ucCRC8:Data after processing
  * @retval  Gets the CRC16 checksum
  */
uint16_t referee_Classdef::Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC)
{
    uint8_t chData;
    if (pchMessage == NULL) {
        return 0xFFFF;
    }
    while(dwLength--) {
        chData = *pchMessage++;
        (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
    }
    return wCRC;
}


void referee_Classdef::unPackDataFromRF(uint8_t *data_buf, uint32_t length)
{
		static uint8_t temp[128];
		static uint8_t RFdataBuff[256];

		static int32_t index,buff_read_index;
		static short CRC16_Function,CRC16_Referee;
		static uint8_t byte;
		static int32_t read_len;
		static uint16_t data_len;
		static uint8_t unpack_step;
		static uint8_t protocol_packet[PROTOCAL_FRAME_MAX_SIZE];
    /*初始化读取状态*/
    buff_read_index = 0;
    memcpy(RFdataBuff,data_buf,length);
    /*从头开始读取 */
    read_len=length;

    while (read_len--) {
        byte = RFdataBuff[buff_read_index++];
        switch(unpack_step) {
        case STEP_HEADER_SOF: {
            if(byte == START_ID) {
                unpack_step = STEP_LENGTH_LOW;
                protocol_packet[index++] = byte;
            } else {
                index = 0;
            }
        }
        break;

        case STEP_LENGTH_LOW: {
            data_len = byte;
            protocol_packet[index++] = byte;
            unpack_step = STEP_LENGTH_HIGH;
        }
        break;

        case STEP_LENGTH_HIGH: {
            data_len |= (byte << 8);
            protocol_packet[index++] = byte;
            if(data_len < (PROTOCAL_FRAME_MAX_SIZE - HEADER_LEN - CRC_ALL_LEN)) {
                unpack_step = STEP_FRAME_SEQ;
            } else {
                unpack_step = STEP_HEADER_SOF;
                index = 0;
            }
        }
        break;

        case STEP_FRAME_SEQ: {
            protocol_packet[index++] = byte;
            unpack_step = STEP_HEADER_CRC8;
        }
        break;

        case STEP_HEADER_CRC8: {
            protocol_packet[index++] = byte;

            if (index == HEADER_LEN+1) {
                if ( Get_CRC8_Check_Sum(protocol_packet, HEADER_LEN,0xff)== protocol_packet[HEADER_LEN]) {
                    unpack_step = STEP_DATA_CRC16;
                } else {
                    unpack_step = STEP_HEADER_SOF;
                    index = 0;
                }
            }
        }
        break;

        case STEP_DATA_CRC16: {
            if (index < (HEADER_LEN + CMD_LEN + data_len + CRC_ALL_LEN)) {
                protocol_packet[index++] = byte;
            }
            if (index >= (HEADER_LEN + CMD_LEN + data_len + CRC_ALL_LEN)) {

                CRC16_Function=Get_CRC16_Check_Sum(protocol_packet, HEADER_LEN + CMD_LEN + data_len +CRC_8_LEN,0xffff);
                CRC16_Referee=* (__packed short *)(&protocol_packet[index-2]);
                if ( CRC16_Function==CRC16_Referee) {
                    RefereeHandle(protocol_packet);
                }
                unpack_step = STEP_HEADER_SOF;
                index = 0;
            }
        }
        break;

        default: {
            unpack_step = STEP_HEADER_SOF;
            index = 0;
        }
        break;
        }
    }
}

/**
  * @brief  Receive and handle referee system data
  * @param  void
  * @retval void
  */
void referee_Classdef::RefereeHandle(uint8_t *data_buf)
{
    switch(((FrameHeader *)data_buf)->CmdID) {

			case GameState_ID: 
					GameState = *(ext_game_status_t*)(&data_buf[7]);
			break;
			
			case GameResult_ID: 
					GameResult = *(ext_game_result_t*)(&data_buf[7]);
			break;
			
			case GameRobotHP_ID: 
					GameRobotHP = *(ext_game_robot_HP_t*)(&data_buf[7]);
			break;	
			
			case DartStatus_ID: 
					DartStatus = *(ext_dart_status_t*)(&data_buf[7]);
			break;			
			
			case EventData_ID: 
					EventData = *(ext_event_data_t*)(&data_buf[7]);
			break;			
			
			case SupplyProjectileAction_ID: 
					SupplyAction = *(ext_supply_projectile_action_t*)(&data_buf[7]);
			break;			
			
			case RefereeWarning_ID: 
					RefereeWarning = *(ext_referee_warning_t*)(&data_buf[7]);
			break;			
			
			case DartRemainingTime_ID: 
					DartRemainTime = *(ext_dart_remaining_time_t*)(&data_buf[7]);
			break;			
			
			case GameRobotState_ID: 
					GameRobotState = *(ext_game_robot_status_t*)(&data_buf[7]);
					Calc_Robot_ID(GameRobotState.robot_id);
			break;			
						
			case PowerHeatData_ID: 
					PowerHeatData = *(ext_power_heat_data_t*)(&data_buf[7]);
			break;			
			
			case GameRobotPos_ID: 
					RobotPos = *(ext_game_robot_pos_t*)(&data_buf[7]);
			break;			
			
			case BuffMusk_ID: 
					RobotBuff = *(ext_buff_t*)(&data_buf[7]);
			break;			
			
			case AerialRobotEnergy_ID: 
					AerialEnergy = *(aerial_robot_energy_t*)(&data_buf[7]);
			break;			
			
			case RobotHurt_ID: 
					RobotHurt = *(ext_robot_hurt_t*)(&data_buf[7]);
			break;			
			
			case ShootData_ID: 
					ShootData = *(ext_shoot_data_t*)(&data_buf[7]);
			break;			
						
			case BulletRemaining_ID: 
					BulletRemaining = *(ext_bullet_remaining_t*)(&data_buf[7]);
			break;						
			
			case RFID_Status_ID: 
					RFID_Status = *(ext_rfid_status_t*)(&data_buf[7]);
			break;			
						
			case RobotComData_ID:
					RobotInteractiveHandle((robot_interactive_data_t*)(&data_buf[7]));
			break;
			
			default:
					break;
    }
}

/**
  * @brief  Calculate robot ID 
  * @param  local robot id
  * @retval 
  */
void referee_Classdef::Calc_Robot_ID(uint8_t local_id)
{
	if(local_id !=0 )	/* referee connection successful */
	{
		if(local_id < 10)	/* red */
		{
			robot_client_ID.hero = 1;
			robot_client_ID.engineer = 2;
			robot_client_ID.infantry_3 = 3;
			robot_client_ID.infantry_4 = 4;
			robot_client_ID.infantry_5 = 5;
			robot_client_ID.aerial = 6;
			robot_client_ID.sentry = 7;
			robot_client_ID.local = local_id;
			robot_client_ID.client = 0x0100 + local_id;
		}
		else	/* blue */
		{
			robot_client_ID.hero = 101;
			robot_client_ID.engineer = 102;
			robot_client_ID.infantry_3 = 103;
			robot_client_ID.infantry_4 = 104;
			robot_client_ID.infantry_5 = 105;
			robot_client_ID.aerial = 106;
			robot_client_ID.sentry = 107;
			robot_client_ID.local = local_id;
			robot_client_ID.client = 0x0164 + (local_id - 100);		
		}
	}
}

/**
  * @brief  Transfer user system data to the server through huart
	* @note  	Referee.CV_ToOtherRobot(, Referee.robot_client_ID.hero, 123);
  * @param  data1,data2,data3,data4:data to send
  * @retval void
  */
void referee_Classdef::CV_ToOtherRobot(uint8_t target_id, uint8_t* _data, uint8_t length)
{
	pack_send_robotData(RobotComData_ID, target_id, (uint8_t*)_data, length);
}

void referee_Classdef::RobotInteractiveHandle(robot_interactive_data_t* RobotInteractiveData_t)
{
	if(GameRobotState.robot_id == RobotInteractiveData_t->receiver_ID && GameRobotState.robot_id != 0) {
		if(RobotInteractiveData_t->data_cmd_id == RobotComData_ID) {
			/* 拷贝到对应的机器人数组 */
			memcpy(&robot_com_data[RobotInteractiveData_t->sender_ID], RobotInteractiveData_t->data, RobotInteractiveData_t->data[0]);
		}
	}
}

/**
 * @brief Set the painting layer
 * @note Referee.line_drawing(ADD_PICTURE, 400,500,1000,900,GREEN, test);
				 A large digital layer covers a small digital layer
 * @param 
 * @retval 
 */
void referee_Classdef::Set_DrawingLayer(uint8_t _layer)
{
	drawing.layer = _layer;
}

/**
 * @brief Draw a straight line at the UI interface
 * @note Referee.line_drawing(ADD_PICTURE, 400,500,1000,900,GREEN, test);
 * @param 
 * @retval 
 */
void referee_Classdef::line_drawing(drawOperate_e _operate_type, uint16_t startx,uint16_t starty,uint16_t endx,uint16_t endy,colorType_e vcolor, uint8_t name[])
{
	memcpy(drawing.graphic_name,name,3);
	drawing.operate_tpye = _operate_type;
	drawing.graphic_tpye = LINE;
	drawing.width=5;
	drawing.color=vcolor;
	drawing.start_x=startx;
	drawing.start_y=starty;
	drawing.end_x=endx;
	drawing.end_y=endy;
	pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)&drawing, sizeof(drawing));
}

/**
 * @brief 
 * @note Referee.rectangle_drawing(ADD_PICTURE, 700,300,200,900,GREEN, test);
 * @param 
 * @retval 
 */
void referee_Classdef::rectangle_drawing(drawOperate_e _operate_type, uint16_t startx,uint16_t starty,uint16_t length_,uint16_t width_,colorType_e vcolor, uint8_t name[])
{
	memcpy(drawing.graphic_name, name, 3);	
	drawing.operate_tpye = _operate_type;
	drawing.graphic_tpye = RECTANGLE;
	drawing.width=5;
	drawing.color=vcolor;
	drawing.start_x=startx;
	drawing.start_y=starty;
	drawing.end_x=startx+length_;
	drawing.end_y=starty+width_;
	pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)&drawing, sizeof(drawing));
}

/**
 * @brief 
 * @note Referee.rectangle_drawing(ADD_PICTURE, 700,300,200,900,GREEN, test);
 * @param 
 * @retval 
 */
void referee_Classdef::circle_drawing(drawOperate_e _operate_type, uint16_t centrex,uint16_t centrey,uint16_t r,colorType_e vcolor, uint8_t name[])
{
	memcpy(drawing.graphic_name, name, 3);		
	drawing.operate_tpye = _operate_type;
	drawing.graphic_tpye = CIRCLE;
	drawing.width=5;
	drawing.color=vcolor;
	drawing.start_x=centrex;
	drawing.start_y=centrey;
	drawing.radius=r;
	pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)&drawing, sizeof(drawing));
}
/**
 * @brief 
 * @note Referee.oval_drawing(ADD_PICTURE, 800,500,200,500,GREEN,test);	 
 * @param 
 * @retval 
 */
void referee_Classdef::oval_drawing(drawOperate_e _operate_type, uint16_t centrex,uint16_t centrey,uint16_t minor_semi_axis,uint16_t major_semi_axis,colorType_e vcolor, uint8_t name[])
{
	memcpy(drawing.graphic_name, name, 3);	
	drawing.operate_tpye = _operate_type;	
	drawing.graphic_tpye=OVAL ;
	drawing.width=5;
	drawing.color=vcolor;
	drawing.start_x=centrex;
	drawing.start_y=centrey;
	drawing.end_x=major_semi_axis;
	drawing.end_y=minor_semi_axis;
	pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)&drawing, sizeof(drawing));
	
}

/**
 * @brief 
 * @note Referee.arc_drawing(ADD_PICTURE, 800,500,300,500,30,150,PURPLE, test);
 * @param 
 * @retval 
 */
void referee_Classdef::arc_drawing(drawOperate_e _operate_type, uint16_t centrex,uint16_t centrey,uint16_t endx,uint16_t endy,int16_t start_angle_,int16_t end_angle_,colorType_e vcolor, uint8_t name[])
{
	memcpy(drawing.graphic_name, name, 3);	
	drawing.operate_tpye = _operate_type;	
	drawing.graphic_tpye=ARC ;
	drawing.width=5;
	drawing.color=vcolor;
	drawing.start_x=centrex;
	drawing.start_y=centrey;
	drawing.end_x=endx;
	drawing.end_y=endy;
	drawing.start_angle=start_angle_;
	drawing.end_angle=end_angle_;
	pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)&drawing, sizeof(drawing));

}
void referee_Classdef::float_drawing(drawOperate_e _operate_type, uint16_t startx,uint16_t starty, colorType_e vcolor, float data, uint8_t name[])
{
	memcpy(drawing.graphic_name, name, 3);	
	drawing.operate_tpye = _operate_type;	
	drawing.graphic_tpye=_FLOAT ;
	drawing.start_angle = 10;//需要调试
	drawing.end_angle = 2;//小数位有效个数
	drawing.width=5;
	drawing.color=vcolor;
	drawing.start_x=startx;
	drawing.start_y=starty;
	
	memcpy((void*)drawing.radius, &data, 4);	

	pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)&drawing, sizeof(drawing));

}
void referee_Classdef::int_drawing(drawOperate_e _operate_type, uint16_t startx,uint16_t starty,uint16_t size, uint8_t length,uint8_t character[], colorType_e vcolor, int32_t data,uint8_t name[])
{
	memcpy(drawing.graphic_name, name, 3);	
	drawing.operate_tpye = _operate_type;	
	drawing.graphic_tpye=_INT ;
	drawing.start_angle = 10;//需要调试
	drawing.width=5;
	drawing.color=vcolor;
	drawing.start_x=startx;
	drawing.start_y=starty;
	
	memcpy((void*)drawing.radius, &data, 4);	

	pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)&drawing, sizeof(drawing));

}


/**
 * @brief 
 * @note Referee.character_drawing(ADD_PICTURE, 800,500,30,3, test, BLUE, test);
				 Character length not exceeding 30		 
 * @param 
 * @retval 
 */
void referee_Classdef::character_drawing(drawOperate_e _operate_type, uint16_t startx,uint16_t starty,uint16_t size, uint8_t char_length,uint8_t character[], colorType_e vcolor, uint8_t name[])
{
	char_length = _referee_Constrain(char_length, (uint8_t)0, (uint8_t)30);
	
	memcpy(drawing.graphic_name, name, 3);		
	drawing.operate_tpye = _operate_type;	
	drawing.graphic_tpye=_CHAR ;
	drawing.width=5;
	drawing.color=vcolor;
	drawing.start_x=startx;
	drawing.start_y=starty;
	drawing.radius=size;
	drawing.start_angle=char_length;
	
	uint8_t drawing_len = sizeof(com_temp);
	memcpy((void*)com_temp, &drawing, drawing_len);	
	memcpy(&com_temp[drawing_len],character, char_length);//字符串
	
	pack_send_robotData(Drawing_Char_ID, robot_client_ID.client, (uint8_t*)&com_temp, drawing_len+char_length);
}

/**
 * @brief 
 * @note Referee.clean_one_picture(2, test);
 * @param 
 * @retval 
 */
void referee_Classdef::clean_one_picture(uint8_t vlayer,uint8_t name[])
{
	memcpy(drawing.graphic_name, name, 3);		
	drawing.layer = vlayer ;
	drawing.operate_tpye=CLEAR_ONE_PICTURE;
	pack_send_robotData(Drawing_1_ID, robot_client_ID.client, (uint8_t*)&drawing, sizeof(drawing));
}
/**
 * @brief 
 * @note Referee.clean_layer(2);
 * @param 
 * @retval 
 */
void referee_Classdef::clean_layer(uint8_t _layer)
{	
	cleaning.layer = _layer;
	cleaning.operate_tpye = CLEAR_ONE_LAYER;
	
	pack_send_robotData(Drawing_Clean_ID, robot_client_ID.client, (uint8_t*)&cleaning, sizeof(cleaning));

}
/**
 * @brief 
 * @note Referee.clean_all();
 * @param 
 * @retval 
 */
void referee_Classdef::clean_all()
{		
	cleaning.operate_tpye = CLEAR_ALL;	
	pack_send_robotData(Drawing_Clean_ID, robot_client_ID.client, (uint8_t*)&cleaning, sizeof(cleaning));
}
/**
 * @brief draw_ruler
 * @note 	#define CIRCLE 24  准心圆半径 
* @param _sys_time, sacle_num多少条刻度线(<9),ruler_tag第几条标尺, startpoint(标尺左上角起点), step(间距),scale_long(长刻度线的长度),scale_short
 * @retval done:1, if no done:0
 */
uint8_t referee_Classdef::UI_ruler(uint32_t _sys_time, uint8_t ruler_tag, uint8_t sacle_num,uint16_t start_x, uint16_t start_y, uint16_t step, uint16_t scale_long, uint16_t scale_short, colorType_e _color)
{	
	static uint8_t scale_cnt = 0;
	static uint8_t draw_cnt = 0;
	static uint32_t last_sys_time = 0;

	uint8_t name[6]="ruler";
	uint16_t axis_temp;
	
	if(draw_cnt<5) /* 画5次 */
	{
		if(_sys_time - 200 >= last_sys_time)	/* 单位: ms ，频率不能大于10Hz*/
		{
			axis_temp = start_y - scale_cnt*step;
			name[3] = ruler_tag;
			name[4] = scale_cnt;
			
			if(scale_cnt == 0)
			{
				line_drawing(ADD_PICTURE, start_x,start_y,start_x,start_y-(sacle_num+1)*step,_color, name); //竖				
			}
			else
			{
				if(scale_cnt%2 == 1)
					line_drawing(ADD_PICTURE, start_x,axis_temp,start_x+scale_long,axis_temp,_color, name);   //长
				else
					line_drawing(ADD_PICTURE, start_x,axis_temp,start_x+scale_short,axis_temp,_color, name);  //短
			}
			scale_cnt++;
			last_sys_time = _sys_time;						
			if(scale_cnt>sacle_num)
			{
				draw_cnt++;
				scale_cnt = 0;
			}
		}
		return 0;
	}
	else
	{
		scale_cnt = 0;
		draw_cnt = 0;
		return 1;
	}
}
/**
 * @brief 画拟合图形
 * @param *data 打包完的数组，格式为[x0,y0,x1,y1...]
 * @param length 数组长度，应该为偶数
 * @note 画图有问题的话可能是name重复了
 */
void referee_Classdef::Draw_FitingGraph(uint8_t *data, uint16_t length,colorType_e _color){
	static int draw_cnt = 0;//辅助命名图形
	static uint8_t name[3] = "xx";
	
	uint16_t point_cnt = length/2;
	for(uint16_t i = 0; i < point_cnt; i+=2){		
		name[0] = draw_cnt/255;
		name[1] = draw_cnt%255;		
		draw_cnt++;
		
		line_drawing(ADD_PICTURE, data[i],data[i+1],data[i+2],data[i+3],_color, name);
		
	}
}

/**
 * @brief 打包机器人间交互数据并发送
 * @param 
 */
void referee_Classdef::pack_send_robotData(uint16_t _data_cmd_id, uint16_t _receiver_ID, uint8_t* _data, uint16_t _data_len)
{
	static uint8_t temp[128];	
	
	DataHeader data_header;
	data_header.data_cmd_id = _data_cmd_id;
	data_header.send_ID = robot_client_ID.local;
	data_header.receiver_ID = _receiver_ID;
	
	uint8_t header_len = sizeof(data_header);
	memcpy((void*)temp, &data_header, header_len);
	memcpy((void*)(temp + header_len), _data, _data_len);
	
	if(data_header.receiver_ID == robot_client_ID.client)
		send_toReferee(StudentInteractiveHeaderData_ID, temp, header_len+_data_len, Client);
	else
		send_toReferee(StudentInteractiveHeaderData_ID, temp, header_len+_data_len, OtherRobot);
}

/**
 * @brief 发数据包给裁判系统，并做速率控制
 * @param _cmd_id，
 * @param _data，
 * @param data_len，
 * @param receiver，接收方是机器人还是客户端，决定数据需不需要发多次（客户端数据经常丢包，所以需要发多次）
 */
void referee_Classdef::send_toReferee(uint16_t _cmd_id, uint8_t* _data, uint16_t _data_len, receiverType_e _receiver)
{
	static uint8_t temp[128];	
	static uint8_t seq = 0;
	
	FrameHeader send_frame_header;

	send_frame_header.SOF = START_ID;
	send_frame_header.DataLength = _data_len;
	send_frame_header.Seq = seq++;
	send_frame_header.CRC8 = Get_CRC8_Check_Sum((uint8_t*)&send_frame_header,4,0xff);
	send_frame_header.CmdID = _cmd_id;
	
	uint8_t header_len = sizeof(send_frame_header);
	
	memcpy((void*)temp, &send_frame_header, header_len);//frame header
	memcpy((void*)(temp+header_len), _data, _data_len);//data
	* (__packed short *)(&temp[header_len + _data_len]) = Get_CRC16_Check_Sum(temp,header_len + _data_len,0xffff);//CRC16
	
	
	uint8_t send_cnt = 3;//传输次数
	uint16_t total_len = header_len + _data_len + 2;//header_len + _data_len + CRC16_len
	
	while(send_cnt != 0)
	{
		uint32_t now_time = Get_SystemTick()/1000;	//ms
		if(now_time > next_send_time)
		{
			HAL_UART_Transmit_DMA(refereeUart,temp,total_len);
			
			/* 计算下一次允许传输的时间 */
			next_send_time = now_time + float(total_len) / 3720 * 1000;
			
			if(_receiver == OtherRobot)
				send_cnt = 0;
			else
				send_cnt--;
		}	
		vTaskDelay(5);
	}
	
}


/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
