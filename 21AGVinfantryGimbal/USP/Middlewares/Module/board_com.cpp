/**
  ******************************************************************************
  * @file   ： Module_BoardCOM.cpp
  * @brief  ： 板间通信协议模块
  * @author ： 苏锌雨 13535064092
  * @date   ： July,2019
  ******************************************************************************
  ==============================================================================
                     ##### How to use this module #####
  ==============================================================================
  1)引入CANopen，通过对象字典实现指令，支持SDO和PDO通信；
	2)SDO与PDO通信的区别：SDO为双向通信，数据段包含对象地址和数据，每次SDO通信包含发送
		和回传两个帧；PDO为单向通信，数据段完全用来传输数据，效率高；
	3)预定义的PDO共有4个；
	4)建立PDO通信的流程：
		- 设置主站PDO1参数；
		- 主站进入预处理阶段，通过SDO配置PDO1的通信参数；
		- 关闭从站的PDO2，PDO3，PDO4；
		- 发送START指令，从站开始运行，此时通信建立；
	5)有可靠传输需求的地方，使用SDO通信实现，利用定时发送的SDO帧实现离线监测；
	6)使用CANopen标准规定的对象字典地址段，所有数据均规定了对象字典地址；如有新增数据，
		请参考标准对象字典地址段进行预定义，并在对象字典表中进行更新并说明记录：
		(规定的对象字典地址段)
		---------------------------------
		| 地址        | 变量入口 				|
		---------------------------------
		| 0000h       | 保留   					|
		| 0001h-009Fh | 保留   					|
		| 00A0h-0FFFh | 保留   					|
		| 1000h-1FFFh | 通信参数	   		|
		| 2000h-5FFFh | 实验室指定参数	|
		| 6000h-9FFFh | 标准设备区域    |
		| A000h-FFFFh | 保留						|
		---------------------------------
		
  ******************************************************************************
  */

#include "board_com.h"

void* Object_Dictionary[OD_MAX_INDEXS][OD_MAX_SUBS] = {NULL};
uint8_t Type_Memery[OD_MAX_INDEXS*OD_MAX_SUBS] = {0};
uint8_t Local_NodeID = 0;
QueueHandle_t SDO_PassBack_Queue = NULL;
	
uint8_t SDO_Mailbox_Classdef::size()
{
	return sizes;
}

/**
 * @brief  往邮箱队列后插入新帧
 * @param  input_Frame SDO帧
 * @retval void
 * @author 苏锌雨
 */ 
void SDO_Mailbox_Classdef::push_back(SDO_Frame_Typedef input_frame)
{
	if(sizes < MAX_MAILBOX_DEEPTH)
	{
		SDO_Mailbox[sizes] = input_frame;
		sizes++;
	}
}

/**
 * @brief  邮箱队列前移
 * @param  void
 * @retval void
 * @author 苏锌雨
 */ 
void SDO_Mailbox_Classdef::erase()
{
	static SDO_Frame_Typedef clear_frame = {0,0,0,0,0,NULL};
	
	if(sizes)
	{
		for(size_t i = 0; i < sizes - 1; i++)
		{
			SDO_Mailbox[i] = SDO_Mailbox[i+1];
		}
		SDO_Mailbox[sizes - 1] = clear_frame;
		sizes--;
	}
}

/**
 * @brief  重载[]操作符
 * @param  void
 * @retval SDO_Frame_Typedef
 * @author 苏锌雨
 */ 
SDO_Frame_Typedef SDO_Mailbox_Classdef::operator [](uint8_t num)
{
	return SDO_Mailbox[num];
}

void SDO_RequireList_Typedef::Register(void* ptr,uint16_t cob_id,uint16_t index,uint8_t sub_index,Object_Type_Enum type)
{
	if(sizes >= MAX_REQUIRE_LIST_DEEPTH)
	{
		return;
	}
	
	/* 注册表 储存注册的数据来源节点ID和地址 */
	SDO_Frame_Typedef data_config = {cob_id,SDO_ReadRecv,index,sub_index,4};
	
	/* 注册 */
	register_list[sizes] = ptr;
	require_list[sizes] = data_config;
	type_list[sizes] = type;
	sizes++;
	
}

uint8_t SDO_RequireList_Typedef::Get_Register_Index(uint16_t cob_id,uint16_t index,uint8_t sub_index)
{
	uint8_t register_index = 255;
	for(size_t i = 0; i < MAX_REQUIRE_LIST_DEEPTH; i++)
	{
		if(require_list[i].COB_ID == cob_id && require_list[i].Index == index && require_list[i].Sub_Index == sub_index)
		{
			register_index = i;
		}
	}
	return register_index;
}

template <typename T> T SDO_RequireList_Typedef::at(uint16_t cob_id,uint16_t index,uint8_t sub_index)
{
	T res;
	T* ptr;
	
	/* 获取对应数据索引 */
	uint8_t register_index = Get_Register_Index(cob_id,index,sub_index);
	
	if(register_index < MAX_REQUIRE_LIST_DEEPTH)
	{
		memcpy(&res,&require_list[register_index].Data[4],sizeof(T));
			
		/* 更新数据 */
		ptr = (T*)register_list[register_index];
		*ptr = res;
	}
	return res;
}

Object_Type_Enum SDO_RequireList_Typedef::Type(uint8_t register_index)
{
	/* 越界 */
	if(register_index >= MAX_REQUIRE_LIST_DEEPTH)
	{
		return FLOAT;
	}
	else
	{
		return type_list[register_index];
	}
}

void SDO_RequireList_Typedef::Update(SDO_Frame_Typedef frame,uint8_t register_index)
{
	require_list[register_index] = frame;
}

/**
 * @brief  往SDO发送邮箱里添加帧
 * @param  SDO_Frame_Typedef SDO_Frame SDO帧
 * @retval uint8_t
 * @author 苏锌雨
 */ 
uint8_t SDOManager_Classdef::SDO_Add_Message_To_Mailbox(SDO_Frame_Typedef SDO_Frame)
{
	/* 规定了发送邮箱的最大值 
		 防止容器溢出或其他奇怪的问题
	 */
	if(SDO_Tx_Mailbox.size() >= MAX_Deepth)
	{
		return ERROR_MAILBOX_FULL;
	}
	
	SDO_Tx_Mailbox.push_back(SDO_Frame);
	return COM_OK;
}

/**
 * @brief  往SDO发送邮箱里添加帧
 * @param  SDO帧具体参数
 * @retval uint8_t
 * @author 苏锌雨
 */ 
uint8_t SDOManager_Classdef::SDO_Add_Message_To_Mailbox(uint16_t device_id, 
																												uint8_t ndoe_id, 
																												SDO_WorkType_Enum work_type, 
																												uint16_t index, 
																												uint8_t sub_index, 
																												uint8_t * data, 
																												SDO_DataLength_Enum data_length)
{
	if(SDO_Tx_Mailbox.size() >= MAX_Deepth)
	{
		return ERROR_MAILBOX_FULL;
	}
	
	uint8_t ctrl_byte = Set_Ctrl_Byte(work_type,data_length);
	
	SDO_Frame_Typedef SDO_Tx_Frame = {
		(uint16_t)(device_id+ndoe_id), ctrl_byte, index, sub_index, data_length
	};
	/* SDO的数据段只有4个字节，前4个字节为控制数据段 */
	memset(&SDO_Tx_Frame.Data[4],0,4);
	memcpy(&SDO_Tx_Frame.Data[4],&data[0],(uint8_t)data_length);
	
	/* 设置SDO的控制段 */
	SDO_Tx_Frame.Data[0] = SDO_Tx_Frame.Ctrl_Byte;
	SDO_Tx_Frame.Data[1] = SDO_Tx_Frame.Index & 0xff;
	SDO_Tx_Frame.Data[2] = (SDO_Tx_Frame.Index >> 8) & 0xff;
	SDO_Tx_Frame.Data[3] = SDO_Tx_Frame.Sub_Index;
	
	SDO_Tx_Mailbox.push_back(SDO_Tx_Frame);
	return COM_OK;
}

/**
 * @brief  带回传等待的SDO帧发送函数 超时等待时间50ms
 * @param  SDO帧具体参数
 * @retval uint8_t
 * @author 苏锌雨
 */ 
uint8_t SDOManager_Classdef::SDO_Transmit(uint16_t device_id, 
																					uint8_t ndoe_id, 
																					SDO_WorkType_Enum work_type, 
																					uint16_t index, 
																					uint8_t sub_index, 
																					uint8_t * data, 
																					SDO_DataLength_Enum data_length,
																					uint8_t max_retran_times)
{
	uint8_t ctrl_byte = Set_Ctrl_Byte(work_type,data_length);
	
	uint8_t wait_ms = 0;
	uint8_t retran_times;
	
	uint32_t SDO_PassBack_Ptr;
	BaseType_t xTaskWokenByReceive = pdFALSE;
	
	SDO_Frame_Typedef SDO_Tx_Frame = {
		(uint16_t)(device_id+ndoe_id), ctrl_byte, index, sub_index, data_length
	};
	/* SDO的数据段只有4个字节，前4个字节为控制数据段 */
	memset(&SDO_Tx_Frame.Data[4],0,4);
	memcpy(&SDO_Tx_Frame.Data[4],&data[0],(uint8_t)data_length);
	
	/* 设置SDO的控制段 */
	SDO_Tx_Frame.Data[0] = SDO_Tx_Frame.Ctrl_Byte;
	SDO_Tx_Frame.Data[1] = SDO_Tx_Frame.Index & 0xff;
	SDO_Tx_Frame.Data[2] = (SDO_Tx_Frame.Index >> 8) & 0xff;
	SDO_Tx_Frame.Data[3] = SDO_Tx_Frame.Sub_Index;
	
	/* 发送SDO帧 */
	do{
		CANx_SendData(hcan,SDO_Tx_Frame.COB_ID,SDO_Tx_Frame.Data,8);
		
		/* 等待直到收到回传 */		
		while(wait_ms++ < 50)
		{
			vTaskDelay(1);
			
			if(xQueueReceiveFromISR(SDO_PassBack_Queue,&SDO_PassBack_Ptr,&xTaskWokenByReceive) == pdTRUE)
			{
				return COM_OK;
			}
		}

	if(retran_times++ >= max_retran_times)
	{
		return ERROR_TIMEOUT;
	}
	
	}while(true);
}

SDOManager_Classdef::SDOManager_Classdef(CAN_HandleTypeDef* hcanx):MAX_Deepth(MAX_MAILBOX_DEEPTH),hcan(hcanx)
{
}

void SDOManager_Classdef::Init()
{
	if(SDO_PassBack_Queue == NULL)
	{
		SDO_PassBack_Queue = xQueueCreate(1,sizeof(uint32_t));
	}
}

/**
 * @brief  接收回调函数
 * @param  CAN包
 * @retval uint8_t
 * @author 苏锌雨
 */ 
void SDOManager_Classdef::SDO_Recv_Callback(CAN_RxMessage* recv_msg)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	uint32_t SDO_PassBack;
	
	SDO_Frame_Typedef SDO_Rx_Frame = {(uint16_t)(recv_msg->header.StdId),recv_msg->data[0],(uint16_t)(recv_msg->data[1]+recv_msg->data[2]*256),recv_msg->data[3],SDO_Data_4Byte};
	memcpy(&SDO_Rx_Frame.Data[0],recv_msg->data,8);
	
	/* 如果为回传信号 通过信号量发送出去 */
	if(SDO_Rx_Frame.Ctrl_Byte == SDO_WriteRecv)
	{
		memcpy(&SDO_PassBack,&SDO_Rx_Frame.Data[4],4);
		xQueueSendFromISR(SDO_PassBack_Queue,(void*)&SDO_PassBack,&xHigherPriorityTaskWoken);
	}
	
	SDO_Rx_Mailbox.push_back(SDO_Rx_Frame);
}

/**
 * @brief  SDO协议执行函数 任务调用
 * @param  void
 * @retval void
 * @author 苏锌雨
 */ 
void SDOManager_Classdef::SDO_Exce_Once()
{
	/* 初始化回传abort code */
	uint8_t abort_code[4] = {0};
	uint8_t read_data[4] = {0};
	
	if(SDO_PassBack_Queue == NULL)
	{
		SDO_PassBack_Queue = xQueueCreate(1,sizeof(uint32_t));
	}
	
	/* 清理发送邮箱 */
	if(SDO_Tx_Mailbox.size())
	{
		if(CANx_SendData(hcan,SDO_Tx_Mailbox[0].COB_ID,&SDO_Tx_Mailbox[0].Data[0],8) == 0)
		{
			SDO_Tx_Mailbox.erase();
		}
	}
	
	/* 处理接收邮箱 */
	while(SDO_Rx_Mailbox.size())
	{
		switch(SDO_Rx_Mailbox[0].Ctrl_Byte & 0xf1)
		{
			case SDO_ReadSend:
				if(Object_Read(SDO_Rx_Mailbox[0].Index,SDO_Rx_Mailbox[0].Sub_Index,read_data) == 0)
				{
					SDO_Add_Message_To_Mailbox(DEVICE_ID_RX,Local_NodeID,SDO_ReadRecv,SDO_Rx_Mailbox[0].Index,SDO_Rx_Mailbox[0].Sub_Index,read_data,SDO_Data_4Byte);
				}
				/* 无法读取 */
				else
				{
					Set_Abort_Code(AC_OUT_OF_MEMERORY,abort_code);
					SDO_Add_Message_To_Mailbox(DEVICE_ID_RX,Local_NodeID,SDO_ReadRecv,SDO_Rx_Mailbox[0].Index,SDO_Rx_Mailbox[0].Sub_Index,abort_code,SDO_Data_4Byte);
				}
				break;
			case SDO_ReadRecv:
				/* 更新数据 */
				Update(SDO_Rx_Mailbox[0]);
				break;
			case SDO_WriteSend:
				if(Object_Write(SDO_Rx_Mailbox[0].Index,SDO_Rx_Mailbox[0].Sub_Index,SDO_Rx_Mailbox[0].Data) == 0)
				{
					Set_Abort_Code(AC_NO_ERROR,abort_code);
					SDO_Add_Message_To_Mailbox(DEVICE_ID_RX,Local_NodeID,SDO_WriteRecv,SDO_Rx_Mailbox[0].Index,SDO_Rx_Mailbox[0].Sub_Index,abort_code,SDO_Data_4Byte);
				}
				/* 不可写入 */
				else
				{
					Set_Abort_Code(AC_OUT_OF_MEMERORY,abort_code);
					SDO_Add_Message_To_Mailbox(DEVICE_ID_RX,Local_NodeID,SDO_WriteRecv,SDO_Rx_Mailbox[0].Index,SDO_Rx_Mailbox[0].Sub_Index,abort_code,SDO_Data_4Byte);
				}
				break;
			case SDO_WriteRecv:
				break;
		}
		SDO_Rx_Mailbox.erase();
	}
	
}

/**
 * @brief  请求读取一个数据
 * @param  地址
 * @retval uint8_t
 * @author 苏锌雨
 */ 
void SDOManager_Classdef::Pull(void * ptr,uint8_t node_id,uint16_t index,uint8_t sub_index,Object_Type_Enum type) 
{
	uint8_t temp_res[4];
	SDO_Add_Message_To_Mailbox(DEVICE_ID_TX,node_id,SDO_ReadSend,index,sub_index,temp_res,SDO_Data_0Byte);
	
	if(SDO_RequireList.Get_Register_Index(DEVICE_ID_RX+node_id,index,sub_index) >= MAX_REQUIRE_LIST_DEEPTH)
	{
		/* 注册请求数据 */
		SDO_RequireList.Register(ptr,DEVICE_ID_RX+node_id,index,sub_index,type);
	}
}

/**
 * @brief  请求写一个数据
 * @param  地址
 * @retval uint8_t
 * @author 苏锌雨
 */ 
void SDOManager_Classdef::Push(void * Ptr,uint8_t node_id,uint16_t index,uint8_t sub_index,Object_Type_Enum type)
{
	uint8_t temp_res[4];
	memcpy(&temp_res[0],Ptr,int(type)/3+1);
	SDO_Add_Message_To_Mailbox(DEVICE_ID_TX,node_id,SDO_WriteSend,index,sub_index,temp_res,SDO_DataLength_Enum(int(type)/3+1));
	
}	

/**
 * @brief  更新一个数据
 * @param  SDO数据帧
 * @retval void
 * @author 苏锌雨
 */ 
void SDOManager_Classdef::Update(SDO_Frame_Typedef frame)
{
	/* 如果该数据已注册 */
	uint8_t register_index = SDO_RequireList.Get_Register_Index(frame.COB_ID,frame.Index,frame.Sub_Index);
	if(register_index < MAX_REQUIRE_LIST_DEEPTH)
	{
		/* 更新数据到注册表 */
		SDO_RequireList.Update(frame,register_index);
		
		switch (SDO_RequireList.Type(register_index))
		{
			case UINT8:
			SDO_RequireList.at<uint8_t>(frame.COB_ID,frame.Index,frame.Sub_Index);
			break;
		case INT8:
			SDO_RequireList.at<int8_t>(frame.COB_ID,frame.Index,frame.Sub_Index);
			break;
		case UINT16:
			SDO_RequireList.at<uint16_t>(frame.COB_ID,frame.Index,frame.Sub_Index);
			break;
		case INT16:
			SDO_RequireList.at<int16_t>(frame.COB_ID,frame.Index,frame.Sub_Index);
			break;
		case UINT32:
			SDO_RequireList.at<uint32_t>(frame.COB_ID,frame.Index,frame.Sub_Index);
			break;
		case INT32:
			SDO_RequireList.at<int32_t>(frame.COB_ID,frame.Index,frame.Sub_Index);
			break;
		case FLOAT:
			SDO_RequireList.at<float>(frame.COB_ID,frame.Index,frame.Sub_Index);
			break;
		}
	}
}

/**
 * @brief  将对象Map到PDO帧
 * @param  对象数和对象(变长参数)
 * @retval 错误码
 * @author 苏锌雨 
 */ 
uint8_t PDO_Typedef::Map(uint8_t n,uint32_t objects[])
{
	uint8_t total_size = 0;
	
	uint32_t temp_object[4] = {0};
	
	for(size_t i = 0; i < n; i++)
	{
		temp_object[i] = objects[i];
		total_size += (temp_object[i]&0xff);
	}

	if(total_size <= 64)
	{
		memcpy(this->Mapped_Objects,temp_object,n*sizeof(uint32_t));
		this->Mapped_Nums = n;
		
		return COM_OK;
	}
	
	return ERROR_MAPERROR;
	
}

/**
 * @brief  获取该PDO Map的对象数
 * 				 等于0则未Map
 * @param  void包
 * @retval 对象数
 * @author 苏锌雨 
 */ 
uint8_t PDO_Typedef::Get_Mapped_Nums()
{
	return this->Mapped_Nums;
}

/**
 * @brief  将RXPDO帧的数据更新到注册的地址中
 * @param  CAN包
 * @retval 错误码
 * @author 苏锌雨 
 */ 
uint8_t RXPDO_Typedef::Update(CAN_RxMessage* recv_msg)
{
	uint8_t bias = 0;
	void* addr = NULL;
	for(size_t i = 0; i < this->Mapped_Nums; i++)
	{
		/* 通过是否注册数据暂存地址判断是主动请求还是写入 */
		if(Register_Buff[i] == NULL)
		{
			/* 写入 */
			addr = Addr_Read(Mapped_Objects[i]>>16,(Mapped_Objects[i]>>8)&0xff);
			memcpy(addr,&recv_msg->data[bias],(Mapped_Objects[i]&0xff)/8);
			
			if(addr == NULL)
			{
				return ERROR_UNREGISTED;
			}
		}
		else
		{
			/* 主动请求 */
			memcpy(Register_Buff[i],&recv_msg->data[bias],(Mapped_Objects[i]&0xff)/8);
		}
		bias += (Mapped_Objects[i]&0xff)/8;
	}
	return COM_OK;
}

/**
 * @brief  注册RXPDO帧的数据缓存地址
 * @param  地址
 * @retval 错误码
 * @author 苏锌雨 
 */ 
void RXPDO_Typedef::Regster(uint8_t n,void* Ptr[])
{
	for(size_t i = 0; i < n ; i++)
	{
		Register_Buff[i] = Ptr[i];
	}
}

/**
 * @brief  把数据打包为TXPDO帧
 * @param  PDO帧的COB-ID
 * @retval PDO帧
 * @author 苏锌雨 
 */ 
PDO_Frame_Typedef TXPDO_Typedef::Pack()
{
	PDO_Frame_Typedef res = {0,NULL};
	if(Mapped_Nums > 0)
	{
		res.COB_ID = this->COB_ID;
		uint8_t bias = 0;
		for(size_t i = 0; i < Mapped_Nums; i++)
		{
			Object_Read(Mapped_Objects[i]>>16,(Mapped_Objects[i]>>8)&0xff,&res.Data[bias]);
			bias += (Mapped_Objects[i]&0xff)/8;
		}
	}
	return res;
}

/**
 * @brief  PDO通信初始化 注册通信参数地址
 * @param  CAN包
 * @retval uint8_t
 * @author 苏锌雨 
 */ 
PDOManager_Classdef::PDOManager_Classdef(CAN_HandleTypeDef* hcanx):hcan(hcanx)
{
	for(size_t i = 0; i < 4; i++)
	{
		/* 通信参数 */
		Object_Dict_Register(&this->RxPDO[i].COB_ID,0x1400+i,0x01,ReadNWrite,UINT32);
		Object_Dict_Register(&this->RxPDO[i].PDO_Type,0x1400+i,0x02,ReadNWrite,UINT8);
		Object_Dict_Register(&this->TxPDO[i].COB_ID,0x1800+i,0x01,ReadNWrite,UINT32);
		Object_Dict_Register(&this->TxPDO[i].PDO_Type,0x1800+i,0x02,ReadNWrite,UINT8);
		
		Object_Dict_Register(&this->RxPDO[i].Mapped_Nums,0x1600+i,0x00,ReadNWrite,UINT8);
		Object_Dict_Register(&this->RxPDO[i].Mapped_Objects[0],0x1600+i,0x01,ReadNWrite,UINT32);
		Object_Dict_Register(&this->RxPDO[i].Mapped_Objects[1],0x1600+i,0x02,ReadNWrite,UINT32);
		Object_Dict_Register(&this->RxPDO[i].Mapped_Objects[2],0x1600+i,0x03,ReadNWrite,UINT32);
		Object_Dict_Register(&this->RxPDO[i].Mapped_Objects[3],0x1600+i,0x04,ReadNWrite,UINT32);
		
		Object_Dict_Register(&this->TxPDO[i].Mapped_Nums,0x1A00+i,0x00,ReadNWrite,UINT8);
		Object_Dict_Register(&this->TxPDO[i].Mapped_Objects[0],0x1A00+i,0x01,ReadNWrite,UINT32);
		Object_Dict_Register(&this->TxPDO[i].Mapped_Objects[1],0x1A00+i,0x02,ReadNWrite,UINT32);
		Object_Dict_Register(&this->TxPDO[i].Mapped_Objects[2],0x1A00+i,0x03,ReadNWrite,UINT32);
		Object_Dict_Register(&this->TxPDO[i].Mapped_Objects[3],0x1A00+i,0x04,ReadNWrite,UINT32);
	}
}

/**
 * @brief  读取接收到的PDO帧
 * @param  CAN包
 * @retval uint8_t
 * @author 苏锌雨 
 */ 
void PDOManager_Classdef::PDO_Recv_Callback(CAN_RxMessage* recv_msg)
{
	switch(recv_msg->header.StdId & 0xf80)
	{
		case 0x180:
			RxPDO[0].Update(recv_msg);
			PDO_WatchDog[0].Feed();
			break;
		case 0x280:
			RxPDO[1].Update(recv_msg);
			PDO_WatchDog[1].Feed();
			break;
		case 0x380:
			RxPDO[2].Update(recv_msg);
			PDO_WatchDog[2].Feed();
			break;
		case 0x480:
			RxPDO[3].Update(recv_msg);
			PDO_WatchDog[3].Feed();
			break;
	}
}

/**
 * @brief  发送所有Map过的PDO帧(一次)
 * @param  void
 * @retval void
 * @author 苏锌雨
 */ 
void PDOManager_Classdef::PDO_Exce_Once()
{
	PDO_Frame_Typedef tx_frame;
	for(size_t i = 0; i < 4; i++)
	{
		if(TxPDO[i].Get_Mapped_Nums() > 0)
		{
			tx_frame = TxPDO[i].Pack();
			if(xTaskGetTickCount() - TxPDO[i].Last_Push_Time > TxPDO[i].Gape_ms)
			{
				CANx_SendData(hcan,tx_frame.COB_ID,&tx_frame.Data[0],8);
				TxPDO[i].Last_Push_Time = xTaskGetTickCount();
			}
		}
	}
	
	/* 更新PDO看门狗 */
	PDOWDog_Bark();
}

/**
 * @brief  通过PDO Mapping请求数据
 * @param  节点ID，PDO索引，对象数，对象，数据缓存地址，触发类型
 * @retval 错误码
 * @author 苏锌雨
 */ 
uint8_t PDOManager_Classdef::Pull(uint8_t node_id,uint8_t PDO_Index,uint8_t n,uint32_t objects[],void* reg_buff[],PDO_TriggerType_Enum trigger_type)
{
	uint8_t temp_data[4] = {(uint8_t)(0x80 + Local_NodeID), (uint8_t)0x01, (uint8_t)0x00, (uint8_t)0x40};
	uint8_t res = COM_OK;

	if(PDO_Index < 4)
	{
		RxPDO[PDO_Index].Map(n,objects);
		RxPDO[PDO_Index].Regster(n,reg_buff);
		
		PDO_WatchDog[PDO_Index].Start();
		
		/* 发送SDO帧 请求建立PDO通信 */
		/* 写入TxPDO参数 */
		/* Map对象数 */
		if(Transmit_SDO(node_id,0x1A00+PDO_Index,0x00,&n,SDO_Data_1Byte,2) == ERROR_TIMEOUT)
		{
			res |= ERROR_TIMEOUT;
		}
		/* COB-ID */
		if(Transmit_SDO(node_id,0x1800+PDO_Index,0x01,temp_data,SDO_Data_4Byte,2) == ERROR_TIMEOUT)
		{
			res |= ERROR_TIMEOUT;
		}
		for(size_t i = 0; i < n; i++)
		{
			memcpy(temp_data,&objects[i],4);
			/* 对象地址 */
			if(Transmit_SDO(node_id,0x1A00+PDO_Index,0x01+i,temp_data,SDO_Data_4Byte,2) == ERROR_TIMEOUT)
			{
				res |= ERROR_TIMEOUT;
			}
		}
		return res;
	}
	else
	{
		return ERROR_PDOMAPPED;
	}

}

uint8_t PDOManager_Classdef::Push(uint8_t node_id,uint8_t PDO_Index,uint8_t n,uint32_t objects[],PDO_TriggerType_Enum trigger_type,uint16_t gape_ms)
{
	uint8_t temp_data[4] = {(uint8_t)(0x00 + Local_NodeID), (uint8_t)0x02, (uint8_t)0x00, (uint8_t)0x00};
	uint8_t res = COM_OK;
	
	if(PDO_Index < 4)
	{
		TxPDO[PDO_Index].Map(n,objects);
		TxPDO[PDO_Index].COB_ID = 0x180+node_id+(PDO_Index<<8);
		TxPDO[PDO_Index].Gape_ms = gape_ms;
		
		/* 发送SDO帧 请求建立PDO通信 */ 
		/* 写入RxPDO参数 */
		/* Map对象数 */
		if(Transmit_SDO(node_id,0x1600+PDO_Index,0x00,&n,SDO_Data_1Byte,2) == ERROR_TIMEOUT)
		{
			res |= ERROR_TIMEOUT;
		}
		/* COB-ID */
		if(Transmit_SDO(node_id,0x1400+PDO_Index,0x01,temp_data,SDO_Data_4Byte,2) == ERROR_TIMEOUT)
		{
			res |= ERROR_TIMEOUT;
		}
		for(size_t i = 0; i < n; i++)
		{
			memcpy(temp_data,&objects[i],4);
			/* 对象地址 */
			if(Transmit_SDO(node_id,0x1600+PDO_Index,0x01+i,temp_data,SDO_Data_4Byte,2) == ERROR_TIMEOUT)
			{
				res |= ERROR_TIMEOUT;
			}
		}
		return res;
	}
	else
	{
		return ERROR_PDOMAPPED;
	}
	
}

void PDOManager_Classdef::Force_Map_Rx(uint8_t RxPDO_Index,uint8_t n,uint32_t objects[])
{
	switch (RxPDO_Index)
	{
		case 0:
			this->RxPDO[0].Mapped_Nums = n;
			memcpy(this->RxPDO[0].Mapped_Objects,objects,n*4);
			break;
		case 1:
			this->RxPDO[1].Mapped_Nums = n;
			memcpy(this->RxPDO[1].Mapped_Objects,objects,n*4);
			break;
		case 2:
			this->RxPDO[2].Mapped_Nums = n;
			memcpy(this->RxPDO[2].Mapped_Objects,objects,n*4);
			break;
		case 3:
			this->RxPDO[3].Mapped_Nums = n;
			memcpy(this->RxPDO[3].Mapped_Objects,objects,n*4);
			break;
	}
}

	/* PDO看门狗计时 */
void PDOManager_Classdef::PDOWDog_Bark()
{
	for(size_t i = 0;i < 4; i++)
	{
		PDO_WatchDog[i].Bark();
	}
}
	/* PDO状态 */
uint8_t PDOManager_Classdef::PDO_Status(uint8_t PDO_Index)
{
	if(PDO_Index >= 4)
	{
		for(size_t i = 0;i < 4; i++)
		{
			if(PDO_WatchDog[i].Status() == ERROR_TIMEOUT)
			{
				return ERROR_TIMEOUT;
			}
		}
		return COM_OK;
	}
	else
	{
		return PDO_WatchDog[PDO_Index].Status(); 
	}
}

uint8_t PDOManager_Classdef::Transmit_SDO(uint8_t ndoe_id, 
																				 uint16_t index, uint8_t sub_index, 
																				 uint8_t * data, 
																				 SDO_DataLength_Enum data_length,
																				 uint8_t max_retran_times) 
{
	uint8_t ctrl_byte;
	uint32_t SDO_PassBack_Ptr; 
	BaseType_t xTaskWokenByReceive = pdFALSE;
	uint8_t wait_ms = 0;
	uint8_t retran_times = 0;
	
	if(data_length == SDO_Data_1Byte)
		ctrl_byte = 0x2f;
	else if(data_length == SDO_Data_2Byte)
		ctrl_byte = 0x2b;
	else if(data_length == SDO_Data_4Byte)
		ctrl_byte = 0x23;
	
	SDO_Frame_Typedef SDO_Tx_Frame = { 
		(uint16_t)(DEVICE_ID_TX+ndoe_id), ctrl_byte, index, sub_index, data_length
	};
	
	/* SDO的数据段只有4个字节，前4个字节为控制数据段 */
	memset(&SDO_Tx_Frame.Data[4],0,4);
	memcpy(&SDO_Tx_Frame.Data[4],&data[0],(uint8_t)data_length);
	
	/* 设置SDO的控制段 */
	SDO_Tx_Frame.Data[0] = SDO_Tx_Frame.Ctrl_Byte;
	SDO_Tx_Frame.Data[1] = SDO_Tx_Frame.Index & 0xff;
	SDO_Tx_Frame.Data[2] = (SDO_Tx_Frame.Index >> 8) & 0xff;
	SDO_Tx_Frame.Data[3] = SDO_Tx_Frame.Sub_Index;
	
	vTaskDelay(3);
	
	/* 发送SDO帧 */
	do{
		CANx_SendData(hcan,SDO_Tx_Frame.COB_ID,SDO_Tx_Frame.Data,8);
		
		wait_ms = 0;
		/* 等待直到收到回传 */		
		while(wait_ms++ < 50)
		{
			vTaskDelay(1);
			
			if(xQueueReceiveFromISR(SDO_PassBack_Queue,&SDO_PassBack_Ptr,&xTaskWokenByReceive) == pdTRUE)
			{
				return COM_OK;
			}
		}

	if(retran_times++ >= max_retran_times) 
	{
		return ERROR_TIMEOUT;
	}
	
	}while(true);
}

void PDO_WatchDog_Classdef::Start()
{
	isStarted = 1;
}

void PDO_WatchDog_Classdef::Bark()
{
	if(isStarted)
	{
		if(Waitms <= MaxWaitms)
		{
				Waitms++;
		}
	}
	else
	{
		Waitms = 0;
	}
}

void PDO_WatchDog_Classdef::Feed()
{
	Waitms = 0;
}	

uint8_t PDO_WatchDog_Classdef::Status()
{
	if((Waitms <= MaxWaitms && isStarted) || !isStarted)
	{
		return COM_OK;
	}
	return ERROR_TIMEOUT;

}

/**
 * @brief  将Abort Code写入数据段
 * @param  Abort Code，数据段地址
 * @retval void
 * @author 苏锌雨
 */ 
void Set_Abort_Code(uint32_t abort_code, uint8_t* data)
{
	data[0] = abort_code&0xff;
	data[1] = (abort_code>>8)&0xff;
	data[2] = (abort_code>>16)&0xff;
	data[3] = (abort_code>>24)&0xff;
}

/**
 * @brief  通过SDO类型和数据长度设置Ctrl Byte
 * @param  工作类型，数据长度
 * @retval Ctrl Byte
 * @author 苏锌雨
 */ 
uint8_t Set_Ctrl_Byte(SDO_WorkType_Enum work_type,SDO_DataLength_Enum data_length)
{
	uint8_t ctrl_byte;
	switch(work_type)
	{
		case SDO_ReadSend:
			ctrl_byte = 0x40;
			break;
		case SDO_ReadRecv:
			if(data_length == SDO_Data_1Byte)
				ctrl_byte = 0x4f;
			else if(data_length == SDO_Data_2Byte)
				ctrl_byte = 0x4b;
			else if(data_length == SDO_Data_4Byte)
				ctrl_byte = 0x43;
			break;
		case SDO_WriteSend:
			if(data_length == SDO_Data_1Byte)
				ctrl_byte = 0x2f;
			else if(data_length == SDO_Data_2Byte)
				ctrl_byte = 0x2b;
			else if(data_length == SDO_Data_4Byte)
				ctrl_byte = 0x23;
			break;
		case SDO_WriteRecv:
			ctrl_byte = 0x60;
			break;
	}
	return ctrl_byte;
}

/**
 * @brief  对象字典注册
 * @param  对象信息，注册地址
 * @retval void
 * @author 苏锌雨
 */ 
void Object_Dict_Register(void* param_ptr, uint16_t index, uint8_t sub_index, bool RW, Object_Type_Enum type)
{
	/* 地址转换 */
	index = Addr_Transfer(index);
	
	if(index >= OD_MAX_INDEXS || sub_index >= OD_MAX_SUBS)
	{
		return;
	}
	
	/* 注册 */
	Object_Dictionary[index][sub_index] = param_ptr;
	Type_Memery[index*OD_MAX_SUBS+sub_index] = type << 1;
	Type_Memery[index*OD_MAX_SUBS+sub_index] |= RW;
}

/**
 * @brief  查询对象R/W
 * @param  对象地址
 * @retval bool
 * @author 苏锌雨
 */ 
bool Object_RW(uint16_t index, uint8_t sub_index)
{
	/* 地址转换 */
	index = Addr_Transfer(index);
	
	if(index*OD_MAX_SUBS+sub_index < OD_MAX_INDEXS*OD_MAX_SUBS)
	{
		return Type_Memery[index*OD_MAX_SUBS+sub_index] & 0x01;
	}
	
	return ReadOnly;
}

/**
 * @brief  查询对象类型
 * @param  对象地址
 * @retval Object_Type_Enum 对象类型枚举
 * @author 苏锌雨
 */ 
Object_Type_Enum Object_Type(uint16_t index, uint8_t sub_index)
{
	/* 地址转换 */
	index = Addr_Transfer(index);
	
	if(index*OD_MAX_SUBS+sub_index < OD_MAX_INDEXS*OD_MAX_SUBS)
	{
		return Object_Type_Enum(Type_Memery[index*OD_MAX_SUBS+sub_index] >> 1);
	}
	
	return UINT8;
}

/**
 * @brief  写入对象
 * @param  对象地址，数据
 * @retval uint8_t(错误类型)
 * @author 苏锌雨
 */ 
uint8_t Object_Write(uint16_t index, uint8_t sub_index, uint8_t * Data)
{
	static uint8_t * data_uint8;
	static int8_t * data_int8;
	static uint16_t * data_uint16;
	static int16_t * data_int16;
	static uint32_t * data_uint32;
	static int32_t * data_int32;
	static float* data_float;
	
	void* addr = Addr_Read(index,sub_index);
	
	if(addr == NULL)
	{
		return ERROR_INDEXERROR;
	}

	switch(Object_Type(index,sub_index))
	{
		case UINT8:
			data_uint8 = (uint8_t*)addr;
			*data_uint8 = uint8_t(Data[4]);
			break;
		case INT8:
			data_int8 = (int8_t*)addr;
			*data_int8 = int8_t(Data[4]);
			break;
		case UINT16:
			data_uint16 = (uint16_t*)addr;
			*data_uint16 = uint16_t(Data[4]|(Data[5]<<8));
			break;
		case INT16:
			data_int16 = (int16_t*)addr;
			*data_int16 = int16_t(Data[4]|(Data[5]<<8));
			break;
		case UINT32:
			data_uint32 = (uint32_t*)addr;
			*data_uint32 = uint32_t(Data[4]|(Data[5]<<8)|(Data[6]<<16)|(Data[7]<<24));
			break;
		case INT32:
			data_int32 = (int32_t*)addr;
			*data_int32 = int32_t(Data[4]|(Data[5]<<8)|(Data[6]<<16)|(Data[7]<<24));
			break;
		case FLOAT:
			data_float = (float*)addr;
			memcpy(&data_float,&Data[4],4);
			break;
	}
	
	return COM_OK;
}

/**
 * @brief  读出对象
 * @param  对象地址，存放数据的内存
 * @retval uint8_t(错误类型)
 * @author 苏锌雨
 */ 
uint8_t Object_Read(uint16_t index, uint8_t sub_index, uint8_t * Data)
{
	static uint8_t * data_uint8;
	static int8_t * data_int8;
	static uint16_t * data_uint16;
	static int16_t * data_int16;
	static uint32_t * data_uint32;
	static int32_t * data_int32;
	static float* data_float;
	
	void* addr = Addr_Read(index,sub_index);
	
	if(addr == NULL)
	{
		return ERROR_INDEXERROR;
	}
	
	switch(Object_Type(index,sub_index))
	{
		case UINT8:
			data_uint8 = (uint8_t*)addr;
			memcpy(&Data[0],data_uint8,1);
			break;
		case INT8:
			data_int8 = (int8_t*)addr;
			memcpy(&Data[0],data_int8,1);
			break;
		case UINT16:
			data_uint16 = (uint16_t*)addr;
			memcpy(&Data[0],data_uint16,2);
			break;
		case INT16:
			data_int16 = (int16_t*)addr;
			memcpy(&Data[0],data_int16,2);
			break;
		case UINT32:
			data_uint32 = (uint32_t*)addr;
			memcpy(&Data[0],data_uint32,4);
			break;
		case INT32:
			data_int32 = (int32_t*)addr;
			memcpy(&Data[0],data_int32,4);
			break;
		case FLOAT:
			data_float = (float*)addr;
			memcpy(&Data[0],data_float,4);
			break;
	}
	
	return COM_OK;
}

void* Addr_Read(uint16_t index, uint8_t sub_index)
{
	index = Addr_Transfer(index);
	
	if(index >= OD_MAX_INDEXS || sub_index >= OD_MAX_SUBS)
	{
		return NULL;
	}
	
	if(Object_Dictionary[index][sub_index] == NULL)
	{
		return NULL;
	}
	
	return Object_Dictionary[index][sub_index];
}

uint16_t Addr_Transfer(uint16_t addr)
{
	/* 实验室指定参数 */
	if(addr >= 0x2000)
	{
		addr -= 0x2000;
	}
	
	/* 通信参数 */
	switch (addr & 0xff00)
	{
		case 0x1400:
			addr = addr - 0x1400 + 20;
			break;
		case 0x1800:
			addr = addr - 0x1800 + 20 + 4;
			break;
		case 0x1600:
			addr = addr - 0x1600 + 20 + 8;
			break;
		case 0x1A00:
			addr = addr - 0x1A00 + 20 + 12;
			break;
	}
	
	return addr;
}
