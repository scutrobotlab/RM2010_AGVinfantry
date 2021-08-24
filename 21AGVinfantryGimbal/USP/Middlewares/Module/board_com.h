/**
  ******************************************************************************
  * @file   ： Module_BoardCOM.h
  * @brief  ： 板间通信统一协议模块
  * @author ： 苏锌雨 13535064092
  * @date   ： July,2019
  ******************************************************************************
  ==============================================================================
                     ##### How to use this module #####
  ==============================================================================
  1)向工业上各种普遍采用CANopen协议的先进嵌入式设备兼容
	2)规定本实验室使用的各种板子的device_id(制造商id)均为0x300(发送)，0x400(接收)用
		户只需关注设备的节点id(node_id)，同一个CAN网络最多支持255个设备；
	3)需要配置本机节点id并储存在Flash芯片中；
	4)需要配置本机对象字典的地址段为：2000h-5FFFh；
	5)板间通信的基本步骤：
		- 将某数据视为对象，在对象字典中规定一段地址储存这个数据，配置R/W；
		- 若采用SDO协议，分为：
			· 读，发送方需要发送该数据的地址，接收方回传该地址映射的数据值；
			· 写，发送方发送该数据的地址和数值，接收方回传确认；
		- 若采用PDO协议，发送方需要先与接收方约定PDO映射的对象字典地址，此后传输的PDO
			帧数据段全为数据，无需传输数据地址，接收方无需回传；
	6)模块使用说明
		- 需要通信的数据，使用Object_Dict_Register函数注册该数据的地址到对象字典中
		
	7)对象字典目录
	  - 对象字典大小可以规定，防止内存浪费
		- 对象字典默认偏移量为0x2000
		----------------------------------------------------
		| 地址        | 变量入口 						| 类型   | R/W |
		----------------------------------------------------
		| 0x2000-0x00 | 陀螺仪AngX			| float  |  R  |
		| 0x2000-0x01 | 陀螺仪AngY			| float  |  R  |
		| 0x2000-0x02 | 陀螺仪AngZ			| float  |  R  |
		| 0x2001-0x00 | 陀螺仪GyroX			| float  |  R  |
		| 0x2001-0x01 | 陀螺仪GyroY			| float  |  R  |
		| 0x2001-0x02 | 陀螺仪GyroZ			| float  |  R  |
		| 0x2002-0x00 | 底盘电机[0]Target		|	int16	 | R/W |
		| 0x2002-0x01 | 底盘电机[1]Target		|	int16	 | R/W |
		| 0x2002-0x02 | 底盘电机[2]Target		|	int16	 | R/W |
		| 0x2002-0x03 | 底盘电机[3]Target		|	int16	 | R/W |
		| 0x2003-0x00 | 云台电机[0]Target		|	int16	 | R/W |
		| 0x2003-0x01 | 云台电机[1]Target		|	int16	 | R/W |
		| 0x2004-0x00 | 发射电机[0]Target		|	int16	 | R/W |
		| 0x2004-0x01 | 发射电机[1]Target		|	int16	 | R/W |
		| 0x2005-0x00 | 超级电容开关				|	uint8	 | R/W |
		| 0x2005-0x01 | 超级电容充电功率		|	uint8	 | R/W |
		| 0x2006-0x00 | 射速								|	float	 |  R  |
		| 0x2006-0x01 | 射频								|	uint8	 |  R  |
		| 0x2006-0x03 | 机器人等级					|	uint8	 |  R  |
		| 0x2006-0x04 | 剩余血量						|	uint16 |  R  |
		| 0x2007-0x00 | 枪口每秒冷却值			|	uint16 |  R  |
		| 0x2007-0x01 | 枪口热量上限				|	uint16 |  R  |
		----------------------------------------------------
		| 0x1016-0x00 | 消费者时间          | uint32 | R/W  |
		| 0x1016-0x01 | 消费者时间          | uint32 | R/W  |
		| 0x1016-0x02 | 消费者时间          | uint32 | R/W  |
		| 0x1016-0x03 | 消费者时间          | uint32 | R/W  |
		| 0x1016-0x04 | 消费者时间          | uint32 | R/W  |
		| 0x1017-0x00 | 生产者时间          | uint16 | R/W  |
		----------------------------------------------------
		| 0x1400-0x01 | RxPDO 1 COB-ID			| uint32 | R/W |
		| 0x1400-0x02 | RxPDO 1 同步/异步		| uint8	 | R/W |
		| 0x1800-0x01 | TxPDO 1 COB-ID			| uint32 | R/W |
		| 0x1800-0x02 | TxPDO 1 触发类型		| uint8	 | R/W |
		| 0x1800-0x03 | TxPDO 1 禁止时间		| uint16 | R/W |
		----------------------------------------------------
		| 0x1600-0x00 | RxPDO 1 Map对象数		| uint8	 | R/W |
		| 0x1600-0x0？ | RxPDO 1 第?个对象		| uint32 | R/W |
		| 0x1601-0x00 | RxPDO 2 Map对象数		| uint8	 | R/W |
		| 0x1601-0x0? | RxPDO 2 第?个对象		| uint32 | R/W |
		| 0x1A00-0x00 | TxPDO 1 Map对象数		| uint8	 | R/W |
		| 0x1A00-0x0? | TxPDO 1 第?个对象		| uint32 | R/W |
		| 0x1A01-0x00 | TxPDO 2 Map对象数		| uint8	 | R/W |
		| 0x1A01-0x0? | TxPDO 2 第?个对象		| uint32 | R/W |

		----------------------------------------------------

  ******************************************************************************
  */


#ifndef _BOARD_COM_H
#define _BOARD_COM_H

#include <string.h>
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "drv_can.h"

/* 预定义错误类型 */
#define COM_OK							0
#define ERROR_MAILBOX_FULL 	1
#define ERROR_INDEXERROR		2
#define ERROR_UNREGISTED		3
#define ERROR_UNWRITEABLE		4
#define ERROR_MAPERROR			5
#define ERROR_PDOMAPPED			6
#define ERROR_TIMEOUT				7

/* 预定义邮箱深度 */
#define MAX_MAILBOX_DEEPTH	10

/* 预定义数据请求寄存器深度 */
#define MAX_REQUIRE_LIST_DEEPTH		10

/* 预定义对象字典大小 */
#define OD_MAX_INDEXS 40
#define OD_MAX_SUBS		5

/* 预定义制造商ID */
#define DEVICE_ID_TX 0x300
#define DEVICE_ID_RX 0x400

/* 预定义PDO COB-ID */
#define TXPDO1_ID		0x180
#define TXPDO2_ID		0x280
#define TXPDO3_ID		0x380
#define TXPDO4_ID		0x480
#define RXPDO1_ID		0x200
#define RXPDO2_ID		0x300
#define RXPDO3_ID		0x400
#define RXPDO4_ID		0x500

/* 预定义Abort Code */
#define AC_NO_ERROR					0x00000000	//无错误
#define AC_OUT_OF_MEMERORY 	0x05040005 	//地址超过预定义的对象字典
#define AC_OBJECT_NOT_EXIT 	0x06040043	//地址定义的对象不存在

/* 宏函数 */
#define isSDO(x) ((x&0x3f0)==DEVICE_ID_TX||(x&0x4f0)==DEVICE_ID_RX)
#define isPDO(x) ((x&TXPDO1_ID)==TXPDO1_ID||(x&TXPDO2_ID)==TXPDO2_ID||(x&TXPDO3_ID)==TXPDO3_ID||(x&TXPDO4_ID)==TXPDO4_ID)
/* 本地节点ID，储存在Flash芯片中 
	 初始化时需要读取
 */
extern uint8_t Local_NodeID;
#define g_Local_NodeID Local_NodeID

extern QueueHandle_t SDO_PassBack_Queue;

/* 对象R/W枚举 */
enum Object_RW_Enum{
	ReadOnly = 0,
	ReadNWrite = 1,
};

/* 对象类型枚举 */
enum Object_Type_Enum{
	UINT8 = 0,
	INT8 = 1,
	UINT16 = 3,
	INT16 = 4,
	UINT32 = 9,
	INT32 = 10,
	FLOAT = 11,
};

/* 对象字典
	 对象字典的大小视数据数量决定
 */
extern void* Object_Dictionary[OD_MAX_INDEXS][OD_MAX_SUBS];

/* 储存对象类型和R/W 
	 高7位储存数据类型
	 最低位为R/W
 */
extern uint8_t Type_Memery[OD_MAX_INDEXS*OD_MAX_SUBS];

/* SDO报文数据结构 */
struct SDO_Frame_Typedef{
	uint16_t COB_ID;
	uint8_t Ctrl_Byte;	//Client 2 Server 或 Server 2 Client
	uint16_t Index;
	uint8_t Sub_Index;
	uint8_t Data_Length;
	uint8_t Data[8];
};

/* SDO报文工作模式枚举 */
enum SDO_WorkType_Enum{
	SDO_ReadSend = 0x40,		//读请求
	SDO_ReadRecv = 0x41,		//读回复
	SDO_WriteSend = 0x21,	//写请求
	SDO_WriteRecv = 0x60,	//写回复
};

/* SDO报文数据段长度枚举 */
enum SDO_DataLength_Enum{
	SDO_Data_0Byte = 0,
	SDO_Data_1Byte = 1,
	SDO_Data_2Byte = 2,
	SDO_Data_4Byte = 4,
};

/* SDO报文邮箱数据结构 */
class SDO_Mailbox_Classdef{
public:
	SDO_Mailbox_Classdef():sizes(0){}
	uint8_t size();
	void push_back(SDO_Frame_Typedef);
	void erase();
	SDO_Frame_Typedef operator [](uint8_t);
private:
	uint8_t sizes;
	SDO_Frame_Typedef SDO_Mailbox[MAX_MAILBOX_DEEPTH];
};

class SDO_RequireList_Typedef{
public:
	SDO_RequireList_Typedef():sizes(0){}
	void Register(void* ptr,uint16_t cob_id,uint16_t index,uint8_t sub_index,Object_Type_Enum type);
	uint8_t Get_Register_Index(uint16_t cob_id,uint16_t index,uint8_t sub_index);
	template <typename T> T at(uint16_t cob_id,uint16_t index,uint8_t sub_index);
	Object_Type_Enum Type(uint8_t register_index);
	void Update(SDO_Frame_Typedef frame,uint8_t register_index);
private:
	uint8_t sizes;
	/* 注册表 储存数据注册的地址 */
	void* register_list[MAX_REQUIRE_LIST_DEEPTH];
	/* 注册表 储存注册的数据来源节点ID和地址 */
	SDO_Frame_Typedef require_list[MAX_REQUIRE_LIST_DEEPTH];
	/* 注册表 储存注册的数据的类型 */
	Object_Type_Enum type_list[MAX_REQUIRE_LIST_DEEPTH];

};

/* 工作模式说明：
	 接收线程中调用接收回调函数SDO_Recv_Callback，将接收到的SDO保存
	 调用执行函数SDO_Exce_Once，处理发送和接收邮箱中的消息队列
	 需要保证以上两个线程实时性最高
   消息邮箱为队列结构，先进先出，长度动态变化
 */
class SDOManager_Classdef
{
public:
	SDOManager_Classdef(CAN_HandleTypeDef* hcanx);

	/* 队列初始化函数 */
	void Init();
	
	/* 在CAN接收任务中调用 */
	void SDO_Recv_Callback(CAN_RxMessage*);
		
	/* 在专门的任务中调用 */
	void SDO_Exce_Once();
		
	/* 简单使用的接口 */
	void Pull(void * ptr,uint8_t node_id,uint16_t index,uint8_t sub_index,Object_Type_Enum type);
	void Push(void * Ptr,uint8_t node_id,uint16_t index,uint8_t sub_index,Object_Type_Enum type);

	/* 带回传等待的发送 */
	uint8_t SDO_Transmit(uint16_t device_id, uint8_t ndoe_id, SDO_WorkType_Enum work_type, uint16_t index, uint8_t sub_index, uint8_t * data, SDO_DataLength_Enum data_length,uint8_t max_retran_times);
	 
private:
	uint8_t SDO_Add_Message_To_Mailbox(SDO_Frame_Typedef);
	uint8_t SDO_Add_Message_To_Mailbox(uint16_t device_id, uint8_t ndoe_id, SDO_WorkType_Enum work_type, uint16_t index, uint8_t sub_index, uint8_t * data, SDO_DataLength_Enum data_length);

	/* 更新接收到的数据 */
	void Update(SDO_Frame_Typedef);

	/* 发送和接收邮箱 */
	SDO_Mailbox_Classdef SDO_Tx_Mailbox;
	SDO_Mailbox_Classdef SDO_Rx_Mailbox;
	
	/* 请求数据注册表 */
	SDO_RequireList_Typedef SDO_RequireList;

	/* 邮箱最大深度 */
	uint8_t MAX_Deepth;
	/* SDO管理器使用的CAN */
	CAN_HandleTypeDef* hcan;
};

enum PDO_TriggerType_Enum
{
	SYNC = 0x01,//同步信号触发
	RTR = 0xfd,	//外部请求触发
	INT = 0xff, //数值改变触发
};

enum PDO_ObjectSize_Enum
{
	PDO_Object_1Byte = 1,
	PDO_Object_2Byte = 2,
	PDO_Object_4Byte = 4,
	PDO_Object_8Byte = 8,
};

struct PDO_Frame_Typedef
{
	uint16_t COB_ID;
	uint8_t Data[8];
};

struct PDO_Object_Typedef
{
	uint8_t Node_ID;
	uint16_t Index;
	uint8_t Sub_Index;
	PDO_ObjectSize_Enum Size;
};

class PDO_Typedef
{
public:
	PDO_Typedef():Mapped_Nums(0){}
	uint8_t Map(uint8_t n,uint32_t objects[]);
	uint8_t Get_Mapped_Nums();
	
	/* Map的对象个数 */
	uint8_t Mapped_Nums;
	/* Map的对象 */
	uint32_t Mapped_Objects[4];
	/* PDO的COB-ID */
	uint32_t COB_ID;
	/* PDO触发类型 */
	PDO_TriggerType_Enum PDO_Type;
protected:	
	
};

class TXPDO_Typedef : public PDO_Typedef
{
public:
	TXPDO_Typedef():Gape_ms(1),Last_Push_Time(0){}
	/* Map的对象数据值打包 */
	PDO_Frame_Typedef Pack();
	uint16_t Gape_ms;
	uint32_t Last_Push_Time;
private:
};

class RXPDO_Typedef : public PDO_Typedef
{
public:
	RXPDO_Typedef(){
		memset(Register_Buff,0,32);
	}
	/* 更新接收到的数据 */
	uint8_t Update(CAN_RxMessage* recv_msg);
	void Regster(uint8_t n,void* Ptr[]);
private:
	void * Register_Buff[4];
};

class PDO_WatchDog_Classdef
{
public:
	PDO_WatchDog_Classdef(uint16_t max = 100):Waitms(0),MaxWaitms(max),isStarted(0){};
	void Start();
	void Bark();
	void Feed();
	uint8_t Status();
private:
	uint16_t Waitms;
	uint16_t MaxWaitms;
	uint8_t isStarted;
};

class PDOManager_Classdef
{
public:
	PDOManager_Classdef(CAN_HandleTypeDef* hcanx);
	/* 在CAN接收任务中调用 */
	void PDO_Recv_Callback(CAN_RxMessage*);
		
	/* 在专门的任务中调用 */
	void PDO_Exce_Once();
		
	/* 读写方法与SDO的使用方法不同 */
	uint8_t Pull(uint8_t node_id,uint8_t TxPDO_Index,uint8_t n,uint32_t objects[],void* reg_buff[],PDO_TriggerType_Enum trigger_type);
	uint8_t Push(uint8_t node_id,uint8_t RxPDO_Index,uint8_t n,uint32_t objects[],PDO_TriggerType_Enum trigger_type,uint16_t gape_ms);
	/* 强制Map接收端 
	 * 防止发送端push时丢包 
	 * 从接收端强制建立稳定通信
	 */
	void Force_Map_Rx(uint8_t RxPDO_Index,uint8_t n,uint32_t objects[]);
	
	/* PDO看门狗计时 */
	void PDOWDog_Bark();
	/* PDO状态 */
	uint8_t PDO_Status(uint8_t PDO_Index = 4);
	
private:
	uint8_t Transmit_SDO(uint8_t ndoe_id,uint16_t index, uint8_t sub_index,uint8_t * data,SDO_DataLength_Enum data_length,uint8_t max_retrain_times);
	TXPDO_Typedef TxPDO[4];
	RXPDO_Typedef RxPDO[4];
	PDO_WatchDog_Classdef PDO_WatchDog[4];

	/* PDO管理器使用的CAN */
	CAN_HandleTypeDef* hcan;
};

class CANopen_ClassDef
{
public:
	CANopen_ClassDef();
	
private:
};

void Set_Abort_Code(uint32_t abort_code, uint8_t* data);

uint8_t Set_Ctrl_Byte(SDO_WorkType_Enum work_type,SDO_DataLength_Enum data_length);

/* 每个需要通信的数据都需要在对象字典中注册 */
void Object_Dict_Register(void* param_ptr, uint16_t index, uint8_t sub_index, bool RW, Object_Type_Enum type);

/* 查询对象类型和R/W */
bool Object_RW(uint16_t index, uint8_t sub_index);
Object_Type_Enum Object_Type(uint16_t index, uint8_t sub_index);

/* 写入对象 */
uint8_t Object_Write(uint16_t index, uint8_t sub_index, uint8_t * Data);

/* 读出对象 */
uint8_t Object_Read(uint16_t index, uint8_t sub_index, uint8_t * Data);

/* 读出对象地址 */
void* Addr_Read(uint16_t index, uint8_t sub_index);

/* 地址转换 */
uint16_t Addr_Transfer(uint16_t addr);


#endif
