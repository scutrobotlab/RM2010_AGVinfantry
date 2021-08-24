/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file   : referee.cpp
  * @author : kainan 837736328@qq.com
  * @brief  : Code for communicating with Referee system of Robomaster 2020.
  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have many 
  * bugs, update the version NO., write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding 
  * through your new brief.
  ******************************************************************************
  */
#ifndef __REFEREE_H__
#define __REFEREE_H__

/* Includes ------------------------------------------------------------------*/
#include  <stdint.h>
#include  <string.h>
#include "../Components/drv_uart.h"
#ifdef __cplusplus
/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
enum RF_status_e
{
  RF_OFFLINE = 0U,
	RF_ONLINE
};

/**
	@brief robot and client ID
*/
typedef	struct { 
	uint8_t hero;
	uint8_t engineer;
	uint8_t infantry_3;
	uint8_t infantry_4;
	uint8_t infantry_5;
	uint8_t aerial;
	uint8_t sentry;
	uint8_t local;
	uint16_t client;
} RC_ID;

/*裁判系统数据结构*/
typedef __packed struct { 
  uint8_t SOF;
  uint16_t DataLength;
  uint8_t Seq;
  uint8_t CRC8;
  uint16_t CmdID;
} FrameHeader;

typedef __packed struct { 
	uint16_t data_cmd_id;
	uint16_t send_ID;
	uint16_t receiver_ID;
} DataHeader;

/**
   @brief 裁判系统各种ID 
*/
typedef enum {
	/* 裁判系统发过来的ID */
  GameState_ID                    = 0x0001,
  GameResult_ID                   = 0x0002,
  GameRobotHP_ID                  = 0x0003,
	DartStatus_ID 									= 0x0004,
	ICRA_DebuffStatus_ID 						= 0x0005,
  EventData_ID                    = 0x0101,
  SupplyProjectileAction_ID       = 0x0102,
  RefereeWarning_ID               = 0x0104,
  DartRemainingTime_ID            = 0x0105,
	GameRobotState_ID               = 0x0201,
  PowerHeatData_ID                = 0x0202,
  GameRobotPos_ID                 = 0x0203,
  BuffMusk_ID                     = 0x0204,
  AerialRobotEnergy_ID            = 0x0205,
  RobotHurt_ID                    = 0x0206,
  ShootData_ID                    = 0x0207,
  BulletRemaining_ID              = 0x0208,
	RFID_Status_ID           				= 0x0209,
  StudentInteractiveHeaderData_ID	= 0x0301, /* 车间通信和客户端都是0x0301 */	
	
	/* 机器人交互数据ID */	
	RobotComData_ID									= 0x0233,	/* 车间交互，队伍自定义 */
	Drawing_Clean_ID								= 0x0100,
	Drawing_1_ID										= 0x0101,
	Drawing_2_ID										= 0x0102,
	Drawing_5_ID										= 0x0103,
	Drawing_7_ID										= 0x0104,
	Drawing_Char_ID									= 0x0110,
	
} RefereeSystemID_t;

/**
  @brief  比赛状态数据：0x0001  1Hz
*/
typedef __packed struct {
  uint8_t game_type : 4;
  uint8_t game_progress : 4;
  uint16_t stage_remain_time;
} ext_game_status_t;

/**
   @brief 比赛结果数据：0x0002	比赛结束后发送
*/
typedef __packed struct {
  uint8_t winner;
} ext_game_result_t;

/**
   @brief 机器人血量数据：0x0003	1Hz
*/
typedef __packed struct {
	uint16_t red_1_robot_HP;
	uint16_t red_2_robot_HP; 
	uint16_t red_3_robot_HP; 
	uint16_t red_4_robot_HP; 
	uint16_t red_5_robot_HP; 
	uint16_t red_7_robot_HP; 
	uint16_t red_outpost_HP;
	uint16_t red_base_HP; 
	
	uint16_t blue_1_robot_HP; 
	uint16_t blue_2_robot_HP; 
	uint16_t blue_3_robot_HP; 
	uint16_t blue_4_robot_HP; 
	uint16_t blue_5_robot_HP; 
	uint16_t blue_7_robot_HP; 
	uint16_t blue_outpost_HP;
	uint16_t blue_base_HP;
} ext_game_robot_HP_t;

/**
   @brief 飞镖发射状态：0x0004  飞镖发射后发送
*/
typedef __packed struct
{
 uint8_t dart_belong; /* 发射方 */
 uint16_t stage_remaining_time; /* 发射时比赛剩余时间 */
} ext_dart_status_t;

/**
   @brief 人工智能挑战赛加成与惩罚区状态：0x0005 1Hz
*/
typedef __packed struct
{
 uint8_t F1_zone_status:1;
 uint8_t F1_zone_buff_debuff_status:3;
 uint8_t F2_zone_status:1;
 uint8_t F2_zone_buff_debuff_status:3; 
 uint8_t F3_zone_status:1;
 uint8_t F3_zone_buff_debuff_status:3; 
 uint8_t F4_zone_status:1;
 uint8_t F4_zone_buff_debuff_status:3; 
 uint8_t F5_zone_status:1;
 uint8_t F5_zone_buff_debuff_status:3; 
 uint8_t F6_zone_status:1;
 uint8_t F6_zone_buff_debuff_status:3;
} ext_ICRA_buff_debuff_zone_status_t;

/**
   @brief 场地事件数据：0x0101	事件改变发送
*/
typedef __packed struct {
  uint32_t event_type;
} ext_event_data_t;

/**
   @brief 补给站动作标识：0x0102	动作改变后发送
*/
typedef __packed struct {
  uint8_t supply_projectile_id;
  uint8_t supply_robot_id;
  uint8_t supply_projectile_step;
  uint8_t supply_projectile_num;  
} ext_supply_projectile_action_t;

/**
   @brief 裁判警告信息：0x0104	警告发生后发送
*/
typedef __packed struct {
	uint8_t level;
	uint8_t foul_robot_id;
} ext_referee_warning_t;

/**
   @brief 飞镖发射口倒计时：0x0105 1Hz
*/
typedef __packed struct
{
 uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;

/**
   @brief 比赛机器人状态：0x0201	10Hz
*/
typedef __packed struct { 
	uint8_t robot_id;
	uint8_t robot_level;
	uint16_t remain_HP;
	uint16_t max_HP;
	uint16_t shooter_heat0_cooling_rate;/* 17mm */
	uint16_t shooter_heat0_cooling_limit;
	uint16_t shooter_heat1_cooling_rate;/* 42mm */
	uint16_t shooter_heat1_cooling_limit;
	uint8_t shooter_heat0_speed_limit;
	uint8_t shooter_heat1_speed_limit;
	uint8_t max_chassis_power;
	uint8_t mains_power_gimbal_output : 1;
	uint8_t mains_power_chassis_output : 1;
	uint8_t mains_power_shooter_output : 1;
} ext_game_robot_status_t;   

/**
   @brief 实时功率热量数据：0x0202	50Hz
*/
typedef __packed struct {
	uint16_t chassis_volt; 
	uint16_t chassis_current; 
	float chassis_power; 
	uint16_t chassis_power_buffer; 
	uint16_t shooter_heat0; 
	uint16_t shooter_heat1; 
	uint16_t mobile_shooter_heat2;
} ext_power_heat_data_t;

/**
   @brief 机器人位置：0x0203	10Hz
*/
typedef __packed struct { 
  float x;
  float y;
  float z;
  float yaw;
} ext_game_robot_pos_t;

/**
   @brief 机器人增益：0x0204	状态改变后发送
*/
typedef __packed struct {
  uint8_t power_rune_buff;
} ext_buff_t;

/**
   @brief 空中机器人能量状态：0x0205	10Hz
*/
typedef __packed struct {
  uint8_t energy_point;
  uint8_t attack_time;
} aerial_robot_energy_t;

/**
   @brief 伤害状态：0x0206	收到伤害后发送
*/
typedef __packed struct {
  uint8_t armor_id : 4;
  uint8_t hurt_type : 4;
} ext_robot_hurt_t;

/**
   @brief 实时射击信息：0x0207	射击后发送
*/
typedef __packed struct {
  uint8_t bullet_type;
  uint8_t bullet_freq;
  float bullet_speed;
} ext_shoot_data_t;

/**
   @brief 子弹剩余发射数：0x0208	1Hz
*/
typedef __packed struct {
  uint16_t bullet_remaining_num;
} ext_bullet_remaining_t;

/**
   @brief 机器人RFID状态：0x0209 1Hz
*/
typedef __packed struct
{
	uint32_t rfid_status;  /* 每个位代表不同地点的RFID状态 */
} ext_rfid_status_t;

/* 裁判系统客户端交互部分 */
/**
   @brief 交互数据接收信息：0x0301	10Hz
*/
typedef __packed struct {
  uint16_t data_cmd_id;
  uint16_t sender_ID;     
  uint16_t receiver_ID;
} ext_student_interactive_header_data_t;

/**
   @brief 客户端自定义数据：0x0301	内容ID：0xD180 10Hz
*/
typedef __packed struct {
	uint16_t data_cmd_id;
	uint16_t sender_ID;
	uint16_t receiver_ID;
	float data1;
	float data2;
	float data3;
	uint8_t masks;
} client_custom_data_t;

/**
   @brief 学生机器人间通信：0x0301	内容ID：0x0201~0x02FF	10Hz
*/
typedef __packed struct {
	uint16_t data_cmd_id;
	uint16_t sender_ID;
	uint16_t receiver_ID;
	uint8_t data[112];           //!<长度需要小于113个字节
} robot_interactive_data_t;

/* 解包用的协议 */
#define START_ID	0XA5
#define PROTOCAL_FRAME_MAX_SIZE	60 /* 这个参数只作为大致的数据包长度判断，可以设置为 >= 最大长度 */
#define HEADER_LEN 	4
#define CRC_ALL_LEN	3
#define CRC_8_LEN	1
#define CMD_LEN	2

typedef enum {
    STEP_HEADER_SOF=0,
    STEP_LENGTH_LOW,
    STEP_LENGTH_HIGH,
    STEP_FRAME_SEQ,
    STEP_HEADER_CRC8,
    STEP_DATA_CRC16
} unPackStep_e;

/* Exported ------------------------------------------------------------------*/
typedef __packed struct 
{
	 uint8_t _SOF;
	 uint8_t seq; //包序列，用于图形命名
	 uint16_t start_x;
	 uint16_t start_y;
	 uint16_t end_x;
	 uint16_t end_y;
	 uint8_t _EOF;             
} uart_pack_s;

typedef struct
{
	uint8_t graphic_name[3]; 
	uint32_t operate_tpye:3; 
	uint32_t graphic_tpye:3; 
	uint32_t layer:4; 
	uint32_t color:4; 
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10; 
	uint32_t start_x:11; 
	uint32_t start_y:11; 
	uint32_t radius:10; 
	uint32_t end_x:11; 
	uint32_t end_y:11;
} __attribute__((packed))ext_client_graphic_draw_t;


/* 删除图层 */
typedef __packed struct
{
	uint8_t operate_tpye; 
	uint8_t layer; 
} ext_client_custom_graphic_delete_t;


typedef enum {
    ADD_PICTURE = 1U,
    MODIFY_PICTURE = 2U,
    CLEAR_ONE_PICTURE = 3U,
} drawOperate_e;

typedef enum {
    CLEAR_ONE_LAYER = 1U,
    CLEAR_ALL = 2U,
} clearOperate_e;

typedef enum {
	LINE = 0U,
	RECTANGLE = 1U,
	CIRCLE = 2U,
	OVAL = 3U,
	ARC = 4U,
	_FLOAT = 5U,
	_INT = 6U,
	_CHAR = 7U,
} graphic_tpye_e;

typedef enum
{
	RED = 0U,
	BLUE = 0U,
	YELLOW,
	GREEN,
	ORANGE,
	PURPLE,
	PINK,
	DARKGREEN,
	BLACK,
	WHITE
}colorType_e;

typedef enum
{
	OtherRobot = 0U,
	Client = 1U,
}receiverType_e;

typedef uint32_t (*SystemTick_Fun)(void);

class referee_Classdef
{
	public:
		/* 裁判系统数据 */
		ext_game_status_t GameState;
		ext_game_result_t GameResult;
		ext_game_robot_HP_t GameRobotHP;
		ext_dart_status_t DartStatus;
		ext_event_data_t EventData;
		ext_supply_projectile_action_t SupplyAction;
		ext_referee_warning_t RefereeWarning;
		ext_dart_remaining_time_t DartRemainTime;
		ext_game_robot_status_t GameRobotState;
		ext_power_heat_data_t PowerHeatData;
		ext_game_robot_pos_t RobotPos;
		ext_buff_t RobotBuff;
		aerial_robot_energy_t AerialEnergy;
		ext_robot_hurt_t RobotHurt;
		ext_shoot_data_t ShootData;
		ext_bullet_remaining_t BulletRemaining;
		ext_rfid_status_t RFID_Status;
		ext_student_interactive_header_data_t StudentInteractiveHeaderData;
		robot_interactive_data_t RobotInteractiveData;
		
		/* 机器人ID */
		RC_ID robot_client_ID;	
		
		/* 裁判系统连接状态,Todo */
		RF_status_e status;
		
		/* 机器人交互数据 */
		robot_interactive_data_t robot_com_data[7];
		
		referee_Classdef(){}
		
		void Init(UART_HandleTypeDef *_huart, uint32_t (*getTick_fun)(void));
				
		void unPackDataFromRF(uint8_t *data_buf, uint32_t length);
		void CV_ToOtherRobot(uint8_t target_id, uint8_t* data1, uint8_t length);
		
		void Set_DrawingLayer(uint8_t _layer);
		void line_drawing(drawOperate_e _operate_type,uint16_t startx,uint16_t starty,uint16_t endx,uint16_t endy,colorType_e vcolor,uint8_t name[]);
		void rectangle_drawing(drawOperate_e _operate_type, uint16_t startx,uint16_t starty,uint16_t length_,uint16_t width_,colorType_e vcolor, uint8_t name[]);
		void circle_drawing(drawOperate_e _operate_type, uint16_t centrex,uint16_t centrey,uint16_t r,colorType_e vcolor, uint8_t name[]);
		void oval_drawing(drawOperate_e _operate_type, uint16_t centrex,uint16_t centrey,uint16_t minor_semi_axis,uint16_t major_semi_axis,colorType_e vcolor, uint8_t name[]);
		void arc_drawing(drawOperate_e _operate_type, uint16_t centrex,uint16_t centrey,uint16_t endx,uint16_t endy,int16_t start_angle_,int16_t end_angle_,colorType_e vcolor, uint8_t name[]);
		void float_drawing(drawOperate_e _operate_type, uint16_t startx,uint16_t starty, colorType_e vcolor, float data, uint8_t name[]);
		void int_drawing(drawOperate_e _operate_type, uint16_t startx,uint16_t starty,uint16_t size, uint8_t length,uint8_t character[], colorType_e vcolor, int32_t data,uint8_t name[]);
		void character_drawing(drawOperate_e _operate_type,uint16_t startx,uint16_t starty,uint16_t size, uint8_t length,uint8_t character[], colorType_e vcolor, uint8_t name[]);		
		void clean_one_picture(uint8_t vlayer,uint8_t name[]);
		void clean_layer(uint8_t _layer);
		void clean_all();
		
		/* 组合图形接口 */
		uint8_t UI_ruler(uint32_t _sys_time, uint8_t ruler_tag, uint8_t sacle_num = 5, uint16_t start_x = 960, uint16_t start_y = 540 - 24, uint16_t step = 40, uint16_t scale_long = 60, uint16_t scale_short = 32, colorType_e _color = WHITE);
		void Draw_FitingGraph(uint8_t *data, uint16_t length,colorType_e _color);

	private:

		/* 外部句柄 */
		UART_HandleTypeDef *refereeUart;
		SystemTick_Fun Get_SystemTick;   /*<! Pointer of function to get system tick */
		
		ext_client_graphic_draw_t drawing;
		ext_client_custom_graphic_delete_t cleaning;
		uint32_t next_send_time;	/* 用于控制发送速率 */
		
		/* 计算我方机器人ID */
		void Calc_Robot_ID(uint8_t local_id);
	
		/* 解包过程用到的变量及函数 */
		uint8_t DataCheck(uint8_t **p);
		unsigned char com_temp[128];	//绘画时用于拼接的空间
		unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
		uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
		void RefereeHandle(uint8_t *data_buf); /* 更新对应ID的数据 */
		void RobotInteractiveHandle(robot_interactive_data_t* RobotInteractiveData_t);

		/* 发送时用到的 */
		void pack_send_robotData(uint16_t _data_cmd_id, uint16_t _receiver_ID, uint8_t* _data, uint16_t _data_len);
		void send_toReferee(uint16_t _com_id, uint8_t* _data, uint16_t length, receiverType_e receiver);

};

/* 这些是当时用上位机调试客户端绘画用的 */
extern uart_pack_s uart_pack;
uint32_t unpack_UI(uint8_t *data_buf,uint16_t length);

#endif
#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
