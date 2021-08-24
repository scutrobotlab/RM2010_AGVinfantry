/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file   : referee.h
  * @author : Lingzi_Xie 1357657340@qq.com
  * @brief  : Code for communicating with Referee system of Robomaster 2021.
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

#define DRAWING_PACK	15
#define	ROBOT_COM_PACK	8
/* Private variables ---------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
/* 裁判系统状态；0：掉线，1：在线 */
enum RF_status_e
{
	RF_OFFLINE = 0U,
	RF_ONLINE
};

/* 枚举变量，用于车间通信数组访问 */
enum{
	HERO = 0U,
	ENGINEER,
	INFANTRY_3,
	INFANTRY_4,
	INFANTRY_5,
	AERIAL,
	SENTRY,
	RADAR = 8U
};

enum{
	Robot_Red = 0U,
	Robot_Blue
};

/* 己方机器人ID及当前机器人客户端ID */
typedef	struct { 
	uint8_t hero;
	uint8_t engineer;
	uint8_t infantry_3;
	uint8_t infantry_4;
	uint8_t infantry_5;
	uint8_t aerial;
	uint8_t sentry;
	uint8_t drat;														//飞镖发射架
	uint8_t radar;														//雷达站
	uint8_t local;														//本机器人ID

	uint8_t robot_where;												//蓝方 or 红方机器人
	uint16_t client;													//对应的客户端ID
} __attribute__((packed))RC_ID;

/* 数据帧帧头结构体 */
typedef struct { 
	uint8_t SOF;
	uint16_t DataLength;
	uint8_t Seq;
	uint8_t CRC8;
	uint16_t CmdID;
} __attribute__((packed))FrameHeader;

/* 交互数据帧的统一数据段段头 */
typedef __packed struct { 					
	uint16_t data_cmd_id;
	uint16_t send_ID;
	uint16_t receiver_ID;
} DataHeader;

/* ----------------------------------各种数据类型对应的ID号--------------------------------------- */
typedef enum {
	/* 裁判系统发过来数据帧ID类型，注意这里是CMD_ID */
	GameState_ID                    = 0x0001,		//比赛状态数据 1Hz
	GameResult_ID                   = 0x0002,		//比赛结果数据 结束后发送
	GameRobotHP_ID                  = 0x0003,		//机器人血量数据 1Hz
	DartStatus_ID 					= 0x0004,		//【05.06的协议中移除了？！】飞镖发射状态数据
	ICRA_DebuffStatus_ID 			= 0x0005,		//人工智能挑战赛加成与惩罚状态 1Hz
	EventData_ID                    = 0x0101,		//场地事件数据 1Hz
	SupplyProjectileAction_ID       = 0x0102,		//补给站动作标识 动作后发送

	RefereeWarning_ID               = 0x0104,		//裁判警告数据 警告后发送
	DartRemainingTime_ID            = 0x0105,		//飞镖发射口倒计时 1Hz
	GameRobotState_ID               = 0x0201,		//机器人状态 10Hz
	PowerHeatData_ID                = 0x0202,		//功率热量数据 50Hz
	GameRobotPos_ID                 = 0x0203,		//机器人位置数据 10Hz
	BuffMusk_ID                     = 0x0204,		//机器人增益 状态改变后发送
	AerialRobotEnergy_ID            = 0x0205,		//空中机器人能量 10Hz
	RobotHurt_ID                    = 0x0206,		//伤害数据 伤害发生后发送
	ShootData_ID                    = 0x0207,   	//实时射击数据 发射后发送
	BulletRemaining_ID              = 0x0208,		//子弹剩余发送数 哨兵及空中 1Hz
	RFID_Status_ID           		= 0x0209,		//RFID状态 1Hz
	ExtDartClientCmd_ID             = 0x020A,		//飞镖机器人客户端发送指令 10Hz
	StudentInteractiveHeaderData_ID	= 0x0301, 		//机器人交互数据 30Hz上限
	CustomControllerData_ID			= 0x0302,	 	//自定义控制器交互数据接口 30Hz
	MiniMapInteractiveData_ID		= 0x0303,		//小地图交互数据 交互发送
	VT_Data_ID						= 0x0304,		//键盘、鼠标数据，通过图传发送
	ClientMapCommand_ID				= 0x0305,		//小地图接收信息

	/* 机器人交互数据段的ID类型，注意这里是数据段内容ID！ */	
	RobotComData_ID					= 0x0233,		//车间交互，该ID由各个队伍自定义
	CustomData_ID					= 0xD180,		//自定义数据ID
	Drawing_Clean_ID				= 0x0100,	
	Drawing_1_ID					= 0x0101,
	Drawing_2_ID					= 0x0102,
	Drawing_5_ID					= 0x0103,
	Drawing_7_ID					= 0x0104,
	Drawing_Char_ID					= 0x0110,

} RefereeSystemID_t;

/* ----------------------------------比赛数据帧的数据段框架--------------------------------------- */
/**
  @brief  比赛状态数据：0x0001，1Hz
*/
typedef struct {
	uint8_t game_type : 4;
	uint8_t game_progress : 4;
	uint16_t stage_remain_time;
	
	uint64_t SyncTimeStamp;
} __attribute__((packed))ext_game_status_t;

/**
   @brief 比赛结果数据：0x0002，在比赛结束后发送
*/
typedef struct {
  uint8_t winner;
} ext_game_result_t;

/**
   @brief 机器人血量数据：0x0003，1Hz
*/
typedef struct {
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
   @brief 飞镖发射状态：0x0004，飞镖发射后发送（05.06删去）
*/
typedef struct
{
 uint8_t dart_belong; 													//发射方
 uint16_t stage_remaining_time; 										//发射时比赛剩余时间 
} __attribute__((packed))ext_dart_status_t;

/**
   @brief 人工智能挑战赛加成与惩罚区状态：0x0005，1Hz
*/
typedef struct
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
	
	uint16_t red1_bullet_felt;
	uint16_t red2_bullet_felt;
	uint16_t blue1_bullet_felt;
	uint16_t blue2_bullet_felt;
	
} __attribute__((packed))ext_ICRA_buff_debuff_zone_status_t;

/**
   @brief 场地事件数据：0x0101，事件改变发送
*/
typedef struct {
  uint32_t event_type;
} ext_event_data_t;

/**
   @brief 补给站动作标识：0x0102	动作改变后发送
*/
typedef struct {
  uint8_t supply_projectile_id;
  uint8_t supply_robot_id;
  uint8_t supply_projectile_step;
  uint8_t supply_projectile_num;  
} ext_supply_projectile_action_t;

/**
   @brief 裁判警告信息：0x0104	警告发生后发送
*/
typedef struct {
	uint8_t level;
	uint8_t foul_robot_id;
} ext_referee_warning_t;

/**
   @brief 飞镖发射口倒计时：0x0105，1Hz
*/
typedef struct
{
 uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;

/**
   @brief 当前比赛机器人状态：0x0201，10Hz
*/
typedef struct { 
	uint8_t robot_id;
	uint8_t robot_level;
	uint16_t remain_HP;
	uint16_t max_HP;
	uint16_t shooter_id1_17mm_cooling_rate;/* 17mm */
	uint16_t shooter_id1_17mm_cooling_limit;
	uint16_t shooter_id1_17mm_speed_limit;
	
	uint16_t shooter_id2_17mm_cooling_rate;
	uint16_t shooter_id2_17mm_cooling_limit;
	uint16_t shooter_id2_17mm_speed_limit;
	
	uint16_t shooter_id1_42mm_cooling_rate;
	uint16_t shooter_id1_42mm_cooling_limit;
	uint16_t shooter_id1_42mm_speed_limit;
	
	uint16_t classis_power_limit;
	uint8_t mains_power_gimbal_output : 1;
	uint8_t mains_power_chassis_output : 1;
	uint8_t mains_power_shooter_output : 1;
} __attribute__((packed))ext_game_robot_status_t;   

/**
   @brief 实时功率热量数据：0x0202，50Hz
*/
typedef struct {
	uint16_t chassis_volt; 
	uint16_t chassis_current; 
	float chassis_power; 
	uint16_t chassis_power_buffer; 
	uint16_t shooter_id1_17mm_cooling_heat; 
	uint16_t shooter_id2_17mm_cooling_heat; 
	uint16_t shooter_id1_42mm_cooling_heat;
} __attribute__((packed))ext_power_heat_data_t;

/**
   @brief 机器人位置：0x0203,10Hz
*/
typedef __packed struct { 
  float x;
  float y;
  float z;
  float yaw;
} ext_game_robot_pos_t;

/**
   @brief 机器人增益：0x0204，状态改变后发送
*/
typedef struct {
  uint8_t power_rune_buff;
} ext_buff_t;

/**
   @brief 空中机器人能量状态：0x0205，10Hz
*/
typedef struct {
  uint8_t attack_time;
} aerial_robot_energy_t;

/**
   @brief 伤害状态：0x0206，受到伤害后发送
*/
typedef struct {
  uint8_t armor_id : 4;
  uint8_t hurt_type : 4;
} ext_robot_hurt_t;

/**
   @brief 实时射击信息：0x0207，射击后发送
*/
typedef struct {
	uint8_t bullet_type;
	uint8_t shooter_id;
	uint8_t bullet_freq;
	float bullet_speed;
} __attribute__((packed))ext_shoot_data_t;

/**
   @brief 子弹剩余发射数：0x0208，10Hz
*/
typedef struct {
  	uint16_t bullet_remaining_num_17mm;
	uint16_t bullet_remaining_num_42mm;
	uint16_t coin_remaining_num;
} ext_bullet_remaining_t;

/**
   @brief 机器人RFID状态：0x0209，1Hz
*/
typedef struct
{
  uint32_t rfid_status;  /* 每个位代表不同地点的RFID状态 */
} ext_rfid_status_t;	/*RFID状态不完全代表对应的增益或处罚状态，例如敌方已占领的高低增益点，不能获取对应的增益效果*/

/**
	@brief 飞镖机器人客户端指令数据：0x020A，10Hz 只对飞镖发送
*/
typedef struct {
  	uint8_t dart_launch_opening_status;
	uint8_t dart_attack_target;
	uint16_t target_change_time;
	uint16_t operate_launch_cmd_time;
} __attribute__((packed))ext_dart_client_cmd_t;


/* ----------------------------------裁判系统客户端交互部分--------------------------------------- */
/**
   @brief 交互数据段通用段头结构体定义：自定义数据的命令ID为：0x0301，10Hz
*/
typedef struct {
  uint16_t data_cmd_id;
  uint16_t sender_ID;     
  uint16_t receiver_ID;
} ext_student_interactive_header_data_t;

/**
   @brief 学生机器人间通信：命令ID为0x0301，内容ID：在0x0200~0x02FF内自由选择，10Hz
   @brief 这里定义了完整的数据段格式，虽然看起来有些冗余，但方便处理其他机器人的发过来的数据报
*/
typedef struct {
	uint16_t data_cmd_id;												//数据段段首
	uint16_t sender_ID;     
	uint16_t receiver_ID;	
	uint8_t data[112];           										//!<长度需要小于113个字节（官方手册约定）
} __attribute__((packed))robot_interactive_data_t;

typedef struct {
	uint8_t data[ROBOT_COM_PACK];           							//接收到的数据
} my_interactive_data_t;

/**
   @brief 自定义控制器交互数据：0x0302，30Hz；
   @brief 注意！该交互数据包数据段没有段首，直接就是数据部分
*/
typedef struct {
	uint8_t data[30];           										//!<长度需要小于30个字节（官方手册约定）
} custom_controller_interactive_data_t;

/**
   @brief 小地图下发数据：0x0303 触发时发送；
*/
typedef struct
{
	float target_position_x;
	float target_position_y;
	float target_position_z;
	uint8_t commd_keyboard;
	uint16_t target_robot_ID;
} __attribute__((packed))ext_mini_map_command_t;

/**
   @brief 键鼠信息，通过图传发送到机器人（图传发送端的3pin接口）：0x0304；
*/
typedef struct
{
	int16_t mouse_x;
	int16_t mouse_y;
	int16_t mouse_z;
	int8_t left_button_down;
	int8_t right_button_down;
	uint16_t keyboard_value;
	uint16_t reserved;
} __attribute__((packed))ext_VT_command_t;

/**
   @brief 雷达站下发数据帧内容定义，将被己方操作手以第一人称看到，0x305
*/
typedef struct
{
	uint16_t target_robot_ID;											//敌方机器人的坐标
	float target_position_x;
	float target_position_y;
	float toward_angle;
} __attribute__((packed))ext_client_map_command_t;

/* ----------------------------------解包用到的宏定义、枚举量--------------------------------------- */
#define START_ID	0XA5												//数据帧的起始ID，官方约定
#define PROTOCAL_FRAME_MAX_SIZE	60 										//这个参数在解包时只作为大致的数据包长度判断，可以设置为 >= 最大长度
#define HEADER_LEN 	4													//数据帧帧头长度（为5），4是为了方便直接用数组访问
#define CRC_ALL_LEN	3													//CRC校验码长度
#define CRC_8_LEN	1									
#define CMD_LEN	2														//数据帧指令ID

/* 数据帧解包状态 */
typedef enum {
    STEP_HEADER_SOF=0,													//帧头SOF，应为0xA5
    STEP_LENGTH_LOW,													//数据段长度低8位
    STEP_LENGTH_HIGH,													//数据段长度高8位
    STEP_FRAME_SEQ,														//包序号
    STEP_HEADER_CRC8,													//帧头CRC8校验码
    STEP_DATA_CRC16														//帧末CRC16校验码
} unPackStep_e;

/* Exported ------------------------------------------------------------------*/

/* ----------------------------------客户端绘图相关--------------------------------------- */
/* 绘制图形操作，数据段 */
typedef struct
{
	uint8_t graphic_name[3]; 											//图形名称，作为客户端中的索引
	uint32_t operate_tpye:3; 											//图形操作（空、增加、修改、删除）
	uint32_t graphic_tpye:3; 											//图形类型（何种图案或数字）
	uint32_t layer:4; 													//图层数
	uint32_t color:4; 													//颜色
	uint32_t start_angle:9;												//起始角度
	uint32_t end_angle:9;												//终止角度
	uint32_t width:10; 													//线宽
	uint32_t start_x:11; 												//起点坐标
	uint32_t start_y:11; 
	uint32_t radius:10; 												//字体大小或半径
	uint32_t end_x:11; 													//终点坐标
	uint32_t end_y:11;
} __attribute__((packed))graphic_data_struct_t;


/* 删除指定图层操作，数据段 */
typedef __packed struct
{
	uint8_t operate_tpye; 
	uint8_t layer; 
} ext_client_custom_graphic_delete_t;

/* 图形配置操作指令，参照官方手册P23 */
typedef enum {
	NULL_OPERATION = 0U,
    ADD_PICTURE = 1U,
    MODIFY_PICTURE = 2U,
    CLEAR_ONE_PICTURE = 3U,
} drawOperate_e;

/* 图形清除操作指令 */
typedef enum {
    CLEAR_ONE_LAYER = 1U,
    CLEAR_ALL = 2U,
} clearOperate_e;

/* 图形绘制类型 */
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

/* 图形色彩配置 */
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

/* 交互数据帧的通信类型，机器人间通信 or 自定义UI or 小地图通信 */
typedef enum
{
	CV_OtherRobot = 0U,
	UI_Client,
	MiniMap_Client
}receive_Type_e;

typedef uint32_t (*SystemTick_Fun)(void);

/* ----------------------------------裁判系统串口通信类--------------------------------------- */
class referee_Classdef
{
	public:
		/* 机器人ID */
		RC_ID robot_client_ID;	
		
		/* 裁判系统连接状态，Todo */
		RF_status_e status;

		/* 比赛接收数据 */
		ext_game_status_t GameState;													//比赛状态数据
		ext_game_result_t GameResult;													//比赛结果数据
		ext_game_robot_HP_t GameRobotHP;												//机器人血量数据
		ext_dart_status_t DartStatus;													//飞镖状态数据
		ext_event_data_t EventData;														//场地事件数据
		ext_supply_projectile_action_t SupplyAction;									//补给站动作数据
		ext_referee_warning_t RefereeWarning;											//裁判警告信息数据
		ext_dart_remaining_time_t DartRemainTime;										//飞镖发射倒计时数据
		ext_game_robot_status_t GameRobotState;											//机器人状态：当前血量、射速、底盘功率等
		ext_power_heat_data_t PowerHeatData;											//机器人功率、热量数据
		ext_game_robot_pos_t RobotPos;													//机器人位置数据
		ext_buff_t RobotBuff;															//机器人增益数据
		aerial_robot_energy_t AerialEnergy;												//空中机器人能量状态数据
		ext_robot_hurt_t RobotHurt;														//机器人伤害状态数据
		ext_shoot_data_t ShootData;														//实时射击信息数据
		ext_bullet_remaining_t BulletRemaining;											//剩余弹丸量数据
		ext_rfid_status_t RFID_Status;													//RFID状态数据
		ext_dart_client_cmd_t DartClientCmd;											//飞镖机器人客户端指令数据
	
		/* 交互数据 */
		my_interactive_data_t robot_rec_data[9];										//存储本方其他机器人发送的信息，添加了雷达站											
		custom_controller_interactive_data_t custom_control_data;						//自定义控制器数据段部分
		ext_mini_map_command_t mini_map_data;											//小地图下发信息
		ext_VT_command_t VT_command_data;												//键鼠信息，通过图传发送端接收
		ext_client_map_command_t radar_map_data;										//雷达站信息

		referee_Classdef(){}															//构造函数定义

		
		void Init(UART_HandleTypeDef *_huart, uint32_t (*getTick_fun)(void));			//初始化学生串口		
		void unPackDataFromRF(uint8_t *data_buf, uint32_t length);						//数据帧整帧解包
		void CV_ToOtherRobot(uint8_t target_id, uint8_t* data1, uint8_t length);		//机器人之间车间通信
		void Radar_dataTransmit(uint8_t target_id, float position_x, float position_y, float toward_angle);	//雷达站信息发送
		
		/* 用于清除图像 */
		void clean_one_picture(uint8_t vlayer,uint8_t name[]);							//清除单个图形
		void clean_two_picture(uint8_t vlayer,uint8_t name1[], uint8_t name2[]);		//清除两张图形
		void clean_layer(uint8_t _layer);												//清除单个图层
		void clean_all();																//清除所有
			
		/* 组合图形用户接口实现【已测试】 */
		/* 字符串绘制 */
		void Draw_Char(uint8_t _layer, uint16_t start_x, uint16_t start_y, uint8_t* char_name, uint8_t* data, uint16_t str_len, uint16_t str_size, colorType_e _color, drawOperate_e _operate_type);
		/* 通用UI标尺 */
		uint8_t UI_ruler(uint8_t _layer, uint16_t start_x, uint16_t start_y, uint16_t scale_step, uint16_t scale_long, uint16_t scale_short, colorType_e _color, drawOperate_e _operate_type);
		/* 通用UI准星 */
		void UI_Collimator(uint8_t _layer, uint16_t start_x, uint16_t start_y, uint16_t line_length, colorType_e _color, drawOperate_e _operate_type);
		
		/* 【图层0】视觉自瞄开启标志 */
		void Draw_Auto_Lock(uint8_t auto_flag, uint16_t center_x, uint16_t center_y, uint16_t line_width, colorType_e _color);
		/* 【图层0】车界线绘制 */
		void Draw_Robot_Limit(uint16_t height, uint16_t distance, uint16_t center_x, uint16_t line_width, colorType_e _color, drawOperate_e _operate_type);
		/* 【图层3】静态弹仓标志，不允许添加其他图形在该图层 */
		void Draw_Bullet(uint8_t bullet_flag, uint16_t center_x, uint16_t center_y, uint16_t line_width, colorType_e _color);
		/* 【图层2】静态小陀螺标志 */
		void Draw_Spin(uint8_t spin_flag, uint16_t center_x, uint16_t center_y, uint16_t line_width, colorType_e _color);
		/* 【图层1】超级电容开启绘制 */		
		void Draw_Boost(uint8_t boost_flag,  uint16_t center_x, uint16_t center_y, uint16_t line_width, colorType_e _color);
		/* 【图层1】电容电压百分比绘制，并配合超级电容开启打印提示字符串 */
		void Draw_Cap_Energy(float current_volt, float max_volt, float min_volt, uint8_t enable_cnt, uint16_t center_x, uint16_t center_y);
		/* 【图层4】无弹丸时提示补弹 */
		void Draw_No_Bullet(uint8_t no_bullet_flag, uint16_t center_x, uint16_t center_y, colorType_e _color);
		
		/* ---------------------英雄机器人榴弹准星，自定义图层----------------------- */
		void Hero_UI_ruler(uint8_t _layer, uint16_t start_x, uint16_t start_y, uint16_t* line_distance, uint16_t* line_length, colorType_e* _color, drawOperate_e _operate_type);
		
		/* ---------------------空中机器人UI绘制，自定义图层----------------------- */
		//空中机器人pitch轴标尺
		void Aerial_PitchRuler_Frame(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t total_length, uint16_t long_scale_length, uint16_t short_scale_length, colorType_e ruler_color, colorType_e current_color, colorType_e target_color);
		void Aerial_Pitch_Update(float pitch_angle, uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t total_length, colorType_e tag_color);			//空中机器人pitch轴浮标
		void Aerial_Pitch_Target(float pitch_target, uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t total_length, colorType_e target_color);		//空中机器人pitch轴目标浮标
		//云台手剩余全局剩余弹量显示
		void Aerial_BulletRemain_Frame(uint8_t layer, uint16_t start_x, uint16_t start_y, uint16_t size);
		void Aerial_BulletRemain_Update(uint16_t _17mm, uint16_t _42mm, uint8_t layer, uint16_t start_x, uint16_t start_y, uint16_t size);
		
		/* ---------------------工程机器人UI绘制，自定义图层----------------------- */
		//工程机器人抬升高度标尺
		void Engineer_HighthRuler_Frame(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t total_length, uint16_t long_scale_length, uint16_t short_scale_length, uint8_t ruler_tag, colorType_e ruler_color, colorType_e current_color);
		//工程机器人当前抬升高度浮标
		void Engineer_Height_Update(float height, uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t total_length, uint8_t ruler_tag, colorType_e tag_color);
		//工程机器人当前抬升高度浮标（两个标尺的浮标合并为一个数据包发送，减少数据包传输延迟带来的动态效果不同步）
		void Engineer_HeightMul_Update(float *height, uint8_t _layer, uint16_t *center_x, uint16_t *center_y, uint16_t *total_length, colorType_e tag_color);
		//工程机器人目标抬升高度浮标
		void Engineer_Target_Height(float target_height, uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t total_length, uint8_t ruler_tag, colorType_e tag_color);
		
		void Mine_Icon(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint8_t size, uint8_t mine_tag, colorType_e _color, drawOperate_e _operate_type);		//矿石图标
		void Engineer_MineRe_Frame(uint8_t _layer, uint16_t start_x, uint16_t start_y, uint8_t size);																//工程机器人当前存储矿石数UI框架
		void Engineer_MineRe_Update(uint8_t mine_num, uint8_t _layer, uint16_t start_x, uint16_t start_y, uint8_t size);											//更新工程机器人存储矿石数
		void Engineer_MineMode_Update(uint8_t mode, uint8_t _layer, uint16_t start_x, uint16_t start_y, uint8_t size, colorType_e _color);							//工程机器人自动/手动组状态
		void Engineer_RescueMode_Update(uint8_t mode, uint8_t _layer, uint16_t start_x, uint16_t start_y, uint8_t size, colorType_e _color);						//工程机器人救援/刷卡状态
		void Engineer_AutoMode_Update(uint8_t status, uint8_t _layer, uint16_t start_x, uint16_t start_y, uint8_t size);											//工程机器人自动组状态
			
		/* ----------------------哨兵机器人UI绘制，自定义图层----------------------- */
		void Sentry_PosRuler_Frame(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t total_length, colorType_e ruler_color, colorType_e current_color);//哨兵机器人移动轨道绘制
		void Sentry_Pos_Update(float pos_percent, uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t total_length, colorType_e tag_color);				//哨兵机器人当前位置浮标
		void Sentry_Patrol_Frame(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t size, uint8_t *keys, colorType_e patrol_color);						//哨兵机器人巡逻区域绘制
		void Sentry_Patrol_Update(uint8_t tag, uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t size, colorType_e _color);							//哨兵机器人索敌状态指示
		void Sentry_Bullet_Frame(uint8_t _layer, uint16_t start_x, uint16_t start_y, uint16_t size, colorType_e _color);											//哨兵机器人发弹状态框架
		void Sentry_Bullet_Update(uint8_t tag, uint8_t _layer, uint16_t start_x, uint16_t start_y, uint16_t size, colorType_e _color);								//哨兵机器人发弹状态
		
		/* ----------------------雷达站策略集显示，占用图层9------------------------
		策略集发生变化时，会占用带宽以绘制对应UI*/
		void Armor(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint8_t size, uint8_t armor_tag, colorType_e _color, drawOperate_e _operate_type);			//盾牌图标
		void Sword(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint8_t size, uint8_t sword_tag, colorType_e _color, drawOperate_e _operate_type);			//剑图标
		void Outpost_Icon(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint8_t size, uint8_t tag, colorType_e _color, drawOperate_e _operate_type);					//前哨站图标
		void Sentry_Icon(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint8_t size, uint8_t tag, colorType_e _color, drawOperate_e _operate_type);						//哨兵图标
		void Missle_Icon(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint8_t size, uint8_t tag, colorType_e _color, drawOperate_e _operate_type);						//飞镖图标
		void High_Land(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint8_t size, uint8_t tag, colorType_e _color, drawOperate_e _operate_type);						//高地图标
		void Windmill_Icon(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint8_t size, uint8_t tag, colorType_e _color, drawOperate_e _operate_type);					//神符图标
		void FlyingSlope_Icon(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint8_t size, uint8_t tag, colorType_e _color, drawOperate_e _operate_type);				//飞坡图标
		void Radar_Strategy_Frame(uint16_t *frame_pos_x, uint16_t *frame_pos_y);																					//雷达站策略UI框架
		void Radar_CStrategy_Update(uint8_t protect, uint8_t attack, uint8_t comment_startegy, uint16_t *pos_x, uint16_t *pos_y);									//雷达站全局策略UI绘制
		void Radar_SStrategy_Update(uint16_t special_startegy, uint16_t *pos_x, uint16_t *pos_y);																	//雷达站专用策略UI绘制

		private:
		/* 外部句柄 */
		UART_HandleTypeDef *refereeUart;
		SystemTick_Fun Get_SystemTick;   												/*<! Pointer of function to get system tick */
		
		uint8_t repeat_cnt = 0;															//UI绘制重复发包
		
		/* 发送数据包缓存区，最大128字节 */
		uint8_t transmit_pack[128];
		/* UI绘制数据包 */
		uint8_t data_pack[DRAWING_PACK*7] = {0};

		/* 客户端删除图形数据包 */
		ext_client_custom_graphic_delete_t cleaning;
										
		/* 计算我方机器人ID */
		void Calc_Robot_ID(uint8_t local_id);

		/* 解包过程用到的变量及函数 */
		uint8_t DataCheck(uint8_t **p);																				//绘画时用于拼接的空间
		unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);		//获取数据包包头的CRC8校验和
		uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);							//获取数据包整包的CRC16校验和
		void RefereeHandle(uint8_t *data_buf); 																		//更新对应ID的数据
		void RobotInteractiveHandle(robot_interactive_data_t* RobotInteractiveData_t);								//机器人间通信

		/* 发送时用到的 */
		void pack_send_robotData(uint16_t _data_cmd_id, uint16_t _receiver_ID, uint8_t* _data, uint16_t _data_len);	//0x301交互数据，包括UI绘制和车间通信	
		void send_toReferee(uint16_t _com_id, uint16_t length, receive_Type_e _receive_type);

		/** 
		具体图形的绘制：空操作、线、矩形、圆、椭圆、圆弧、浮点数、整数、字符；并指定绘制的图层
		*/
		graphic_data_struct_t* null_drawing(uint8_t _layer,uint8_t name[]);
		graphic_data_struct_t* line_drawing(uint8_t _layer,drawOperate_e _operate_type,uint16_t startx,uint16_t starty,uint16_t endx,uint16_t endy, uint16_t line_width, colorType_e vcolor,uint8_t name[]);
		graphic_data_struct_t* rectangle_drawing(uint8_t _layer,drawOperate_e _operate_type, uint16_t startx,uint16_t starty,uint16_t length,uint16_t width, uint16_t line_width, colorType_e vcolor, uint8_t name[]);
		graphic_data_struct_t* circle_drawing(uint8_t _layer,drawOperate_e _operate_type, uint16_t centrex,uint16_t centrey,uint16_t r, uint16_t line_width, colorType_e vcolor, uint8_t name[]);
		graphic_data_struct_t* oval_drawing(uint8_t _layer,drawOperate_e _operate_type, uint16_t centrex,uint16_t centrey,uint16_t minor_semi_axis,uint16_t major_semi_axis, uint16_t line_width, colorType_e vcolor, uint8_t name[]);
		graphic_data_struct_t* arc_drawing(uint8_t _layer,drawOperate_e _operate_type, uint16_t centrex,uint16_t centrey,uint16_t minor_semi_axis,uint16_t major_semi_axis,int16_t start_angle,int16_t end_angle,uint16_t line_width,colorType_e vcolor, uint8_t name[]);
		graphic_data_struct_t* float_drawing(uint8_t _layer,drawOperate_e _operate_type, uint16_t startx,uint16_t starty,uint16_t size, uint16_t width,colorType_e vcolor, float data, uint8_t name[]);
		graphic_data_struct_t* int_drawing(uint8_t _layer, drawOperate_e _operate_type, uint16_t startx,uint16_t starty,uint16_t size, uint16_t width, colorType_e vcolor, int32_t data,uint8_t name[]);
		graphic_data_struct_t* character_drawing(uint8_t _layer,drawOperate_e _operate_type,uint16_t startx,uint16_t starty,uint16_t size, uint8_t width,uint8_t* data, uint16_t str_len, colorType_e vcolor, uint8_t name[]);		
};

#endif

#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
