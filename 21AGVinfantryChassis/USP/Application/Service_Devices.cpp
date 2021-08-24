/**
  ******************************************************************************
  * @file   Service_Devices.cpp
  * @brief  Devices service running file.
  ******************************************************************************
  * @note
  *  - Before running your devices, just do what you want ~ !
  *  - More devices or using other classification is decided by yourself ~ !
  ===============================================================================
                                    Task List
  ===============================================================================
  * <table>
  * <tr><th>Task Name     <th>Priority          <th>Frequency/Hz    <th>Stack/Byte
  * <tr><td>              <td>                  <td>                <td>    
  * </table>
  *
 */
/* Includes ------------------------------------------------------------------*/
#include "Service_Devices.h"
#include "Service_Communication.h"
#include "autoInfantryConfig.h"
/* Private define ------------------------------------------------------------*/
TaskHandle_t DeviceActuators_Handle;
TaskHandle_t DeviceDR16_Handle;
TaskHandle_t Device_Referee_UI_Handle;
TaskHandle_t DeviceSensors_Handle;
TaskHandle_t RecvReferee_Handle;
TaskHandle_t DeviceAGVcontrol_Handle;
extern referee_Classdef Referee;
extern C_SourceManage_Classdef SourceManage;
/* Private variables ---------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
void Device_Actuators(void *arg);
void Device_Sensors(void *arg);
void Device_Referee_UI(void *arg);
void Device_DR16(void *arg);
void Recv_Referee(void *arg);
void Device_AGVcontrol(void *arg);
/* Exported devices ----------------------------------------------------------*/
/* Motor & ESC & Other actuators*/
//Motor_AK80_9  Test_Motor(1, 0, 0);
/* Remote control */

/* IMU & NUC & Other sensors */

/* Other boards */

/* Function prototypes -------------------------------------------------------*/
/**
* @brief  Initialization of device management service
* @param  None.
* @return None.
*/
void Service_Devices_Init(void)
{
//  xTaskCreate(Device_Actuators, "Dev.Actuator" , Tiny_Stack_Size,    NULL, PrioritySuperHigh,   &DeviceActuators_Handle);
  xTaskCreate(Device_Referee_UI,      "Dev.Referee_UI"     , Normal_Stack_Size,    NULL, PriorityHigh,        &Device_Referee_UI_Handle);
  xTaskCreate(Device_Sensors,   "Dev.Sensors"  , Small_Stack_Size,    NULL, PriorityRealtime,        &DeviceSensors_Handle);
	xTaskCreate(Recv_Referee,    "Rx.Referee"   , Large_Stack_Size,    NULL, PriorityRealtime,  &RecvReferee_Handle);
	xTaskCreate(Device_AGVcontrol,   "Dev.AGVcontrol"  , Small_Stack_Size,    NULL, PriorityHigh,        &DeviceAGVcontrol_Handle);
}




/**
* @brief    裁判系统可视化UI绘制任务
* @note     绘制车界线、电容电压（百分比）以及三种指示标志：小陀螺、超级电容、弹仓
* @return   None.
*/
void Device_Referee_UI(void *arg)
{
    static TickType_t _xPreviousWakeTime;

    //起始绘制时的绘制次数。若服务器丢包率较高，该值可适当给大
    static uint8_t enable_cnt = 20;             
 
    //下坠UI标尺的水平刻度线长度、距离、颜色；垂直线总长度由为各水平刻度线距离之和
		uint16_t line_distance[6] = {10,30,30,35/*哨兵*/,30,50};
		uint16_t line_length[6] = {120,80,70,60,20,20};
		colorType_e ruler_color[7] = {WHITE, WHITE, WHITE, WHITE, YELLOW, YELLOW, WHITE};     //最后一个为垂直线颜色

		//雷达站策略集坐标
		uint16_t cpos_x[4] = {100,160,220,260};						                        //全局UI推荐坐标
		uint16_t cpos_y[4] = {730,730,730,730};
		uint16_t frame_x[2] = {100,100};							
		uint16_t frame_y[2] = {800,670};

		uint16_t spos_x[3] = {100,200,80};							                        //专用UI推荐坐标
		uint16_t spos_y[3] = {610,610,560};	

    //图传稳定需要一段时间的延时
    vTaskDelay(500);
    Referee.clean_all();

    vTaskDelay(2000);                        
 
    for(;;)
    { 
				if(auto_infantry.write_state2[4] == 1)
				{
					enable_cnt =20;
					auto_infantry.write_state2[4] = false;
				}
        //刚开始时多次绘制图形，确保能在动态UI刚开启时顺利绘制图形
        if(enable_cnt)
        {
            //车界线、下坠标尺绘制
            Referee.Draw_Robot_Limit(180, 80, 961, 3, YELLOW, ADD_PICTURE);
            Referee.Hero_UI_ruler(5, 961, 538, line_distance, line_length, ruler_color, ADD_PICTURE);	

            //绘制电容剩余能量
            Referee.Draw_Cap_Energy(SourceManage.capObj.Voltage, 24, 17, enable_cnt, 420,800);			

            //雷达站策略集部分
						Referee.Radar_Strategy_Frame(frame_x, frame_y);

            enable_cnt--;
        }
        else
        {
            Referee.Draw_Cap_Energy(SourceManage.capObj.Voltage, 24, 17, enable_cnt, 420,800);	//绘制电容剩余能量
						Referee.Draw_Boost(auto_infantry.write_state2[1], 1600, 740, 10, PINK);					//绘制超级电容状态
						Referee.Draw_Spin(auto_infantry.write_state2[2], 1400, 740, 10, BLUE);					//绘制小陀螺开启状态
						Referee.Draw_Bullet(auto_infantry.write_state2[3], 1800, 740, 8, GREEN);					//绘制弹仓开启状态
						Referee.Draw_Auto_Lock(1-auto_infantry.write_state2[5], 1400, 680, 8, WHITE);					//绘制自瞄开启状态
//						Referee.Draw_No_Bullet(Chassis.bulletNum, 861, 738, ORANGE);						//绘制空弹提示
			
						//Radar
						Referee.Radar_CStrategy_Update(0, 0, Referee.robot_rec_data[RADAR].data[0], cpos_x, cpos_y);		//雷达站通用策略集
						Referee.Radar_SStrategy_Update(Referee.robot_rec_data[RADAR].data[1], spos_x, spos_y);				//雷达站专用策略集
        }
    }
}






void Device_Sensors(void *arg)
{
  /* Cache for Task */
  
  /* Pre-Load for task */
	
  /* Infinite loop */
  TickType_t xLastWakeTime_t;
  xLastWakeTime_t = xTaskGetTickCount();
	
	
//	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADCReadBuff, 7);
//	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
//	SourceManage.SourceManage_Init();
	
  for(;;)
  {
		vTaskDelayUntil(&xLastWakeTime_t,10);
    SourceManage.Update(ADCReadBuff);		//更新电压、电流等数据
		SourceManage.Set_ChargePower(PowerCtrl.Get_capChargePower());		//更新电容的允许充电电流
		SourceManage.Manage();		//进行供电逻辑控制
		
		HAL_DAC_SetValue(&hdac, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, SourceManage.capObj.charge_DAC_Value);		//设置电容充电电流
    
  /* Pass control to the next task ------------------------------------------*/
    
  }
}





void Recv_Referee(void *arg)
{
  /* Pre-Load for task */
	static USART_COB* referee_pack;
  static TickType_t xLastWakeTime_t = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
	{
		vTaskDelayUntil(&xLastWakeTime_t,1);
//		Sent_Contorl(&huart1);
		if(xTaskNotifyWait(0x00000000, 0xFFFFFFFF, (uint32_t *) &referee_pack, 0) == pdTRUE)
		{
			Referee.unPackDataFromRF((uint8_t*)referee_pack->address, referee_pack->len);		//更新裁判系统数据
		}
		/* Pass control to the next task */
    
  }
}



void Device_AGVcontrol(void *arg)
{
  /* Cache for Task */

  /* Pre-Load for task */

  /* Infinite loop */
  TickType_t xLastWakeTime_t;
  xLastWakeTime_t = xTaskGetTickCount();
	vTaskDelay(2000);
  for(;;)
  {
		/* Pass control to the next task */
		vTaskDelayUntil(&xLastWakeTime_t,1);
		
    auto_infantry.AGVControl();		//底盘控制函数
    
    
  }
}
/* User Code End Here ---------------------------------*/

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
