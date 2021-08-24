/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    dr16.cpp
  * @author  Zelong.Xu (8762322@qq.com)
  * @brief   Code for DJI-DR16 driver in embedded software system.
  * @date    2019-11-08
  * @version 2.1
  * @par Change Log：
  * <table>
  * <tr><th>Date        <th>Version  <th>Author    		<th>Description
  * <tr><td>2019-04-00  <td> 2.0     <td>YuyongHu     <td>Creator
  * <tr><td>2019-11-08  <td> 2.1     <td>Zelong.Xu    <td>Re-format for API.
  * </table>
  *
  ==============================================================================
                            How to use this driver     
  ==============================================================================
    @note 
      -# 实例化DR16_Classdef类
      -# 调用Check_Link()进行DR16在线检查
      -# 通过串口接收DR16传来的数据，并传入DataCapture()处理
      -# 需要时调用各通道获取函数获取数据，或者获取接收机的连接状态

  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have any 
  * bugs, update the version Number, write dowm your name and the date. The most
  * important thing is make sure the users will have clear and definite under-
  * standing through your new brief.
  *
  * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/ 
#include "dr16.h"

/* Private function declarations ---------------------------------------------*/
float DeadZone_Process(float num,float DZ_min, float DZ_max, float DZ_num);
/* function prototypes -------------------------------------------------------*/
/**
    * @brief  Initialize DR16 Class
    * @param  None
    * @retval None
    */
DR16_Classdef::DR16_Classdef()
{
    MouseCoefficient = 128;      //鼠标系数初始化
    Status = Connection_Lost;    //状态初始化
    for(short i=0;i<16;i++)      //键值初始化
    {
        Key[i].isPressed=false;
        Key[i].isTriggered=false;
    }
    //摇杆值初始化
    DataPack.ch0 = 1024;
    DataPack.ch1 = 1024;
    DataPack.ch2 = 1024;
    DataPack.ch3 = 1024;
}

/**
 * @brief  获取数据包函数，将DR16接收器收到的数据包的值赋给DR16的成员变量DataPack并进行键盘按键消抖处理
 * @param  captureData:收到的数据包指针
 * @return None.
 */
void DR16_Classdef::DataCapture(DR16_DataPack_Typedef* captureData)
{
    DataPack = *captureData; 

    /*设置在线，开始再次检测连接*/
    last_check_time = 0;
    Status = Connection_Established;

    /*各杂七杂八通道值归一化处理*/
    RX_Norm = DeadZone_Process((float)(DataPack.ch0-1024)/660.0f,-Ignore_Limit,Ignore_Limit,0);
    RY_Norm = DeadZone_Process((float)(DataPack.ch1-1024)/660.0f,-Ignore_Limit,Ignore_Limit,0);
    LX_Norm = DeadZone_Process((float)(DataPack.ch2-1024)/660.0f,-Ignore_Limit,Ignore_Limit,0);
    LY_Norm = DeadZone_Process((float)(DataPack.ch3-1024)/660.0f,-Ignore_Limit,Ignore_Limit,0);

    float temp;

    temp = MouseCoefficient*((float)DataPack.x)/32768.0f;
    temp=temp>1?1:temp;
    temp=temp<-1?-1:temp;
    MouseX_Norm = temp;
    
    temp = MouseCoefficient*((float)DataPack.y)/32768.0f;
    temp=temp>1?1:temp;
    temp=temp<-1?-1:temp;
    MouseY_Norm = temp;

    
    temp = MouseCoefficient*((float)DataPack.z)/32768.0f;
    temp=temp>1?1:temp;
    temp=temp<-1?-1:temp;
    MouseZ_Norm = temp;

    /*按键处理*/
    Key_Process();
}

/**
 * @brief  按键处理 Key Process
 * @param  None
 * @return None
 */
void DR16_Classdef::Key_Process(void)
{
    for(short i=0;i<16;i++)
    {
        //检测到对应按键按下就置key结构数组相关位
        if(DataPack.key & (0x01<<i)) 
            Key[i].isPressed = true;
        else
        {
            Key[i].isPressed = false;
            Key[i].isTriggered = false;
        }
    }
    //鼠标左右键处理
    if(DataPack.press_l == 0x01)
        Key[_Mouse_L].isPressed = true;
    else
    {
        Key[_Mouse_L].isPressed = false;
        Key[_Mouse_L].isTriggered = false;
    }
    if(DataPack.press_r == 0x01)
        Key[_Mouse_R].isPressed = true;
    else
    {
        Key[_Mouse_R].isPressed = false;
        Key[_Mouse_R].isTriggered = false;
    }
}

/**
 * @brief  注册按键回调函数
 * @param  _Key 按下的按键
 * @param  Fun_Ptr 对应按键要回调的函数
 * @return None
 */
void DR16_Classdef::Register_Click_Fun(int _Key, CLICK_EXCE Fun_Ptr)
{
    Click_Fun[_Key] = Fun_Ptr;
}

/**
 * @brief  按键回调函数
 * @param  None
 * @return None
 */
void DR16_Classdef::Exce_Click_Fun()
{
    for(size_t i = 0; i < 18; i++)
    {
        if(Click_Fun[i]!= NULL &&IsKeyPress(i)&&!IsKeyPress(_CTRL))
        {
            if(!Key[i].isTriggered)
            {
            Key[i].isTriggered = 1;
            Click_Fun[i]();
            }
        }
    }
}

/**
 * @brief  以下Getxxx函数功能都是获得数据包中的Get后面数据的值。
 * @param  None
 * @return Get后面的数据的值
 */
uint64_t DR16_Classdef::GetCh0(void)
{
    return DataPack.ch0;
}

uint64_t DR16_Classdef::GetCh1(void)
{
    return DataPack.ch1;
}


uint64_t DR16_Classdef::GetCh2(void)
{
    return DataPack.ch2;
}


uint64_t DR16_Classdef::GetCh3(void)
{
    return DataPack.ch3;
}

SW_Status_Typedef DR16_Classdef::GetS2(void)
{
    return (SW_Status_Typedef)DataPack.s2;
}
SW_Status_Typedef DR16_Classdef::GetS1(void)
{
    return (SW_Status_Typedef)DataPack.s1;
}
int64_t DR16_Classdef::GetMouseX(void)
{
    return DataPack.x;
}
int64_t DR16_Classdef::GetMouseY(void)
{
    return DataPack.y;
}
int64_t DR16_Classdef::GetMouseZ(void)
{
    return DataPack.z;
}
uint64_t DR16_Classdef::GetPress_L(void)
{
    return DataPack.press_l;
}
uint64_t DR16_Classdef::GetPress_R(void)
{
	return DataPack.press_r;
}
uint64_t DR16_Classdef::Getkey(void)
{
    return DataPack.key;
}
/**
 * @brief  归一化后的通道0123、鼠标XYZ值(Left_X_Axis,Right_Y_Axis,balabala)
 * @param  None
 * @retval -1~1之间的通道值
 */
float DR16_Classdef::Get_RX_Norm(void)
{
    return RX_Norm;
}
float DR16_Classdef::Get_RY_Norm(void)
{
    return RY_Norm;
}
float DR16_Classdef::Get_LX_Norm(void)
{
    return LX_Norm;
}
float DR16_Classdef::Get_LY_Norm(void)
{
    return LY_Norm;
}
float DR16_Classdef::Get_MouseX_Norm(void)
{
    return MouseX_Norm;
}
float DR16_Classdef::Get_MouseY_Norm(void)
{
    return MouseY_Norm;
}
float DR16_Classdef::Get_MouseZ_Norm(void)
{
    return MouseZ_Norm;
}

/**
 * @brief  用于判断某个按键是否按下,组合键立马判断
 * @param  _key 头文件中宏定义的key键，如_w,_s等
 * @retval 按下为ture，没按下为false
 */
bool DR16_Classdef::IsKeyPress(int _key)
{
    return Key[_key].isPressed;
}

/**
 * @brief   得到DR16成员变量status的值，常用于判断DR16是否在线
 * @param   None
 * @return  Connection_Lost         DR16离线  
 * @return  Connection_Established  DR16在线
 */
LinkageStatus_Typedef DR16_Classdef::GetStatus(void)
{
  return Status;
}


/**
* @brief  连接确认，更新设备的连接状态。每100ms内没有调用DataCapture()将
*         进入离线模式。
* @param  current_check_time 当前系统时间（毫秒）.
* @return None
*/
void DR16_Classdef::Check_Link(uint32_t current_check_time)
{
  static uint32_t dt = 0;

  /*开始检测*/
  if(last_check_time == 0)
  {
    last_check_time = current_check_time;
  }
  else
  {
      dt = current_check_time - last_check_time;
      if (dt > 100)
      {
          /*时钟计时溢出*/
          if (dt > 1000)
              last_check_time = 0;
          /*每100ms不置位就认为掉线*/
          else
              Status = Connection_Lost;

          /*重新开始检测*/
          last_check_time = 0;
      }
      else {}
  }
}

/**
 * @brief  死区处理，常用于消除零点附近的微小误差
 * @param  num:要处理的数; DZ_min,DZ_max:死区范围;DZ_num:落在死区内时返回的值
 * @return 处理后的结果
 */
float DeadZone_Process(float num,float DZ_min, float DZ_max, float DZ_num)
{
    //若在死区内则返回死区值
    if(num<DZ_max&&num>DZ_min)
    {
        return DZ_num;
    }
    else
        return num;
}

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
