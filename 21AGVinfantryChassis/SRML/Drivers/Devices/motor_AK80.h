/**
  ******************************************************************************
  * Copyright (c) 2020 - ~, SCUT-RobotLab Development Team
  * @file    motor_AK80.h
  * @author  Mentos Seetoo 1356046979@qq.com
  * @brief   Driver for T-Motor A-Series Dynamic Module(AK80-6, AK80-9), which is
  *          highly intergrated with internal gear and controller, on embedded 
  *          platform. Extension package of 'motor.h'.
  * @date    2020-07-11
  * @version 1.0
  * @par Change Log:
  * <table>
  * <tr><th>Date        <th>Version  <th>Author    		  <th>Description
  * <tr><td>2020-07-11  <td> 1.0     <td>Mentos Seetoo  <td>Creator
  * </table>
  *
  ==============================================================================
                          How to use this library 
  ==============================================================================
    @usage
      1. Construct a new object of 'Motor_AK80_9'.
      2. Optionally call `setZero_cmd()` & `set_pid()`.
      3. MUST call `enterCtrlMode_cmd()` once and send by CAN before control loop.
      4. Use `pack_cmd()` & `unpack_cmd()` to pack/unpack CAN meaasge data.
      
      - To view more details about this dynamic module, please read  
        'AK80-6_Cook_Book_v1.pdf' following in the folder.
        
    @warning
      - Standard C++11 required!
      - Unsafe under multithreading.
  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have many 
  * bugs, update the version Number, write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding 
  * through your new brief.
  *
  * <h2><center>&copy; Copyright (c) 2020 - ~, SCUT-RobotLab Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
  */
#pragma once
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <math.h>
#ifdef __cplusplus
/* Private macros ------------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
struct _AKParamRange_t
{
  float P_MIN;
  float P_MAX;
  float V_MIN;
  float V_MAX;
  float T_MIN;
  float T_MAX;
  float KP_MIN;
  float KP_MAX;
  float KD_MIN;
  float KD_MAX;
  
  _AKParamRange_t(float p_min, float p_max, float v_min, float v_max,  \
                  float t_min, float t_max, float kp_min, float kp_max, \
                  float kd_min, float kd_max):P_MIN(p_min), P_MAX(p_max),\
                  V_MIN(v_min), V_MAX(v_max), T_MIN(t_min), T_MAX(t_max), \
                  KP_MIN(kp_min), KP_MAX(kp_max), KD_MIN(kd_min), KD_MAX(kd_max){}
};

static int float_to_uint(float x, float x_min, float x_max, int bits)
{
  /* 
    Converts a float to an unsigned int, given range and number of bits
  */
  float span = x_max - x_min;
  float offset = x_min;
  return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

static float uint_to_float(int x_int, float x_min, float x_max, int bits){
  /* 
    converts unsigned int to float, given range and number of bits
  */
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/* Exported function declarations --------------------------------------------*/

class Motor_AK80_9 
{
private:
  const _AKParamRange_t PARAM =                //!< Parameter range for specific model
    _AKParamRange_t(-95.5, 95.5, -30, 30, -18, 18, 0, 500, 0, 5);

  float position_des, velocity_des, torque_ff;  //!< Desire value & feed forward torque.
  float Kp, Kd;                                 //!< internal pid controller.
  bool  is_ctrlMode;                            //!< motor only works under control mode.
  int32_t ID;
  
public:
  float position_fb, velocity_fb, torque_fb;    //!< Feed back value.

  Motor_AK80_9(uint8_t id, float p_gain, float d_gain)
  { 
    ID = id;
    is_ctrlMode = false;
    Kp = fminf(fmaxf(PARAM.KP_MIN, p_gain), PARAM.KP_MAX);
    Kd = fminf(fmaxf(PARAM.KD_MIN, d_gain), PARAM.KD_MAX);
  };
  void set_pid(float p_gain, float d_gain)
  { 
    Kp = fminf(fmaxf(PARAM.KP_MIN, p_gain), PARAM.KP_MAX);
    Kd = fminf(fmaxf(PARAM.KD_MIN, d_gain), PARAM.KD_MAX);
  }

  /*  
    Be careful of the input array.
  */
  void pack_cmd(uint8_t _package[],float p_des, float v_des, float t_ff)
  {
    if(is_ctrlMode)
    {
      //Constrain
      position_des = fminf(fmaxf(PARAM.P_MIN, p_des), PARAM.P_MAX);
      velocity_des = fminf(fmaxf(PARAM.V_MIN, v_des), PARAM.V_MAX);
      torque_ff = fminf(fmaxf(PARAM.T_MIN, t_ff), PARAM.T_MAX);
      
      //Convert float to uint.
      int p_int = float_to_uint(position_des, PARAM.P_MIN, PARAM.P_MAX, 16);
      int v_int = float_to_uint(velocity_des, PARAM.V_MIN, PARAM.V_MAX, 12);
      int t_int = float_to_uint(torque_ff, PARAM.T_MIN, PARAM.T_MAX, 12);
      int kp_int = float_to_uint(Kp, PARAM.KP_MIN, PARAM.KP_MAX, 12);
      int kd_int = float_to_uint(Kd, PARAM.KD_MIN, PARAM.KD_MAX, 12);
      
      //Pack
      _package[0] = p_int >> 8;
      _package[1] = p_int & 0xff;
      _package[2] = v_int >> 4;
      _package[3] = ((v_int&0xF)<<4)|(kp_int>>8);
      _package[4] = kp_int&0xFF;
      _package[5] = kd_int >> 4;
      _package[6] = ((kd_int&0xF)<<4)|(t_int>>8);
      _package[7] = t_int&0xff;
    }
    else
    {
      for(uint8_t i = 0; i < 8; i++)
        _package[i] = 0;
    }
  }
  
  void unpack_reply(uint8_t _package[])
  {
    //Unpack
    int p_int = (_package[1] << 8)|_package[2];
    int v_int = (_package[3] << 4)|(_package[4] >> 4);
    int i_int = ((_package[4] & 0x0f) << 8)|_package[5];
    
    //Convert int to float
    position_fb = uint_to_float(p_int, PARAM.P_MIN, PARAM.P_MAX, 16);
    velocity_fb = uint_to_float(v_int, PARAM.V_MIN, PARAM.V_MAX, 12);
    torque_fb = uint_to_float(i_int, PARAM.T_MIN, PARAM.T_MAX, 12);
  }
  
  void setZero_cmd(uint8_t _package[])
  { 
    const uint8_t temp[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0XFE};
    
    for(uint8_t i = 0; i < 8; i++)
      _package[i] = temp[i];
  }
  
  void enterCtrlMode_cmd(uint8_t _package[])
  {
    const uint8_t temp[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,0XFC };
    for(uint8_t i = 0; i < 8; i++)
      _package[i] = temp[i];
    
    is_ctrlMode = true;
  }
};

#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
