/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    sys_analysis.h
  * @author  Mentos_Seetoo
  * @brief   Sys_analysis is an tool package for control system analysis running
  *          in MCU. This file provide common analysis methods like frequency-
  *          domain analysis, phase plot for robotic developer to optimize the 
  *          performance of the physical system, such as gimbal, robotic arm...
  * @date    2020-1-1
  * @version 0.1
  * @par Change Log：
  * <table>
  * <tr><th>Date        <th>Version  <th>Author         <th>Description
  * <tr><td>2020-01-01  <td> 0.1     <td>Mentos Seetoo  <td>Creator
  * </table>
  *
  ============================================================================== 
                              How to use this library  
  ==============================================================================
    @note
      The following methods is provied by this lib: \n
      - Frequency Domain Analysis.
      - Phase plane.
      
    @see
      - To view more about this lib, please visit: \n
      
    @warning
      - ARM_MATH & DSP is required!!!
      
    @Todo
      - 解决高频区难以采样，生成不准确等问题
      - 完善其他小功能与接口
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
#include "sys_analysis.h"
#include "myAssert.h"

#include <stm32f4xx.h>
#include "arm_math.h"
/* Private define ------------------------------------------------------------*/
float result_series1[MAX_BUFF_LEN];
float result_series2[MAX_BUFF_LEN];
float result_series3[MAX_BUFF_LEN];
/* Private variables ---------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
ANLS_SystemTick_Fun CAnalyzer::Get_SystemTick = NULL;
/* Function prototypes -------------------------------------------------------*/
CAnalyzer::CAnalyzer()
{
  Param.series_len   = 0;
  Param.Ampt_Series  = result_series1;
  Param.Phase_Series = result_series2;
  Param.Freq_Series  = result_series3;
}

/**
 * @brief  Regist get time function(1Tick = 1us)
 * @param  realTime_fun: Pointer of function to get system real time
 * @retval 1: success
           0: error input param
 * @author
 */
uint8_t CAnalyzer::getMicroTick_regist(uint32_t (*getTick_fun)(void))
{
  if(getTick_fun != NULL)
  {
    CAnalyzer::Get_SystemTick = getTick_fun;
    return 1;
  }
  else 
    return 0;
}

/**
 * @brief  Sample control function. This function is reentrant, and it's
 *         recommanded to run at least 1KHz. Because the frquency that 
 *         the function run determined the maximun sample frequency.
 * @param  sample_value: respond value of control system.
 * @param  output_percent: Control output of system. We use this to judge
 *         if saturation is happened.
 * @retval PCB_t:
 */
PCB_t CAnalyzer::sample_control(float sample_value, float output_percent)
{
  switch(Param.state)
  {
    case ANLS_PROCESSING:
      /* Pre-process of sample data */
      last_dSample = this_dSample;
      this_dSample = sample_value - val_sample;
      val_sample = sample_value;
      
      /*Main process*/
      calculation_core();
      break;
    default:
      break;
  }
  
  return Param;
}

uint8_t CAnalyzer::calculation_core()
{
  const float rad2deg = 180.0f/PI;
  float w = Param.Freq_Series[sampled_cnt];
  
  my_assert(Get_SystemTick != NULL);
  /* 
    Resolute sample signal
  */
  /*Reach the sample point.(Only use maximum value)*/
  if(last_dSample >=0 && this_dSample <= 0 && last_dSample != this_dSample) 
  {
    ampt_sample = val_sample - fixed_point;
    phase_lost  = fmodf(w*t, 2*PI)*rad2deg - 90.0f;
    
    ampt_gain = 20*log10f(fabs(ampt_sample)/ampt_signal);
    
    /*Effective sampling(default error rate is 5%)*/
    if(fabs((ampt_gain - last_ampt_gain)/ampt_gain) < AMPT_ERROR_RATE)
    {
      Ampt_Gains += ampt_gain;
      Phase_Losts += phase_lost;
      hit_cnt ++;
    }
    else
      hit_cnt = 0;
    
    last_ampt_gain = ampt_gain;
    
    /*Record and Shift to next sample frequence.*/
    if(hit_cnt == hit_num_per_freq)
    {
      /* Write mean value to buff */
      hit_cnt = 0;
      ampt_gain = Ampt_Gains/hit_num_per_freq;
      phase_lost = Phase_Losts/hit_num_per_freq;
      
      Param.Ampt_Series[sampled_cnt] = ampt_gain;
      Param.Phase_Series[sampled_cnt] = phase_lost;
      
      if(++sampled_cnt == Param.series_len)
      { /* Sample Finished*/
        Param.state = ANLS_FINISHED;
        sampled_cnt = 0;
      }
      else
      { /* Set w to next frequency, phase shift is zero, 
           stabilizing sample time of every frequency.*/
        //hit_num_per_freq = Param.Freq_Series[sampled_cnt] * base_hit_num;
        t_base = Get_SystemTick()*0.000001f - 0.5f*PI/Param.Freq_Series[sampled_cnt];
      }
    }
  }
  else void(0);

  /* Generate new input */
  t = Get_SystemTick()*0.000001f - t_base;  
  val_signal = ampt_signal * arm_sin_f32(w * t) + fixed_point;
  
  return 1;
}

/*Other function*/
uint8_t CAnalyzer::start(EMode_t mode)
{
  Param.mode = mode;
  if(Param.state == ANLS_WAITING || Param.state == ANLS_FINISHED)
  {
    my_assert(Get_SystemTick != NULL);
    
    Param.state = ANLS_PROCESSING;
    t_base = Get_SystemTick()*0.000001f;
    return 1;
  }
  else
    return 0;
}

uint8_t CAnalyzer::stop()
{
  Param.state = ANLS_WAITING;
  return 1;
}

void CAnalyzer::set(int16_t resolution, int16_t max_frequence, float fixed_point, float init_ampt, uint16_t hit_num_per_freq)
{
  /*Calculate the series of frequency*/
  int16_t decade_num = (int16_t)log10f(max_frequence);
  float rest = 0;
  
  if(log10f(max_frequence) - decade_num > 0) decade_num ++;
  for(uint8_t j = 0; j < decade_num; ++j)
  {
    for(uint8_t k = 0; k < resolution; ++k)
    {
      rest = powf(10.0f,j) * (1 + k * 9.0f/(float)resolution);
      if(rest <= max_frequence)
      {
        Param.series_len ++;
        Param.Freq_Series[j*resolution + k] = rest;
      }
      else
        break;
    }
  }
  
  this->fixed_point = fixed_point;
  this->ampt_signal = init_ampt;
  this->hit_num_per_freq = hit_num_per_freq;
}

CAnalyzer::~CAnalyzer()
{
  delete[] Param.Ampt_Series;
  delete[] Param.Phase_Series;
  delete[] Param.Freq_Series;
}
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
