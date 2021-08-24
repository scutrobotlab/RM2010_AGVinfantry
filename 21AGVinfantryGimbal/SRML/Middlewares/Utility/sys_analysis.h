/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    sys_analysis.h
  * @author  Mentos_Seetoo 1356046979@qq.com
  * @brief   Sys_analysis is an tool package for control system analysis running
  *          in MCU. This file provide common analysis methods like frequency-
  *          domain analysis, phase plot for robotic developer to optimize the 
  *          performance of a physical system, such as gimbal, robotic arm...
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
#ifndef __SYS_ANALYSIS_H__
#define __SYS_ANALYSIS_H__
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <math.h>
#include "linux_list.h"
#include "myAssert.h"
#ifdef __cplusplus

/* Private define ------------------------------------------------------------*/
#define AMPT_ERROR_RATE       0.05f
#define MAX_BUFF_LEN          300
/* Private variables ---------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
typedef uint32_t (*ANLS_SystemTick_Fun)(void);

enum EState_t
{
  ANLS_WAITING = 0U,
  ANLS_PROCESSING,
  ANLS_FINISHED
};

enum EMode_t
{
  AUTO_FREQ = 0U,
  MANUAL_FREQ,
  PHASE_PLANE
};

typedef struct 
{
  EState_t  state;
  EMode_t   mode;
  uint16_t  series_len;
  float    *Ampt_Series;
  float    *Phase_Series;
  float    *Freq_Series;
}PCB_t;
/* Exported function declarations --------------------------------------------*/
class CAnalyzer
{
//private:
  public:
    PCB_t Param;
    /*auxiliary variable*/
    float val_sample, this_dSample, last_dSample, ampt_sample, val_signal,ampt_signal;
    float Ampt_Gains, last_ampt_gain, ampt_gain, Phase_Losts, phase_lost;
    uint16_t  hit_cnt, hit_num_per_freq;
    uint16_t sampled_cnt;
    float fixed_point;
    /*time variable*/
    float t,t_base;//Unit:second
    uint8_t calculation_core();
public:
    static uint8_t getMicroTick_regist(uint32_t (*getTick_fun)(void));
                                                  /*<! Regist get time function */
    static ANLS_SystemTick_Fun Get_SystemTick;    /*<! Pointer of function to get system tick */
    CAnalyzer();
    ~CAnalyzer();
    PCB_t   sample_control(float sample_value, float output_percent);
    uint8_t start(EMode_t mode);
    uint8_t stop();
    void    set(int16_t resolution, int16_t max_frequence, float fixed_point, float init_ampt, uint16_t hit_num_per_freq);
    void    results();

};


#endif
#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
