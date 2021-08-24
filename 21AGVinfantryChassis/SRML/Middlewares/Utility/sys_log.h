/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    sys_log.h
  * @author  Mentos_Seetoo 1356046979@qq.com
  * @brief   Syslog is a light-weight log module in embedded software system for
  *          SCUT-RobotLab.
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
#ifndef __SYS_LOG_H__
#define __SYS_LOG_H__
/* Includes ------------------------------------------------------------------*/
#include "linux_list.h"

#ifdef __cplusplus
#include <stdarg.h>
#include <string>
#include <stdint.h>

/* Private define ------------------------------------------------------------*/
#define LOG_OK              1
#define LOG_ERROR           0
#define LOG_NAME_MAX_LEN    16
#define LOG_LINE_BUF_SIZE   256
#define DEFAULT_TAG         default_tag_info
#define _DEBUG_             LOG_LVL_DEBUG
#define _INFO_              LOG_LVL_INFO
#define _WARN_              LOG_LVL_WARN
#define _ERROR_             LOG_LVL_ERROR
#define _FATAL_             LOG_LVL_FATAL

/* Private variables ---------------------------------------------------------*/
static const char *const default_tag_info = "Default";

/* Private type --------------------------------------------------------------*/
enum eLogLevel
{
  LOG_LVL_ALL = 0U,
  LOG_LVL_DEBUG,
  LOG_LVL_INFO,
  LOG_LVL_WARN,
  LOG_LVL_ERROR,
  LOG_LVL_FATAL
};

typedef struct 
{
  eLogLevel threshold_level;
  bool   out_File_Line;
  bool   init_flag;
}LogConfig_t;

typedef uint32_t (*LogBacken_t)(uint8_t *msg, uint16_t len);

typedef struct 
{
  char         tag[LOG_NAME_MAX_LEN + 1];
  eLogLevel    threshold_level;
  LogBacken_t  output_backen;
  list_t       list;
}LogFilter_t;

typedef struct 
{
  uint8_t min;
  uint8_t sec;
  uint16_t msec;
}LogTimeStamp_t;

typedef struct
{
  char      log_array[LOG_LINE_BUF_SIZE + 2];
  uint16_t  cur_len;
  size_t write_string(const char *src);
}LogBuff_t;

/* Exported function declarations --------------------------------------------*/
class CLogger
{
    static CLogger* InstanceHandle;
private:
    LogTimeStamp_t  SystemTime;
    LogConfig_t     Configurator;
    LogBuff_t       Buffer;
    LogFilter_t     Filter;
    uint32_t (*getSysTime)(void);
    
    void updateTimeStamp();
    void manager(const char *tag, eLogLevel msg_level,const char *format, va_list args);
    LogFilter_t* check_filter(const char *tag, eLogLevel msg_level);
    int32_t formatter(LogFilter_t* , eLogLevel msg_level, const char *format, va_list args);
public:
    CLogger();
    int32_t getMilliTick_regist(uint32_t (*pFunc)());
    int32_t global_conf(eLogLevel threshold_level, bool out_File_Line);
    int32_t filter_conf(const char* tag, eLogLevel threshold_level, LogBacken_t backen, LogFilter_t* new_Filter = NULL);
    void Record(eLogLevel msg_level, const char *tag, const char *format, ...);
    void Record(eLogLevel msg_level, const char *format, ...);
};


#endif
#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
