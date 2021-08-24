/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    sys_log.cpp
  * @author  Mentos_Seetoo 1356046979@qq.com
  * @brief   Syslog is a light-weight log module in embedded software system for
  *          SCUT-RobotLab.
  * @date    2019-9-28
  * @version 1.0
  * @par Change Log：
  * <table>
  * <tr><th>Date        <th>Version  <th>Author         <th>Description
  * <tr><td>2019-06-12  <td> 1.0     <td>Mentos Seetoo  <td>Creator
  * </table>
  *
  ============================================================================== 
                            How to use this library  
  ==============================================================================
    @note
      -# `getMilliTick_regist()` 注册获取系统时间的回调函数(1Tick = 1ms)
      
      -# `global_conf()`配置全局等级参数，默认情况下等级阈值为"ALL",不输出文件名和行名。
      
      -# `filter_conf()`配置一个过滤器的参数，根据配置的过滤器类型分两种情况：
        - 新建一个过滤器结构体作为容器（不需要初始化）
        - 使用sysLog自带的一个默认过滤器，不需要传入过滤器结构体容器的指针，但标签名
          字要为“DEFAULT_TAG”。
          
      -# `Record()`在需要的地方使用这个函数。
      
    @see
      - To view more about this module, please visit: \n
        https://www.scut-robotlab.cn/confluence/x/K4QxAg
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
#include "sys_log.h"
#include "myAssert.h"

/* Private define ------------------------------------------------------------*/
using namespace std;

/* Private variables ---------------------------------------------------------*/
static const char * const level_info[] = { "[ DEBUG ]",
                                           "[ INFO ]",
                                           "[ WARN ]",
                                           "[ ERROR ]",
                                           "[ FATAL ]"};
CLogger* CLogger::InstanceHandle = NULL;
                                          
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
size_t LogBuff_t::write_string(const char *src)
{
  /* Check input param */
  
  /* Write to buff */
  while(*src != 0)
  {
    /* make sure destination has enough space */
    if(cur_len <= LOG_LINE_BUF_SIZE)
    {
      log_array[cur_len] = *src++;
      cur_len ++;
    }
    else break;
  }
  return cur_len;
}

void Error_Handle();
/* Function prototypes -------------------------------------------------------*/
CLogger::CLogger()
{
  /* Single instance for logger. */
  if(InstanceHandle == NULL)
  {
    /* Init lists and parameters */
    INIT_LIST_HEAD(&Filter.list);
    strcpy(Filter.tag, DEFAULT_TAG);
    InstanceHandle = this;
  }
  else 
    this->~CLogger();
}

/**
* @brief  Deploy a new filter or update an exsisting filter.
* @note   Config at least one filter before using logger. If 
*         usr need to config the default filter, set tag as
*         `DEFAULT_TAG` and the first param is disabled.
* @param  &new_Filter: make sure new_Filter is allocated and 
*         will never be deleted 
* @param  tag_name: Tag name of this filter
* @param  level: level of this filter
* @param  backen: output backen of this filter
* @retval LOG_OK: operate successed
* @retval LOG_ERROR: Wrong parameters
*/
int32_t CLogger::filter_conf(const char* tag, eLogLevel threshold_level, LogBacken_t backen, LogFilter_t* new_Filter)
{
  list_t *node = &Filter.list;
  LogFilter_t *temp_Filter = NULL;
  
  /* Check filter param */
  if(tag == NULL || backen == NULL) return LOG_ERROR;
  
  /* Configure the default filter*/
  if(strstr(tag, DEFAULT_TAG))
  {
    Filter.threshold_level = threshold_level;
    Filter.output_backen   = backen;
    Configurator.init_flag = LOG_OK;
    return LOG_OK;
  }
  else
  {
    if(new_Filter == NULL) return LOG_ERROR;
    new_Filter->output_backen   = backen;
    new_Filter->threshold_level = threshold_level;
    strcpy(new_Filter->tag, tag);
    
    /* Iterate to find if this filter already exsists */
    do
    {  
      temp_Filter = list_entry(node, LogFilter_t, list);
      
      if(strstr(tag, temp_Filter->tag))
      {
        /* Find the mapped filter.*/
        list_replace(&temp_Filter->list, &new_Filter->list);
        Configurator.init_flag = LOG_OK;
        return LOG_OK;
      }
      else 
      {
        node = node->next;
      }
    }while(node != &Filter.list);
    
    /* Add new filter */
    list_add_tail(&new_Filter->list, &Filter.list);
    Configurator.init_flag = LOG_OK;
    return LOG_OK;
  }
}

int32_t CLogger::global_conf(eLogLevel threshold_level, bool out_File_Line)
{
  Configurator.threshold_level = threshold_level;
  Configurator.out_File_Line   = out_File_Line;
  return LOG_OK;
}

/**
* @brief  Add and Output a Log message.
* @param  tag:  tag
* @param  msg_level level information including: \n
*         `_DEBUG_`, `_INFO_`, `_WARN_`, `_ERROR_`, `_FATAL_`
* @param  format: message string
*/
void CLogger::Record(eLogLevel msg_level, const char *tag, const char *format, ...)
{
  va_list args;
  
  if(Configurator.init_flag != LOG_OK)
  {
    return;
  }
  /* args point to the first variable parameter */
  va_start(args, format);
  
  manager(tag,msg_level,format,args);
  
  va_end(args);
}

/**
* @brief  Add and Output a Log message using `DEFAULT_TAG`.
* @note   Default filter must be configured before using this API.
* @param  msg_level level information including: \n
*         `_DEBUG_`, `_INFO_`, `_WARN_`, `_ERROR_`, `_FATAL_`
* @param  format: message string
*/
void CLogger::Record(eLogLevel msg_level, const char *format, ...)
{
  va_list args;
  
  if(Configurator.init_flag != LOG_OK)
  {
    return;
  }
  /* args point to the first variable parameter */
  va_start(args, format);
  
  manager(DEFAULT_TAG,msg_level,format,args);
  
  va_end(args);
}


void CLogger::manager(const char *tag, eLogLevel msg_level,const char *format, va_list args)
{
  LogFilter_t *Mapped_Filter;
  
  
  /*!< 
    Check the global config & filter list.
  */
  if(msg_level < Configurator.threshold_level || msg_level == 0) 
  {
    return;
  }
  if((Mapped_Filter = check_filter(tag, msg_level)) == NULL)
  {
    return;
  }
  else {};
  
  /*!< 
    Format the message.
  */
  formatter(Mapped_Filter, msg_level, format, args);
  
  /*!<
    Output message to used backen.
  */
  if(Mapped_Filter->output_backen != NULL)
  {
    Mapped_Filter->output_backen((uint8_t*)Buffer.log_array, Buffer.cur_len);
  }
  else
  {
    return;
  }
}


__inline LogFilter_t* CLogger::check_filter(const char *tag, eLogLevel msg_level)
{
  list_t *node = &Filter.list;
  LogFilter_t *temp_Filter = NULL;
 
  /*!<
      Iterate through the filter list.
  */
  do
  {  
    temp_Filter = list_entry(node, LogFilter_t, list);
    
    if(strstr(tag, temp_Filter->tag))
    {
      /* Find the mapped filter.*/
      if(msg_level >= temp_Filter->threshold_level) 
        return temp_Filter;
      else {};
    }
    else 
    {
      node = node->next;
    }
  }while(node != &Filter.list);
  
  /* Not find the right filter. */
  return NULL;
}


__inline int32_t CLogger::formatter(LogFilter_t* Filter2fmt, eLogLevel msg_level, const char *format, va_list args)
{
  int16_t fmt_result;
  Buffer.cur_len = 0;
  
  /* Add time stamp header.*/
  updateTimeStamp();
  snprintf(Buffer.log_array, LOG_LINE_BUF_SIZE - Buffer.cur_len, "[%02d:%02d.%03d]", SystemTime.min, SystemTime.sec, SystemTime.msec);
  Buffer.cur_len = strlen(Buffer.log_array);
  
  /* Add level & tag specificed infomation.*/
  Buffer.write_string(level_info[msg_level -1]);
  Buffer.write_string("<");
  Buffer.write_string(Filter2fmt->tag);
  Buffer.write_string(">: ");
  
  /* Add File & Line infomation */
  if(Configurator.out_File_Line == true)
  {
    Buffer.cur_len += snprintf(Buffer.log_array + Buffer.cur_len,LOG_LINE_BUF_SIZE - Buffer.cur_len,"File:%s, Line:%d",__FILE__ , __LINE__);
  }
  
  /* Add user detail information.*/
  fmt_result = vsnprintf(Buffer.log_array + Buffer.cur_len, LOG_LINE_BUF_SIZE - Buffer.cur_len, format, args);
  
  /* calculate final log length */
  if ((Buffer.cur_len + fmt_result <= LOG_LINE_BUF_SIZE) && (fmt_result > -1))
  {
      Buffer.cur_len += fmt_result;
  }
  else
  {
      /* using max length */
      Buffer.cur_len = LOG_LINE_BUF_SIZE;
  }
  
  /* Log message newline sign */
  Buffer.write_string("\r\n");
  
  return true;
}

/**
 * @brief  Regist the implement of `getSysTime()`
 * @param  pFunc: Pointer of get-time function.
 * @retval LOG_OK: operate successed.
 * @retval LOG_ERROR:null pointer of param.
 */
int32_t CLogger::getMilliTick_regist(uint32_t (*pFunc)())
{
  if(pFunc != NULL)
  {
    getSysTime = pFunc;
    return LOG_OK;
  }
  else 
    return LOG_ERROR;
}

/**
* @brief  Update time stamp by sysTime and convert it to "minute:second:millisecond"
* @param  sysTime: millisecond that system had run.
* @author Mentos Seetoo
*/
void CLogger::updateTimeStamp()
{
  /* Check input param */
  
  uint32_t sysTime = getSysTime();
  SystemTime.msec = sysTime%1000;
  SystemTime.sec  = (sysTime/1000)%60;
  SystemTime.min  = sysTime/60000;
}

/**
 * @brief Normally your programm would never reach here, unless an error occurred.
 */
void Error_Handle()
{
  while(1);
}

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
