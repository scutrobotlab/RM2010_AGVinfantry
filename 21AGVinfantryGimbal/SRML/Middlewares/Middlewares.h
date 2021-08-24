/**
  ******************************************************************************
  * @file    Middlewares.h
  * @brief   Header to include all Middlewares.
  * @version 0.0.1
  ******************************************************************************
  * MOST IMPORTANTLY, this library is not open source for developers from other
  * schools' robot team currently. Plz make sure your code is under your supervision.
  * 
  * Thank for @mannychen @HuanpengLu and other pioneers who have put forward such 
  * architechure, and the endeavors of all developers.
  * 
  * By downloading, copying, installing or using the software you agree to this license.
  * If you do not agree to this license, do not download, install,
  * copy or use the software.
  * 
  *                          License Agreement
  *                For SCUT RobotLab Middleware Layer Library
  * 
  * Copyright (c) 2019 - ~, SCUT RobotLab Development Team, all rights reserved.
  * 
  * This file includes all of the headers of SRML.
  * 
  * Before using this library, plz make sure that u have read the README document
  * carefully,
  *    @note
  *     - Plz do not modifiy this file(Except for developer).
  *     - Plz remember to update the version number.
  */
#pragma once
/** @addtogroup Middlewares
  * @{
  */
#include <srml_config.h>
/* Algorithms header begin */
#if USE_SRML_PID
#include "Algorithm/PID.h"
#endif
#if USE_SRML_FILTER
#include "Algorithm/filters.h"
#endif
#if USE_SRML_MYMAT
//#include "Algorithm/myMat.h"
#endif
#if USE_SRML_KALMAN
//#include "Algorithm/kalman.h"
#endif
#if USE_SRML_TRACK_GENERATOR
//#include "Algorithm/track_generator.h"
#endif
/* Algorithms header end */

/* Protocols header begin */
#if USE_SRML_SERIAL_LINE_IP
#include "Protocol/SerialLineIP.h"
#endif
/* Protocols header end */

/* Utilities header begin */
#if USE_SRML_MYASSERT
#include "Utility/myAssert.h"
#endif
#if USE_SRML_LIST
#include "Utility/linux_list.h"
#endif
#if USE_SRML_SYSLOG
#include "Utility/sys_log.h"
#endif
#if USE_SRML_SYSANALYSIS
#include "Utility/sys_analysis.h"
#endif
/* Utilities header end */

/* Modules header begin */
#if USE_SRML_GIMBAL
//#include "Module/gimbal.h"
#endif
#if USE_SRML_CHASSIS
#include "Module/chassis.h"
#endif
#if USE_SRML_POW_CTRL
#include "Module/power_ctrl.h"
#endif
/* Modules header end */

/**
  * @}
  */
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
