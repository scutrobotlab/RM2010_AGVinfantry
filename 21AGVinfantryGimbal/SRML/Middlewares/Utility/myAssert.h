/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    myAssert.h
  * @author  Mentos_Seetoo 1356046979@qq.com
  * @brief   myAssert is a customized assert module for SCUT-RobotLab.
  * @date    2019-11-13
  * @version 1.0
  ============================================================================== 
                            How to use this module  
  ==============================================================================
    @note
    
    @warning
		
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
#ifndef _MY_ASSERT_H
#define _MY_ASSERT_H

#ifdef USE_MY_ASSERT
  #define my_assert(expr) {if(!(expr))while(1){} else void(0);}
#else
  #define my_assert(x)    (void(0))
#endif


#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
