/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    chassis.cpp
  * @author  Kainan.Su 15013073869
  * @brief   Header file of chassis.
  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have many 
  * bugs, update the version Number, write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding 
  * through your new brief.
  *
  * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
  */
#ifndef _CHASSIS_H
#define _CHASSIS_H

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdint.h>

#if defined(ARM_MATH_CM4) && defined(STM32F405xx)
  #define  ARM_MATH_USED
  #include <stm32f4xx.h>
  #include <arm_math.h>
#else 
  #define _USE_MATH_DEFINES
  #define PI         M_PI
  #include <math.h>
#endif

/* Private  ------------------------------------------------------------------*/
#ifdef __cplusplus
#define POSITIVE   1
#define NEGATIVE  -1
#if defined(SWERVE_CHASSIS)
  #define WHEEL_NUM             4
  #define STEER_NUM             4
  enum _chassis_WheelType
  {
    LF = 0U,
    RF,
    RB,
    LB
  };
  enum _chassis_SteerType
  {
    s_LF = 0U,
    s_RF,
    s_RB,
    s_LB
  };
#else 
  #if defined(LUNAR_CHASSIS)
    #define WHEEL_NUM             6
    enum _chassis_WheelType
    {
      LF = 0U,
      RF,
      RB,
      LB,
      LM,
      RM
    };
  #else
    #define WHEEL_NUM             4
    enum _chassis_WheelType
    {
      LF = 0U,
      RF,
      RB,
      LB
    };
  #endif
#endif
  
enum _chassis_Mode
{
  Normal_Speed = 0U,
  Normal_Position,
  Follow_Position,
  Halt
};
  
enum _chassis_SpeedGears
{
  SLOW_GEAR = 0U,
  NORMAL_GEAR,
  FAST_GEAR
};


typedef struct _chassis_GlobalPos
{
  float x;
  float y;
  float roll;
  float pitch;
  float yaw;

  _chassis_GlobalPos& operator = (_chassis_GlobalPos& value)
  {
    if(&value != this)
    {
        x = value.x ; y = value.y; roll = value.roll; pitch = value.pitch; yaw = value.yaw;
    }
    return *this;
  }
  
  _chassis_GlobalPos& operator - (_chassis_GlobalPos& value){
    x -= value.x;
    y -= value.y;
    roll -= value.roll ;
    pitch -= value.pitch;
    yaw -= value.yaw;
    return *this;
  }
}_chassis_GlobalPos;

typedef struct _chassis_Velocity
{
  float x_speed;
  float y_speed;
  float z_speed;
  
  _chassis_Velocity& operator = (_chassis_Velocity& value){
    x_speed = value.x_speed; y_speed = value.y_speed; z_speed = value.z_speed;
    return *this;
  }
  
  _chassis_Velocity& operator + (_chassis_Velocity& value){
    x_speed += value.x_speed;
    y_speed += value.y_speed;
    z_speed += value.z_speed;
    return *this;
  }
}_chassis_Velocity;

#if defined(SWERVE_CHASSIS)
/**
  @brief  `str_angle` maybe is an angle of motor encoder or servo,
          while 'vect_angle' is a mathematical angle(polar angle) of a vector.
          When the `str_angle` is unlimited, `str_angle` is equal to `vect_angle`,
          and `speed_direction` is always `1`;
*/
typedef struct _Steer_Type
{
  float  vect_angle;     //!<Velocity angle(-oo, +oo) 
  float  vect_angleAbs;  //!<Absolute velocity angle(-PI, PI).  
  float  str_angle;      //!<Steer angle(-range/2, range/2)
  int8_t speed_direction;//!<Speed direction of steer, only contains two state{-1,1}
}_Steer_Type;
#endif

typedef struct 
{
  bool  torqueOptimize_flag;
  bool  attitudeOptimize_flag;
  int16_t   wheel_max_speed;
  
  /* Time interval for speed resolve. */
  float  task_run_interval;
  
  /* Acceleration for different stages(Unit: rpm/s) */
  int16_t   max_launch_acceleration;
  int16_t   max_normal_acceleration;
  uint16_t   max_brake_acceleration;
  int16_t   launch_speed;
  
  /* Coefficients for different gears(Max_gear_speed = Coefficients*wheel_max_speed).(Between 0 and 1) */
  float  coefficient_slow;
  float  coefficient_normal;
  float  coefficient_fast;
  float  coefficient_z;
	
	/* User Specific Parameters */
#if defined(SWERVE_CHASSIS)
  float STEER_RANGE;
#endif
	float CHASSIS_WHEEL_TRACK;
	float CHASSIS_WHEEL_BASE;
	float WHEEL_RADIUS;
	int16_t MAX_MOTOR_OUTPUT;
	float RUN_INTERVAL;	
}_chassis_Parameter;


template<typename Type>
Type _Chassis_Constrain(Type input,Type min,Type max){
  if (input <= min)
    return min;
  else if(input >= max)
    return max;
  else return input;
}

template<typename Type>
void _Chassis_BubbleSort(Type a[], int n)
{
  int i,j;
  Type temp;
  for (j = 0; j < n - 1; j++){
    for (i = 0; i < n - 1 - j; i++){
      if(a[i] > a[i + 1]){
          temp = a[i];
          a[i] = a[i + 1];
          a[i + 1] = temp;
      }
    }
  }
}

template<typename Type> 
Type _Chassis_Abs(Type x) {return ((x > 0) ? x : -x);}
/* Exported ------------------------------------------------------------------*/
/**
   @brief User Interface
*/
class CChassis
{
  private:
    _chassis_Mode       Mode;
    _chassis_Parameter  Param;
    _chassis_SpeedGears current_gear;

    void model_resolve();
    uint8_t position_control();
    uint8_t speed_control();
    uint8_t torque_optimization();
    uint8_t attitude_control();
    _chassis_Velocity*(*attitude_controller)(const _chassis_GlobalPos* Current, const _chassis_GlobalPos* Target);
    _chassis_Velocity*(*position_controller)(const _chassis_GlobalPos* Current, const _chassis_GlobalPos* Target);
    int32_t*(*speed_controller)(const int16_t* current, const int16_t* target);
#if defined(SWERVE_CHASSIS)
    void steer_angle_convert();
#endif
  public:
    _chassis_GlobalPos   Zero_Pos;                 //!<零位姿点
    _chassis_GlobalPos   Command_Pos;              //!<指令位姿
    _chassis_Velocity    Command_Velocity;         //!<指令速度矢量
    _chassis_Velocity    Target_Velocity;          //!<优化后输出目标速度矢量
    int16_t              wheel_rpmOut[WHEEL_NUM];  //!<轮子解算后转速
    _chassis_GlobalPos   Current_Pos;              //!<当前位姿
    int16_t              wheel_rpm[WHEEL_NUM];     //!<轮子转速
    int16_t              wheel_torque[WHEEL_NUM];  //!<轮子转矩（电流）
    int32_t              wheel_Out[WHEEL_NUM];     //!<控制器作用后输出
#if defined(SWERVE_CHASSIS)
    _Steer_Type          steer_Set[WHEEL_NUM];     //!<轮组角度信息
#endif

    /* 设置初始参数 */
    CChassis( float chassis_wheel_track, float chassis_wheel_base, float wheel_radius,\
							float max_motor_output, int16_t wheel_max_speed,\
							bool optimize_flag = 1, float run_interval = 0.001f, float str_range = 2*PI){
			/* User Specific Parameters */
			Param.CHASSIS_WHEEL_TRACK = chassis_wheel_track;
			Param.CHASSIS_WHEEL_BASE  = chassis_wheel_base;
			Param.WHEEL_RADIUS        = wheel_radius;
			Param.MAX_MOTOR_OUTPUT    = max_motor_output;
      Param.task_run_interval   = run_interval;
      Param.wheel_max_speed     = wheel_max_speed;
			Param.launch_speed        = Param.wheel_max_speed;
      Param.torqueOptimize_flag     = optimize_flag;
      Param.attitudeOptimize_flag   = 0;
      Param.max_launch_acceleration = 6666;
      Param.max_normal_acceleration = 9999;
      Param.max_brake_acceleration  = 23333;
      Param.coefficient_slow  = 0.25;
      Param.coefficient_normal= 0.6;
      Param.coefficient_fast  = 1.0;
      /* Limit rotate max speed  */
      Param.coefficient_z     = 0.8f;
#if defined(SWERVE_CHASSIS)
      Param.STEER_RANGE = str_range;
#endif
    }
    /*
      设置目标值：
      1.速度模式 --> 设置三轴运动速度比例系数(X,Y,Rotate)
      2.位置模式 --> 设置目标位姿（X,Y,Rotate）
    */
    void Set_Target(float, float, float);
    
    /* 设置所有模式运动速度档位（不同档位速度最大值不同）*/
    void Set_SpeedGear(_chassis_SpeedGears);
    
    /* 设置底盘运动参数(后续有需要可继续添加修改其他参数的接口) */
    void Set_SpeedParam(float slow, float normal, float fast, float z);
    void Set_AccelerationParam(int16_t launch, int16_t normal, uint16_t brake);
    void Set_TorqueOptimizeFlag(bool flag);
    void Set_AttitudeOptimizeFlag(bool flag);
    
    /* 从外部加载位置环、速度环控制器，参数列表见函数注释*/
    void Load_AttitudeController(_chassis_Velocity*(*pFunc)(const _chassis_GlobalPos*, const _chassis_GlobalPos*));
    void Load_PositionController(_chassis_Velocity*(*pFunc)(const _chassis_GlobalPos*, const _chassis_GlobalPos*));
    void Load_SpeedController(int32_t*(*pFunc)(const int16_t*, const int16_t*));   
		
    /* 更新普通位置模式下的零位姿点为当前位姿点*/
    void Update_ZeroPose();
    
    /* 更新当前位姿（X,Y,Roll,Pitch,Yaw）*/
    void Update_CurrentPosition(float, float);
    void Update_CurrentAttitude(float, float, float);
    
   /* 切换底盘运行状态 */
    _chassis_Mode Get_Mode();
    uint8_t Switch_Mode(_chassis_Mode target_mode);
    
   /* 控制主函数 */
    _chassis_Mode Chassis_Control();
};
#endif
#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
