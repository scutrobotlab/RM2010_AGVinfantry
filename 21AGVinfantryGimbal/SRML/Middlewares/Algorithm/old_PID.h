#ifndef _TOOL_PID_H__
#define _TOOL_PID_H__

#include  <arm_math.h>
#include  <drv_timer.h>
#ifdef  __cplusplus
class PIDBase{
public:
  float MODE;//目标值
  float Target;//目标值
  float Current;//当前值
  float Out;//解算后的输出值
  float PTerm,ITerm,DTerm;//PID各个分量输出值
  float Kp, Ki, Kd;//PID各个参数
  float Rpi, Rii, Rdi, Ti;//最小值
  float Rpa, Ria, Rda, Ta;//最大值
  float DeadZone;
  short counter;
  float now_error,last_error,lastlast_error;

  PIDBase(){;}
  void SetPIDBase(float m_Kp, float m_Ki, float m_Kd, float m_Target,
                float m_Rpi, float m_Rpa, float m_Rii, float m_Ria, float m_Rdi, float m_Rda, 
          float m_Ti, float m_Ta, float m_DeadZone);
  void SetTarget(float target);
  void SetTargetOffset(float target);
  void SetCurrent(float current);
  void SetPIDPara(float m_Kp, float m_Ki, float m_Kd);
  void SetTargetMaxMin( float m_Ti, float m_Ta);
  void SetPMaxMin(float m_Rpi, float m_Rpa);
  void SetIMaxMin(float m_Rii, float m_Ria);
  void SetDMaxMin(float m_Rdi, float m_Rda);
  void SetDeadZone(float m_DeadZone);
  };

  class PIDTimer
  {
  public:
  float dt;
  uint32_t   last_time;//上次系统时间记录
  unsigned char UpdataTime(void);
  };

  class PID:public PIDBase,public PIDTimer
  {
  public:

  PID(){;};
  void SetPID(float m_Kp, float m_Ki, float m_Kd, float m_Target,
            float m_Rpi,float m_Rii,float m_Rdi,  
            float m_Rpa,float m_Ria,float m_Rda, 
          float m_Ti, float m_Ta,float m_DeadZone);
  void AdjustPID(void);
  private:
  struct filter_struct
  {
    unsigned char num[3];
    float filterbuff[3][5];//滤波器的数组BUFF
  }filterstruct;

};

class FUZZYPID:public PIDBase,public PIDTimer
{
public:
  float NB,PB,NM,PM,NS,PS;
  float NB_kp,PB_kp,NM_kp,PM_kp,NS_kp,PS_kp;
  float NB_ki,PB_ki,NM_ki,PM_ki,NS_ki,PS_ki;
  float NB_kd,PB_kd,NM_kd,PM_kd,NS_kd,PS_kd;
  float Omega,Omega_Offset;
  float errorC,Kec;
  float a[6];
  FUZZYPID(){;};
  void Set_FUZZYPID(float m_Kp, float m_Ki, float m_Kd, float m_Target,\
                    float m_Rpi,float m_Rpa,float m_Rii,float m_Ria,\
                    float m_Rdi,float m_Rda,float m_Ti, float m_Ta, float m_DeadZone,\
                    float m_NB,  float m_PB,  float m_NM,  float m_PM, float m_NS, float m_PS,\
                    float m_NB_kp,float m_PB_kp, float m_NM_kp,float m_PM_kp, float m_NS_kp,float m_PS_kp,\
                    float m_NB_ki,float m_PB_ki, float m_NM_ki,float m_PM_ki, float m_NS_ki,float m_PS_ki,\
                    float m_NB_kd,float m_PB_kd, float m_NM_kd,float m_PM_kd, float m_NS_kd,float m_PS_kd); 
  void AdjustPID(void);
  void Defuzzification(float error,float errorC);
  void SetOmega(float data,unsigned char dir)
  {	
    if(dir)
      Omega = -(data - Omega_Offset);
    else 
      Omega = (data - Omega_Offset);
  };
  void SetOmegaOffset(float data){
    Omega_Offset = data;
  };

  private:
  struct filter_struct
  {
    unsigned char num[3];
    float filterbuff[3][5];//滤波器的数组BUFF
  }filterstruct;
};


class FUZZYPID_without_gyroscope:public FUZZYPID
{
public:
    FUZZYPID_without_gyroscope(){;};
  void AdjustPID(void);
private:
  struct filter_struct
  {
    unsigned char num[3];
    float filterbuff[3][5];//滤波器的数组BUFF
  }filterstruct;
};


class DUAL_PID:public PIDBase,public PIDTimer
{
public:
float Kpp,Kpn,Kip,Kin,Kdp,Kdn;

  DUAL_PID(){;};
void SetPID(float m_Kpp, float m_Kip, float m_Kdp,
             float m_Kpn, float m_Kin, float m_Kdn, float m_Target,
             float m_Rpi, float m_Rpa, float m_Rii, float m_Ria,
             float m_Rdi, float m_Rda, float m_Ti, float m_Ta,float m_DeadZone);
  void AdjustPID(void);
private:
  
  struct filter_struct
  {
    unsigned char num[3];
    float filterbuff[3][5];//滤波器的数组BUFF
  }filterstruct;

};

typedef struct _PID_Parameter
{
  float Kp;
  float Ki;
  float Kd;
  float Kp_min;
  float Ki_min;
  float Kd_min;
  float Kp_max;
  float Ki_max;
  float Kd_max;
  float Target_min;
  float Target_max;
  float DeadZone; 
}_PID_Parameter;

typedef struct _DUAL_PID_Parameter
{
  float Kpp;
  float Kpn;
  float Kip;
  float Kin;
  float Kdp;
  float Kdn;
  float Kp_min;
  float Ki_min;
  float Kd_min;
  float Kp_max;
  float Ki_max;
  float Kd_max;
  float Target_min;
  float Target_max;
  float DeadZone; 
}_DUAL_PID_Parameter;

/*加载PID参数函数*/
void Load_PIDParameter(PIDBase* p, _PID_Parameter* para);
void Load_PIDParameter(DUAL_PID* p, _DUAL_PID_Parameter* para);
/*设置PID参数结构体*/
void set_PIDParameter(_DUAL_PID_Parameter* p, float Kpp, float Kip, float Kdp,
             float Kpn, float Kin, float Kdn, 
             float Kp_min, float Kp_max, float Ki_min, float Ki_max,
             float Kd_min, float Kd_max, float Target_min, float Target_max,float DeadZone);
void set_PIDParameter(_PID_Parameter* p, float Kp, float Ki, float Kd,
             float Kp_min, float Kp_max, float Ki_min, float Ki_max,
             float Kd_min, float Kd_max, float Target_min, float Target_max,float DeadZone);
#endif
#endif
