#include "Tool_PID.h"
#include "Tool_Filter.h"
static float TRI(float X,float DOWN,float MID,float UP);
static float LAD_DOWN(float X,float MID,float UP);
static float LAD_UP(float X,float DOWN,float MID);
static float N_DOWN(float X,float NM,float NS);
static float N_TRI(float X,float NB,float NM,float NS);
static float N_UP(float X,float NB,float NM);
static float P_DOWN(float X,float PS,float PM);
static float P_TRI(float X,float PS,float PM,float PB);
static float P_UP(float X,float PM,float PB);
#define myabs(x)      ((x)>0? (x):(-(x)))
/**
* @brief  加载PID参数(普通PID)
*/
void Load_PIDParameter(PIDBase* p, _PID_Parameter* para)
{
  p->Kp = para->Kp;
  p->Ki = para->Ki;
  p->Kd = para->Kd;
  p->Rpa = para->Kp_max;
  p->Rpi = para->Kp_min;
  p->Ria = para->Ki_max;
  p->Rii = para->Ki_min;
  p->Rda = para->Kd_max ;
  p->Rdi = para->Kd_min;
  p->Ta = para->Target_max ;
  p->Ti = para->Target_min ;
  p->DeadZone = para->DeadZone;
}

/**
* @brief  加载PID参数(对偶PID)
*/
void Load_PIDParameter(DUAL_PID* p, _DUAL_PID_Parameter* para)
{
  p->Kpp = para->Kpp;
  p->Kpn = para->Kpn;
  p->Kip = para->Kip;
  p->Kin = para->Kin;
  p->Kdp = para->Kdp;
  p->Kdn = para->Kdn;
  p->Rpa = para->Kp_max;
  p->Rpi = para->Kp_min;
  p->Ria = para->Ki_max;
  p->Rii = para->Ki_min;
  p->Rda = para->Kd_max ;
  p->Rdi = para->Kd_min;
  p->Ta = para->Target_max ;
  p->Ti = para->Target_min ;
  p->DeadZone = para->DeadZone;
}

/**
* @brief  设置PID参数结构体（普通PID）
*/
void set_PIDParameter(_PID_Parameter* p, float Kp, float Ki, float Kd,
             float Kp_min, float Kp_max, float Ki_min, float Ki_max,
             float Kd_min, float Kd_max, float Target_min, float Target_max,float DeadZone)
{
  p->Kp = Kp;
  p->Ki = Ki;
  p->Kd = Kd;
  p->Kp_max = Kp_max;
  p->Kp_min = Kp_min;
  p->Ki_max = Ki_max;
  p->Ki_min = Ki_min;
  p->Kd_max = Kd_max ;
  p->Kd_min = Kd_min;
  p->Target_max = Target_max ;
  p->Target_min = Target_min ;
  p->DeadZone = DeadZone;
}

/**
* @brief  设置PID参数结构体（对偶PID）
*/
void set_PIDParameter(_DUAL_PID_Parameter* p, float Kpp, float Kip, float Kdp,
             float Kpn, float Kin, float Kdn, 
             float Kp_min, float Kp_max, float Ki_min, float Ki_max,
             float Kd_min, float Kd_max, float Target_min, float Target_max,float DeadZone)
{
  p->Kpp = Kpp;
  p->Kpn = Kpn;
  p->Kip = Kip;
  p->Kin = Kin;
  p->Kdp = Kdp;
  p->Kdn = Kdn;
  p->Kp_max = Kp_max;
  p->Kp_min = Kp_min;
  p->Ki_max = Ki_max;
  p->Ki_min = Ki_min;
  p->Kd_max = Kd_max ;
  p->Kd_min = Kd_min;
  p->Target_max = Target_max ;
  p->Target_min = Target_min ;
  p->DeadZone = DeadZone;
}


float TRI(float X,float DOWN,float MID,float UP)
{
  if( X >= DOWN && X <= UP)
    return ( 1 - myabs(X - MID)/( MID - DOWN) );
  else
    return 0;
}
float LAD_DOWN(float X,float MID,float UP)
{
  if(	X <= MID)
    return 1.0f;
  else if( X <= UP )
    return (X - UP) / (MID - UP);
  else
    return 0;
}

float LAD_UP(float X,float DOWN,float MID)
{
  if( X >= MID)
    return 1.0f;
  else if( X >= DOWN )
    return (X - DOWN) / (MID - DOWN);
  else
    return 0;
}

float N_DOWN(float X,float NM,float NS)
{
  if( X >=NS && X<= 0)
    return 1.0f;
  else if(X >=NM && X< NS)
    return (X-NM)/(NS-NM);
  else 
    return 0;
}


float N_TRI(float X,float NB,float NM,float NS)
{
  if( X >=NM && X<= NS)
  return myabs(NS - X)/( NS - NM);
  else if(X<NM && X>=NB)
      return ( X - NB )/( NM - NB );
  else 
  return 0;
}

float N_UP(float X, float NB,float NM)
{
  if( X <=NB)
    return 1.0f;
  else if(X <= NM)
    return (NM - X)/(NM - NB);
  else
    return 0;
}

float P_DOWN(float X,float PS,float PM)
{
  if( X >=0 && X <= PS)
    return 1.0f;
  else if(X > PS && X <= PM)
    return (PM - X)/(PM - PS);
  else
    return 0;
}


float P_TRI(float X,float PS,float PM,float PB)
{
  if(X >=PS && X < PM)
  return  (X - PS)/( PM - PS); 
  else if(X<=PB && X >= PM)
      return  (PB - X)/( PB - PM);
  else
  return 0 ;
}

float P_UP(float X, float PM,float PB)
{
  if(X >=PB)
    return 1.0f;
  else if(X >=PM)
    return (X - PM)/(PB-PM);
  else 
    return 0;
}

void PIDBase::SetPIDBase(float m_Kp,  float m_Ki,  float m_Kd,  float m_Target,
                 float m_Rpi, float m_Rpa, float m_Rii, float m_Ria, float m_Rdi, float m_Rda, 
                 float m_Ti,  float m_Ta,  float m_DeadZone)
{
  Kp       = m_Kp;
  Ki       = m_Ki;
  Kd       = m_Kd;
  Target   = m_Target;
  Rpi      = m_Rpi;
  Rii      = m_Rii;
  Rdi      = m_Rdi;
  Rpa      = m_Rpa;
  Ria      = m_Ria;
  Rda      = m_Rda;
  Ti       = m_Ti;
  Ta       = m_Ta;
  DeadZone = m_DeadZone; 
}

void PIDBase::SetTarget(float target)
{
  this->Target = Constrain(target,Ti,Ta);
}

void PIDBase::SetCurrent(float current)
{
  Current = current;
}

void PIDBase::SetPIDPara(float m_Kp, float m_Ki, float m_Kd)
{
  Kp = m_Kp;
  Ki = m_Ki;
  Kd = m_Kd;
}

unsigned char PIDTimer::UpdataTime(void)
{
  uint32_t now_time;
  //系统时间的获取
  if(last_time == 0)
  {
    last_time = micros();
    return 1;
  }
  now_time = micros();

  if(now_time < last_time)
  {
      dt = (float)(now_time + (0xFFFFFFFF - last_time) );
  }
  else
  {
    dt = (float)(now_time - last_time);
}

    last_time = now_time ;

  dt /= 1000000.0f;
  return 0;
}
void PID::SetPID(float m_Kp, float m_Ki, float m_Kd, float m_Target,
             float m_Rpi, float m_Rpa, float m_Rii, float m_Ria,
             float m_Rdi, float m_Rda, float m_Ti, float m_Ta,float m_DeadZone)	 
{
  Kp  = m_Kp; Ki = m_Ki; Kd = m_Kd; Target =m_Target;			 
  Rpi = m_Rpi; Rpa= m_Rpa; Rii= m_Rii; Ria =m_Ria;
  Rdi = m_Rdi; Rda= m_Rda; Ti = m_Ti; Ta =m_Ta;DeadZone=m_DeadZone;
}
  void PID::AdjustPID(void)
{
  if(UpdataTime()) return;

  last_error = now_error;
  now_error = Target - Current;
  now_error = Slider_Filter(filterstruct.filterbuff[0],&filterstruct.num[0],5,now_error);

  //P
  PTerm = Kp * now_error;
  PTerm = Constrain(PTerm, Rpi, Rpa);

  //I
  ITerm +=  Ki * now_error * dt;
  ITerm = Constrain(ITerm, Rii, Ria);

  //D
  DTerm =   Kd * (now_error - last_error)/dt;
  DTerm = Constrain(DTerm, Rdi, Rda);
  DTerm = Slider_Filter(filterstruct.filterbuff[1],&filterstruct.num[1],5,DTerm);//滤波，输出更加平滑

  Out = PTerm + ITerm + DTerm;
  Out = Slider_Filter(filterstruct.filterbuff[2],&filterstruct.num[2],5,Out);//滤波，输出更加平滑

  if(myabs(now_error) <= DeadZone)
  {
    counter++;
    if(counter >= 1024)
      counter = 1024;
  }
  else
  {
    counter--;
    if(counter <= 0)
      counter = 0;
  }
  if(counter >= 1000)
  {
    Out = 0;
    ITerm = 0;
  }
}

void FUZZYPID::Set_FUZZYPID(float m_Kp, float m_Ki, float m_Kd, float m_Target,\
                            float m_Rpi,float m_Rpa,float m_Rii,float m_Ria,\
                            float m_Rdi,float m_Rda,float m_Ti, float m_Ta, float m_DeadZone,\
                            float m_NB, float m_PB, float m_NM, float m_PM, float m_NS, float m_PS,\
                            float m_NB_kp,float m_PB_kp, float m_NM_kp,float m_PM_kp, float m_NS_kp,float m_PS_kp,\
                            float m_NB_ki,float m_PB_ki, float m_NM_ki,float m_PM_ki, float m_NS_ki,float m_PS_ki,\
                            float m_NB_kd,float m_PB_kd, float m_NM_kd,float m_PM_kd, float m_NS_kd,float m_PS_kd) 
    {
    Kp = m_Kp;Ki = m_Ki;Kd = m_Kd;Target = m_Target;
    Rpi = m_Rpi;Rii = m_Rii;Rdi = m_Rdi;
    Rpa = m_Rpa;Ria = m_Ria;Rda = m_Rda;
    Ti  = m_Ti; Ta  = m_Ta;
    DeadZone = m_DeadZone;  
    NB=m_NB; PB=m_PB; NM=m_NM; PM=m_PM; NS=m_NS; PS=m_PS;
    NB_kp=m_NB_kp; PB_kp=m_PB_kp; NM_kp=m_NM_kp; PM_kp=m_PM_kp; NS_kp=m_NS_kp; PS_kp=m_PS_kp;
    NB_ki=m_NB_ki; PB_ki=m_PB_ki; NM_ki=m_NM_ki; PM_ki=m_PM_ki; NS_ki=m_NS_ki; PS_ki=m_PS_ki;
    NB_kd=m_NB_kd; PB_kd=m_PB_kd; NM_kd=m_NM_kd; PM_kd=m_PM_kd; NS_kd=m_NS_kd; PS_kd=m_PS_kd;
    }

  void FUZZYPID::Defuzzification(float error,float errorC)
  {
  float Ux_NS,Ux_PS,Ux_NM,Ux_PM,Ux_NB,Ux_PB;
  float kp,ki,kd;

  Ux_NS  = N_DOWN(error,this->NM,this->NS);
  Ux_PS  = P_DOWN(error,this->PS,this->PM);
  Ux_NM  = N_TRI(error,this->NB,this->NM,this->NS);
  Ux_PM  = P_TRI(error,this->PS,this->PM,this->PB);
  Ux_NB  = N_UP(error,this->NB,this->NM);
  Ux_PB  = P_UP(error,this->PM,this->PB);

  this->a[0]=Ux_NB;
  this->a[1]=Ux_NM;
  this->a[2]=Ux_NS;
  this->a[3]=Ux_PS;
  this->a[4]=Ux_PM;
  this->a[5]=Ux_PB;

  kp = (Ux_NS * this->NS_kp + Ux_PS * this->PS_kp +Ux_NM * this->NM_kp + Ux_PM * this->PM_kp + Ux_NB * this->NB_kp + Ux_PB * this->PB_kp)/(Ux_NS + Ux_PS + Ux_NM + Ux_PM + Ux_NB + Ux_PB);
  ki = (Ux_NS * this->NS_ki + Ux_PS * this->PS_ki +Ux_NM * this->NM_ki + Ux_PM * this->PM_ki + Ux_NB * this->NB_ki + Ux_PB * this->PB_ki)/(Ux_NS + Ux_PS + Ux_NM + Ux_PM + Ux_NB + Ux_PB);
  kd = (Ux_NS * this->NS_kd + Ux_PS * this->PS_kd +Ux_NM * this->NM_kd + Ux_PM * this->PM_kd + Ux_NB * this->NB_kd + Ux_PB * this->PB_kd)/(Ux_NS + Ux_PS + Ux_NM + Ux_PM + Ux_NB + Ux_PB);

  SetPIDPara(kp,ki,kd);
}

void FUZZYPID::AdjustPID(void)
{
  if(UpdataTime()) return;

  now_error = Target - Current;
  now_error = Slider_Filter(filterstruct.filterbuff[0],&filterstruct.num[0],5,now_error);
  Omega = Slider_Filter(filterstruct.filterbuff[1],&filterstruct.num[1],5,Omega);

  errorC = Kec*Omega/dt;

  Defuzzification(now_error,errorC);

  //P
  PTerm = Kp * now_error;
  PTerm = Constrain(PTerm, Rpi, Rpa);

  //I

  ITerm +=  Ki * now_error * dt;
  ITerm = Constrain(ITerm, Rii, Ria);

  //D
  DTerm =   Kd * (Omega - Omega_Offset);

  Out = PTerm + ITerm + DTerm;
  Out = Slider_Filter(filterstruct.filterbuff[2],&filterstruct.num[2],5,Out);//滤波，输出更加平滑

  if(myabs(now_error) <= DeadZone)
  {
    counter++;
    if(counter >= 1024)
      counter = 1024;
  }
  else
  {
    counter--;
    if(counter <= 0)
      counter = 0;
  }
  if(counter >= 1000)
  {
    Out = 0;
    ITerm = 0;
  }
}

void FUZZYPID_without_gyroscope::AdjustPID(void)
{
  if(UpdataTime()) return;

  last_error = now_error;
  now_error = Target - Current;
  now_error = Slider_Filter(filterstruct.filterbuff[0],&filterstruct.num[0],5,now_error);

  errorC = Kec*Omega/dt;

  Defuzzification(now_error,errorC);

  //P
  PTerm = Kp * now_error;
  PTerm = Constrain(PTerm, Rpi, Rpa);

  //I

  ITerm +=  Ki * now_error * dt;
  ITerm = Constrain(ITerm, Rii, Ria);

  //D
  DTerm =   Kd * (now_error - last_error)/dt;
  DTerm = Constrain(DTerm, Rdi, Rda);
  DTerm = Slider_Filter(filterstruct.filterbuff[1],&filterstruct.num[1],5,DTerm);//滤波，输出更加平滑

  Out = PTerm + ITerm + DTerm;
  Out = Slider_Filter(filterstruct.filterbuff[2],&filterstruct.num[2],5,Out);//滤波，输出更加平滑

  if(myabs(now_error) <= DeadZone)
  {
    counter++;
    if(counter >= 1024)
      counter = 1024;
  }
  else
  {
    counter--;
    if(counter <= 0)
      counter = 0;
  }
  if(counter >= 1000)
  {
    Out = 0;
    ITerm = 0;
  }
}





/*对偶PID*/
void DUAL_PID::SetPID(float m_Kpp, float m_Kip, float m_Kdp,
             float m_Kpn, float m_Kin, float m_Kdn, float m_Target,
             float m_Rpi, float m_Rpa, float m_Rii, float m_Ria,
             float m_Rdi, float m_Rda, float m_Ti, float m_Ta,float m_DeadZone)
{
  Kpp  = m_Kpp; Kip = m_Kip; Kdp = m_Kdp; 
  Kpn  = m_Kpn; Kin = m_Kin; Kdn = m_Kdn; 
  Target =m_Target;
  Rpi = m_Rpi; Rpa= m_Rpa; Rii= m_Rii; Ria =m_Ria;
  Rdi = m_Rdi; Rda= m_Rda; Ti = m_Ti; Ta =m_Ta;DeadZone=m_DeadZone;
}



  void DUAL_PID::AdjustPID(void)
{
  if(UpdataTime()) return;

  //lastlast_error = last_error;
  last_error = now_error;
  now_error = Target - Current;
  now_error = Slider_Filter(filterstruct.filterbuff[0],&filterstruct.num[0],5,now_error);
  if(now_error>DeadZone)
  {
    Kp=Kpp;Ki=Kip;Kd=Kdp;
    
  }
  else if(now_error<-DeadZone)
  {
    Kp=Kpn;Ki=Kin;Kd=Kdn;
  }
  else
  {
    Kp=Kpn+(Kpp-Kpn)*(now_error+DeadZone)/2/DeadZone;
  }
  //P
  PTerm = Kp * now_error;
  //PTerm = Kp * (now_error - last_error);
  PTerm = Constrain(PTerm, Rpi, Rpa);

  //I
  ITerm +=  Ki * now_error * dt;
  ITerm = Constrain(ITerm, Rii, Ria);

  //D
  DTerm =   Kd * (now_error - last_error)/dt;
  //DTerm =   Kd * (now_error - 2*last_error + lastlast_error)/dt;
  DTerm = Constrain(DTerm, Rdi, Rda);
  DTerm = Slider_Filter(filterstruct.filterbuff[1],&filterstruct.num[1],5,DTerm);//滤波，输出更加平滑

  Out = PTerm + ITerm + DTerm;
  Out = Slider_Filter(filterstruct.filterbuff[2],&filterstruct.num[2],5,Out);//滤波，输出更加平滑

//  if(myabs(now_error) <= DeadZone)
//  {
//    counter++;
//    if(counter >= 1024)
//      counter = 1024;
//  }
//  else
//  {
//    counter--;
//    if(counter <= 0)
//      counter = 0;
//  }
//  if(counter >= 1000)
//  {
//    Out = 0;
//    ITerm = 0;
//  }
}


