#ifndef __SRML_CONFIG_H__
#define __SRML_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif
/* Drivers ----------------------------------------------------*/
#define USE_SRML_REFEREE                  0
#define USE_SRML_MOTOR                    1
#define USE_SRML_DR16                     1
#define USE_SRML_BMX055                   0
#define USE_SRML_MPU6050                  1
#define USE_SRML_W25Qx                    1 

#define USE_SRML_I2C                      1
#define USE_SRML_SPI                      1
#define USE_SRML_CAN                      1
#define USE_SRML_UART                     1
#define USE_SRML_TIMER                    1
#define USE_SRML_FLASH                    1
/* Middlewares -----------------------------------------------*/
#define USE_SRML_PID                      1
#define USE_SRML_FILTER                   1
#define USE_SRML_MYMAT                    0
#define USE_SRML_KALMAN                   0
#define USE_SRML_TRACK_GENERATOR          0

#define USE_SRML_SERIAL_LINE_IP           1
  
#define USE_SRML_MYASSERT                 0
#define USE_SRML_LIST                     1
#define USE_SRML_SYSLOG                   1
#define USE_SRML_SYSANALYSIS              1

#define USE_SRML_GIMBAL                   0
#define USE_SRML_CHASSIS                  1
#define USE_SRML_POW_CTRL                 1


#ifdef __cplusplus
}
#endif

#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
