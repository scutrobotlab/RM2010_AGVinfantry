#ifndef PITCH_YAW_H
#define PITCH_YAW_H

#include "FreeRTOS.h"
#include "PID.h"
#include <main.h>
#include "motor.h"
#include "mpu6050_config.h"

#define on 1
#define off 0

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern MPUData_Typedef MPUData;
/*pitchyaw_test below*/
extern Motor_GM6020 GrimalMotor[2];

extern myPID Yaw[2];//测试用，结束后更改为auto_gimbal类
extern myPID Pitch[2];//测试用，结束后更改为auto_gimbal类

extern float calcangle_yaw(float now_imu_data , int8_t status);//测试用，结束后更改为auto_gimbal类
extern float calcangle_pitch(float now_imu_data , int8_t status);//测试用，结束后更改为auto_gimbal类

/*pitchyaw_test above*/

extern Motor_C610 Turnplate[1];//拨盘电机
extern Motor_C620 FriMotor[4];



class gimbal//步兵云台，待测试结束后使用
{
	public:
		gimbal();//初始化
		void calculatePitchIMU(float, int8_t, uint8_t);//pitch轴360°制角度带圈数计算
		void calculateYawIMU(float, int8_t, uint8_t);//yaw轴360°制角度带圈数计算
		float getPitchIMU();//获取360°制pitch轴带圈数数据
		float getYawIMU();//获取360°制yaw轴带圈数数据
		float getPitchScale();//获取pitch轴遥控倍数
		float getYawScale();//获取yaw轴遥控倍数
		myPID pitchAngle,pitchSpeed,yawAngle,yawSpeed;//SRML库pid
		void targetGet(uint8_t);//目标值获取
		void pitchContralCalculate(float*, short*);//pitch轴pid计算
		void yawContralCalculate(float*, short*);//yaw轴pid计算
		void gimbalControl(Motor_GM6020*, MPUData_Typedef*, uint8_t);//云台控制
		void sendMassageClear();//can发送数据清零
		void sendMassageWrite(int16_t, int16_t, int16_t, int16_t);//can发送数据填充
	protected:
		float pitchMPU_total_angle;//360°制pitch轴带圈数数据
		float yawMPU_total_angle;//360°制yaw轴带圈数数据
		float pitchSpeedOffset;
		float yawSpeedOffset;
		float pitch_scale;//pitch轴遥控速率倍数
		float yaw_scale;//yaw轴遥控速率倍数
		uint8_t gimbalctrl[8];//can发送数据存储
		/*待添加私有成员*/
};

class auto_gimbal:public gimbal//自动步兵云台，增加一个yaw轴
{
	public:
		auto_gimbal();
		void calculateYawBelowIMU(float, int8_t, uint8_t);//yaw轴360°制角度带圈数计算
		float getBelowYawIMU();//获取360°制yaw轴带圈数数据
		float getBelowYawScale();//获取yaw轴遥控倍数
		myPID yawBelowAngle,yawBelowSpeed;
		void targetBelowGet(uint8_t);//目标值获取
		void yawBelowControlCalculate();//下yaw轴pid计算
		void autoGimbalControl(Motor_GM6020*, uint8_t);//云台控制
	protected:
		float yawMPU_below_total_angle;//360°制下yaw轴带圈数数据
		float yaw_below_scale;//下yaw轴遥控速率倍数
};

#endif
