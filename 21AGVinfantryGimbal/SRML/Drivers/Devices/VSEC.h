#ifndef _VSEC_H
#define _VSEC_H

#include "stdint.h"
#include "string.h"
#include "drv_can.h"

//CAN 发送的标志位   还有其他接口但是没有给出  所以请勿发送超出下面的标志位
//如果超出标志位 驱动器也会执行 但是会产生意外错误
typedef enum {
	CAN_PACKET_SET_DUTY = 0,
	CAN_PACKET_SET_CURRENT,
	CAN_PACKET_SET_CURRENT_BRAKE,
	CAN_PACKET_SET_RPM,
	CAN_PACKET_SET_POS,
	CAN_PACKET_SET_PLANNING_POS,
	CAN_PACKET_SET_RESETPOS,
	CAN_PACKET_SET_CURRENT_REL,
	CAN_PACKET_SET_CURRENT_BRAKE_REL,
	CAN_PACKET_SET_CURRENT_HANDBRAKE,
	CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,
	CAN_PACKET_STATUS,
} CAN_PACKET_ID;

void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index);

void buffer_append_float32(uint8_t* buffer, float number, float scale, int32_t *index);

/* 设置电流值 单位A */
void VSEC_Set_Current(CAN_HandleTypeDef *hcan, uint8_t controller_id, float current);

/* 电调回传数据解包 */
void VSEC_UnPack(CAN_RxBuffer *can_msg, float *temp_fet, float *temp_motor, float *current, float *pos);

#endif
