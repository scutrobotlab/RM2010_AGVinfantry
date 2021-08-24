#include "VSEC.h"

void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index) {
	int16_t res =	((uint16_t) buffer[*index]) << 8 |
					((uint16_t) buffer[*index + 1]);
	*index += 2;
	return res;
}

void buffer_append_float32(uint8_t* buffer, float number, float scale, int32_t *index) {
    buffer_append_int32(buffer, (int32_t)(number * scale), index);
}

/* 设置电流值 单位A */
void VSEC_Set_Current(CAN_HandleTypeDef *hcan, uint8_t controller_id, float current) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_float32(buffer, current, 1e3, &send_index);
	CANx_SendExtData(hcan,(controller_id|((uint32_t)CAN_PACKET_SET_CURRENT<<8)),buffer,8);
}

/* 电调回传数据解包 */
void VSEC_UnPack(CAN_RxBuffer *can_msg, float *temp_fet, float *temp_motor, float *current, float *pos) {
	int32_t id = 0;
	temp_fet = (float)buffer_get_int16(can_msg->data, &id) / 10.0;
	temp_motor = (float)buffer_get_int16(can_msg->data, &id) / 10.0;
	current = (float)buffer_get_int16(can_msg->data, &id) / 10.0;
	pos = (float)buffer_get_int16(can_msg->data, &id) / 50.0;
}
