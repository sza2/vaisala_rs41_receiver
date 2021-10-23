/*
 * extract_block.c
 *
 *  Created on: Aug 10, 2021
 *      Author: sza2
 */

#include "string.h"
#include "extract_block.h"

void extract_block_status(uint8_t *block_data, RS41_Status_t *data)
{
  data->frame_number = (*((uint16_t *)(block_data + RS41_BLOCK_STATUS_OFFSET + RS41_OFFSET_STATUS_FRAME_NUMBER)));
  memcpy(data->serial_number, block_data + RS41_BLOCK_STATUS_OFFSET + RS41_OFFSET_STATUS_SERIAL_NUMBER, 8);
  data->serial_number[8] = 0;
  data->battery_voltage = (float)(*((uint8_t *)(block_data + RS41_BLOCK_STATUS_OFFSET + RS41_OFFSET_STATUS_BATTERY_VOLTAGE))) / 10;
  data->bitfield_status = (*((uint16_t *)(block_data + RS41_BLOCK_STATUS_OFFSET + RS41_OFFSET_STATUS_BITFIELD_STATUS)));
  data->pcb_temperature = (*((uint8_t *)(block_data + RS41_BLOCK_STATUS_OFFSET + RS41_OFFSET_STATUS_PCB_TEMPERATURE)));
  data->humidity_sensor_heating_pwm = (*((uint16_t *)(block_data + RS41_BLOCK_STATUS_OFFSET + RS41_OFFSET_STATUS_HUMIDITY_SENSOR_HEATING_PWM)));
  data->transimt_power = (*((uint8_t *)(block_data + RS41_BLOCK_STATUS_OFFSET + RS41_OFFSET_STATUS_TRANSMIT_POWER)));
  data->max_subframe_number = (*((uint8_t *)(block_data + RS41_BLOCK_STATUS_OFFSET + RS41_OFFSET_STATUS_MAX_SUBFRAME_NUMBER)));
  data->subframe_number = (*((uint8_t *)(block_data + RS41_BLOCK_STATUS_OFFSET + RS41_OFFSET_STATUS_SUBFRAME_NUMBER)));
  memcpy(data->subframe_payload, block_data + RS41_BLOCK_STATUS_OFFSET + RS41_OFFSET_STATUS_SUBFRAME_PAYLOAD, 16);
  return;
}

void extract_block_gpspos(uint8_t *block_data, RS41_Gps_Position_t *data)
{
  data->ecef_position.x = (float)(*((uint32_t *)(block_data + RS41_BLOCK_GPSPOS_OFFSET + RS41_OFFSET_GPSPOS_ECEF_POSITION_X))) / 100.0;
  data->ecef_position.y = (float)(*((uint32_t *)(block_data + RS41_BLOCK_GPSPOS_OFFSET + RS41_OFFSET_GPSPOS_ECEF_POSITION_Y))) / 100.0;
  data->ecef_position.z = (float)(*((uint32_t *)(block_data + RS41_BLOCK_GPSPOS_OFFSET + RS41_OFFSET_GPSPOS_ECEF_POSITION_Z))) / 100.0;
  data->ecef_velocity.x = (float)(*((uint16_t *)(block_data + RS41_BLOCK_GPSPOS_OFFSET + RS41_OFFSET_GPSPOS_ECEF_VELOCITY_X))) / 100.0;
  data->ecef_velocity.y = (float)(*((uint16_t *)(block_data + RS41_BLOCK_GPSPOS_OFFSET + RS41_OFFSET_GPSPOS_ECEF_VELOCITY_Y))) / 100.0;
  data->ecef_velocity.z = (float)(*((uint16_t *)(block_data + RS41_BLOCK_GPSPOS_OFFSET + RS41_OFFSET_GPSPOS_ECEF_VELOCITY_Z))) / 100.0;
  data->satellite_vehicle_number = (*((uint8_t *)(block_data + RS41_BLOCK_GPSPOS_OFFSET + RS41_OFFSET_GPSPOS_SATELLITE_VEHICLE_NUMBER)));
  data->speed_accuraccy = (float)(*((uint8_t *)(block_data + RS41_BLOCK_GPSPOS_OFFSET + RS41_OFFSET_GPSPOS_SPEED_ACCURACY))) * 10.0;
  data->pdop = (float)(*((uint8_t *)(block_data + RS41_BLOCK_GPSPOS_OFFSET + RS41_OFFSET_GPSPOS_PDOP))) / 10.0;
  return;
}
