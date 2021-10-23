/*
 * ble_data.h
 *
 *  Created on: Aug 11, 2021
 *      Author: sza2
 */

#ifndef BLE_DATA_H_
#define BLE_DATA_H_

/// storage for the info sent over BLE
typedef struct __attribute__((packed)) {
  float latitude;
  float longitude;
  float altitude;
  int8_t rssi;
  uint16_t frame_number;
  uint8_t serial_number[9];
  uint8_t battery_voltage;
  uint16_t bitfied_status;
  uint8_t satellite_vehicle_number;
  uint8_t pdop;
} BLE_Data_t;

#endif /* BLE_DATA_H_ */
