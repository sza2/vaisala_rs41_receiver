/*
 * rs41_packet_structure.h
 *
 *  Created on: Jul 19, 2021
 *      Author: sza2
 */

/// The first half of RS41 header is used as sync word
/// EFR32 supports max 32 bit sync word, the remaining 32 bits treated as
/// payload
#define SYNC_WORD_LENGTH (4u)


#ifndef RS41_PACKET_STRUCTURE_H_
#define RS41_PACKET_STRUCTURE_H_

#define RS41_PACKET_LENGTH_MAX      518
#define RS41_PACKET_LENGTH_NORMAL   320
#define RS41_PACKET_LENGTH_EXTENDED RS41_PACKET_LENGTH_MAX
#define RS41_HEADER_OFFSET          0x0
#define RS41_HEADER_LENGTH          8
#define RS41_ECC_OFFSET             0x8
#define RS41_ECC_LENGTH             48
#define RS41_FRAME_TYPE_OFFSET      0x38

/// block length is the payload length, not including the ID, length byte and
/// the CRC
#define RS41_BLOCK_STATUS_ID        0x79
#define RS41_BLOCK_STATUS_OFFSET    0x39
#define RS41_BLOCK_STATUS_LENGTH    40

#define RS41_BLOCK_MEAS_ID          0x7a
#define RS41_BLOCK_MEAS_OFFSET      0x65
#define RS41_BLOCK_MEAS_LENGTH      42

#define RS41_BLOCK_GPSINFO_ID       0x7c
#define RS41_BLOCK_GPSINFO_OFFSET   0x65
#define RS41_BLOCK_GPSINFO_LENGTH   42

#define RS41_BLOCK_GPSRAW_ID        0x7d
#define RS41_BLOCK_GPSRAW_OFFSET    0x85
#define RS41_BLOCK_GPSRAW_LENGTH    89

#define RS41_BLOCK_GPSPOS_ID        0x7b
#define RS41_BLOCK_GPSPOS_OFFSET    0x112
#define RS41_BLOCK_GPSPOS_LENGTH    21

#define RS41_BLOCK_EMPTY_ID         0x76
#define RS41_BLOCK_EMPTY_OFFSET     0x12b
#define RS41_BLOCK_EMPTY_LENGTH     17


#define RS41_OFFSET_STATUS_FRAME_NUMBER                 0x02
#define RS41_OFFSET_STATUS_SERIAL_NUMBER                0x04
#define RS41_OFFSET_STATUS_BATTERY_VOLTAGE              0x0c
#define RS41_OFFSET_STATUS_BITFIELD_UNKNOWN0            0x0d
#define RS41_OFFSET_STATUS_BITFIELD_STATUS              0x0f
#define RS41_OFFSET_STATUS_UNKNOWN0                     0x11
#define RS41_OFFSET_STATUS_PCB_TEMPERATURE              0x12
#define RS41_OFFSET_STATUS_BITFIELD_ERROR               0x13
#define RS41_OFFSET_STATUS_HUMIDITY_SENSOR_HEATING_PWM  0x15
#define RS41_OFFSET_STATUS_TRANSMIT_POWER               0x17
#define RS41_OFFSET_STATUS_MAX_SUBFRAME_NUMBER          0x18
#define RS41_OFFSET_STATUS_SUBFRAME_NUMBER              0x19
#define RS41_OFFSET_STATUS_SUBFRAME_PAYLOAD             0x1a

typedef struct {
  uint16_t frame_number;
  uint8_t serial_number[9];               // 8 bytes +1 for string termination
  float battery_voltage;                  //battery voltage * 10
  uint16_t bitfield_unknown0;             // currently unknown
  uint16_t bitfield_status;               // bit 0:
                                          //  0: start phase
                                          //  1: flight mode
                                          // bit 1:
                                          //  0: ascent
                                          //  1: descent
                                          // bit 12:
                                          //  0: VBAT ok
                                          //  1: VBAT low
  uint8_t unknown0;                       // currently unknown
  uint8_t pcb_temperature;
  uint16_t bitfield_error;                // currently unknown
  uint16_t humidity_sensor_heating_pwm;
  uint8_t transimt_power;                 // 0: min, 7: max
  uint8_t max_subframe_number;
  uint8_t subframe_number;
  uint8_t subframe_payload[16];
} RS41_Status_t;

#define RS41_OFFSET_GPSPOS_ECEF_POSITION_X          0x02
#define RS41_OFFSET_GPSPOS_ECEF_POSITION_Y          0x06
#define RS41_OFFSET_GPSPOS_ECEF_POSITION_Z          0x0a
#define RS41_OFFSET_GPSPOS_ECEF_VELOCITY_X          0x0e
#define RS41_OFFSET_GPSPOS_ECEF_VELOCITY_Y          0x10
#define RS41_OFFSET_GPSPOS_ECEF_VELOCITY_Z          0x12
#define RS41_OFFSET_GPSPOS_SATELLITE_VEHICLE_NUMBER 0x14
#define RS41_OFFSET_GPSPOS_SPEED_ACCURACY           0x15
#define RS41_OFFSET_GPSPOS_PDOP                     0x16

typedef struct {
  float x;
  float y;
  float z;
} RS41_Ecef_Position_t;

typedef struct {
  float x;
  float y;
  float z;
} RS41_Ecef_Velocity_t;

typedef struct {
  RS41_Ecef_Position_t ecef_position;
  RS41_Ecef_Velocity_t ecef_velocity;
  uint8_t satellite_vehicle_number;
  float speed_accuraccy;
  float pdop;
} RS41_Gps_Position_t;

#endif /* RS41_PACKET_STRUCTURE_H_ */
