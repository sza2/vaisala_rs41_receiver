/*
 * validate_block.h
 *
 *  Created on: Aug 9, 2021
 *      Author: sza2
 */

#ifndef VALIDATE_BLOCK_H_
#define VALIDATE_BLOCK_H_

#include "stdint.h"
#include "stdbool.h"

#include "rs41_packet_structure.h"

typedef uint8_t Block_Validation_Result_t;

enum Block_Validation_Result_t {
  BLOCK_OK,
  BLOCK_LENGTH_ERROR, // last byte is over RS41 packet length
  BLOCK_MISMATCH_TYPE,
  BLOCK_MISMATCH_SIZE,
  BLOCK_CRC_ERROR,
};

typedef uint8_t Block_Type_t;

enum Block_Type_t {
  RS41_BLOCK_STATUS,
  RS41_BLOCK_MEAS,
  RS41_BLOCK_GPSINFO,
  RS41_BLOCK_GPSRAW,
  RS41_BLOCK_GPSPOS,
  RS41_BLOCK_EMPTY,
};

typedef struct {
  uint8_t id;
  uint16_t offset;
  uint8_t length;
} Block_Parameters_t;


Block_Validation_Result_t validate_block(Block_Type_t type, uint8_t *data);

#endif /* VALIDATE_BLOCK_H_ */
