/*
 * validate_block.c
 *
 *  Created on: Aug 9, 2021
 *      Author: sza2
 */

#include "validate_block.h"
#include "em_gpcrc.h"
#include "stdio.h"

Block_Parameters_t block_types[] = {
  {
    RS41_BLOCK_STATUS_ID,
    RS41_BLOCK_STATUS_OFFSET,
    RS41_BLOCK_STATUS_LENGTH,
  },
  {
    RS41_BLOCK_MEAS_ID,
    RS41_BLOCK_MEAS_OFFSET,
    RS41_BLOCK_MEAS_LENGTH,
  },
  {
    RS41_BLOCK_GPSINFO_ID,
    RS41_BLOCK_GPSINFO_OFFSET,
    RS41_BLOCK_GPSINFO_LENGTH,
  },
  {
    RS41_BLOCK_GPSRAW_ID,
    RS41_BLOCK_GPSRAW_OFFSET,
    RS41_BLOCK_GPSRAW_LENGTH,
  },
  {
    RS41_BLOCK_GPSPOS_ID,
    RS41_BLOCK_GPSPOS_OFFSET,
    RS41_BLOCK_GPSPOS_LENGTH,
  },
  {
    RS41_BLOCK_EMPTY_ID,
    RS41_BLOCK_EMPTY_OFFSET,
    RS41_BLOCK_EMPTY_LENGTH,
  },
};

Block_Validation_Result_t validate_block(Block_Type_t type, uint8_t *data)
{
  Block_Parameters_t block_type = block_types[type];
  // offset + ID field (1 byte) + length field (1 byte) + length + CRC (2 bytes)
  // must not be greater than the available buffer size
  if (block_type.offset + 1 + 1 + block_type.length + 2 > RS41_PACKET_LENGTH_MAX) {
    return BLOCK_LENGTH_ERROR;
  }
  if (block_type.id != data[block_type.offset]) {
    return BLOCK_MISMATCH_TYPE;
  }
  if (block_type.length != data[block_type.offset +1]) {
    return BLOCK_MISMATCH_SIZE;
  }
  uint16_t calculated_crc;

  GPCRC_Init_TypeDef init = GPCRC_INIT_DEFAULT;
  init.crcPoly = 0x1021;
  init.initValue = 0xffff;
  init.reverseBits = true;
  GPCRC_Init(GPCRC, &init);

  GPCRC_Start(GPCRC);
  for (uint8_t cnt = 0; cnt < block_type.length; cnt++) {
    GPCRC_InputU8(GPCRC, data[block_type.offset + cnt + 2]);
  }
  calculated_crc = (uint16_t) GPCRC_DataReadBitReversed(GPCRC);

  uint16_t received_crc =
    *((uint16_t *)(data + block_type.offset + 2 + block_type.length));

  if (calculated_crc != received_crc) {
    return BLOCK_CRC_ERROR;
  }
  return BLOCK_OK;
}
