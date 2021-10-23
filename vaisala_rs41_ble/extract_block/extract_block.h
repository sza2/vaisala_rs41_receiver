/*
 * extract_block.h
 *
 *  Created on: Aug 10, 2021
 *      Author: sza2
 */

#ifndef EXTRACT_BLOCK_H_
#define EXTRACT_BLOCK_H_

#include "stdint.h"
#include "rs41_packet_structure.h"

void extract_block_status(uint8_t *block_data, RS41_Status_t *data);
void extract_block_gpspos(uint8_t *block_data, RS41_Gps_Position_t *data);

#endif /* EXTRACT_BLOCK_H_ */
