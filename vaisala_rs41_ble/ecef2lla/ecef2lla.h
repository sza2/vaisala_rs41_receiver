/*
 * ecef2lla.h
 *
 *  Created on: Aug 10, 2021
 *      Author: sza2
 */

#ifndef ECEF2LLA_H_
#define ECEF2LLA_H_

#include "rs41_packet_structure.h"

#define EARTH_A  6378137.0
#define EARTH_B  6356752.31424518
#define EARTH_A2_B2  (EARTH_A * EARTH_A - EARTH_B * EARTH_B)

typedef struct {
  float latitude;
  float longitude;
  float altitude;
} LLA_t;

LLA_t ecef2lla(RS41_Ecef_Position_t ecef_position);

#endif /* ECEF2LLA_H_ */
