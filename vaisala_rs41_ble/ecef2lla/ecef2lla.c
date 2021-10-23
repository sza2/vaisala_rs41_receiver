/*
 * ecef2lla.c
 *
 *  Created on: Aug 10, 2021
 *      Author: sza2
 */

#include "math.h"
#include "stdint.h"
#include "stdbool.h"
#include "ecef2lla.h"

#define M_PI    3.14159265358979323846

static float a;
static float a_square;
static float b;
static float b_square;
static float ea_square;
static float eb_square;

static bool init = false;

static void ecef2lla_init(void)
{
  a = EARTH_A;
  a_square = pow(a, 2);
  b = EARTH_B;
  b_square = pow(b, 2);
  ea_square = EARTH_A2_B2 / a_square;
  eb_square = EARTH_A2_B2 / b_square;

  init = true;
}

LLA_t ecef2lla(RS41_Ecef_Position_t ecef_position)
{
  if (!init) {
    ecef2lla_init();
  }

  LLA_t lla;
  float phi, p, theta;

  p = sqrt(pow(ecef_position.x, 2) + pow(ecef_position.y, 2));
  theta = atan2(ecef_position.z * a, p * b);
  phi = atan2(ecef_position.z + eb_square * b * pow(sin(theta), 3), p - ea_square * a * pow(cos(theta), 3));
  lla.altitude = p / cos(phi) - a / sqrt(1 - ea_square * pow(sin(phi), 2));
  lla.latitude = phi * 180 / M_PI;
  lla.longitude = atan2(ecef_position.y, ecef_position.x) * 180 / M_PI;

  return lla;
}
