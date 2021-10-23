#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "reedsolomon.h"
#include "rs41_packet_structure.h"

uint8_t *data;

static uint8_t rs41_null = 0;

static uint8_t *get_address_short_even (int index)
{
  if (index < 132) {
    return &data[48 + 2 * index];
  } else {
    if (index >= 231) {
      return &data[index - 231];
    } else {
      return &rs41_null;
    }
  }
}

static uint8_t *get_address_short_odd (int index)
{
  if (index < 132) {
    return &data[49 + 2 * index];
  } else {
    if (index >= 231) {
      return &data[index - 207];
    } else {
      return &rs41_null;
    }
  }
}

static uint8_t *get_address_long_even (int index)
{
  if (index < 231) {
    return &data[48 + 2 * index];
  } else {
   return &data[index - 231];
  }
}

static uint8_t *get_address_long_odd (int index)
{
  if (index < 231) {
    return &data[49 + 2 * index];
  } else {
    return &data[index - 207];
  }
}

int32_t rs41_process_block (uint8_t *buffer, int length)
{
  int errors;
  int corrected_errors;

  // the header is not part of the error correction data
  data = buffer + 8;

  switch(length) {
    case RS41_PACKET_LENGTH_NORMAL:
      if (REEDSOLOMON_process(get_address_short_even, &errors) != LPCLIB_SUCCESS) {
        return LPCLIB_ERROR;
      } else {
        corrected_errors = errors;
        if (REEDSOLOMON_process(get_address_short_odd, &errors) != LPCLIB_SUCCESS) {
          return LPCLIB_ERROR;
        }
        corrected_errors += errors;
      }
      break;
    case RS41_PACKET_LENGTH_EXTENDED:
      if (REEDSOLOMON_process(get_address_long_even, &errors) != LPCLIB_SUCCESS) {
        return LPCLIB_ERROR;
      } else {
        corrected_errors = errors;
        if (REEDSOLOMON_process(get_address_long_odd, &errors) != LPCLIB_SUCCESS) {
          return LPCLIB_ERROR;
        }
        corrected_errors += errors;
      }
      break;
    default:
      return LPCLIB_ERROR;
  }

  printf("Number of errors: %d\n", corrected_errors);

  return 0;
}
