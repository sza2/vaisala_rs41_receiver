/*
 * de_whitening.c
 *
 *  Created on: Jul 19, 2021
 *      Author: sza2
 */

#include <rs41_de_whitening.h>

#define DE_WHITENING_MASK_LENGTH 64
uint8_t de_whitening_mask[DE_WHITENING_MASK_LENGTH] = {
  0x96, 0x83, 0x3E, 0x51, 0xB1, 0x49, 0x08, 0x98,
  0x32, 0x05, 0x59, 0x0E, 0xF9, 0x44, 0xC6, 0x26,
  0x21, 0x60, 0xC2, 0xEA, 0x79, 0x5D, 0x6D, 0xA1,
  0x54, 0x69, 0x47, 0x0C, 0xDC, 0xE8, 0x5C, 0xF1,
  0xF7, 0x76, 0x82, 0x7F, 0x07, 0x99, 0xA2, 0x2C,
  0x93, 0x7C, 0x30, 0x63, 0xF5, 0x10, 0x2E, 0x61,
  0xD0, 0xBC, 0xB4, 0xB6, 0x06, 0xAA, 0xF4, 0x23,
  0x78, 0x6E, 0x3B, 0xAE, 0xBF, 0x7B, 0x4C, 0xC1
};

void de_whiten(uint8_t *de_whitened_data, uint8_t *whitened_data, uint16_t length)
{
  for (uint16_t cnt = 0; cnt < length; cnt++) {
    de_whitened_data[cnt] = whitened_data[cnt] ^ de_whitening_mask[cnt % DE_WHITENING_MASK_LENGTH];
  }
}
