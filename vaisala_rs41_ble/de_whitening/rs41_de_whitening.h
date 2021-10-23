/*
 * de_whitening_rs41.h
 *
 *  Created on: Jul 19, 2021
 *      Author: sza2
 */

#ifndef DE_WHITENING_RS41_H_
#define DE_WHITENING_RS41_H_

#include "stdint.h"

void de_whiten(uint8_t *de_whitened_data, uint8_t *whitened_data, uint16_t length);

#endif /* DE_WHITENING_RS41_H_ */
