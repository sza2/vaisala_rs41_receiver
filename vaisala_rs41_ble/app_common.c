/*
 * app_common.c
 *
 *  Created on: Aug 14, 2021
 *      Author: sza2
 */

#include "app_common.h"

// OS event to prevent cyclic execution of the task proprietary main loop.
OS_FLAG_GRP app_event_flags;
