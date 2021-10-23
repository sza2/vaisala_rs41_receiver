/*
 * app_common.h
 *
 *  Created on: Aug 3, 2021
 *      Author: sza2
 */

#ifndef APP_COMMON_H_
#define APP_COMMON_H_

#include "os.h"

// OS event flag
#define APP_PROPRIETARY_EVENT_FLAG              ((OS_FLAGS)0x01)
#define APP_BLUETOOTH_CHANNEL_NUMBER_EVENT_FLAG ((OS_FLAGS)0x02)
#define APP_BUTTON0_EVENT_FLAG                  ((OS_FLAGS)0x04)
#define APP_BUTTON1_EVENT_FLAG                  ((OS_FLAGS)0x08)

#define CHARACTERISTIC_CHANGED ((OS_FLAGS)0x01)

extern OS_FLAG_GRP app_event_flags;

#endif /* APP_COMMON_H_ */
