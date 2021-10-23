/***************************************************************************//**
 * @file
 * @brief Core proprietary application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2021 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/
#include "stdio.h"
#include "os.h"
#include "rail.h"
#include "sl_rail_util_init.h"
#include "em_common.h"
#include "app_assert.h"
#include "app_proprietary.h"
#include "gatt_db.h"
#include "sl_bt_api.h"
#include "sl_simple_led_instances.h"
#include "sl_simple_button_instances.h"
#include "em_cmu.h"
#include "nvm3.h"

#include "app_common.h"
#include "rs41_packet_structure.h"
#include "rs41_de_whitening.h"
#include "validate_block.h"
#include "extract_block.h"
#include "ecef2lla.h"
#include "ble_data.h"
#include "draw_qrcode.h"

#if defined(SL_CATALOG_GLIB_PRESENT)
#include "glib.h"
GLIB_Context_t glibContext;
#include <glib_font_modified_8x8.h>
#endif

#if defined(SL_CATALOG_GLIB_PRESENT)
#endif

#define TMP_STR_LEN 32

/// Contains the last RAIL Rx/Tx events
static volatile uint64_t radio_events = 0;

/// Contains the status of RAIL Calibration
static volatile RAIL_Status_t calibration_status = 0;

static volatile bool calibration_error = false;
static volatile bool packet_recieved = false;
static volatile bool rx_error = false;

// if the scheduled RX finished w/o receiving a packet
static volatile bool packet_not_received = false;

/// set RX fifo size >512 as the extended frame can be 518 bytes
#define RX_FIFO_SIZE (1024u)
uint8_t rx_fifo[RX_FIFO_SIZE];

#define BUTTON_CH_STEP 10
uint32_t proprietary_channel = 320;
extern const RAIL_ChannelConfigEntry_t rs41_channels[];

/// Receive and Send FIFO
/// actually, RX data is set to rx_data + SYNC_WORD_SIZE, as the first half of
/// the header is used as sync word but the first half of the header is placed
/// to the rx_data array to ease further processing
uint8_t rx_data[RS41_PACKET_LENGTH_MAX] = {0x86, 0x35, 0xf4, 0x40};
uint8_t rs41_data[RS41_PACKET_LENGTH_MAX];

BLE_Data_t ble_data;

// OS task parameters
#define APP_PROPRIETARY_TASK_PRIO         6u
#define APP_PROPRIETARY_TASK_STACK_SIZE   (2048 / sizeof(CPU_STK))

// OS task variables
static CPU_STK app_proprietary_task_stack[APP_PROPRIETARY_TASK_STACK_SIZE];
static OS_TCB app_proprietary_task_tcb;

bool enable_lcd = true;

void display_info(char *str, int x, int y, bool lcd)
{
  printf("%s\n", str);
  if (lcd) {
#if defined(SL_CATALOG_GLIB_PRESENT)
  GLIB_Rectangle_t rect = {
    .xMin = x,
    .yMin = y,
    .xMax = x + 128,
    .yMax = y + 8,
  };
  glibContext.foregroundColor = White;
  GLIB_drawRectFilled(&glibContext, &rect);
  glibContext.foregroundColor = Black;
  GLIB_drawString(&glibContext, str, strlen(str), x, y, false);
#endif
  }
}

void display_mark(bool status, int x, int y, bool lcd)
{
  if (lcd) {
#if defined(SL_CATALOG_GLIB_PRESENT)
  GLIB_Rectangle_t rect = {
    .xMin = x,
    .yMin = y,
    .xMax = x + 8,
    .yMax = y + 8,
  };
  glibContext.foregroundColor = White;
  GLIB_drawRectFilled(&glibContext, &rect);
  glibContext.foregroundColor = Black;
  GLIB_drawString(&glibContext, status ? "}" : "{", 1, x, y, false);
#endif
  }
}

/**************************************************************************//**
 * Proprietary application task.
 *
 * @param[in] p_arg Unused parameter required by the OS API.
 *****************************************************************************/
static void app_proprietary_task(void *p_arg);

/**************************************************************************//**
 * Proprietary application init.
 *****************************************************************************/
void app_proprietary_init()
{
  char *app_name = "RS41 to BLE";
  printf("%s\n", app_name);

  size_t numberOfObjects;

  CMU_ClockEnable(cmuClock_GPCRC, true);

#if defined(SL_CATALOG_GLIB_PRESENT)
  EMSTATUS emstatus;

  /* Initialize the DMD module for the DISPLAY device driver. */
  emstatus = DMD_init(0);
  if (DMD_OK != emstatus) {
    while (1) ;
  }

  /* Initialize the glib context */
  emstatus = GLIB_contextInit(&glibContext);
  if (GLIB_OK != emstatus) {
    while (1) ;
  }
  glibContext.backgroundColor = White;
  glibContext.foregroundColor = Black;

  GLIB_clear(&glibContext);
  GLIB_setFont(&glibContext, (GLIB_Font_t *)&GLIB_FontModified8x8);
  GLIB_drawString(&glibContext, app_name, strlen(app_name), 0, 0, false);
  DMD_updateDisplay();
#endif

  printf("NVM3 object count: %d\n", numberOfObjects = nvm3_countObjects(nvm3_defaultHandle));
  if (numberOfObjects < 1) {
    Ecode_t nvm_status;
    printf("Initializing NVM objects\n");
    // Erase all objects and write initial data to NVM3
    nvm_status = nvm3_eraseAll(nvm3_defaultHandle);
    printf("NVM erased (status: %08lx)\n", nvm_status);
    nvm_status = nvm3_writeCounter(nvm3_defaultHandle, 1, proprietary_channel);
    printf("NVM counter written (status: %08lx)\n", nvm_status);
  }
  nvm3_readCounter(nvm3_defaultHandle, 1, &proprietary_channel);
  printf("Proprietary channel: %lu\n", proprietary_channel);

  RTOS_ERR err;

  // Create the Proprietary Application task.
  OSTaskCreate(&app_proprietary_task_tcb,
               "App Proprietary Task",
               app_proprietary_task,
               0u,
               APP_PROPRIETARY_TASK_PRIO,
               &app_proprietary_task_stack[0u],
               (APP_PROPRIETARY_TASK_STACK_SIZE / 10u),
               APP_PROPRIETARY_TASK_STACK_SIZE,
               0u,
               0u,
               0u,
               (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               &err);
  app_assert(err.Code == RTOS_ERR_NONE,
             "[E: 0x%04x] Task creation failed" APP_LOG_NEW_LINE,
             (int)err.Code);

  // Initialize the flag group for the proprietary task.
  OSFlagCreate(&app_event_flags, "Prop. flags", (OS_FLAGS)0, &err);
  app_assert(err.Code == RTOS_ERR_NONE,
             "[E: 0x%04x] Event flag creation failed" APP_LOG_NEW_LINE,
             (int)err.Code);
}

static void app_proprietary_task(void *p_arg)
{
  PP_UNUSED_PARAM(p_arg);
  RTOS_ERR err;

  /////////////////////////////////////////////////////////////////////////////
  //                                                                         //
  // The following code snippet shows how to start simple receiving, as an   //
  // example, within a DMP application. However, it is commented out to      //
  // demonstrate the lowest possible power consumption as well.              //
  //                                                                         //
  // PLEASE NOTE: ENABLING CONSTANT RECIEVING HAS HEAVY IMPACT ON POWER      //
  // CONSUMPTION OF YOUR PRODUCT.                                            //
  //                                                                         //
  // For further examples on sending / recieving in a DMP application, and   //
  // also on reducing the overall power demand, the following Flex projects  //
  // could serve as a good starting point:                                   //
  //                                                                         //
  //  - Flex (Connect) - Soc Empty Example DMP                               //
  //  - Flex (RAIL) - Range Test DMP                                         //
  //  - Flex (RAIL) - Energy Mode                                            //
  //                                                                         //
  // See also: AN1134: Dynamic Multiprotocol Development with Bluetooth and  //
  //                   Proprietary Protocols on RAIL in GSDK v2.x            //
  /////////////////////////////////////////////////////////////////////////////


   RAIL_Handle_t rail_handle;
   RAIL_Status_t status;

   rail_handle = sl_rail_util_get_handle(SL_RAIL_UTIL_HANDLE_INST0);
   // Start reception.
   RAIL_SchedulerInfo_t rxSchedulerInfo = (RAIL_SchedulerInfo_t){
     .priority = 15,
   };
   RAIL_ScheduleRxConfig_t rxScheduleConfig = {
     .startMode = RAIL_TIME_DELAY,
     .start = 300000,
     .endMode = RAIL_TIME_DELAY,
     .end = 1800000,
     .rxTransitionEndSchedule = 0,
     .hardWindowEnd = 0,
   };
   status = RAIL_ScheduleRx(rail_handle, 0, &rxScheduleConfig, &rxSchedulerInfo);
   app_assert(status == RAIL_STATUS_NO_ERROR,
             "[E: 0x%04x] Failed to start RAIL reception" APP_LOG_NEW_LINE,
             (int)status);


  // Start task main loop.
  while (DEF_TRUE) {

    // Wait for the event flag to be set.

    OS_FLAGS flags = OSFlagPend(&app_event_flags,
      APP_PROPRIETARY_EVENT_FLAG + \
      APP_BLUETOOTH_CHANNEL_NUMBER_EVENT_FLAG + \
      APP_BUTTON0_EVENT_FLAG + \
      APP_BUTTON1_EVENT_FLAG,
      (OS_TICK)0,
      OS_OPT_PEND_BLOCKING +     \
      OS_OPT_PEND_FLAG_SET_ANY + \
      OS_OPT_PEND_FLAG_CONSUME,
      NULL,
      &err);

    app_assert(err.Code == RTOS_ERR_NONE,
               "[E: 0x%04x] Prop event flag pend error" APP_LOG_NEW_LINE,
               (int)err.Code);

    char tmp_str[32];

    if( flags & APP_BLUETOOTH_CHANNEL_NUMBER_EVENT_FLAG) {
      size_t bt_len;
      uint16_t bt_value;
      sl_bt_gatt_server_read_attribute_value(gattdb_channel_number, 0, sizeof(proprietary_channel), &bt_len, (uint8_t *)&bt_value);
      if(RAIL_IsValidChannel(rail_handle, bt_value) == RAIL_STATUS_NO_ERROR) {
        proprietary_channel = bt_value;
      }
      snprintf(tmp_str, TMP_STR_LEN, "Freq: 40%03lu0kHz", proprietary_channel);
      display_info(tmp_str, 0, 0, enable_lcd);
    }

    if( flags & APP_BUTTON0_EVENT_FLAG) {
//      if(RAIL_IsValidChannel(rail_handle, proprietary_channel + BUTTON_CH_STEP) == RAIL_STATUS_NO_ERROR) {
//      }
      proprietary_channel += BUTTON_CH_STEP;
      if (proprietary_channel > rs41_channels[0].channelNumberEnd) {
        proprietary_channel = rs41_channels[0].channelNumberStart;
      }
      Ecode_t nvm_status = nvm3_writeCounter(nvm3_defaultHandle, 1, proprietary_channel);
      printf("NVM counter write (status: %08lx)\n", nvm_status);
      snprintf(tmp_str, TMP_STR_LEN, "Freq: 40%03lu0kHz", proprietary_channel);
      display_info(tmp_str, 0, 0, enable_lcd);
    }

    if( flags & APP_BUTTON1_EVENT_FLAG) {
/*
      if(RAIL_IsValidChannel(rail_handle, proprietary_channel - BUTTON_CH_STEP) == RAIL_STATUS_NO_ERROR) {
        proprietary_channel -= BUTTON_CH_STEP;
      }
      printf("Channel (by button1): %u\n", proprietary_channel);
*/
      enable_lcd = !enable_lcd;
      if (enable_lcd) {
        // if enable_lcd becomes true possibly QR code displayed thus clear it
        // and display current frequency
        GLIB_clear(&glibContext);
        snprintf(tmp_str, TMP_STR_LEN, "Freq: 40%03lu0kHz", proprietary_channel);
        display_info(tmp_str, 0, 0, enable_lcd);
      } else {
        // otherwise indicate QR mode on the display (QR code displayed only if
        // coordinates successfully received)
        display_info("QR", 0, 0, true);
      }
    }

    RAIL_RxPacketHandle_t rx_packet_handle;
    RAIL_RxPacketInfo_t packet_info;
    RAIL_RxPacketDetails_t packet_details;
    RAIL_Status_t rail_status = RAIL_STATUS_NO_ERROR;
    if (flags & APP_PROPRIETARY_EVENT_FLAG) {
      if (packet_recieved) {
        packet_recieved = false;
        // Packet received:
        //  - Check whether RAIL_HoldRxPacket() was successful, i.e. packet handle is valid
        //  - Copy it to the application FIFO
        //  - Free up the radio FIFO
        rx_packet_handle = RAIL_GetRxPacketInfo(rail_handle, RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE, &packet_info);
        while (rx_packet_handle != RAIL_RX_PACKET_HANDLE_INVALID) {
          RAIL_CopyRxPacket(rx_data + SYNC_WORD_LENGTH, &packet_info);

          rail_status = RAIL_GetRxPacketDetails(rail_handle, rx_packet_handle, &packet_details);
          if (rail_status != RAIL_STATUS_NO_ERROR) {
            printf("RAIL_GetRxPacketDetails() result:%d\n", rail_status);
          }

          rail_status = RAIL_ReleaseRxPacket(rail_handle, rx_packet_handle);
          if (rail_status != RAIL_STATUS_NO_ERROR) {
            printf("RAIL_ReleaseRxPacket() result:%d\n", rail_status);
          }

          de_whiten(rs41_data, rx_data, 320);

  #if defined(SL_CATALOG_LED0_PRESENT)
          sl_led_toggle(&sl_led_led0);
  #endif

          LLA_t lla = {};
          RS41_Status_t rs41_status = {};

          snprintf(tmp_str, TMP_STR_LEN, "Freq: 40%03lu0kHz", proprietary_channel);
          display_info(tmp_str, 0, 0, enable_lcd);
          snprintf(tmp_str, TMP_STR_LEN, "RSSI: %d", packet_details.rssi);
          display_info(tmp_str, 0, 10, enable_lcd);
  /*
          printf("Length: %d\n", packet_info.packetBytes);
          for(uint16_t i = 0; i < 320; i++) {
            printf("0x%02x ", rs41_data[i]);
          }
          printf("\n");
  */

          if (validate_block(RS41_BLOCK_STATUS, rs41_data) == BLOCK_OK) {
            extract_block_status(rs41_data, &rs41_status);
            ble_data.frame_number = rs41_status.frame_number;
            memcpy(ble_data.serial_number, rs41_status.serial_number, sizeof(ble_data.serial_number));
            ble_data.battery_voltage = (uint8_t)(rs41_status.battery_voltage * 10);
            ble_data.bitfied_status = rs41_status.bitfield_status;

            snprintf(tmp_str, TMP_STR_LEN, "Fr:   %u", rs41_status.frame_number);
            display_info(tmp_str, 10, 24, enable_lcd);
            snprintf(tmp_str, TMP_STR_LEN, "SN:   %s", rs41_status.serial_number);
            display_info(tmp_str, 10, 34, enable_lcd);
            snprintf(tmp_str, TMP_STR_LEN, "Batt: %1.1f", rs41_status.battery_voltage);
            display_info(tmp_str, 10, 44, enable_lcd);
            snprintf(tmp_str, TMP_STR_LEN, "Stat: %04x", rs41_status.bitfield_status);
            display_info(tmp_str, 10, 54, enable_lcd);

  //          snprintf(temporary_string, TEMPORARY_STRING_SIZE, "%d\n", status.pcb_temperature);
  //          printf("%s\n", tmp_str);
  //          snprintf(temporary_string, TEMPORARY_STRING_SIZE, "%d\n", status.humidity_sensor_heating_pwm);
  //          printf("%s\n", tmp_str);
  //          snprintf(temporary_string, TEMPORARY_STRING_SIZE, "%u\n", status.transimt_power);
  //          printf("%s\n", tmp_str);
  //          snprintf(temporary_string, TEMPORARY_STRING_SIZE, "%u\n", status.subframe_number);
  //          printf("%s\n", tmp_str);
            display_mark(true, 0, 28, enable_lcd);
          } else {
            printf("STATUS validation failed\n");
            display_mark(false, 0, 72, enable_lcd);
          }

          if (validate_block(RS41_BLOCK_GPSPOS, rs41_data) == BLOCK_OK) {
            RS41_Gps_Position_t gps_position;
            extract_block_gpspos(rs41_data, &gps_position);
            ble_data.satellite_vehicle_number = gps_position.satellite_vehicle_number;
            ble_data.pdop = (uint8_t)(gps_position.pdop * 10);

            lla = ecef2lla(gps_position.ecef_position);
            ble_data.latitude = lla.latitude;
            ble_data.longitude = lla.longitude;
            ble_data.altitude = lla.altitude;

            snprintf(tmp_str, TMP_STR_LEN, "Lat: %f", lla.latitude);
            display_info(tmp_str, 10, 68, enable_lcd);
            snprintf(tmp_str, TMP_STR_LEN, "Lon: %f", lla.longitude);
            display_info(tmp_str, 10, 78, enable_lcd);
            snprintf(tmp_str, TMP_STR_LEN, "Alt: %.1f", lla.altitude);
            display_info(tmp_str, 10, 88, enable_lcd);
            snprintf(tmp_str, TMP_STR_LEN, "Sat:  %d", gps_position.satellite_vehicle_number);
            display_info(tmp_str, 10, 98, enable_lcd);
            snprintf(tmp_str, TMP_STR_LEN, "PDOP: %1.1f", gps_position.pdop);
            display_info(tmp_str, 10, 108, enable_lcd);
            display_mark(true, 0, 72, enable_lcd);

            if(!enable_lcd) {
              char qr_text[60];
              snprintf(qr_text, 80, "https://www.openstreetmap.org/?mlat=%.6f&mlon=%.6f", lla.latitude, lla.longitude);
              draw_qrcode(qr_text);
            }
          } else {
            printf("GPSPOS validation failed\n");
            display_mark(false, 0, 72, enable_lcd);
          }

  #if defined(SL_CATALOG_LED0_PRESENT)
          sl_led_toggle(&sl_led_led0);
  #endif

          sl_bt_gatt_server_write_attribute_value(gattdb_rs41_data, 0, sizeof(ble_data), (uint8_t *)&ble_data);
          sl_bt_external_signal(CHARACTERISTIC_CHANGED);

          rx_packet_handle = RAIL_GetRxPacketInfo(rail_handle, RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE, &packet_info);
          status = RAIL_ScheduleRx(rail_handle, proprietary_channel, &rxScheduleConfig, &rxSchedulerInfo);
          if (rail_status != RAIL_STATUS_NO_ERROR) {
            printf("RAIL_StartRx() result:%d\n", rail_status);
          }
        }
      }
      if (rx_error) {
        rx_error = false;
        printf("Radio RX Error occurred\nEvents: %llX\n", radio_events);
        status = RAIL_ScheduleRx(rail_handle, proprietary_channel, &rxScheduleConfig, &rxSchedulerInfo);
        if (rail_status != RAIL_STATUS_NO_ERROR) {
          printf("RAIL_StartRx() result:%d\n", rail_status);
        }
      }
      if (packet_not_received) {
        packet_not_received = false;
        status = RAIL_ScheduleRx(rail_handle, proprietary_channel, &rxScheduleConfig, &rxSchedulerInfo);
        if (rail_status != RAIL_STATUS_NO_ERROR) {
          printf("RAIL_StartRx() result:%d\n", rail_status);
        }
      }
    }
#if defined(SL_CATALOG_GLIB_PRESENT)
    DMD_updateDisplay();
#endif

  }
}

/**************************************************************************//**
 * This callback is called on registered RAIL events.
 * Overrides dummy weak implementation.
 *****************************************************************************/
void sl_rail_util_on_event(RAIL_Handle_t rail_handle,
                           RAIL_Events_t events)
{
  radio_events = events;
  // Handle Rx events
  if ( events & RAIL_EVENTS_RX_COMPLETION ) {
    if (events & RAIL_EVENT_RX_PACKET_RECEIVED) {
      // Keep the packet in the radio buffer, download it later at the state machine
      RAIL_HoldRxPacket(rail_handle);
      packet_recieved = true;
    } else {
      // Handle Rx error
      rx_error = true;
    }
  }

  if (events & RAIL_EVENT_RX_SCHEDULED_RX_END) {
    if (!(events & RAIL_EVENT_RX_PACKET_RECEIVED)) {
      packet_not_received = true;
    }
  }

  // Handle Tx events - currently not used
  if ( events & RAIL_EVENTS_TX_COMPLETION) {
    if (events & RAIL_EVENT_TX_PACKET_SENT) {
    } else {
      // Handle Tx error
    }
  }

  // Perform all calibrations when needed
  if ( events & RAIL_EVENT_CAL_NEEDED ) {
    calibration_status = RAIL_Calibrate(rail_handle, NULL, RAIL_CAL_ALL_PENDING);
    if (calibration_status != RAIL_STATUS_NO_ERROR) {
      calibration_error = true;
    }
  }

#if defined(SL_CATALOG_KERNEL_PRESENT)
  RTOS_ERR err;
  OSFlagPost(&app_event_flags, APP_PROPRIETARY_EVENT_FLAG, OS_OPT_POST_FLAG_SET, &err);
#endif
}
/*
RAIL_Status_t RAILCb_SetupRxFifo(RAIL_Handle_t rail_handle)
{
  uint16_t rx_fifo_size = RX_FIFO_SIZE;
  RAIL_Status_t status = RAIL_SetRxFifo(rail_handle, &rx_fifo[0], &rx_fifo_size);
  if (rx_fifo_size != RX_FIFO_SIZE) {
    // We set up an incorrect FIFO size
    printf("Error: RAIL_SetRxFifo() result:%d ", status);
    return RAIL_STATUS_INVALID_PARAMETER;
  }
  return status;
}
*/

void sl_button_on_change(const sl_button_t *handle)
{
  if (sl_button_get_state(handle) == SL_SIMPLE_BUTTON_PRESSED) {
    if (handle == &sl_button_btn0) {
      RTOS_ERR err;
      OSFlagPost(&app_event_flags,
        APP_BUTTON0_EVENT_FLAG,
        OS_OPT_POST_FLAG_SET,
        &err);
    }
    if(handle == &sl_button_btn1) {
      RTOS_ERR err;
      OSFlagPost(&app_event_flags,
        APP_BUTTON1_EVENT_FLAG,
        OS_OPT_POST_FLAG_SET,
        &err);
    }
  }
}
