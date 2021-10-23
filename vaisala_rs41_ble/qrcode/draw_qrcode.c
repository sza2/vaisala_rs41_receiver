/*
 * draw_qrcode.c
 *
 *  Created on: Aug 19, 2021
 *      Author: imborbas
 */

#include "qrcode.h"
#include "glib.h"

#define QRCODE_VERSION 5
#define QRCODE_DISPLAY_SCALE 3
#define QRCODE_DISPLAY_OFFSET 8

extern GLIB_Context_t glibContext;

QRCode qrcode;
uint8_t qrcodeData[QRCODE_BUFFERSIZE(QRCODE_VERSION)];

 // Create the QR code
int draw_qrcode (char *qrcode_text)
{
  uint8_t x, y, xx, yy;
  qrcode_initText(&qrcode, qrcodeData, QRCODE_VERSION, ECC_MEDIUM, qrcode_text); //"https://www.openstreetmap.org/?mlat=47.55905&mlon=19.05191#map=18/47.55905/19.05191");
  GLIB_clear(&glibContext);
  for (y = 0; y < qrcode.size; y++) {
      for (x = 0; x < qrcode.size; x++) {
          if (qrcode_getModule(&qrcode, x, y)) {
              for (xx=0; xx < QRCODE_DISPLAY_SCALE; xx++) {
                  for (yy=0; yy < QRCODE_DISPLAY_SCALE; yy++) {
                      GLIB_drawPixel(&glibContext, QRCODE_DISPLAY_SCALE*x+xx+QRCODE_DISPLAY_OFFSET, QRCODE_DISPLAY_SCALE*y+yy+QRCODE_DISPLAY_OFFSET);
                  }
              }
          }
      }
  }
  DMD_updateDisplay();

  return 0;
}
