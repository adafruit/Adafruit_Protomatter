/*!
 * @file Adafruit_Protomatter.cpp
 *
 * @mainpage Adafruit Protomatter RGB LED matrix library.
 *
 * @section intro_sec Introduction
 *
 * This is documentation for Adafruit's protomatter library for HUB75-style
 * RGB LED matrices. It is designed to work with various matrices sold by
 * Adafruit ("HUB75" is a vague term and other similar matrices are not
 * guaranteed to work). This file is the Arduino-specific calls; the
 * underlying C code is more platform-neutral.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing products
 * from Adafruit!
 *
 * @section dependencies Dependencies
 *
 * This library depends on
 * <a href="https://github.com/adafruit/Adafruit-GFX-Library">Adafruit_GFX</a>
 * being present on your system. Please make sure you have installed the
 * latest version before using this library.
 *
 * @section author Author
 *
 * Written by Phil "Paint Your Dragon" Burgess and Jeff Epler for
 * Adafruit Industries, with contributions from the open source community.
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

// Arduino-specific wrapper for the Protomatter C library (provides
// constructor and so forth, builds on Adafruit_GFX). There should
// not be any device-specific #ifdefs here. See notes in core.c and
// arch/arch.h regarding portability.

#include "Adafruit_Protomatter.h" // Also includes core.h & Adafruit_GFX.h

extern Protomatter_core *_PM_protoPtr; ///< In core.c (via arch.h)

Adafruit_Protomatter::Adafruit_Protomatter(uint16_t bitWidth, uint8_t bitDepth,
                                           uint8_t rgbCount, uint8_t *rgbList,
                                           uint8_t addrCount, uint8_t *addrList,
                                           uint8_t clockPin, uint8_t latchPin,
                                           uint8_t oePin, bool doubleBuffer,
                                           int8_t tile, void *timer)
    : GFXcanvas16(bitWidth, (2 << min((int)addrCount, 5)) *
                                min((int)rgbCount, 5) *
                                (tile ? abs(tile) : 1)) {
  if (bitDepth > 6)
    bitDepth = 6; // GFXcanvas16 color limit (565)

  // Arguments are passed through to the C _PM_init() function which does
  // some input validation and minor allocation. Return value is ignored
  // because we can't really do anything about it in a C++ constructor.
  // The class begin() function checks rgbPins for NULL to determine
  // whether to proceed or indicate an error.
  (void)_PM_init(&core, bitWidth, bitDepth, rgbCount, rgbList, addrCount,
                 addrList, clockPin, latchPin, oePin, doubleBuffer, tile,
                 timer);
}

Adafruit_Protomatter::~Adafruit_Protomatter(void) {
  _PM_deallocate(&core);
  _PM_protoPtr = NULL;
}

ProtomatterStatus Adafruit_Protomatter::begin(void) {
  _PM_protoPtr = &core;
  return _PM_begin(&core);
}

// Transfer data from GFXcanvas16 to the matrix framebuffer's weird
// internal format. The actual conversion functions referenced below
// are in core.c, reasoning is explained there.
void Adafruit_Protomatter::show(void) {
  _PM_convert_565(&core, getBuffer(), WIDTH);
  _PM_swapbuffer_maybe(&core);
}

// Returns current value of frame counter and resets its value to zero.
// Two calls to this, timed one second apart (or use math with other
// intervals), can be used to get a rough frames-per-second value for
// the matrix (since this is difficult to estimate beforehand).
uint32_t Adafruit_Protomatter::getFrameCount(void) {
  return _PM_getFrameCount(_PM_protoPtr);
}

// This is based on the HSV function in Adafruit_NeoPixel.cpp, but with
// 16-bit RGB565 output for GFX lib rather than 24-bit. See that code for
// an explanation of the math, this is stripped of comments for brevity.
uint16_t Adafruit_Protomatter::colorHSV(uint16_t hue, uint8_t sat,
                                        uint8_t val) {
  uint8_t r, g, b;

  hue = (hue * 1530L + 32768) / 65536;

  if (hue < 510) { //         Red to Green-1
    b = 0;
    if (hue < 255) { //         Red to Yellow-1
      r = 255;
      g = hue;       //           g = 0 to 254
    } else {         //         Yellow to Green-1
      r = 510 - hue; //           r = 255 to 1
      g = 255;
    }
  } else if (hue < 1020) { // Green to Blue-1
    r = 0;
    if (hue < 765) { //         Green to Cyan-1
      g = 255;
      b = hue - 510;  //          b = 0 to 254
    } else {          //        Cyan to Blue-1
      g = 1020 - hue; //          g = 255 to 1
      b = 255;
    }
  } else if (hue < 1530) { // Blue to Red-1
    g = 0;
    if (hue < 1275) { //        Blue to Magenta-1
      r = hue - 1020; //          r = 0 to 254
      b = 255;
    } else { //                 Magenta to Red-1
      r = 255;
      b = 1530 - hue; //          b = 255 to 1
    }
  } else { //                 Last 0.5 Red (quicker than % operator)
    r = 255;
    g = b = 0;
  }

  // Apply saturation and value to R,G,B, pack into 16-bit 'RGB565' result:
  uint32_t v1 = 1 + val;  // 1 to 256; allows >>8 instead of /255
  uint16_t s1 = 1 + sat;  // 1 to 256; same reason
  uint8_t s2 = 255 - sat; // 255 to 0
  return (((((r * s1) >> 8) + s2) * v1) & 0xF800) |
         ((((((g * s1) >> 8) + s2) * v1) & 0xFC00) >> 5) |
         (((((b * s1) >> 8) + s2) * v1) >> 11);
}
