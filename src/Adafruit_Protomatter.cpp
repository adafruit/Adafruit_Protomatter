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
    : GFXcanvas16(bitWidth,
                  (2 << min((int)addrCount, 5)) * min((int)rgbCount, 5)) {
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
