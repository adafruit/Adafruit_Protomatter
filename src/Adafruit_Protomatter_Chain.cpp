/*!
 * @file Adafruit_Protomatter_Chain.cpp
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
 * Modified by Mark Komus to support two matricies.
 * Original code by Phil "Paint Your Dragon" Burgess and Jeff Epler for
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

#include "Adafruit_Protomatter_Chain.h" // Also includes core.h & Adafruit_GFX.h

extern Protomatter_core *_PM_protoPtr; ///< In core.c (via arch.h)

Adafruit_Protomatter_Chain::Adafruit_Protomatter_Chain(uint16_t displayWidth, uint16_t displayHeight, uint16_t bitWidth, uint8_t bitDepth,
                                           uint8_t rgbCount, uint8_t *rgbList,
                                           uint8_t addrCount, uint8_t *addrList,
                                           uint8_t clockPin, uint8_t latchPin,
                                           uint8_t oePin, bool doubleBuffer,
                                           void *timer)
    : GFXcanvas16(displayWidth, displayHeight) {
  if (bitDepth > 6)
    bitDepth = 6; // GFXcanvas16 color limit (565)

  bWidth = bitWidth;
  uint32_t bytes = displayWidth * displayHeight * 2;
  if ((correctedBuffer = (uint16_t *)malloc(bytes))) {
    memset(correctedBuffer, 0, bytes);
  }

  // Arguments are passed through to the C _PM_init() function which does
  // some input validation and minor allocation. Return value is ignored
  // because we can't really do anything about it in a C++ constructor.
  // The class begin() function checks rgbPins for NULL to determine
  // whether to proceed or indicate an error.
  (void)_PM_init(&core, bitWidth, bitDepth, rgbCount, rgbList, addrCount,
                 addrList, clockPin, latchPin, oePin, doubleBuffer, timer);
}

Adafruit_Protomatter_Chain::~Adafruit_Protomatter_Chain(void) {
  _PM_deallocate(&core);
  _PM_protoPtr = NULL;
  if (correctedBuffer)
    free(correctedBuffer);
}

ProtomatterStatus Adafruit_Protomatter_Chain::begin(void) {
  _PM_protoPtr = &core;
  return _PM_begin(&core);
}

// Transfer data from GFXcanvas16 to the matrix framebuffer's weird
// internal format. The actual conversion functions referenced below
// are in core.c, reasoning is explained there.
void Adafruit_Protomatter_Chain::show(void) {
  correctBuffer();
  _PM_convert_565(&core, correctedBuffer, bWidth);
  _PM_swapbuffer_maybe(&core);
}

void Adafruit_Protomatter_Chain::correctBuffer() {
  int16_t displayWidth = width();
  int16_t displayHeight = height();

  uint16_t *upperdisplay = getBuffer();
  uint16_t *lowerdisplay = upperdisplay+(displayWidth*displayHeight/2);
  int size = sizeof(uint16_t);

  // Loop through the entire display
  // Map the pixels from the display to the correct pixels on the matricies
  for (int row = 0; row < (displayHeight/2); row++) {
    for (int col = 0; col < displayWidth; col++) {
      uint16_t *upperSource = upperdisplay+(row*displayWidth)+col;
      uint16_t *lowerSource = lowerdisplay+(row*displayWidth)+col;

      uint16_t *upperDest = correctedBuffer + ((displayHeight/2 - row) * displayWidth * 2) - (displayWidth + col + 1);
      uint16_t *lowerDest = correctedBuffer + (2 * row * displayWidth) + displayWidth + col;

      *upperDest = *upperSource;
      *lowerDest = *lowerSource;
    }
  }

}

// Returns current value of frame counter and resets its value to zero.
// Two calls to this, timed one second apart (or use math with other
// intervals), can be used to get a rough frames-per-second value for
// the matrix (since this is difficult to estimate beforehand).
uint32_t Adafruit_Protomatter_Chain::getFrameCount(void) {
  return _PM_getFrameCount(_PM_protoPtr);
}
