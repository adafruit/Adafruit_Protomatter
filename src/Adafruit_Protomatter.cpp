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

extern Protomatter_core* _PM_protoPtr; ///< In core.c (via arch.h)

/**
 * @brief  Adafruit_Protomatter constructor.
 * @param  bitWidth      Total width of RGB matrix chain, in pixels.
 *                       Usu. some multiple of 32, but maybe exceptions.
 * @param  bitDepth      Color "depth" in bitplanes, determines range of
 *                       shades of red, green and blue. e.g. passing 4
 *                       bits = 16 shades ea. R,G,B = 16x16x16 = 4096
 *                       colors. Max is 6, since the GFX library works
 *                       with "565" RGB colors (6 bits green, 5 red/blue).
 * @param  rgbCount      Number of "sets" of RGB data pins, each set
 *                       containing 6 pins (2 ea. R,G,B). Typically 1,
 *                       indicating a single matrix (or matrix chain).
 *                       In theory (but not yet extensively tested),
 *                       multiple sets of pins can be driven in parallel,
 *                       up to 5 on some devices (if the hardware design
 *                       provides all those bits on one PORT).
 * @param  rgbList       A uint8_t array of pins (Arduino pin numbering),
 *                       6X the prior rgbCount value, corresponding to
 *                       the 6 output color bits for a matrix (or chain).
 *                       Order is upper-half red, green, blue, lower-half
 *                       red, green blue (repeat for each add'l chain).
 *                       All the RGB pins (plus the clock pin below on
 *                       some architectures) MUST be on the same PORT
 *                       register. It's recommended (but not required)
 *                       that all RGB pins (and clock depending on arch)
 *                       be within the same byte of a PORT (but do not
 *                       need to be sequential or contiguous within that
 *                       byte) for more efficient RAM utilization. For
 *                       two concurrent chains, same principle but 16-bit
 *                       word instead of byte.
 * @param  addrCount     Number of row address lines required of matrix.
 *                       Total pixel height is then 2 x 2^addrCount, e.g.
 *                       32-pixel-tall matrices have 4 row address lines.
 *                       In ABC mode, use 5 for 64 rows even though
 *                       addrList contains only three pins.
 * @param  addrList      A uint8_t array of pins (Arduino pin numbering),
 *                       one per row address line in binary mode. In ABC
 *                       mode, exactly three: A clock, B enable, C data.
 * @param  clockPin      RGB clock pin (Arduino pin #).
 * @param  latchPin      RGB data latch pin (Arduino pin #).
 * @param  oePin         Output enable pin (Arduino pin #), active low.
 * @param  doubleBuffer  If true, two matrix buffers are allocated,
 *                       so changing display contents doesn't introduce
 *                       artifacts mid-conversion. Requires ~2X RAM.
 * @param  tile          If multiple matrices are chained and stacked
 *                       vertically (rather than or in addition to
 *                       horizontally), the number of vertical tiles is
 *                       specified here. Positive values indicate a
 *                       "progressive" arrangement (always left-to-right),
 *                       negative for a "serpentine" arrangement (alternating
 *                       180 degree orientation). Horizontal tiles are implied
 *                       in the 'bitWidth' argument.
 * @param  timer         Pointer to timer peripheral or timer-related
 *                       struct (architecture-dependent), or NULL to
 *                       use a default timer ID (also arch-dependent).
 * @param rowAddressMode Row protocol. Defaults to binary A-E addressing.
 *                       Use PROTOMATTER_ROW_ADDRESS_ABC for serial rows.
 */
Adafruit_Protomatter::Adafruit_Protomatter(
    uint16_t bitWidth, uint8_t bitDepth, uint8_t rgbCount, uint8_t* rgbList,
    uint8_t addrCount, uint8_t* addrList, uint8_t clockPin, uint8_t latchPin,
    uint8_t oePin, bool doubleBuffer, int8_t tile, void* timer,
    ProtomatterRowAddressMode rowAddressMode)
    : GFXcanvas16(bitWidth, (2 << min((int)addrCount, 5)) *
                                min((int)rgbCount, 5) *
                                (tile ? abs(tile) : 1)) {
  if (bitDepth > 6)
    bitDepth = 6; // GFXcanvas16 color limit (565)

  // Arguments are passed through to the C initialization function which does
  // some input validation and minor allocation. Return value is ignored
  // because we can't really do anything about it in a C++ constructor.
  // The class begin() function checks rgbPins for NULL to determine
  // whether to proceed or indicate an error.
  (void)_PM_init_with_row_address_mode(
      &core, bitWidth, bitDepth, rgbCount, rgbList, addrCount, addrList,
      clockPin, latchPin, oePin, doubleBuffer, tile, timer, rowAddressMode);
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
