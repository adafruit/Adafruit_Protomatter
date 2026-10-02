// Arduino-specific header, accompanies Adafruit_Protomatter.cpp.
// There should not be any device-specific #ifdefs here.

#pragma once

#include <Adafruit_GFX.h>

#include "core.h"

/*!
    @brief  Class representing the Arduino-facing side of the Protomatter
            library. Subclass of Adafruit_GFX's GFXcanvas16 to allow all
            the drawing operations.
*/
class Adafruit_Protomatter : public GFXcanvas16 {
 public:
  Adafruit_Protomatter(uint16_t bitWidth, uint8_t bitDepth, uint8_t rgbCount,
                       uint8_t* rgbList, uint8_t addrCount, uint8_t* addrList,
                       uint8_t clockPin, uint8_t latchPin, uint8_t oePin,
                       bool doubleBuffer, int8_t tile = 1, void* timer = NULL,
                       ProtomatterRowAddressMode rowAddressMode =
                           PROTOMATTER_ROW_ADDRESS_BINARY);
  ~Adafruit_Protomatter(void);

  /*!
    @brief  Start a Protomatter matrix display running -- initialize
            pins, timer and interrupt into existence.
    @return A ProtomatterStatus status, one of:
            PROTOMATTER_OK if everything is good.
            PROTOMATTER_ERR_PINS if data and/or clock pins are split
            across different PORTs.
            PROTOMATTER_ERR_MALLOC if insufficient RAM to allocate
            display memory.
            PROTOMATTER_ERR_ARG if a bad value was passed to the
            constructor.
  */
  ProtomatterStatus begin(void);

  /*!
    @brief Process data from GFXcanvas16 to the matrix framebuffer's
           internal format for display.
  */
  void show(void);

  /*!
    @brief Disable (but do not deallocate) a Protomatter matrix.
  */
  void stop(void) {
    _PM_stop(&core);
  }

  /*!
    @brief Resume a previously-stopped matrix.
  */
  void resume(void) {
    _PM_resume(&core);
  }

  /*!
    @brief  Returns current value of frame counter and resets its value
            to zero. Two calls to this, timed one second apart (or use
            math with other intervals), can be used to get a rough
            frames-per-second value for the matrix (since this is
            difficult to estimate beforehand).
    @return Frame count since previous call to function, as a uint32_t.
  */
  uint32_t getFrameCount(void);

  /*!
    @brief  Converts 24-bit color (8 bits red, green, blue) used in a lot
            a lot of existing graphics code down to the "565" color format
            used by Adafruit_GFX. Might get further quantized by matrix if
            using less than 6-bit depth.
    @param  red    Red brightness, 0 (min) to 255 (max).
    @param  green  Green brightness, 0 (min) to 255 (max).
    @param  blue   Blue brightness, 0 (min) to 255 (max).
    @return Packed 16-bit (uint16_t) color value suitable for GFX drawing
            functions.
  */
  uint16_t color565(uint8_t red, uint8_t green, uint8_t blue) {
    return ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | (blue >> 3);
  }

  /*!
    @brief   Convert hue, saturation and value into a packed 16-bit RGB color
             that can be passed to GFX drawing functions.
    @param   hue  An unsigned 16-bit value, 0 to 65535, representing one full
                  loop of the color wheel, which allows 16-bit hues to "roll
                  over" while still doing the expected thing (and allowing
                  more precision than the wheel() function that was common to
                  older graphics examples).
    @param   sat  Saturation, 8-bit value, 0 (min or pure grayscale) to 255
                  (max or pure hue). Default of 255 if unspecified.
    @param   val  Value (brightness), 8-bit value, 0 (min / black / off) to
                  255 (max or full brightness). Default of 255 if unspecified.
    @return  Packed 16-bit '565' RGB color. Result is linearly but not
             perceptually correct (no gamma correction).
  */
  uint16_t colorHSV(uint16_t hue, uint8_t sat = 255, uint8_t val = 255);

  /*!
    @brief   Adjust HUB clock signal duty cycle on architectures that support
             this (currently SAMD51 only) (else ignored).
    @param   d Duty setting, 0 minimum. Increasing values generate higher clock
             duty cycles at the same frequency. Arbitrary granular units, max
             varies by architecture and CPU speed, if supported at all.
             e.g. SAMD51 @ 120 MHz supports 0 (~50% duty) through 2 (~75%).
  */
  void setDuty(uint8_t d) {
    _PM_setDuty(d);
  };

 private:
  Protomatter_core core;             // Underlying C struct
  void convert_byte(uint8_t* dest);  // GFXcanvas16-to-matrix
  void convert_word(uint16_t* dest); // conversion functions
  void convert_long(uint32_t* dest); // for 8/16/32 bit bufs
};
