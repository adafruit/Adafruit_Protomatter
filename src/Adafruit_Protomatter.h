// Arduino-specific header, accompanies Adafruit_Protomatter.cpp.
// There should not be any device-specific #ifdefs here.

#pragma once

#include "core.h"
#include <Adafruit_GFX.h>

/*!
    @brief  Class representing the Arduino-facing side of the Protomatter
            library. Subclass of Adafruit_GFX's GFXcanvas16 to allow all
            the drawing operations.
*/
class Adafruit_Protomatter : public GFXcanvas16 {
public:
  /*!
    @brief  Adafruit_Protomatter constructor.
    @param  bitWidth      Total width of RGB matrix chain, in pixels.
                          Usu. some multiple of 32, but maybe exceptions.
    @param  bitDepth      Color "depth" in bitplanes, determines range of
                          shades of red, green and blue. e.g. passing 4
                          bits = 16 shades ea. R,G,B = 16x16x16 = 4096
                          colors. Max is 6, since the GFX library works
                          with "565" RGB colors (6 bits green, 5 red/blue).
    @param  rgbCount      Number of "sets" of RGB data pins, each set
                          containing 6 pins (2 ea. R,G,B). Typically 1,
                          indicating a single matrix (or matrix chain).
                          In theory (but not yet extensively tested),
                          multiple sets of pins can be driven in parallel,
                          up to 5 on some devices (if the hardware design
                          provides all those bits on one PORT).
    @param  rgbList       A uint8_t array of pins (Arduino pin numbering),
                          6X the prior rgbCount value, corresponding to
                          the 6 output color bits for a matrix (or chain).
                          Order is upper-half red, green, blue, lower-half
                          red, green blue (repeat for each add'l chain).
                          All the RGB pins (plus the clock pin below on
                          some architectures) MUST be on the same PORT
                          register. It's recommended (but not required)
                          that all RGB pins (and clock depending on arch)
                          be within the same byte of a PORT (but do not
                          need to be sequential or contiguous within that
                          byte) for more efficient RAM utilization. For
                          two concurrent chains, same principle but 16-bit
                          word instead of byte.
    @param  addrCount     Number of row address lines required of matrix.
                          Total pixel height is then 2 x 2^addrCount, e.g.
                          32-pixel-tall matrices have 4 row address lines.
    @param  addrList      A uint8_t array of pins (Arduino pin numbering),
                          one per row address line.
    @param  clockPin      RGB clock pin (Arduino pin #).
    @param  latchPin      RGB data latch pin (Arduino pin #).
    @param  oePin         Output enable pin (Arduino pin #), active low.
    @param  doubleBuffer  If true, two matrix buffers are allocated,
                          so changing display contents doesn't introduce
                          artifacts mid-conversion. Requires ~2X RAM.
    @param  tile          If multiple matrices are chained and stacked
                          vertically (rather than or in addition to
                          horizontally), the number of vertical tiles is
                          specified here. Positive values indicate a
                          "progressive" arrangement (always left-to-right),
                          negative for a "serpentine" arrangement (alternating
                          180 degree orientation). Horizontal tiles are implied
                          in the 'bitWidth' argument.
    @param  timer         Pointer to timer peripheral or timer-related
                          struct (architecture-dependent), or NULL to
                          use a default timer ID (also arch-dependent).
  */
  Adafruit_Protomatter(uint16_t bitWidth, uint8_t bitDepth, uint8_t rgbCount,
                       uint8_t *rgbList, uint8_t addrCount, uint8_t *addrList,
                       uint8_t clockPin, uint8_t latchPin, uint8_t oePin,
                       bool doubleBuffer, int8_t tile = 1, void *timer = NULL);
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
  void stop(void) { _PM_stop(&core); }

  /*!
    @brief Resume a previously-stopped matrix.
  */
  void resume(void) { _PM_resume(&core); }

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
    @param   Duty setting, 0 minimum. Increasing values generate higher clock
             duty cycles at the same frequency. Arbitrary granular units, max
             varies by architecture and CPU speed, if supported at all.
             e.g. SAMD51 @ 120 MHz supports 0 (~50% duty) through 2 (~75%).
  */
  void setDuty(uint8_t d) { _PM_setDuty(d); };

private:
  Protomatter_core core;             // Underlying C struct
  void convert_byte(uint8_t *dest);  // GFXcanvas16-to-matrix
  void convert_word(uint16_t *dest); // conversion functions
  void convert_long(uint32_t *dest); // for 8/16/32 bit bufs
};
