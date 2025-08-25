/*!
 * @file esp32.h
 *
 * Part of Adafruit's Protomatter library for HUB75-style RGB LED matrices.
 * This file contains ORIGINAL-ESP32-SPECIFIC CODE.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Phil "Paint Your Dragon" Burgess and Jeff Epler for
 * Adafruit Industries, with contributions from the open source community.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#pragma once

// NOTE: there is some intentional repetition in the macros and functions
// for some ESP32 variants. Previously they were all one file, but complex
// preprocessor directives were turning into spaghetti. THEREFORE, if making
// a change or bugfix in one variant-specific header, check the others to
// see if the same should be applied!

#if defined(CONFIG_IDF_TARGET_ESP32) // ORIGINAL ESP32, NOT S2/S3/etc.

#define _PM_portOutRegister(pin)                                               \
  (volatile uint32_t *)((pin < 32) ? &GPIO.out : &GPIO.out1.val)
#define _PM_portSetRegister(pin)                                               \
  (volatile uint32_t *)((pin < 32) ? &GPIO.out_w1ts : &GPIO.out1_w1ts.val)
#define _PM_portClearRegister(pin)                                             \
  (volatile uint32_t *)((pin < 32) ? &GPIO.out_w1tc : &GPIO.out1_w1tc.val)

#define _PM_portBitMask(pin) (1U << ((pin) & 31))

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define _PM_byteOffset(pin) ((pin & 31) / 8)
#define _PM_wordOffset(pin) ((pin & 31) / 16)
#else
#define _PM_byteOffset(pin) (3 - ((pin & 31) / 8))
#define _PM_wordOffset(pin) (1 - ((pin & 31) / 16))
#endif

// No special peripheral setup on OG ESP32, just use common timer init...
#define _PM_timerInit(core) _PM_esp32commonTimerInit(core);

// Return current count value (timer enabled or not).
// Timer must be previously initialized.
// This function is the same on all ESP32 parts EXCEPT S3.
IRAM_ATTR inline uint32_t _PM_timerGetCount(Protomatter_core *core) {
  return (uint32_t)timerRead((hw_timer_t *)core->timer);
}

#if defined(ARDUINO) // COMPILING FOR ARDUINO ------------------------------

// ESP32 requires a custom PEW declaration (issues one set of RGB color bits
// followed by clock pulse). Turns out the bit set/clear registers are not
// actually atomic. If two writes are made in quick succession, the second
// has no effect. One option is NOPs, other is to write a 0 (no effect) to
// the opposing register (set vs clear) to synchronize the next write.
// S2, S3 replace the whole "blast" functions and don't use PEW. C3 can use
// the default PEW.
#if !defined(CONFIG_IDF_TARGET_ESP32S3) && !defined(CONFIG_IDF_TARGET_ESP32S2)
#define PEW                                                                    \
  *set = *data++;         /* Set RGB data high */                              \
  *clear_full = 0;        /* ESP32 MUST sync before 2nd 'set' */               \
  *set_full = clock;      /* Set clock high */                                 \
  *clear_full = rgbclock; /* Clear RGB data + clock */                         \
  ///< Bitbang one set of RGB data bits to matrix
#endif // end !ESP32S3/S2

#elif defined(CIRCUITPY) // COMPILING FOR CIRCUITPYTHON --------------------

#define _PM_STRICT_32BIT_IO (1)

// ESP32 requires a custom PEW declaration (issues one set of RGB color bits
// followed by clock pulse). Turns out the bit set/clear registers are not
// actually atomic. If two writes are made in quick succession, the second
// has no effect. One option is NOPs, other is to write a 0 (no effect) to
// the opposing register (set vs clear) to synchronize the next write.
#define PEW                                                                    \
  *set = (*data++) << shift; /* Set RGB data high */                           \
  *clear_full = 0;           /* ESP32 MUST sync before 2nd 'set' */            \
  *set = clock;              /* Set clock high */                              \
  *clear_full = rgbclock;    /* Clear RGB data + clock */                      \
  ///< Bitbang one set of RGB data bits to matrix

#endif // END CIRCUITPYTHON ------------------------------------------------

#endif // END ESP32
