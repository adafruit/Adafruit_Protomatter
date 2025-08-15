/*!
 * @file esp32-c3.h
 *
 * Part of Adafruit's Protomatter library for HUB75-style RGB LED matrices.
 * This file contains ESP32-C3-SPECIFIC CODE.
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

#if defined(CONFIG_IDF_TARGET_ESP32C3)

#define _PM_portOutRegister(pin) (volatile uint32_t *)&GPIO.out
#define _PM_portSetRegister(pin) (volatile uint32_t *)&GPIO.out_w1ts
#define _PM_portClearRegister(pin) (volatile uint32_t *)&GPIO.out_w1tc

#define _PM_portBitMask(pin) (1U << ((pin) & 31))

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define _PM_byteOffset(pin) ((pin & 31) / 8)
#define _PM_wordOffset(pin) ((pin & 31) / 16)
#else
#define _PM_byteOffset(pin) (3 - ((pin & 31) / 8))
#define _PM_wordOffset(pin) (1 - ((pin & 31) / 16))
#endif

// No special peripheral setup on ESP32C3, just use common timer init...
#define _PM_timerInit(core) _PM_esp32commonTimerInit(core);

#if defined(ARDUINO) // COMPILING FOR ARDUINO ------------------------------
// Return current count value (timer enabled or not).
// Timer must be previously initialized.
// This function is the same on all ESP32 parts EXCEPT S3.
IRAM_ATTR inline uint32_t _PM_timerGetCount(Protomatter_core *core) {
  return (uint32_t)timerRead((hw_timer_t *)core->timer);
}

#elif defined(CIRCUITPY) // COMPILING FOR CIRCUITPYTHON --------------------

#endif // END CIRCUITPYTHON ------------------------------------------------

#endif // END ESP32C3
