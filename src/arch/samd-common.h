/*!
 * @file samd-common.h
 *
 * Part of Adafruit's Protomatter library for HUB75-style RGB LED matrices.
 * This file contains SAMD-SPECIFIC CODE (SAMD51 & SAMD21).
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

#if defined(__SAMD51__) || defined(SAM_D5X_E5X) || defined(_SAMD21_) ||        \
    defined(SAMD21)

#if defined(ARDUINO) // COMPILING FOR ARDUINO ------------------------------

// g_APinDescription[] table and pin indices are Arduino specific:
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define _PM_byteOffset(pin) (g_APinDescription[pin].ulPin / 8)
#define _PM_wordOffset(pin) (g_APinDescription[pin].ulPin / 16)
#else
#define _PM_byteOffset(pin) (3 - (g_APinDescription[pin].ulPin / 8))
#define _PM_wordOffset(pin) (1 - (g_APinDescription[pin].ulPin / 16))
#endif

// Arduino implementation is tied to a specific timer/counter & freq:
#if defined(TC4)
#define _PM_TIMER_DEFAULT TC4
#define _PM_IRQ_HANDLER TC4_Handler
#else // No TC4 on some M4's
#define _PM_TIMER_DEFAULT TC3
#define _PM_IRQ_HANDLER TC3_Handler
#endif
#define _PM_timerFreq 48000000
// Partly because IRQs must be declared at compile-time, and partly
// because we know Arduino's already set up one of the GCLK sources
// for 48 MHz.

// Because it's tied to a specific timer right now, there can be only
// one instance of the Protomatter_core struct. The Arduino library
// sets up this pointer when calling begin().
void *_PM_protoPtr = NULL;

// Timer interrupt service routine
void _PM_IRQ_HANDLER(void) {
  // Clear overflow flag:
  _PM_TIMER_DEFAULT->COUNT16.INTFLAG.reg = TC_INTFLAG_OVF;
  _PM_row_handler(_PM_protoPtr); // In core.c
}

#elif defined(CIRCUITPY) // COMPILING FOR CIRCUITPYTHON --------------------

#include "hal_gpio.h"

#define _PM_pinOutput(pin) gpio_set_pin_direction(pin, GPIO_DIRECTION_OUT)
#define _PM_pinInput(pin) gpio_set_pin_direction(pin, GPIO_DIRECTION_IN)
#define _PM_pinHigh(pin) gpio_set_pin_level(pin, 1)
#define _PM_pinLow(pin) gpio_set_pin_level(pin, 0)
#define _PM_portBitMask(pin) (1u << ((pin) & 31))

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define _PM_byteOffset(pin) ((pin & 31) / 8)
#define _PM_wordOffset(pin) ((pin & 31) / 16)
#else
#define _PM_byteOffset(pin) (3 - ((pin & 31) / 8))
#define _PM_wordOffset(pin) (1 - ((pin & 31) / 16))
#endif

// CircuitPython implementation is tied to a specific freq (but the counter
// is dynamically allocated):
#define _PM_timerFreq 48000000

// As currently implemented, there can be only one instance of the
// Protomatter_core struct. This pointer is set up when starting the matrix.
void *_PM_protoPtr = NULL;

// Timer interrupt service routine
void _PM_IRQ_HANDLER(void) {
  ((Tc *)(((Protomatter_core *)_PM_protoPtr)->timer))->COUNT16.INTFLAG.reg =
      TC_INTFLAG_OVF;
  _PM_row_handler(_PM_protoPtr); // In core.c
}

#else // END CIRCUITPYTHON -------------------------------------------------

// Byte offset macros, timer and ISR work for other environments go here.

#endif

#endif // END SAMD5x/SAME5x/SAMD21
