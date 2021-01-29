/*!
 * @file rp2040.h
 *
 * Part of Adafruit's Protomatter library for HUB75-style RGB LED matrices.
 * This file contains RP2040 (Raspberry Pi Pico, etc.) SPECIFIC CODE.
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

// TO DO: PUT A *PROPER* RP2040 CHECK HERE
#if defined(PICO_BOARD)

#include "pico/stdlib.h" // For sio_hw, etc.
#include "hardware/timer.h"
#include "hardware/irq.h"
#include "time.h"

// RP2040 only allows full 32-bit aligned writes to GPIO.
#define _PM_STRICT_32BIT_IO ///< Change core.c behavior for long accesses only

// TEMPORARY: FORCING ARDUINO COMPILATION FOR INITIAL C TESTING
#define ARDUINO

#if defined(ARDUINO) // COMPILING FOR ARDUINO ------------------------------

// 'pin' here is GPXX # -- that might change in Arduino implementation
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define _PM_byteOffset(pin) ((pin & 31) / 8)
#define _PM_wordOffset(pin) ((pin & 31) / 16)
#else
#define _PM_byteOffset(pin) (3 - ((pin & 31) / 8))
#define _PM_wordOffset(pin) (1 - ((pin & 31) / 16))
#endif

// Arduino implementation is tied to a specific timer & freq:
#define _PM_ALARM_NUM 0
#define _PM_IRQ_HANDLER TIMER_IRQ_0
#define _PM_timerFreq 1000000
#define _PM_TIMER_DEFAULT NULL

// Because it's tied to a specific timer right now, there can be only
// one instance of the Protomatter_core struct. The Arduino library
// sets up this pointer when calling begin().
void *_PM_protoPtr = NULL;

#define _PM_portOutRegister(pin) ((void *)&sio_hw->gpio_out)
#define _PM_portSetRegister(pin) ((volatile uint32_t *)&sio_hw->gpio_set)
#define _PM_portClearRegister(pin) ((volatile uint32_t *)&sio_hw->gpio_clr)
#define _PM_portToggleRegister(pin) ((volatile uint32_t *)&sio_hw->gpio_togl)
// 'pin' here is GPXX # -- that might change in Arduino implementation
#define _PM_portBitMask(pin) (1UL << pin)
// Same for these -- using GPXX #, but Arduino might assign different order
#define _PM_pinOutput(pin) gpio_set_dir(pin, GPIO_OUT)
#define _PM_pinLow(pin) gpio_clr_mask(1UL << pin)
#define _PM_pinHigh(pin) gpio_set_mask(1UL << pin)

#define _PM_delayMicroseconds(n) sleep_us(n)

static void _PM_timerISR(void) {
  hw_clear_bits(&timer_hw->intr, 1u << _PM_ALARM_NUM); // Clear alarm flag
  _PM_row_handler(_PM_protoPtr); // In core.c
}

// Initialize, but do not start, timer.
void _PM_timerInit(void *tptr) {
  timer_hw->alarm[_PM_ALARM_NUM] = timer_hw->timerawl; // Clear any timer
  hw_set_bits(&timer_hw->inte, 1u << _PM_ALARM_NUM);
  irq_set_exclusive_handler(_PM_IRQ_HANDLER, _PM_timerISR); // Set IRQ handler
}

// Unlike timers on other devices, on RP2040 you don't reset a counter to
// zero at the start of a cycle. To emulate that behavior (for determining
// elapsed times), the timer start time must be saved somewhere...
static uint64_t _PM_timerSave;

// Set timer period and enable timer.
inline void _PM_timerStart(void *tptr, uint32_t period) {
  _PM_timerSave = timer_hw->timerawl; // Time at start
  uint64_t target = _PM_timerSave + period; // Time at end
  timer_hw->alarm[_PM_ALARM_NUM] = (uint32_t)target;

  irq_set_enabled(_PM_IRQ_HANDLER, true); // Enable alarm IRQ
}

// Return current count value (timer enabled or not).
// Timer must be previously initialized.
inline uint32_t _PM_timerGetCount(void *tptr) {
  return (uint32_t)(timer_hw->timerawl - _PM_timerSave);
}

// Disable timer and return current count value.
// Timer must be previously initialized.
uint32_t _PM_timerStop(void *tptr) {
  irq_set_enabled(_PM_IRQ_HANDLER, false); // Disable alarm IRQ
  return _PM_timerGetCount(tptr);
}

#elif defined(CIRCUITPY) // COMPILING FOR CIRCUITPYTHON --------------------

// RP2040 CircuitPython magic goes here.
// Put anything common to Arduino & CircuitPython above the platform ifdefs.

#endif // END CIRCUITPYTHON ------------------------------------------------

#endif // END PICO_BOARD

#define _PM_chunkSize 1 ///< DON'T unroll loop






#if 0

#define _PM_clockHoldHigh                                                      \
  asm("nop; nop; nop; nop; nop; nop; nop;");                                   \
  asm("nop; nop; nop; nop; nop; nop; nop;");
#define _PM_clockHoldLow                                                       \
  asm("nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;");                    \
  asm("nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;");

#endif // 0
