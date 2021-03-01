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
 * RP2040 NOTES: This initial implementation does NOT use PIO. That's normal
 * for Protomatter, which was written for simple GPIO + timer interrupt for
 * broadest portability. While not entirely optimal, it's not pessimal
 * either...no worse than any other platform where we're not taking
 * advantage of device-specific DMA or peripherals. Would require changes to
 * the 'blast' functions or possibly the whole _PM_row_handler() (both
 * currently in core.c). CPU load is just a few percent for a 64x32
 * matrix @ 6-bit depth, so I'm not losing sleep over this.
 *
 */

#pragma once

// TO DO: PUT A *PROPER* RP2040 CHECK HERE
#if defined(PICO_BOARD) || defined(__RP2040__)

#include "../../hardware_pwm/include/hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "pico/stdlib.h" // For sio_hw, etc.

// RP2040 only allows full 32-bit aligned writes to GPIO.
#define _PM_STRICT_32BIT_IO ///< Change core.c behavior for long accesses only

// TEMPORARY: FORCING ARDUINO COMPILATION FOR INITIAL C TESTING
#if !defined(CIRCUITPY)
#define ARDUINO
#endif

// Enable this to use PWM for bitplane timing, else a timer alarm is used.
// PWM has finer resolution, but alarm is adequate -- this is more about
// which peripheral we'd rather use, as both are finite resources.
#ifndef _PM_CLOCK_PWM
#define _PM_CLOCK_PWM (1)
#endif

#if _PM_CLOCK_PWM // Use PWM for timing
static void _PM_PWM_ISR(void);
#else // Use timer alarm for timing
static void _PM_timerISR(void);
#endif

#if defined(ARDUINO) // COMPILING FOR ARDUINO ------------------------------

// 'pin' here is GPXX # -- that might change in Arduino implementation
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define _PM_byteOffset(pin) ((pin & 31) / 8)
#define _PM_wordOffset(pin) ((pin & 31) / 16)
#else
#define _PM_byteOffset(pin) (3 - ((pin & 31) / 8))
#define _PM_wordOffset(pin) (1 - ((pin & 31) / 16))
#endif

#if _PM_CLOCK_PWM

// Arduino implementation is tied to a specific PWM slice & frequency
#define _PM_PWM_SLICE 0
#define _PM_PWM_DIV 3 // ~41.6 MHz, similar to SAMD
#define _PM_timerFreq (125000000 / _PM_PWM_DIV)
#define _PM_TIMER_DEFAULT NULL

#else // Use alarm for timing

// Arduino implementation is tied to a specific timer alarm & frequency
#define _PM_ALARM_NUM 1
#define _PM_IRQ_HANDLER TIMER_IRQ_1
#define _PM_timerFreq 1000000
#define _PM_TIMER_DEFAULT NULL

// Initialize, but do not start, timer.
void _PM_timerInit(void *tptr) {
#if _PM_CLOCK_PWM
  // Enable PWM wrap interrupt
  pwm_clear_irq(_PM_PWM_SLICE);
  pwm_set_irq_enabled(_PM_PWM_SLICE, true);
  irq_set_exclusive_handler(PWM_IRQ_WRAP, _PM_PWM_ISR);
  irq_set_enabled(PWM_IRQ_WRAP, true);

  // Config but do not start PWM
  pwm_config config = pwm_get_default_config();
  pwm_config_set_clkdiv_int(&config, _PM_PWM_DIV);
  pwm_init(_PM_PWM_SLICE, &config, true);
#else
  timer_hw->alarm[_PM_ALARM_NUM] = timer_hw->timerawl; // Clear any timer
  hw_set_bits(&timer_hw->inte, 1u << _PM_ALARM_NUM);
  irq_set_exclusive_handler(_PM_IRQ_HANDLER, _PM_timerISR); // Set IRQ handler
#endif
}

#endif

#elif defined(CIRCUITPY) // COMPILING FOR CIRCUITPYTHON --------------------

// 'pin' here is GPXX #
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define _PM_byteOffset(pin) ((pin & 31) / 8)
#define _PM_wordOffset(pin) ((pin & 31) / 16)
#else
#define _PM_byteOffset(pin) (3 - ((pin & 31) / 8))
#define _PM_wordOffset(pin) (1 - ((pin & 31) / 16))
#endif

#if _PM_CLOCK_PWM

int _PM_pwm_slice;
#define _PM_PWM_SLICE (_PM_pwm_slice & 0xff)
#define _PM_PWM_DIV 3 // ~41.6 MHz, similar to SAMD
#define _PM_timerFreq (125000000 / _PM_PWM_DIV)
#define _PM_TIMER_DEFAULT NULL

#else // Use alarm for timing

// Currently tied to a specific timer alarm & frequency
#define _PM_ALARM_NUM 1
#define _PM_IRQ_HANDLER TIMER_IRQ_1
#define _PM_timerFreq 1000000
#define _PM_TIMER_DEFAULT NULL

#endif

// Initialize, but do not start, timer.
void _PM_timerInit(void *tptr) {
#if _PM_CLOCK_PWM
  _PM_pwm_slice = (int)tptr & 0xff;
  // Enable PWM wrap interrupt
  pwm_clear_irq(_PM_PWM_SLICE);
  pwm_set_irq_enabled(_PM_PWM_SLICE, true);
  irq_set_exclusive_handler(PWM_IRQ_WRAP, _PM_PWM_ISR);
  irq_set_enabled(PWM_IRQ_WRAP, true);

  // Config but do not start PWM
  pwm_config config = pwm_get_default_config();
  pwm_config_set_clkdiv_int(&config, _PM_PWM_DIV);
  pwm_init(_PM_PWM_SLICE, &config, true);
#else
  timer_hw->alarm[_PM_ALARM_NUM] = timer_hw->timerawl; // Clear any timer
  hw_set_bits(&timer_hw->inte, 1u << _PM_ALARM_NUM);
  irq_set_exclusive_handler(_PM_IRQ_HANDLER, _PM_timerISR); // Set IRQ handler
#endif
}

#endif

#if !_PM_CLOCK_PWM
// Unlike timers on other devices, on RP2040 you don't reset a counter to
// zero at the start of a cycle. To emulate that behavior (for determining
// elapsed times), the timer start time must be saved somewhere...
static volatile uint32_t _PM_timerSave;

#endif

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
#define _PM_pinOutput(pin)                                                     \
  {                                                                            \
    gpio_init(pin);                                                            \
    gpio_set_dir(pin, GPIO_OUT);                                               \
  }
#define _PM_pinLow(pin) gpio_clr_mask(1UL << pin)
#define _PM_pinHigh(pin) gpio_set_mask(1UL << pin)

#ifndef _PM_delayMicroseconds
#define _PM_delayMicroseconds(n) sleep_us(n)
#endif

#if _PM_CLOCK_PWM // Use PWM for timing
static void _PM_PWM_ISR(void) {
  pwm_clear_irq(_PM_PWM_SLICE);  // Reset PWM wrap interrupt
  _PM_row_handler(_PM_protoPtr); // In core.c
}
#else // Use timer alarm for timing
static void _PM_timerISR(void) {
  hw_clear_bits(&timer_hw->intr, 1u << _PM_ALARM_NUM); // Clear alarm flag
  _PM_row_handler(_PM_protoPtr);                       // In core.c
}
#endif

// Set timer period and enable timer.
inline void _PM_timerStart(void *tptr, uint32_t period) {
#if _PM_CLOCK_PWM
  pwm_set_counter(_PM_PWM_SLICE, 0);
  pwm_set_wrap(_PM_PWM_SLICE, period);
  pwm_set_enabled(_PM_PWM_SLICE, true);
#else
  irq_set_enabled(_PM_IRQ_HANDLER, true);                  // Enable alarm IRQ
  _PM_timerSave = timer_hw->timerawl;                      // Time at start
  timer_hw->alarm[_PM_ALARM_NUM] = _PM_timerSave + period; // Time at end
#endif
}

// Return current count value (timer enabled or not).
// Timer must be previously initialized.
inline uint32_t _PM_timerGetCount(void *tptr) {
#if _PM_CLOCK_PWM
  return pwm_get_counter(_PM_PWM_SLICE);
#else
  return timer_hw->timerawl - _PM_timerSave;
#endif
}

// Disable timer and return current count value.
// Timer must be previously initialized.
uint32_t _PM_timerStop(void *tptr) {
#if _PM_CLOCK_PWM
  pwm_set_enabled(_PM_PWM_SLICE, false);
#else
  irq_set_enabled(_PM_IRQ_HANDLER, false); // Disable alarm IRQ
#endif
  return _PM_timerGetCount(tptr);
}

#define _PM_chunkSize 8
#define _PM_clockHoldLow asm("nop; nop;");
#if _PM_CLOCK_PWM
#define _PM_minMinPeriod 100
#else
#define _PM_minMinPeriod 8
#endif

#endif // END PICO_BOARD
