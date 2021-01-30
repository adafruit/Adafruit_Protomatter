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
 * broadest portability (using PWM as a timer in this case). While not
 * entirely optimal, it's not pessimal either...no worse than any other
 * platform where we're not taking advantage of device-specific DMA or
 * peripherals. Would require changes to the 'blast' functions or possibly
 * the whole _PM_row_handler() (both currently in core.c). CPU load is just
 * a few percent for a 64x32 matrix @ 6-bit depth.
 *
 */

#pragma once

// TO DO: PUT A *PROPER* RP2040 CHECK HERE
#if defined(PICO_BOARD)

#include "../../hardware_pwm/include/hardware/pwm.h"
#include "hardware/irq.h"
#include "pico/stdlib.h" // For sio_hw, etc.

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

// Arduino implementation is tied to a specific PWM slice & frequency
#define _PM_PWM_SLICE 0
#define _PM_PWM_DIV 3 // ~41.6 MHz, similar to SAMD
#define _PM_timerFreq (125000000 / _PM_PWM_DIV)
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
#define _PM_pinOutput(pin)                                                     \
  {                                                                            \
    gpio_init(pin);                                                            \
    gpio_set_dir(pin, GPIO_OUT);                                               \
  }
#define _PM_pinLow(pin) gpio_clr_mask(1UL << pin)
#define _PM_pinHigh(pin) gpio_set_mask(1UL << pin)

#define _PM_delayMicroseconds(n) sleep_us(n)

static void _PM_PWM_ISR(void) {
  pwm_clear_irq(_PM_PWM_SLICE);  // Reset PWM wrap interrupt
  _PM_row_handler(_PM_protoPtr); // In core.c
}

// Initialize, but do not start, timer.
void _PM_timerInit(void *tptr) {

  // Enable PWM wrap interrupt
  pwm_clear_irq(_PM_PWM_SLICE);
  pwm_set_irq_enabled(_PM_PWM_SLICE, true);
  irq_set_exclusive_handler(PWM_IRQ_WRAP, _PM_PWM_ISR);
  irq_set_enabled(PWM_IRQ_WRAP, true);

  // Config but do not start PWM
  pwm_config config = pwm_get_default_config();
  pwm_config_set_clkdiv_int(&config, _PM_PWM_DIV);
  pwm_init(_PM_PWM_SLICE, &config, true);
}

// Set timer period and enable timer.
inline void _PM_timerStart(void *tptr, uint32_t period) {
  pwm_set_counter(_PM_PWM_SLICE, 0);
  pwm_set_wrap(_PM_PWM_SLICE, period);
  pwm_set_enabled(_PM_PWM_SLICE, true);
}

// Return current count value (timer enabled or not).
// Timer must be previously initialized.
inline uint32_t _PM_timerGetCount(void *tptr) {
  return pwm_get_counter(_PM_PWM_SLICE);
}

// Disable timer and return current count value.
// Timer must be previously initialized.
uint32_t _PM_timerStop(void *tptr) {
  pwm_set_enabled(_PM_PWM_SLICE, false);
  return _PM_timerGetCount(tptr);
}

#elif defined(CIRCUITPY) // COMPILING FOR CIRCUITPYTHON --------------------

// RP2040 CircuitPython magic goes here.
// Put anything common to Arduino & CircuitPython ABOVE the platform ifdefs.

#endif // END CIRCUITPYTHON ------------------------------------------------

#define _PM_chunkSize 8
#define _PM_clockHoldLow asm("nop; nop;");
#define _PM_minMinPeriod 100

#endif // END PICO_BOARD
