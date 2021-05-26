/*!
 * @file esp32.h
 *
 * Part of Adafruit's Protomatter library for HUB75-style RGB LED matrices.
 * This file contains ESP32-SPECIFIC CODE.
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

#if defined(ESP32)

#include "driver/timer.h"

#define _PM_portOutRegister(pin)                                               \
  (volatile uint32_t *)((pin < 32) ? &GPIO.out : &GPIO.out1.val)

#define _PM_portSetRegister(pin)                                               \
  (volatile uint32_t *)((pin < 32) ? &GPIO.out_w1ts : &GPIO.out1_w1ts.val)

#define _PM_portClearRegister(pin)                                             \
  (volatile uint32_t *)((pin < 32) ? &GPIO.out_w1tc : &GPIO.out1_w1tc.val)

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define _PM_byteOffset(pin) ((pin & 31) / 8)
#define _PM_wordOffset(pin) ((pin & 31) / 16)
#else
#define _PM_byteOffset(pin) (3 - ((pin & 31) / 8))
#define _PM_wordOffset(pin) (1 - ((pin & 31) / 16))
#endif

// As written, because it's tied to a specific timer right now, the
// Arduino lib only permits one instance of the Protomatter_core struct,
// which it sets up when calling begin().
void *_PM_protoPtr = NULL;

#define _PM_timerFreq 40000000 // 40 MHz (1:2 prescale)

#if defined(ARDUINO) // COMPILING FOR ARDUINO ------------------------------

// ESP32 requires a custom PEW declaration (issues one set of RGB color bits
// followed by clock pulse). Turns out the bit set/clear registers are not
// actually atomic. If two writes are made in quick succession, the second
// has no effect. One option is NOPs, other is to write a 0 (no effect) to
// the opposing register (set vs clear) to synchronize the next write.
#define PEW                                                                    \
  *set = *data++;         /* Set RGB data high */                              \
  *clear_full = 0;        /* ESP32 MUST sync before 2nd 'set' */               \
  *set_full = clock;      /* Set clock high */                                 \
  *clear_full = rgbclock; /* Clear RGB data + clock */                         \
  ///< Bitbang one set of RGB data bits to matrix

#define _PM_timerNum 0 // Timer #0 (can be 0-3)

// This is the default aforementioned singular timer. IN THEORY, other
// timers could be used, IF an Arduino sketch passes the address of its
// own hw_timer_t* to the Protomatter constructor and initializes that
// timer using ESP32's timerBegin(). All of the timer-related functions
// below pass around a handle rather than accessing _PM_esp32timer
// directly, in case that's ever actually used in the future.
static hw_timer_t *_PM_esp32timer = NULL;
#define _PM_TIMER_DEFAULT &_PM_esp32timer

extern IRAM_ATTR void _PM_row_handler(Protomatter_core *core);

// Timer interrupt handler. This, _PM_row_handler() and any functions
// called by _PM_row_handler() should all have the IRAM_ATTR attribute
// (RAM-resident functions). This isn't really the ISR itself, but a
// callback invoked by the real ISR (in arduino-esp32's esp32-hal-timer.c)
// which takes care of interrupt status bits & such.
IRAM_ATTR static void _PM_esp32timerCallback(void) {
  _PM_row_handler(_PM_protoPtr); // In core.c
}

// Initialize, but do not start, timer.
void _PM_timerInit(void *tptr) {
  hw_timer_t **timer = (hw_timer_t **)tptr; // pointer-to-pointer
  if (timer == _PM_TIMER_DEFAULT) {
    *timer = timerBegin(_PM_timerNum, 2, true); // 1:2 prescale, count up
  }
  timerAttachInterrupt(*timer, &_PM_esp32timerCallback, true);
}

// Set timer period, initialize count value to zero, enable timer.
IRAM_ATTR inline void _PM_timerStart(void *tptr, uint32_t period) {
  hw_timer_t *timer = *(hw_timer_t **)tptr;
  timerAlarmWrite(timer, period, true);
  timerAlarmEnable(timer);
  timerStart(timer);
}

// Return current count value (timer enabled or not).
// Timer must be previously initialized.
IRAM_ATTR inline uint32_t _PM_timerGetCount(void *tptr) {
  hw_timer_t *timer = *(hw_timer_t **)tptr;
  return (uint32_t)timerRead(timer);
}

// Disable timer and return current count value.
// Timer must be previously initialized.
IRAM_ATTR uint32_t _PM_timerStop(void *tptr) {
  hw_timer_t *timer = *(hw_timer_t **)tptr;
  timerStop(timer);
  return _PM_timerGetCount(tptr);
}

#elif defined(CIRCUITPY) // COMPILING FOR CIRCUITPYTHON --------------------

// ESP32 CircuitPython magic goes here. If any of the above Arduino-specific
// defines, structs or functions are useful as-is, don't copy them, just
// move them above the ARDUINO check so fixes/changes carry over, thx.

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

#include "driver/gpio.h"
#include "hal/timer_ll.h"
#include "peripherals/timer.h"

#define _PM_STRICT_32BIT_IO (1)

#define _PM_TIMER_DEFAULT NULL

#define _PM_pinOutput(pin) gpio_set_direction((pin), GPIO_MODE_OUTPUT)

#define _PM_pinLow(pin) gpio_set_level((pin), false)

#define _PM_pinHigh(pin) gpio_set_level((pin), true)

#define _PM_portBitMask(pin) (1U << ((pin)&31))

// Timer interrupt handler. This, _PM_row_handler() and any functions
// called by _PM_row_handler() should all have the IRAM_ATTR attribute
// (RAM-resident functions). This isn't really the ISR itself, but a
// callback invoked by the real ISR (in arduino-esp32's esp32-hal-timer.c)
// which takes care of interrupt status bits & such.
IRAM_ATTR bool _PM_esp32timerCallback(void *unused) {
  if (_PM_protoPtr) {
    _PM_row_handler(_PM_protoPtr); // In core.c
  }
  return false;
};

// Initialize, but do not start, timer.
void _PM_timerInit(void *tptr) {
  const timer_config_t config = {
      .alarm_en = false,
      .counter_en = false,
      .intr_type = TIMER_INTR_LEVEL,
      .counter_dir = TIMER_COUNT_UP,
      .auto_reload = true,
      .divider = 2 // 40MHz
  };

  timer_index_t *timer = (timer_index_t *)tptr;
  timer_init(timer->group, timer->idx, &config);
  timer_isr_callback_add(timer->group, timer->idx, _PM_esp32timerCallback, NULL,
                         0);
  timer_enable_intr(timer->group, timer->idx);
}

// Set timer period, initialize count value to zero, enable timer.
IRAM_ATTR void _PM_timerStart(void *tptr, uint32_t period) {
  timer_index_t *timer = (timer_index_t *)tptr;
  timer_ll_set_counter_enable(timer->hw, timer->idx, false);
  timer_ll_set_counter_value(timer->hw, timer->idx, 0);
  timer_ll_set_alarm_value(timer->hw, timer->idx, period);
  timer_ll_set_alarm_enable(timer->hw, timer->idx, true);
  timer_ll_set_counter_enable(timer->hw, timer->idx, true);
}

IRAM_ATTR uint32_t _PM_timerGetCount(void *tptr) {
  timer_index_t *timer = (timer_index_t *)tptr;
  timer->hw->hw_timer[timer->idx].update.update = 1;
  return timer->hw->hw_timer[timer->idx].cnt_low;
}

// Disable timer and return current count value.
// Timer must be previously initialized.
IRAM_ATTR uint32_t _PM_timerStop(void *tptr) {
  timer_index_t *timer = (timer_index_t *)tptr;
  timer_ll_set_counter_enable(timer->hw, timer->idx, false);
  return _PM_timerGetCount(tptr);
}

#endif // END CIRCUITPYTHON ------------------------------------------------

#endif // END ESP32
