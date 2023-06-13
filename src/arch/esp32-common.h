/*!
 * @file esp32-common.h
 *
 * Part of Adafruit's Protomatter library for HUB75-style RGB LED matrices.
 * This file contains ESP32-SPECIFIC CODE (common to all ESP variants).
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

#if defined(ESP32) ||                                                          \
    defined(ESP_PLATFORM) // *All* ESP32 variants (OG, S2, S3, etc.)

#include "esp_idf_version.h"

// NOTE: there is some intentional repetition in the macros and functions
// for some ESP32 variants. Previously they were all one file, but complex
// preprocessor directives were turning into spaghetti. THEREFORE, if making
// a change or bugfix in one variant-specific header, check the others to
// see if the same should be applied!

#include "driver/timer.h"
#include "soc/gpio_periph.h"

// As currently written, only one instance of the Protomatter_core struct
// is allowed, set up when calling begin()...so it's just a global here:
Protomatter_core *_PM_protoPtr;

#define _PM_timerFreq 40000000 // 40 MHz (1:2 prescale)

#if defined(ARDUINO) // COMPILING FOR ARDUINO ------------------------------

#define _PM_timerNum 0 // Timer #0 (can be 0-3)

// The following defines and functions are common to all ESP32 variants in
// the Arduino platform. Anything unique to one variant (or a subset of
// variants) is declared in the corresponding esp32-*.h header(s); please
// no #if defined(CONFIG_IDF_TARGET_*) action here...if you find yourself
// started down that path, it's okay, but move the code out of here and
// into the variant-specific headers.

// This is the default aforementioned singular timer. IN THEORY, other
// timers could be used, IF an Arduino sketch passes the address of its
// own hw_timer_t* to the Protomatter constructor and initializes that
// timer using ESP32's timerBegin(). All of the timer-related functions
// below pass around a handle rather than accessing _PM_esp32timer directly,
// in case that's ever actually used in the future.
static hw_timer_t *_PM_esp32timer = NULL;
#define _PM_TIMER_DEFAULT &_PM_esp32timer

extern IRAM_ATTR void _PM_row_handler(Protomatter_core *core); // In core.c

// Timer interrupt handler. This, _PM_row_handler() and any functions
// called by _PM_row_handler() should all have the IRAM_ATTR attribute
// (RAM-resident functions). This isn't really the ISR itself, but a
// callback invoked by the real ISR (in arduino-esp32's esp32-hal-timer.c)
// which takes care of interrupt status bits & such.
IRAM_ATTR static void _PM_esp32timerCallback(void) {
  _PM_row_handler(_PM_protoPtr); // In core.c
}

// Set timer period, initialize count value to zero, enable timer.
IRAM_ATTR inline void _PM_timerStart(Protomatter_core *core, uint32_t period) {
  hw_timer_t *timer = (hw_timer_t *)core->timer;
  timerAlarmWrite(timer, period, true);
  timerAlarmEnable(timer);
  timerStart(timer);
}

// Disable timer and return current count value.
// Timer must be previously initialized.
IRAM_ATTR uint32_t _PM_timerStop(Protomatter_core *core) {
  timerStop((hw_timer_t *)core->timer);
  return _PM_timerGetCount(core);
}

// Initialize, but do not start, timer. This function contains timer setup
// that's common to all ESP32 variants; code in variant-specific files might
// set up its own special peripherals, then call this.
void _PM_esp32commonTimerInit(Protomatter_core *core) {
  hw_timer_t *timer = (hw_timer_t *)core->timer; // pointer-to-pointer
  if (timer == _PM_TIMER_DEFAULT) {
    core->timer = timerBegin(_PM_timerNum, 2, true); // 1:2 prescale, count up
  }
  timerAttachInterrupt(timer, &_PM_esp32timerCallback, true);
}

#elif defined(CIRCUITPY) // COMPILING FOR CIRCUITPYTHON --------------------

// The following defines and functions are common to all ESP32 variants in
// the CircuitPython platform. Anything unique to one variant (or a subset
// of variants) is declared in the corresponding esp32-*.h header(s);
// please no #if defined(CONFIG_IDF_TARGET_*) action here...if you find
// yourself started down that path, it's okay, but move the code out of
// here and into the variant-specific headers.

#include "driver/gpio.h"
#include "esp_idf_version.h"
#include "hal/timer_ll.h"
#include "peripherals/timer.h"

#define _PM_TIMER_DEFAULT NULL
#define _PM_pinOutput(pin) gpio_set_direction((pin), GPIO_MODE_OUTPUT)
#define _PM_pinLow(pin) gpio_set_level((pin), false)
#define _PM_pinHigh(pin) gpio_set_level((pin), true)

// Timer interrupt handler. This, _PM_row_handler() and any functions
// called by _PM_row_handler() should all have the IRAM_ATTR attribute
// (RAM-resident functions). This isn't really the ISR itself, but a
// callback invoked by the real ISR (in arduino-esp32's esp32-hal-timer.c)
// which takes care of interrupt status bits & such.
static IRAM_ATTR bool _PM_esp32timerCallback(void *unused) {
  if (_PM_protoPtr) {
    _PM_row_handler(_PM_protoPtr); // In core.c
  }
  return false;
};

// Set timer period, initialize count value to zero, enable timer.
#if (ESP_IDF_VERSION_MAJOR == 5)
static IRAM_ATTR void _PM_timerStart(Protomatter_core *core, uint32_t period) {
  timer_index_t *timer = (timer_index_t *)core->timer;
  timer_ll_enable_counter(timer->hw, timer->idx, false);
  timer_ll_set_reload_value(timer->hw, timer->idx, 0);
  timer_ll_trigger_soft_reload(timer->hw, timer->idx);
  timer_ll_set_alarm_value(timer->hw, timer->idx, period);
  timer_ll_enable_alarm(timer->hw, timer->idx, true);
  timer_ll_enable_counter(timer->hw, timer->idx, true);
}
#else
IRAM_ATTR void _PM_timerStart(Protomatter_core *core, uint32_t period) {
  timer_index_t *timer = (timer_index_t *)core->timer;
  timer_ll_set_counter_enable(timer->hw, timer->idx, false);
  timer_ll_set_counter_value(timer->hw, timer->idx, 0);
  timer_ll_set_alarm_value(timer->hw, timer->idx, period);
  timer_ll_set_alarm_enable(timer->hw, timer->idx, true);
  timer_ll_set_counter_enable(timer->hw, timer->idx, true);
}
#endif

// Disable timer and return current count value.
// Timer must be previously initialized.
IRAM_ATTR uint32_t _PM_timerStop(Protomatter_core *core) {
  timer_index_t *timer = (timer_index_t *)core->timer;
#if (ESP_IDF_VERSION_MAJOR == 5)
  timer_ll_enable_counter(timer->hw, timer->idx, false);
#else
  timer_ll_set_counter_enable(timer->hw, timer->idx, false);
#endif
  return _PM_timerGetCount(core);
}

#if !defined(CONFIG_IDF_TARGET_ESP32S3)
IRAM_ATTR uint32_t _PM_timerGetCount(Protomatter_core *core) {
  timer_index_t *timer = (timer_index_t *)core->timer;
  uint64_t result;
  timer_ll_get_counter_value(timer->hw, timer->idx, &result);
  return (uint32_t)result;
}
#endif

// Initialize, but do not start, timer. This function contains timer setup
// that's common to all ESP32 variants; code in variant-specific files might
// set up its own special peripherals, then call this.
static void _PM_esp32commonTimerInit(Protomatter_core *core) {
  timer_index_t *timer = (timer_index_t *)core->timer;
  const timer_config_t config = {
      .alarm_en = false,
      .counter_en = false,
      .intr_type = TIMER_INTR_LEVEL,
      .counter_dir = TIMER_COUNT_UP,
      .auto_reload = true,
      .divider = 2 // 40MHz
  };

  timer_init(timer->group, timer->idx, &config);
  timer_isr_callback_add(timer->group, timer->idx, _PM_esp32timerCallback, NULL,
                         0);
  timer_enable_intr(timer->group, timer->idx);
}

#endif // END CIRCUITPYTHON ------------------------------------------------

#endif // END ESP32 (all variants)
