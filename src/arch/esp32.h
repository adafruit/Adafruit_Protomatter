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

#if defined(CONFIG_IDF_TARGET_ESP32C3)

#define _PM_portOutRegister(pin) (volatile uint32_t *)&GPIO.out
#define _PM_portSetRegister(pin) (volatile uint32_t *)&GPIO.out_w1ts
#define _PM_portClearRegister(pin) (volatile uint32_t *)&GPIO.out_w1tc

#else

// Is this needed? Maybe not.
#define _PM_STRICT_32BIT_IO ///< Change core.c behavior for long accesses only

// Oh wait - this is only true on S2/S3. On others, use orig line.
#define _PM_portOutRegister(pin)                                               \
  (volatile uint32_t *)((pin < 32) ? &GPIO.out : &GPIO.out1.val)
#define _PM_portSetRegister(pin)                                               \
  (volatile uint32_t *)((pin < 32) ? &GPIO.out_w1ts : &GPIO.out1_w1ts.val)
#define _PM_portClearRegister(pin)                                             \
  (volatile uint32_t *)((pin < 32) ? &GPIO.out_w1tc : &GPIO.out1_w1tc.val)

#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32S2)

#define _PM_bytesPerElement 1
// These all have the 7th bit (doubled up, because toggle) set on purpose,
// because that's how the code always works.
static uint16_t _bit_toggle[64] = {
    0x3000, 0x3003, 0x300C, 0x300F, 0x3030, 0x3033, 0x303C, 0x303F, 0x30C0,
    0x30C3, 0x30CC, 0x30CF, 0x30F0, 0x30F3, 0x30FC, 0x30FF, 0x3300, 0x3303,
    0x330C, 0x330F, 0x3330, 0x3333, 0x333C, 0x333F, 0x33C0, 0x33C3, 0x33CC,
    0x33CF, 0x33F0, 0x33F3, 0x33FC, 0x33FF, 0x3C00, 0x3C03, 0x3C0C, 0x3C0F,
    0x3C30, 0x3C33, 0x3C3C, 0x3C3F, 0x3CC0, 0x3CC3, 0x3CCC, 0x3CCF, 0x3CF0,
    0x3CF3, 0x3CFC, 0x3CFF, 0x3F00, 0x3F03, 0x3F0C, 0x3F0F, 0x3F30, 0x3F33,
    0x3F3C, 0x3F3F, 0x3FC0, 0x3FC3, 0x3FCC, 0x3FCF, 0x3FF0, 0x3FF3, 0x3FFC,
    0x3FFF,
};
// How to tell core code to always use 1 byte even if registers span
// PORTs?
// That's in core->bytesPerElement...and by the time our timer init func
// is called later, that value's already set and memory is already alloc'd.
// UNLESS _PM_portOutRegiser is fiddled to return same address for all,
// then it will alloc one byte.
// Need to define custom _PM_clearReg() and _PM_setReg() macros, and
// have core.c only define those if not already present.
// Ah! 'x' is a pin struct, so there should be a mask element?
// OH, OF COURSE. the gpio_out_idv register is specifically used
// by the RGB & clock bits. ALL other bits are on different
// registers and not through Direct GPIO periph.
//#define _PM_setReg(x) DEDIC_GPIO.gpio_out_idv.val = _bit_set[x.bit]
//#define _PM_clearReg(x) DEDIC_GPIO.gpio_out_idv.val = _bit_clear[x.bit]

#include <driver/dedic_gpio.h>
// These files are S2 only:
#include <soc/dedic_gpio_reg.h>
#include <soc/dedic_gpio_struct.h>
// Are there S3 equivs?

// Override the behavior of _PM_portBitMask macro so instead of returning
// a 32-bit mask for a pin within its corresponding GPIO register, it instead
// returns a 7-bit mask for the pin within the Direct GPIO register...this
// requires comparing against pin numbers in the core struct.
static uint32_t _PM_directBitMask(Protomatter_core *core, int pin) {
  if (pin == core->clockPin) return 0x40;
  for (uint8_t i=0; i<6; i++) {
    if (pin == core->rgbPins[i]) return 1 << i;
  }
  // Else return the bit that would normally be used for regular GPIO
  return (1U << (pin & 31));
}
// Thankfully, at present, any code which calls _PM_portBitMask() currently
// has a 'core' variable.
#define _PM_portBitMask(pin) _PM_directBitMask(core, pin)

// Because S2/S3 go through Dedicated GPIO, everythings always in one byte:
#define _PM_byteOffset(pin) 0
#define _PM_wordOffset(pin) 0
// Downside to Dedicated GPIO is we can only do one chain.
// Parallel chains require more bits...maybe change to I2S LCD for that.

// ESP32-S2 and S3 require a complete replacement of the "blast" functions
// in order to get sufficient speed.
#define _PM_CUSTOM_BLAST
IRAM_ATTR static void blast_byte(Protomatter_core *core, uint8_t *data) {
  volatile uint32_t *gpio = &DEDIC_GPIO.gpio_out_idv.val;

  // PORT has already been initialized with RGB data + clock bits
  // all LOW, so we don't need to initialize that state here.

// OK, with some chunk loop unrolling, this is starting to look
// promising. RGB and clock are toggling! However, OR, latch and
// address pins are not. Need to work on those.
  for (uint32_t bits = core->chainBits / 8; bits--; ) {
    *gpio = _bit_toggle[*data++]; // Toggle in new data + toggle clock low
    *gpio = 0b11000000000000;     // Toggle clock high
    *gpio = _bit_toggle[*data++];
    *gpio = 0b11000000000000;
    *gpio = _bit_toggle[*data++];
    *gpio = 0b11000000000000;
    *gpio = _bit_toggle[*data++];
    *gpio = 0b11000000000000;
    *gpio = _bit_toggle[*data++];
    *gpio = 0b11000000000000;
    *gpio = _bit_toggle[*data++];
    *gpio = 0b11000000000000;
    *gpio = _bit_toggle[*data++];
    *gpio = 0b11000000000000;
    *gpio = _bit_toggle[*data++];
    *gpio = 0b11000000000000;
  }

  // Want the PORT left with RGB data and clock LOW on function exit
  // (so it's easier to see on 'scope, and to prime it for the next call).
  // This is implicit in the no-toggle case (due to how the PEW macro
  // works), but toggle case requires explicitly clearing those bits.
  // rgbAndClockMask is an 8-bit value when toggling, hence offset here.
  *gpio = 0b10101010101010; // Clear RGB + clock bits
}
// If using custom "blast" function(s), all three must be declared.
// Unused ones can be empty, that's fine, just need to exist.
IRAM_ATTR static void blast_word(Protomatter_core *core, uint16_t *data) {}
IRAM_ATTR static void blast_long(Protomatter_core *core, uint32_t *data) {}

// Might not need this now:
//#define _PM_minMinPeriod 50 ///< Minimum timer interval for least bit

#else

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define _PM_byteOffset(pin) ((pin & 31) / 8)
#define _PM_wordOffset(pin) ((pin & 31) / 16)
#else
#define _PM_byteOffset(pin) (3 - ((pin & 31) / 8))
#define _PM_wordOffset(pin) (1 - ((pin & 31) / 16))
#endif

#endif // end ESP32S3/S2

#endif // end !ESP32C3

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
// S3 & S2 can use stock PEW define
#if !defined(CONFIG_IDF_TARGET_ESP32S3) && !defined(CONFIG_IDF_TARGET_ESP32S2)
#define PEW                                                                    \
  *set = *data++;         /* Set RGB data high */                              \
  *clear_full = 0;        /* ESP32 MUST sync before 2nd 'set' */               \
  *set_full = clock;      /* Set clock high */                                 \
  *clear_full = rgbclock; /* Clear RGB data + clock */                         \
  ///< Bitbang one set of RGB data bits to matrix
#endif // end !ESP32S3/S2

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

// Initialize, but do not start, timer (and any other peripherals).
void _PM_timerInit(Protomatter_core *core) {
#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32S2)
  // On S2 & S3, initialize the Dedicated GPIO peripheral using the RGB
  // pin list from the core struct, plus the clock pin (7 pins total).
  // Unsure if these structs & arrays need to be persistent. Declaring
  // static just in case, will remove that one by one.
  static int pins[7];
  for (uint8_t i=0; i<6; i++) pins[i] = core->rgbPins[i];
  pins[6] = core->clockPin;
  static dedic_gpio_bundle_config_t config_in = {
    .gpio_array = pins, // Array of GPIO numbers
    .array_size = 7, // RGB pins + clock pin
    .flags = {
      .in_en = 0, // Disable input
      .out_en = 1, // Enable output
      .out_invert = 0, // Non-inverted
    }
  };
  static dedic_gpio_bundle_handle_t bundle;
  (void)dedic_gpio_new_bundle(&config_in, &bundle);
  dedic_gpio_bundle_write(bundle, config_in.array_size, 1);
  DEDIC_GPIO.gpio_out_cpu.val = 0; // Use GPIO registers, not CPU instructions
#endif

  hw_timer_t **timer = (hw_timer_t **)core->timer; // pointer-to-pointer
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
#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32S2)
  timer->hw->hw_timer[timer->idx].update.tn_update = 1;
  return timer->hw->hw_timer[timer->idx].lo.tn_lo;
#else
  timer->hw->hw_timer[timer->idx].update.tx_update = 1;
  return timer->hw->hw_timer[timer->idx].lo.tx_lo;
#endif
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
