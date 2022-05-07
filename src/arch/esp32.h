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

// Oh wait - this is only true on S2/S3. On others, use orig line.
#define _PM_portOutRegister(pin)                                               \
  (volatile uint32_t *)((pin < 32) ? &GPIO.out : &GPIO.out1.val)
#define _PM_portSetRegister(pin)                                               \
  (volatile uint32_t *)((pin < 32) ? &GPIO.out_w1ts : &GPIO.out1_w1ts.val)
#define _PM_portClearRegister(pin)                                             \
  (volatile uint32_t *)((pin < 32) ? &GPIO.out_w1tc : &GPIO.out1_w1tc.val)

#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32S2)

static uint16_t _bit_set[128] = {
    0x0000, 0x0001, 0x0004, 0x0005, 0x0010, 0x0011, 0x0014, 0x0015, 0x0040,
    0x0041, 0x0044, 0x0045, 0x0050, 0x0051, 0x0054, 0x0055, 0x0100, 0x0101,
    0x0104, 0x0105, 0x0110, 0x0111, 0x0114, 0x0115, 0x0140, 0x0141, 0x0144,
    0x0145, 0x0150, 0x0151, 0x0154, 0x0155, 0x0400, 0x0401, 0x0404, 0x0405,
    0x0410, 0x0411, 0x0414, 0x0415, 0x0440, 0x0441, 0x0444, 0x0445, 0x0450,
    0x0451, 0x0454, 0x0455, 0x0500, 0x0501, 0x0504, 0x0505, 0x0510, 0x0511,
    0x0514, 0x0515, 0x0540, 0x0541, 0x0544, 0x0545, 0x0550, 0x0551, 0x0554,
    0x0555, 0x1000, 0x1001, 0x1004, 0x1005, 0x1010, 0x1011, 0x1014, 0x1015,
    0x1040, 0x1041, 0x1044, 0x1045, 0x1050, 0x1051, 0x1054, 0x1055, 0x1100,
    0x1101, 0x1104, 0x1105, 0x1110, 0x1111, 0x1114, 0x1115, 0x1140, 0x1141,
    0x1144, 0x1145, 0x1150, 0x1151, 0x1154, 0x1155, 0x1400, 0x1401, 0x1404,
    0x1405, 0x1410, 0x1411, 0x1414, 0x1415, 0x1440, 0x1441, 0x1444, 0x1445,
    0x1450, 0x1451, 0x1454, 0x1455, 0x1500, 0x1501, 0x1504, 0x1505, 0x1510,
    0x1511, 0x1514, 0x1515, 0x1540, 0x1541, 0x1544, 0x1545, 0x1550, 0x1551,
    0x1554, 0x1555,
};
static uint16_t _bit_clear[128] = {
    0x0000, 0x0002, 0x0008, 0x000A, 0x0020, 0x0022, 0x0028, 0x002A, 0x0080,
    0x0082, 0x0088, 0x008A, 0x00A0, 0x00A2, 0x00A8, 0x00AA, 0x0200, 0x0202,
    0x0208, 0x020A, 0x0220, 0x0222, 0x0228, 0x022A, 0x0280, 0x0282, 0x0288,
    0x028A, 0x02A0, 0x02A2, 0x02A8, 0x02AA, 0x0800, 0x0802, 0x0808, 0x080A,
    0x0820, 0x0822, 0x0828, 0x082A, 0x0880, 0x0882, 0x0888, 0x088A, 0x08A0,
    0x08A2, 0x08A8, 0x08AA, 0x0A00, 0x0A02, 0x0A08, 0x0A0A, 0x0A20, 0x0A22,
    0x0A28, 0x0A2A, 0x0A80, 0x0A82, 0x0A88, 0x0A8A, 0x0AA0, 0x0AA2, 0x0AA8,
    0x0AAA, 0x2000, 0x2002, 0x2008, 0x200A, 0x2020, 0x2022, 0x2028, 0x202A,
    0x2080, 0x2082, 0x2088, 0x208A, 0x20A0, 0x20A2, 0x20A8, 0x20AA, 0x2200,
    0x2202, 0x2208, 0x220A, 0x2220, 0x2222, 0x2228, 0x222A, 0x2280, 0x2282,
    0x2288, 0x228A, 0x22A0, 0x22A2, 0x22A8, 0x22AA, 0x2800, 0x2802, 0x2808,
    0x280A, 0x2820, 0x2822, 0x2828, 0x282A, 0x2880, 0x2882, 0x2888, 0x288A,
    0x28A0, 0x28A2, 0x28A8, 0x28AA, 0x2A00, 0x2A02, 0x2A08, 0x2A0A, 0x2A20,
    0x2A22, 0x2A28, 0x2A2A, 0x2A80, 0x2A82, 0x2A88, 0x2A8A, 0x2AA0, 0x2AA2,
    0x2AA8, 0x2AAA,
};
static uint16_t _bit_toggle[128] = {
    0x0000, 0x0003, 0x000C, 0x000F, 0x0030, 0x0033, 0x003C, 0x003F, 0x00C0,
    0x00C3, 0x00CC, 0x00CF, 0x00F0, 0x00F3, 0x00FC, 0x00FF, 0x0300, 0x0303,
    0x030C, 0x030F, 0x0330, 0x0333, 0x033C, 0x033F, 0x03C0, 0x03C3, 0x03CC,
    0x03CF, 0x03F0, 0x03F3, 0x03FC, 0x03FF, 0x0C00, 0x0C03, 0x0C0C, 0x0C0F,
    0x0C30, 0x0C33, 0x0C3C, 0x0C3F, 0x0CC0, 0x0CC3, 0x0CCC, 0x0CCF, 0x0CF0,
    0x0CF3, 0x0CFC, 0x0CFF, 0x0F00, 0x0F03, 0x0F0C, 0x0F0F, 0x0F30, 0x0F33,
    0x0F3C, 0x0F3F, 0x0FC0, 0x0FC3, 0x0FCC, 0x0FCF, 0x0FF0, 0x0FF3, 0x0FFC,
    0x0FFF, 0x3000, 0x3003, 0x300C, 0x300F, 0x3030, 0x3033, 0x303C, 0x303F,
    0x30C0, 0x30C3, 0x30CC, 0x30CF, 0x30F0, 0x30F3, 0x30FC, 0x30FF, 0x3300,
    0x3303, 0x330C, 0x330F, 0x3330, 0x3333, 0x333C, 0x333F, 0x33C0, 0x33C3,
    0x33CC, 0x33CF, 0x33F0, 0x33F3, 0x33FC, 0x33FF, 0x3C00, 0x3C03, 0x3C0C,
    0x3C0F, 0x3C30, 0x3C33, 0x3C3C, 0x3C3F, 0x3CC0, 0x3CC3, 0x3CCC, 0x3CCF,
    0x3CF0, 0x3CF3, 0x3CFC, 0x3CFF, 0x3F00, 0x3F03, 0x3F0C, 0x3F0F, 0x3F30,
    0x3F33, 0x3F3C, 0x3F3F, 0x3FC0, 0x3FC3, 0x3FCC, 0x3FCF, 0x3FF0, 0x3FF3,
    0x3FFC, 0x3FFF,
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
#define _PM_setReg(x) DEDIC_GPIO.gpio_out_idv.val = _bit_set[x.bit]
#define _PM_clearReg(x) DEDIC_GPIO.gpio_out_idv.val = _bit_clear[x.bit]

#include <driver/dedic_gpio.h>
#include <soc/dedic_gpio_reg.h>
#include <soc/dedic_gpio_struct.h>

#define _PM_portToggleRegister(pin)                                            \
  (volatile uint32_t *)(DEDIC_GPIO.gpio_out_idv.val)

// Override the behavior of _PM_portBitMask macro so instead of returning
// a 32-bit mask for a pin within its corresponding GPIO register, it instead
// returns a 7-bit mask for the pin within the Direct GPIO register...this
// requires comparing against pin numbers in the core struct.
static uint32_t _PM_directBitMask(Protomatter_core *core, int pin) {
  if (pin == core->clockPin) return 0x40;
  for (uint8_t i=0; i<7; i++) {
    if (pin == core->rgbPins[i]) return 1 << i;
  }
  return 0;
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
// This is what we'll use when toggling in the custom blast func:
//  (volatile uint8_t *)(DEDIC_GPIO.gpio_out_drt.gpio_out_drt_vlaue)
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
  hw_timer_t **timer = (hw_timer_t **)core->timer; // pointer-to-pointer
  if (timer == _PM_TIMER_DEFAULT) {
    *timer = timerBegin(_PM_timerNum, 2, true); // 1:2 prescale, count up
  }
  timerAttachInterrupt(*timer, &_PM_esp32timerCallback, true);

#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32S2)
  // On S2 & S3, initialize the Dedicated GPIO peripheral using the RGB
  // pin list from the core struct, plus the clock pin (7 pins total).
  int pins[7];
  for (uint8_t i=0; i<6; i++) pins[i] = core->rgbPins[i];
  pins[6] = core->clockPin;
  dedic_gpio_bundle_config_t config_in = {
    .gpio_array = pins, // Array of GPIO numbers
    .array_size = 7, // RGB pins + clock
    .flags = {
      .in_en = 0, // Disable input
      .out_en = 1, // Enable output
      .out_invert = 0, // Non-inverted
    }
  };
  dedic_gpio_bundle_handle_t bundle;
  (void)dedic_gpio_new_bundle(&config_in, &bundle);
  dedic_gpio_bundle_write(bundle, config_in.array_size, 1);
  DEDIC_GPIO.gpio_out_cpu.val = 0; // Use GPIO registers, not CPU instructions
#endif
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
