/*!
 * @file esp32-s2.h
 *
 * Part of Adafruit's Protomatter library for HUB75-style RGB LED matrices.
 * This file contains ESP32-S2-SPECIFIC CODE.
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

#if defined(CONFIG_IDF_TARGET_ESP32S2)

#define _PM_portOutRegister(pin)                                               \
  (volatile uint32_t *)((pin < 32) ? &GPIO.out : &GPIO.out1.val)
#define _PM_portSetRegister(pin)                                               \
  (volatile uint32_t *)((pin < 32) ? &GPIO.out_w1ts : &GPIO.out1_w1ts.val)
#define _PM_portClearRegister(pin)                                             \
  (volatile uint32_t *)((pin < 32) ? &GPIO.out_w1tc : &GPIO.out1_w1tc.val)

// On ESP32-S2, use the Dedicated GPIO peripheral, which allows faster bit-
// toggling than the conventional GPIO registers. Unfortunately NOT present
// on S3 or other ESP32 devices. Normal GPIO has a bottleneck where toggling
// a pin is limited to 8 MHz max. Dedicated GPIO can work around this and
// get over twice this rate, but requires some very weird hoops!
// Dedicated GPIO only supports 8-bit output, so parallel output isn't
// supported, but positives include that there's very flexible pin MUXing,
// so matrix data in RAM can ALWAYS be stored in byte format regardless how
// the RGB+clock bits are distributed among pins.
#define _PM_bytesPerElement 1
#define _PM_byteOffset(pin) 0
#define _PM_wordOffset(pin) 0

// Odd thing with Dedicated GPIO is that the bit-toggle operation is faster
// than bit set or clear (perhaps some underlying operation is atomic rather
// than read-modify-write). So, instruct core.c to format the matrix data in
// RAM as if we're using a port toggle register, even though
// _PM_portToggleRegister is NOT defined because ESP32 doesn't have that in
// conventional GPIO. Good times.
#define _PM_USE_TOGGLE_FORMAT

// This table is used to remap 7-bit (RGB+RGB+clock) data to the 2-bits-per
// GPIO format used by Dedicated GPIO. Bits corresponding to clock output
// are always set here, as we want that bit toggled low at the same time new
// RGB data is set up.
static uint16_t _bit_toggle[128] = {
    0x3000, 0x3003, 0x300C, 0x300F, 0x3030, 0x3033, 0x303C, 0x303F, 0x30C0,
    0x30C3, 0x30CC, 0x30CF, 0x30F0, 0x30F3, 0x30FC, 0x30FF, 0x3300, 0x3303,
    0x330C, 0x330F, 0x3330, 0x3333, 0x333C, 0x333F, 0x33C0, 0x33C3, 0x33CC,
    0x33CF, 0x33F0, 0x33F3, 0x33FC, 0x33FF, 0x3C00, 0x3C03, 0x3C0C, 0x3C0F,
    0x3C30, 0x3C33, 0x3C3C, 0x3C3F, 0x3CC0, 0x3CC3, 0x3CCC, 0x3CCF, 0x3CF0,
    0x3CF3, 0x3CFC, 0x3CFF, 0x3F00, 0x3F03, 0x3F0C, 0x3F0F, 0x3F30, 0x3F33,
    0x3F3C, 0x3F3F, 0x3FC0, 0x3FC3, 0x3FCC, 0x3FCF, 0x3FF0, 0x3FF3, 0x3FFC,
    0x3FFF, 0x3000, 0x3003, 0x300C, 0x300F, 0x3030, 0x3033, 0x303C, 0x303F,
    0x30C0, 0x30C3, 0x30CC, 0x30CF, 0x30F0, 0x30F3, 0x30FC, 0x30FF, 0x3300,
    0x3303, 0x330C, 0x330F, 0x3330, 0x3333, 0x333C, 0x333F, 0x33C0, 0x33C3,
    0x33CC, 0x33CF, 0x33F0, 0x33F3, 0x33FC, 0x33FF, 0x3C00, 0x3C03, 0x3C0C,
    0x3C0F, 0x3C30, 0x3C33, 0x3C3C, 0x3C3F, 0x3CC0, 0x3CC3, 0x3CCC, 0x3CCF,
    0x3CF0, 0x3CF3, 0x3CFC, 0x3CFF, 0x3F00, 0x3F03, 0x3F0C, 0x3F0F, 0x3F30,
    0x3F33, 0x3F3C, 0x3F3F, 0x3FC0, 0x3FC3, 0x3FCC, 0x3FCF, 0x3FF0, 0x3FF3,
    0x3FFC, 0x3FFF,
};

#include <driver/dedic_gpio.h>
#include <soc/dedic_gpio_reg.h>
#include <soc/dedic_gpio_struct.h>

// Override the behavior of _PM_portBitMask macro so instead of returning
// a 32-bit mask for a pin within its corresponding GPIO register, it instead
// returns a 7-bit mask for the pin within the Direct GPIO register *IF* it's
// one of the RGB bits or the clock bit...this requires comparing against pin
// numbers in the core struct.
static uint32_t _PM_directBitMask(Protomatter_core *core, int pin) {
  if (pin == core->clockPin)
    return 1 << 6;
  for (uint8_t i = 0; i < 6; i++) {
    if (pin == core->rgbPins[i])
      return 1 << i;
  }
  // Else return the bit that would normally be used for regular GPIO
  return (1U << (pin & 31));
}
// Thankfully, at present, any core code which calls _PM_portBitMask()
// currently has a 'core' variable, so we can safely do this...
#define _PM_portBitMask(pin) _PM_directBitMask(core, pin)

// Dedicated GPIO requires a complete replacement of the "blast" functions
// in order to get sufficient speed.
#define _PM_CUSTOM_BLAST // Disable blast_*() functions in core.c
IRAM_ATTR static void blast_byte(Protomatter_core *core, uint8_t *data) {
  volatile uint32_t *gpio = &DEDIC_GPIO.gpio_out_idv.val;

  // GPIO has already been initialized with RGB data + clock bits
  // all LOW, so we don't need to initialize that state here.

  for (uint32_t bits = core->chainBits / 8; bits--;) {
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

  // Want the pins left with RGB data and clock LOW on function exit
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

#if defined(ARDUINO) // COMPILING FOR ARDUINO ------------------------------

void _PM_timerInit(Protomatter_core *core) {

  // On S2, initialize the Dedicated GPIO peripheral using the RGB pin list
  // list from the core struct, plus the clock pin (7 pins total). Unsure if
  // these structs & arrays need to be persistent. Declaring static just in
  // case...could experiment with removing one by one.
  static int pins[7];
  for (uint8_t i = 0; i < 6; i++)
    pins[i] = core->rgbPins[i];
  pins[6] = core->clockPin;
  static dedic_gpio_bundle_config_t config_in = {
      .gpio_array = pins, // Array of GPIO numbers
      .array_size = 7,    // RGB pins + clock pin
      .flags = {
          .in_en = 0,      // Disable input
          .out_en = 1,     // Enable output
          .out_invert = 0, // Non-inverted
      }};
  static dedic_gpio_bundle_handle_t bundle;
  (void)dedic_gpio_new_bundle(&config_in, &bundle);
  dedic_gpio_bundle_write(bundle, config_in.array_size, 1);
  DEDIC_GPIO.gpio_out_cpu.val = 0; // Use GPIO registers, not CPU instructions

  _PM_esp32commonTimerInit(core); // In esp32-common.h
}

// Return current count value (timer enabled or not).
// Timer must be previously initialized.
// This function is the same on all ESP32 parts EXCEPT S3.
IRAM_ATTR inline uint32_t _PM_timerGetCount(Protomatter_core *core) {
  return (uint32_t)timerRead((hw_timer_t *)core->timer);
}

#elif defined(CIRCUITPY) // COMPILING FOR CIRCUITPYTHON --------------------

void _PM_timerInit(Protomatter_core *core) {

  // TO DO: adapt this function for any CircuitPython-specific changes.
  // If none are required, this function can be deleted and the version
  // above can be moved before the ARDUIO/CIRCUITPY checks. If minimal
  // changes, consider a single _PM_timerInit() implementation with
  // ARDUINO/CIRCUITPY checks inside.

  // On S2, initialize the Dedicated GPIO peripheral using the RGB pin list
  // list from the core struct, plus the clock pin (7 pins total). Unsure if
  // these structs & arrays need to be persistent. Declaring static just in
  // case...could experiment with removing one by one.
  static int pins[7];
  for (uint8_t i = 0; i < 6; i++)
    pins[i] = core->rgbPins[i];
  pins[6] = core->clockPin;
  static dedic_gpio_bundle_config_t config_in = {
      .gpio_array = pins, // Array of GPIO numbers
      .array_size = 7,    // RGB pins + clock pin
      .flags = {
          .in_en = 0,      // Disable input
          .out_en = 1,     // Enable output
          .out_invert = 0, // Non-inverted
      }};
  static dedic_gpio_bundle_handle_t bundle;
  (void)dedic_gpio_new_bundle(&config_in, &bundle);
  dedic_gpio_bundle_write(bundle, config_in.array_size, 1);
  DEDIC_GPIO.gpio_out_cpu.val = 0; // Use GPIO registers, not CPU instructions

  _PM_esp32commonTimerInit(core); // In esp32-common.h
}

#endif // END CIRCUITPYTHON ------------------------------------------------

#endif // END ESP32S2
