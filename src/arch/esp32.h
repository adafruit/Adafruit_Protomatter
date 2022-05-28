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

// Just FYI there's a bit of repetition in the defines/funcs for the
// different ESP32 variants. The if-this-then-that permutations started
// to get really complicated and it was cleaner just to break out each
// variant and replicate those items for each.

#include "driver/timer.h"

#if defined(CONFIG_IDF_TARGET_ESP32S2) // **********************************
// *** S2 *** S2 *** S2 *** S2 *** S2 *** S2 *** S2 *** S2 *** S2 *** S2 ***
//

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
// conventional GPIO.
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
// returns a 7-bit mask for the pin within the Direct GPIO register...this
// requires comparing against pin numbers in the core struct.
static uint32_t _PM_directBitMask(Protomatter_core *core, int pin) {
  if (pin == core->clockPin) return 1 << 6;
  for (uint8_t i=0; i<6; i++) {
    if (pin == core->rgbPins[i]) return 1 << i;
  }
  // Else return the bit that would normally be used for regular GPIO
  return (1U << (pin & 31));
}
// Thankfully, at present, any code which calls _PM_portBitMask() currently
// has a 'core' variable, so we can do this...
#define _PM_portBitMask(pin) _PM_directBitMask(core, pin)

// Dedicated GPIO requires a complete replacement of the "blast" functions
// in order to get sufficient speed.
#define _PM_CUSTOM_BLAST // Disable blast_*() functions in core.c
IRAM_ATTR static void blast_byte(Protomatter_core *core, uint8_t *data) {
  volatile uint32_t *gpio = &DEDIC_GPIO.gpio_out_idv.val;

  // GPIO has already been initialized with RGB data + clock bits
  // all LOW, so we don't need to initialize that state here.

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

//
// ***** END S2 *** END S2 *** END S2 *** END S2 *** END S2 *** END S2 *****
#elif defined(CONFIG_IDF_TARGET_ESP32S3) // ********************************
// *** S3 *** S3 *** S3 *** S3 *** S3 *** S3 *** S3 *** S3 *** S3 *** S3 ***
//

#define _PM_portOutRegister(pin)                                               \
  (volatile uint32_t *)((pin < 32) ? &GPIO.out : &GPIO.out1.val)
#define _PM_portSetRegister(pin)                                               \
  (volatile uint32_t *)((pin < 32) ? &GPIO.out_w1ts : &GPIO.out1_w1ts.val)
#define _PM_portClearRegister(pin)                                             \
  (volatile uint32_t *)((pin < 32) ? &GPIO.out_w1tc : &GPIO.out1_w1tc.val)

// On ESP32-S3, use the LCD_CAM peripheral for fast parallel output.
// Thanks to ESP's pin MUXing, matrix data in RAM can ALWAYS be stored in
// byte format regardless how RGB+clock bits are distributed among pins.
#define _PM_bytesPerElement 1
#define _PM_byteOffset(pin) 0
#define _PM_wordOffset(pin) 0

#include <soc/lcd_cam_reg.h>
#include <soc/lcd_cam_struct.h>
#include <hal/dma_types.h>
#include <esp_private/gdma.h>
#include <hal/gpio_hal.h>
#include <esp_rom_gpio.h>
#include <driver/periph_ctrl.h>

// Override the behavior of _PM_portBitMask macro so instead of returning
// a 32-bit mask for a pin within its corresponding GPIO register, it instead
// returns a 6-bit mask for the pin in the LCD_CAM output.
static uint32_t _PM_directBitMask(Protomatter_core *core, int pin) {
  if (pin == core->clockPin) return 1 << 6;
  for (uint8_t i=0; i<6; i++) {
    if (pin == core->rgbPins[i]) return 1 << i;
  }
  // Else return the bit that would normally be used for regular GPIO
  return (1U << (pin & 31));
}

// Thankfully, at present, any code which calls _PM_portBitMask() currently
// has a 'core' variable, so we can do this...
#define _PM_portBitMask(pin) _PM_directBitMask(core, pin)

// DMA buffer, one row (64 columns x 00RGBRGB)
// Probably doesn't need alignment, but trying anyway
static dma_descriptor_t desc;
static gdma_channel_handle_t dma_chan;
static volatile bool xfer_done = true; // DMA completion flag

// DMA callback for end-of-transfer interrupt
static IRAM_ATTR bool dma_callback(gdma_channel_handle_t dma_chan,
  gdma_event_data_t *event_data, void *user_data) {
  xfer_done = true;
  return true;
}

// LCD_CAM requires a complete replacement of the "blast" functions in order
// to use the DMA-based peripheral.
#define _PM_CUSTOM_BLAST // Disable blast_*() functions in core.c
IRAM_ATTR static void blast_byte(Protomatter_core *core, uint8_t *data) {
//  while (!xfer_done); // Wait for completion of prior xfer
gdma_reset(dma_chan);                 // Stop any current DMA
LCD_CAM.lcd_user.lcd_start = 0;       // Stop any current LCD transfer

  LCD_CAM.lcd_misc.lcd_afifo_reset = 1; // Reset TX FIFO (required)

  // Re-init parts of descriptor each time (required)
  desc.dw0.size = desc.dw0.length = core->chainBits;
  desc.buffer = data;

  xfer_done = false;
  gdma_start(dma_chan, (intptr_t)&desc);
  esp_rom_delay_us(1); // Necessary? Another ESP32 example suggests so

  // This begins the DMA xfer out the LCD periph...
  LCD_CAM.lcd_user.lcd_start = 1;
}

// If using custom "blast" function(s), all three must be declared.
// Unused ones can be empty, that's fine, just need to exist.
IRAM_ATTR static void blast_word(Protomatter_core *core, uint16_t *data) {}
IRAM_ATTR static void blast_long(Protomatter_core *core, uint32_t *data) {}

//
// ***** END S3 *** END S3 *** END S3 *** END S3 *** END S3 *** END S3 *****
#elif defined(CONFIG_IDF_TARGET_ESP32C3) // ********************************
// *** C3 *** C3 *** C3 *** C3 *** C3 *** C3 *** C3 *** C3 *** C3 *** C3 ***
//

#define _PM_portOutRegister(pin) (volatile uint32_t *)&GPIO.out
#define _PM_portSetRegister(pin) (volatile uint32_t *)&GPIO.out_w1ts
#define _PM_portClearRegister(pin) (volatile uint32_t *)&GPIO.out_w1tc

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define _PM_byteOffset(pin) ((pin & 31) / 8)
#define _PM_wordOffset(pin) ((pin & 31) / 16)
#else
#define _PM_byteOffset(pin) (3 - ((pin & 31) / 8))
#define _PM_wordOffset(pin) (1 - ((pin & 31) / 16))
#endif

//
// ***** END C3 *** END C3 *** END C3 *** END C3 *** END C3 *** END C3 *****
#else // *******************************************************************
// *** OG *** OG *** OG *** OG *** OG *** OG *** OG *** OG *** OG *** OG ***
//

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

//
// ***** END OG *** END OG *** END OG *** END OG *** END OG *** END OG *****
#endif // End S2/S3/C3/OG **************************************************

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

static void pinmux(int8_t pin, uint8_t signal) {
  esp_rom_gpio_connect_out_signal(pin, signal, false, false);
  gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[pin], PIN_FUNC_GPIO);
  gpio_set_drive_capability((gpio_num_t)pin, (gpio_drive_cap_t)3);
}

// Initialize, but do not start, timer (and any other peripherals).
void _PM_timerInit(Protomatter_core *core) {
#if defined(CONFIG_IDF_TARGET_ESP32S2)

  // On S2, initialize the Dedicated GPIO peripheral using the RGB pin list
  // list from the core struct, plus the clock pin (7 pins total). Unsure if
  // these structs & arrays need to be persistent. Declaring static just in
  // case...could experiment with removing one by one.
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

#elif defined(CONFIG_IDF_TARGET_ESP32S3)

  // On S3, initialize the LCD_CAM peripheral and DMA.

  // LCD_CAM isn't enabled by default -- MUST begin with this:
  periph_module_enable(PERIPH_LCD_CAM_MODULE);
  periph_module_reset(PERIPH_LCD_CAM_MODULE);

  // Configure LCD clock
  LCD_CAM.lcd_clock.clk_en = 1;             // Enable clock
  LCD_CAM.lcd_clock.lcd_clk_sel = 3;        // PLL160M source
  LCD_CAM.lcd_clock.lcd_clkm_div_a = 0;     // 1/1 fractional divide,
  LCD_CAM.lcd_clock.lcd_clkm_div_b = 1;     // plus '7' below yields...
  LCD_CAM.lcd_clock.lcd_clkm_div_num = 7;   // 1:8 prescale (20 MHz CLK)
  LCD_CAM.lcd_clock.lcd_ck_out_edge = 0;    // PCLK low in first half of cycle
  LCD_CAM.lcd_clock.lcd_ck_idle_edge = 0;   // PCLK low idle
  LCD_CAM.lcd_clock.lcd_clk_equ_sysclk = 1; // PCLK = CLK (ignore CLKCNT_N)

  // Configure signal pins
  // IN THEORY this could be expanded to support 2 parallel chains,
  // but the rest of the LCD & DMA setup isn't currently written for
  // that, so it's limited to a single chain.
  const uint8_t signal[] = {
    LCD_DATA_OUT0_IDX,
    LCD_DATA_OUT1_IDX,
    LCD_DATA_OUT2_IDX,
    LCD_DATA_OUT3_IDX,
    LCD_DATA_OUT4_IDX,
    LCD_DATA_OUT5_IDX };
  for (int i = 0; i < 6; i++) pinmux(core->rgbPins[i], signal[i]);
  pinmux(core->clockPin, LCD_PCLK_IDX);

  // Configure frame format
  LCD_CAM.lcd_ctrl.lcd_rgb_mode_en = 0;    // i8080 mode (not RGB)
  LCD_CAM.lcd_rgb_yuv.lcd_conv_bypass = 0; // Disable RGB/YUV converter
  LCD_CAM.lcd_misc.lcd_next_frame_en = 0;  // Do NOT auto-frame
  LCD_CAM.lcd_data_dout_mode.val = 0;      // No data delays
  LCD_CAM.lcd_user.lcd_dout_cyclelen = core->chainBits - 1;
  LCD_CAM.lcd_user.lcd_always_out_en = 0;
  LCD_CAM.lcd_user.lcd_8bits_order = 0; // Do not swap bytes
  LCD_CAM.lcd_user.lcd_bit_order = 0;   // Do not reverse bit order
  LCD_CAM.lcd_user.lcd_2byte_en = 0;    // 8-bit data mode
  LCD_CAM.lcd_user.lcd_dout = 1;        // Enable data out
  // MUST enable at least one dummy phase at start of output, else clock
  // and data are intermittently misaligned. Since HUB75 is just a shift
  // register, the extra clock tick is harmless and the zero-data shifts
  // off the end of the chain. Unsure if this is a lack of understanding
  // on my part, a documentation issue, or silicon errata. Regardless,
  // harmless now that it's known.
  LCD_CAM.lcd_user.lcd_dummy = 1;       // Enable dummy phase at LCD start
  LCD_CAM.lcd_user.lcd_cmd = 0;         // No command at LCD start
  LCD_CAM.lcd_user.lcd_dummy_cyclelen = 0;
  LCD_CAM.lcd_user.lcd_cmd_2_cycle_en = 0;

  LCD_CAM.lcd_user.lcd_update = 1; // Update registers before 1st xfer

  // Set up DMA TX descriptor
  desc.dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
  desc.dw0.suc_eof = 1;
  // Size, length and buffer are set on each DMA setup
  desc.next = NULL;

  // Alloc DMA channel & connect it to LCD periph
  gdma_channel_alloc_config_t dma_chan_config = {
    .direction = GDMA_CHANNEL_DIRECTION_TX,
  };
  esp_err_t ret = gdma_new_channel(&dma_chan_config, &dma_chan);
  gdma_connect(dma_chan, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_LCD, 0));
  gdma_strategy_config_t strategy_config = {
    .owner_check = false,
    .auto_update_desc = false
  };
  gdma_apply_strategy(dma_chan, &strategy_config);
  gdma_transfer_ability_t ability = {
    .sram_trans_align = 0,
    .psram_trans_align = 0,
  };
  gdma_set_transfer_ability(dma_chan, &ability);
  gdma_start(dma_chan, (intptr_t)&desc);

  // Enable DMA transfer callback
  gdma_tx_event_callbacks_t tx_cbs = {
    .on_trans_eof = dma_callback
  };
  gdma_register_tx_event_callbacks(dma_chan, &tx_cbs, NULL);

#endif // end ESP32S2/S3

  // Timer stuff is the same for all ESP32 variants...

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
