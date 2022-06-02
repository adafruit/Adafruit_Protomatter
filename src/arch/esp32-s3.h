/*!
 * @file esp32-s3.h
 *
 * Part of Adafruit's Protomatter library for HUB75-style RGB LED matrices.
 * This file contains ESP32-S3-SPECIFIC CODE.
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

#if defined(CONFIG_IDF_TARGET_ESP32S3)

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

// On ESP32-S3, use the LCD_CAM peripheral for fast parallel output.
// Thanks to ESP's pin MUXing, matrix data in RAM can ALWAYS be stored in
// byte format regardless how RGB+clock bits are distributed among pins.
#define _PM_bytesPerElement 1
#define _PM_byteOffset(pin) 0
#define _PM_wordOffset(pin) 0

#include <driver/periph_ctrl.h>
#include <esp_private/gdma.h>
#include <esp_rom_gpio.h>
#include <hal/dma_types.h>
#include <hal/gpio_hal.h>
#include <soc/lcd_cam_reg.h>
#include <soc/lcd_cam_struct.h>

// Override the behavior of _PM_portBitMask macro so instead of returning
// a 32-bit mask for a pin within its corresponding GPIO register, it instead
// returns a 7-bit mask for the pin within the LCD_CAM data order *IF* it's
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

static dma_descriptor_t desc;
static gdma_channel_handle_t dma_chan;
static volatile bool xfer_done = true; // DMA completion flag

// DMA callback for end-of-transfer interrupt
static IRAM_ATTR bool dma_callback(gdma_channel_handle_t dma_chan,
                                   gdma_event_data_t *event_data,
                                   void *user_data) {
  xfer_done = true;
  return true;
}

// LCD_CAM requires a complete replacement of the "blast" functions in order
// to use the DMA-based peripheral.
#define _PM_CUSTOM_BLAST // Disable blast_*() functions in core.c
IRAM_ATTR static void blast_byte(Protomatter_core *core, uint8_t *data) {
  //  while (!xfer_done); // Wait for completion of prior xfer
  gdma_reset(dma_chan);           // Stop any current DMA
  LCD_CAM.lcd_user.lcd_start = 0; // Stop any current LCD transfer

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

static void pinmux(int8_t pin, uint8_t signal) {
  esp_rom_gpio_connect_out_signal(pin, signal, false, false);
  gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[pin], PIN_FUNC_GPIO);
  gpio_set_drive_capability((gpio_num_t)pin, (gpio_drive_cap_t)3);
}

#if defined(ARDUINO) // COMPILING FOR ARDUINO ------------------------------

void _PM_timerInit(Protomatter_core *core) {
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
  const uint8_t signal[] = {LCD_DATA_OUT0_IDX, LCD_DATA_OUT1_IDX,
                            LCD_DATA_OUT2_IDX, LCD_DATA_OUT3_IDX,
                            LCD_DATA_OUT4_IDX, LCD_DATA_OUT5_IDX};
  for (int i = 0; i < 6; i++)
    pinmux(core->rgbPins[i], signal[i]);
  pinmux(core->clockPin, LCD_PCLK_IDX);

  // Configure frame format
  LCD_CAM.lcd_ctrl.lcd_rgb_mode_en = 0;    // i8080 mode (not RGB)
  LCD_CAM.lcd_rgb_yuv.lcd_conv_bypass = 0; // Disable RGB/YUV converter
  LCD_CAM.lcd_misc.lcd_next_frame_en = 0;  // Do NOT auto-frame
  LCD_CAM.lcd_data_dout_mode.val = 0;      // No data delays
  LCD_CAM.lcd_user.lcd_dout_cyclelen = core->chainBits;
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
  LCD_CAM.lcd_user.lcd_dummy = 1; // Enable dummy phase at LCD start
  LCD_CAM.lcd_user.lcd_cmd = 0;   // No command at LCD start
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
  gdma_strategy_config_t strategy_config = {.owner_check = false,
                                            .auto_update_desc = false};
  gdma_apply_strategy(dma_chan, &strategy_config);
  gdma_transfer_ability_t ability = {
      .sram_trans_align = 0,
      .psram_trans_align = 0,
  };
  gdma_set_transfer_ability(dma_chan, &ability);
  gdma_start(dma_chan, (intptr_t)&desc);

  // Enable DMA transfer callback
  gdma_tx_event_callbacks_t tx_cbs = {.on_trans_eof = dma_callback};
  gdma_register_tx_event_callbacks(dma_chan, &tx_cbs, NULL);

  _PM_esp32commonTimerInit(core); // In esp32-common.h
}

#elif defined(CIRCUITPY) // COMPILING FOR CIRCUITPYTHON --------------------

void _PM_timerInit(Protomatter_core *core) {

  // TO DO: adapt this function for any CircuitPython-specific changes.
  // If none are required, this function can be deleted and the version
  // above can be moved before the ARDUIO/CIRCUITPY checks. If minimal
  // changes, consider a single _PM_timerInit() implementation with
  // ARDUINO/CIRCUITPY checks inside.

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
  const uint8_t signal[] = {LCD_DATA_OUT0_IDX, LCD_DATA_OUT1_IDX,
                            LCD_DATA_OUT2_IDX, LCD_DATA_OUT3_IDX,
                            LCD_DATA_OUT4_IDX, LCD_DATA_OUT5_IDX};
  for (int i = 0; i < 6; i++)
    pinmux(core->rgbPins[i], signal[i]);
  pinmux(core->clockPin, LCD_PCLK_IDX);

  // Configure frame format
  LCD_CAM.lcd_ctrl.lcd_rgb_mode_en = 0;    // i8080 mode (not RGB)
  LCD_CAM.lcd_rgb_yuv.lcd_conv_bypass = 0; // Disable RGB/YUV converter
  LCD_CAM.lcd_misc.lcd_next_frame_en = 0;  // Do NOT auto-frame
  LCD_CAM.lcd_data_dout_mode.val = 0;      // No data delays
  LCD_CAM.lcd_user.lcd_dout_cyclelen = core->chainBits;
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
  LCD_CAM.lcd_user.lcd_dummy = 1; // Enable dummy phase at LCD start
  LCD_CAM.lcd_user.lcd_cmd = 0;   // No command at LCD start
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
  gdma_strategy_config_t strategy_config = {.owner_check = false,
                                            .auto_update_desc = false};
  gdma_apply_strategy(dma_chan, &strategy_config);
  gdma_transfer_ability_t ability = {
      .sram_trans_align = 0,
      .psram_trans_align = 0,
  };
  gdma_set_transfer_ability(dma_chan, &ability);
  gdma_start(dma_chan, (intptr_t)&desc);

  // Enable DMA transfer callback
  gdma_tx_event_callbacks_t tx_cbs = {.on_trans_eof = dma_callback};
  gdma_register_tx_event_callbacks(dma_chan, &tx_cbs, NULL);

  _PM_esp32commonTimerInit(core); // In esp32-common.h
}

#endif // END CIRCUITPYTHON ------------------------------------------------

#endif // END ESP32S3
