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

#define GPIO_DRIVE_STRENGTH GPIO_DRIVE_CAP_3
#define LCD_CLK_PRESCALE 9 // 8, 9, 10 allowed. Bit clock = 160 MHz / this.

#if defined(CIRCUITPY) // COMPILING FOR CIRCUITPYTHON --------------------
#include "components/esp_rom/include/esp_rom_sys.h"
#include "components/heap/include/esp_heap_caps.h"
#endif

// Use DMA-capable RAM (not PSRAM) for framebuffer:
#define _PM_allocate(x) heap_caps_malloc(x, MALLOC_CAP_DMA | MALLOC_CAP_8BIT)
#define _PM_free(x) heap_caps_free(x)

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

// On most architectures, _PM_timerGetCount() is used to measure bitbang
// speed for one scanline, which is then used for bitplane 0 time, and each
// subsequent plane doubles that. Since ESP32-S3 uses DMA and we don't have
// an end-of-transfer interrupt, we make an informed approximation.
// dmaSetupTime (measured in blast_byte()) measures the number of timer
// cycles to set up and trigger the DMA transfer...
static uint32_t dmaSetupTime = 100;
// ...then, the version of _PM_timerGetCount() here uses that as a starting
// point, plus the known constant DMA xfer speed (160/LCD_CLK_PRESCALE MHz)
// and timer frequency (40 MHz), to return an estimate of the one-scanline
// transfer time, from which everything is extrapolated:
IRAM_ATTR inline uint32_t _PM_timerGetCount(Protomatter_core *core) {
  // Time estimate seems to come in a little high, so the -10 here is an
  // empirically-derived fudge factor that may yield ever-so-slightly better
  // refresh in some edge cases. If visual glitches are encountered, might
  // need to dial back this number a bit or remove it.
  return dmaSetupTime + core->chainBits * 40 * LCD_CLK_PRESCALE / 160 - 10;
}
// Note that dmaSetupTime can vary from line to line, potentially influenced
// by interrupts, nondeterministic DMA channel clearing times, etc., which is
// why we don't just use a constant value. Each scanline might show for a
// slightly different length of time, but duty cycle scales with this so it's
// perceptually consistent; don't see bright or dark rows.

#define _PM_minMinPeriod                                                       \
  (200 + (uint32_t)core->chainBits * 40 * LCD_CLK_PRESCALE / 160)

#if (ESP_IDF_VERSION_MAJOR == 5)
#include <esp_private/periph_ctrl.h>
#else
#include <driver/periph_ctrl.h>
#endif
#include <driver/gpio.h>
#include <esp_private/gdma.h>
#include <esp_rom_gpio.h>
#include <hal/dma_types.h>
#include <hal/gpio_hal.h>
#include <hal/lcd_ll.h>
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

// If using custom "blast" function(s), all three must be declared.
// Unused ones can be empty, that's fine, just need to exist.
IRAM_ATTR static void blast_word(Protomatter_core *core, uint16_t *data) {}
IRAM_ATTR static void blast_long(Protomatter_core *core, uint32_t *data) {}

static void pinmux(int8_t pin, uint8_t signal) {
  esp_rom_gpio_connect_out_signal(pin, signal, false, false);
  PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[pin], PIN_FUNC_GPIO);
  gpio_set_drive_capability((gpio_num_t)pin, GPIO_DRIVE_STRENGTH);
}

// LCD_CAM requires a complete replacement of the "blast" functions in order
// to use the DMA-based peripheral.
#define _PM_CUSTOM_BLAST // Disable blast_*() functions in core.c
IRAM_ATTR static void blast_byte(Protomatter_core *core, uint8_t *data) {
  // Reset LCD DOUT parameters each time (required).
  // IN PRINCIPLE, cyclelen should be chainBits-1 (resulting in chainBits
  // cycles). But due to the required dummy phases at start of transfer,
  // extend by 1; set to chainBits, issue chainBits+1 cycles.
  LCD_CAM.lcd_user.lcd_dout_cyclelen = core->chainBits;
  LCD_CAM.lcd_user.lcd_dout = 1;
  LCD_CAM.lcd_user.lcd_update = 1;

  // Reset LCD TX FIFO each time, else we see old data. When doing this,
  // it's REQUIRED in the setup code to enable at least one dummy pulse,
  // else the PCLK & data are randomly misaligned by 1-2 clocks!
  LCD_CAM.lcd_misc.lcd_afifo_reset = 1;

  // Partially re-init descriptor each time (required)
  desc.dw0.size = desc.dw0.length = core->chainBits;
  desc.buffer = data;
  gdma_start(dma_chan, (intptr_t)&desc);
  esp_rom_delay_us(1); // Necessary before starting xfer

  LCD_CAM.lcd_user.lcd_start = 1; // Begin LCD DMA xfer

  // Timer was cleared to 0 before calling blast_byte(), so this
  // is the state of the timer immediately after DMA started:
#if defined(ARDUINO)
  dmaSetupTime = (uint32_t)timerRead((hw_timer_t *)core->timer);
#elif defined(CIRCUITPY)
  uint64_t value;
#if (ESP_IDF_VERSION_MAJOR == 5)
  gptimer_handle_t timer = (gptimer_handle_t)core->timer;
  gptimer_get_raw_count(timer, &value);
#else
  timer_index_t *timer = (timer_index_t *)core->timer;
  timer_get_counter_value(timer->group, timer->idx, &value);
#endif
  dmaSetupTime = (uint32_t)value;
#endif
  // See notes near top of this file for what's done with this info.
}

static void _PM_timerInit(Protomatter_core *core) {
  // On S3, initialize the LCD_CAM peripheral and DMA.

  // LCD_CAM isn't enabled by default -- MUST begin with this:
  periph_module_enable(PERIPH_LCD_CAM_MODULE);
  periph_module_reset(PERIPH_LCD_CAM_MODULE);

  // Reset LCD bus
  LCD_CAM.lcd_user.lcd_reset = 1;
  esp_rom_delay_us(100);

  // Configure LCD clock
  LCD_CAM.lcd_clock.clk_en = 1;         // Enable clock
  LCD_CAM.lcd_clock.lcd_clk_sel = 3;    // PLL160M source
  LCD_CAM.lcd_clock.lcd_clkm_div_a = 1; // 1/1 fractional divide,
  LCD_CAM.lcd_clock.lcd_clkm_div_b = 1; // plus prescale below yields...
#if LCD_CLK_PRESCALE == 8
  LCD_CAM.lcd_clock.lcd_clkm_div_num = 7; // 1:8 prescale (20 MHz CLK)
#elif LCD_CLK_PRESCALE == 9
  LCD_CAM.lcd_clock.lcd_clkm_div_num = 8; // 1:9 prescale (17.8 MHz CLK)
#else
  LCD_CAM.lcd_clock.lcd_clkm_div_num = 9; // 1:10 prescale (16 MHz CLK)
#endif
  LCD_CAM.lcd_clock.lcd_ck_out_edge = 0;    // PCLK low in first half of cycle
  LCD_CAM.lcd_clock.lcd_ck_idle_edge = 0;   // PCLK low idle
  LCD_CAM.lcd_clock.lcd_clk_equ_sysclk = 1; // PCLK = CLK (ignore CLKCNT_N)

  // Configure frame format. Some of these could probably be skipped and
  // use defaults, but being verbose for posterity...
  LCD_CAM.lcd_ctrl.lcd_rgb_mode_en = 0;    // i8080 mode (not RGB)
  LCD_CAM.lcd_rgb_yuv.lcd_conv_bypass = 0; // Disable RGB/YUV converter
  LCD_CAM.lcd_misc.lcd_next_frame_en = 0;  // Do NOT auto-frame
  LCD_CAM.lcd_data_dout_mode.val = 0;      // No data delays
  LCD_CAM.lcd_user.lcd_always_out_en = 0;  // Only when requested
  LCD_CAM.lcd_user.lcd_8bits_order = 0;    // Do not swap bytes
  LCD_CAM.lcd_user.lcd_bit_order = 0;      // Do not reverse bit order
  LCD_CAM.lcd_user.lcd_2byte_en = 0;       // 8-bit data mode
  // MUST enable at least one dummy phase at start of output, else clock and
  // data are randomly misaligned by 1-2 cycles following required TX FIFO
  // reset in blast_byte(). One phase MOSTLY works but sparkles a tiny bit
  // (as in still very occasionally misaligned by 1 cycle). Two seems ideal;
  // no sparkle. Since HUB75 is just a shift register, the extra clock ticks
  // are harmless and the zero-data shifts off end of the chain.
  LCD_CAM.lcd_user.lcd_dummy = 1;          // Enable dummy phase(s) @ LCD start
  LCD_CAM.lcd_user.lcd_dummy_cyclelen = 1; // 2 dummy phases
  LCD_CAM.lcd_user.lcd_cmd = 0;            // No command at LCD start
  LCD_CAM.lcd_user.lcd_cmd_2_cycle_en = 0;
  LCD_CAM.lcd_user.lcd_update = 1;

  // Configure signal pins. IN THEORY this could be expanded to support
  // 2 parallel chains, but the rest of the LCD & DMA setup is not currently
  // written for that, so it's limited to a single chain for now.
  const uint8_t signal[] = {LCD_DATA_OUT0_IDX, LCD_DATA_OUT1_IDX,
                            LCD_DATA_OUT2_IDX, LCD_DATA_OUT3_IDX,
                            LCD_DATA_OUT4_IDX, LCD_DATA_OUT5_IDX};
  for (int i = 0; i < 6; i++)
    pinmux(core->rgbPins[i], signal[i]);
  pinmux(core->clockPin, LCD_PCLK_IDX);
  gpio_set_drive_capability(core->latch.pin, GPIO_DRIVE_STRENGTH);
  gpio_set_drive_capability(core->oe.pin, GPIO_DRIVE_STRENGTH);
  for (uint8_t i = 0; i < core->numAddressLines; i++) {
    gpio_set_drive_capability(core->addr[i].pin, GPIO_DRIVE_STRENGTH);
  }

  // Disable LCD_CAM interrupts, clear any pending interrupt
  LCD_CAM.lc_dma_int_ena.val &= ~LCD_LL_EVENT_TRANS_DONE;
  LCD_CAM.lc_dma_int_clr.val = 0x03;

  // Set up DMA TX descriptor
  desc.dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
  desc.dw0.suc_eof = 1;
  desc.dw0.size = desc.dw0.length = core->chainBits;
  desc.buffer = core->screenData;
  desc.next = NULL;

  // Alloc DMA channel & connect it to LCD periph
#if defined(CIRCUITPY)
  if (dma_chan == NULL) {
#endif
    gdma_channel_alloc_config_t dma_chan_config = {
        .sibling_chan = NULL,
        .direction = GDMA_CHANNEL_DIRECTION_TX,
        .flags = {.reserve_sibling = 0}};
    gdma_new_channel(&dma_chan_config, &dma_chan);
    gdma_connect(dma_chan, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_LCD, 0));
    gdma_strategy_config_t strategy_config = {.owner_check = false,
                                              .auto_update_desc = false};
    gdma_apply_strategy(dma_chan, &strategy_config);
    gdma_transfer_ability_t ability = {
        .sram_trans_align = 0,
        .psram_trans_align = 0,
    };
    gdma_set_transfer_ability(dma_chan, &ability);
#if defined(CIRCUITPY)
  }
#endif
  gdma_start(dma_chan, (intptr_t)&desc);

  // Enable TRANS_DONE interrupt. Note that we do NOT require nor install
  // an interrupt service routine, but DO need to enable the TRANS_DONE
  // flag to make the LCD DMA transfer work.
  LCD_CAM.lc_dma_int_ena.val |= LCD_LL_EVENT_TRANS_DONE & 0x03;

  _PM_esp32commonTimerInit(core); // In esp32-common.h
}

#endif // END ESP32S3
