/*!
 * @file renesas.h
 *
 * Part of Adafruit's Protomatter library for HUB75-style RGB LED matrices.
 * This file contains RENESAS RA-SPECIFIC CODE (e.g. RA4M1 on Arduino UNO R4
 * Minima and WiFi).
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

#if defined(ARDUINO_ARCH_RENESAS)

// RA GPIO PORTs are 16 bits wide. Keep core.c to full-PORT accesses rather
// than poking individual bytes.
#define _PM_PORT_TYPE uint16_t
#define _PM_STRICT_32BIT_IO ///< Change core.c behavior: full-PORT accesses only

#if defined(ARDUINO) // COMPILING FOR ARDUINO ------------------------------

#define _PM_portBase(pin) \
  ((R_PORT0_Type*)IOPORT_PRV_PORT_ADDRESS(digitalPinToPort(pin)))
#define _PM_portOutRegister(pin) ((void*)&_PM_portBase(pin)->PCNTR1)

// All pin set/clear goes through 32-bit PCNTR3 writes: set bits in the low
// half (POSR), reset bits in the high half (PORR), as the core's
// SoftwareSerial does. 16-bit accesses to the POSR/PORR halves don't map
// the way the 32-bit layout suggests, so they're avoided entirely. Both
// "registers" are therefore the same address; the macros differ by shift.
#define _PM_portSetRegister(pin) \
  ((volatile uint32_t*)&_PM_portBase(pin)->PCNTR3)
#define _PM_portClearRegister(pin) _PM_portSetRegister(pin)
#define _PM_setReg(x) (*(volatile uint32_t*)((x).setReg) = (x).bit)
#define _PM_clearReg(x) (*(volatile uint32_t*)((x).clearReg) = (x).bit << 16)

// Defined here (not left to arch.h default) because blast_byte below uses it
#define _PM_portBitMask(pin) digitalPinToBitMask(pin)

// RA parts are little-endian only
#define _PM_byteOffset(pin) ((digitalPinToBspPin(pin) & 0xFF) / 8)
#define _PM_wordOffset(pin) 0

// As written, because it's tied to a specific timer right now, the
// Arduino lib only permits one instance of the Protomatter_core struct,
// which it sets up when calling begin().
void* _PM_protoPtr = NULL;

// The GPT channel is allocated through the core's FspTimer class (C++, see
// renesas.cpp) so it doesn't collide with PWM or other timer users. Once
// allocated, core->timer is replaced with that channel's register block,
// and start/stop/count go straight to the registers (called from the ISR,
// so FSP API overhead is avoided).
#define _PM_TIMER_DEFAULT ((void*)R_GPT0) ///< Placeholder, see _PM_timerInit

// GPT is clocked from PCLKD, divided by 4 so 16-bit channels can still
// hold the longest bitplane period.
#define _PM_timerFreq (R_FSP_SystemClockHzGet(FSP_PRIV_CLOCK_PCLKD) / 4)

// In renesas.cpp. Returns GPT channel register block, or NULL on failure.
extern void* _PM_renesasTimerInit(void);

// Initialize, but do not start, timer.
void _PM_timerInit(Protomatter_core* core) {
  core->timer = _PM_renesasTimerInit();
}

// Set timer period, initialize count value to zero, enable timer.
inline void _PM_timerStart(Protomatter_core* core, uint32_t period) {
  R_GPT0_Type* gpt = (R_GPT0_Type*)core->timer;
  if (!gpt)
    return;
  gpt->GTCR_b.CST = 0;
  gpt->GTCNT = 0;
  gpt->GTPR = period;
  gpt->GTPBR = period; // In case FSP enabled period buffering
  gpt->GTCR_b.CST = 1;
}

// Return current count value (timer enabled or not).
// Timer must be previously initialized.
inline uint32_t _PM_timerGetCount(Protomatter_core* core) {
  R_GPT0_Type* gpt = (R_GPT0_Type*)core->timer;
  return gpt ? gpt->GTCNT : 0;
}

// Disable timer and return current count value.
// Timer must be previously initialized.
uint32_t _PM_timerStop(Protomatter_core* core) {
  R_GPT0_Type* gpt = (R_GPT0_Type*)core->timer;
  if (!gpt)
    return 0;
  gpt->GTCR_b.CST = 0;
  return gpt->GTCNT;
}

#define _PM_minMinPeriod 25 ///< ~2 us at 12 MHz timer clock

// On UNO-shaped boards (e.g. Adafruit RGB Matrix Shield) the RGB pins
// D2-D7 share one PORT byte but CLK (D8) is on another PORT, which the
// core's normal same-PORT check would reject. Declaring one byte per
// element skips that check, and the custom blast below drives clock
// through its own PORT. RGB pins must still all be in one PORT byte.
#define _PM_bytesPerElement 1
#define _PM_CUSTOM_BLAST
#define _PM_chunkSize 8 ///< Matches core default; set here since blast uses it

// ~200 ns per clock edge at 48 MHz. Longer holds (e.g. 1 us) make the
// top-priority ISR starve native USB, and the board drops off the bus.
#define _PM_clockHoldHigh \
  asm("nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;");
#define _PM_clockHoldLow \
  asm("nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;");

static void blast_byte(Protomatter_core* core, uint8_t* data) {
  // RGB is written as one 32-bit PCNTR3 access (set bits in the low half,
  // reset bits in the high half, as the core's SoftwareSerial does), so
  // each line only changes when its value does and other pins are untouched.
  volatile uint32_t* rgbSetReset =
      (volatile uint32_t*)_PM_portSetRegister(core->rgbPins[0]);
  volatile uint32_t* clk = (volatile uint32_t*)core->setReg; // PCNTR3
  uint8_t shift = core->portOffset * 8;
  uint32_t clock = _PM_portBitMask(core->clockPin);
  uint32_t rgb = 0;
  for (uint8_t i = 0; i < 6; i++) {
    rgb |= ((uint8_t*)core->rgbMask)[i];
  }
  rgb <<= shift;
  uint32_t columns =
      _PM_chunkSize * ((core->chainBits + (_PM_chunkSize - 1)) / _PM_chunkSize);

  while (columns--) {
    uint32_t bits = (uint32_t)*data++ << shift;
    *rgbSetReset = bits | ((~bits & rgb) << 16); // RGB data
    _PM_clockHoldLow;
    *clk = clock; // Clock high
    _PM_clockHoldHigh;
    *clk = clock << 16; // Clock low
  }
  *rgbSetReset = rgb << 16; // Leave RGB low
}

// Never called (one byte per element), but core.c references them
static void blast_word(Protomatter_core* core, uint16_t* data) {
  (void)core;
  (void)data;
}
static void blast_long(Protomatter_core* core, uint32_t* data) {
  (void)core;
  (void)data;
}

#elif defined(CIRCUITPY) // COMPILING FOR CIRCUITPYTHON --------------------

// Renesas CircuitPython magic goes here.

#endif // END CIRCUITPYTHON ------------------------------------------------

#endif // END ARDUINO_ARCH_RENESAS
