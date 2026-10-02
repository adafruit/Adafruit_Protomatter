/*!
 * @file samd51.h
 *
 * Part of Adafruit's Protomatter library for HUB75-style RGB LED matrices.
 * This file contains SAMD51-SPECIFIC CODE.
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

#if defined(__SAMD51__) ||                                                     \
    defined(SAM_D5X_E5X) // Arduino, Circuitpy SAMD5x / E5x defs

#if defined(ARDUINO) // COMPILING FOR ARDUINO ------------------------------

// g_APinDescription[] table and pin indices are Arduino specific:
#define _PM_portOutRegister(pin)                                               \
  &PORT->Group[g_APinDescription[pin].ulPort].OUT.reg

#define _PM_portSetRegister(pin)                                               \
  &PORT->Group[g_APinDescription[pin].ulPort].OUTSET.reg

#define _PM_portClearRegister(pin)                                             \
  &PORT->Group[g_APinDescription[pin].ulPort].OUTCLR.reg

#define _PM_portToggleRegister(pin)                                            \
  &PORT->Group[g_APinDescription[pin].ulPort].OUTTGL.reg

#elif defined(CIRCUITPY) // COMPILING FOR CIRCUITPYTHON --------------------

#define _PM_portOutRegister(pin) (&PORT->Group[(pin / 32)].OUT.reg)

#define _PM_portSetRegister(pin) (&PORT->Group[(pin / 32)].OUTSET.reg)

#define _PM_portClearRegister(pin) (&PORT->Group[(pin / 32)].OUTCLR.reg)

#define _PM_portToggleRegister(pin) (&PORT->Group[(pin / 32)].OUTTGL.reg)

#define F_CPU (120000000)

// Enable high output driver strength on one pin. Arduino does this by
// default on pinMode(OUTPUT), but CircuitPython requires the motions...
static void _hi_drive(uint8_t pin) {
  // For Arduino testing only:
  // pin = g_APinDescription[pin].ulPort * 32 + g_APinDescription[pin].ulPin;

  // Input, pull-up and peripheral MUX are disabled as we're only using
  // vanilla PORT writes on Protomatter GPIO.
  PORT->Group[pin / 32].WRCONFIG.reg =
      (pin & 16)
          ? PORT_WRCONFIG_WRPINCFG | PORT_WRCONFIG_DRVSTR |
                PORT_WRCONFIG_HWSEL | (1 << (pin & 15))
          : PORT_WRCONFIG_WRPINCFG | PORT_WRCONFIG_DRVSTR | (1 << (pin & 15));
}

#else

// Other port register lookups go here

#endif

// CODE COMMON TO ALL ENVIRONMENTS -----------------------------------------

// Initialize, but do not start, timer
void _PM_timerInit(Protomatter_core *core) {
  static const struct {
    Tc *tc;          // -> Timer/counter peripheral base address
    IRQn_Type IRQn;  // Interrupt number
    uint8_t GCLK_ID; // Peripheral channel # for clock source
  } timer[] = {
#if defined(TC0)
      {TC0, TC0_IRQn, TC0_GCLK_ID},
#endif
#if defined(TC1)
      {TC1, TC1_IRQn, TC1_GCLK_ID},
#endif
#if defined(TC2)
      {TC2, TC2_IRQn, TC2_GCLK_ID},
#endif
#if defined(TC3)
      {TC3, TC3_IRQn, TC3_GCLK_ID},
#endif
#if defined(TC4)
      {TC4, TC4_IRQn, TC4_GCLK_ID},
#endif
#if defined(TC5)
      {TC5, TC5_IRQn, TC5_GCLK_ID},
#endif
#if defined(TC6)
      {TC6, TC6_IRQn, TC6_GCLK_ID},
#endif
#if defined(TC7)
      {TC7, TC7_IRQn, TC7_GCLK_ID},
#endif
#if defined(TC8)
      {TC8, TC8_IRQn, TC8_GCLK_ID},
#endif
#if defined(TC9)
      {TC9, TC9_IRQn, TC9_GCLK_ID},
#endif
#if defined(TC10)
      {TC10, TC10_IRQn, TC10_GCLK_ID},
#endif
#if defined(TC11)
      {TC11, TC11_IRQn, TC11_GCLK_ID},
#endif
#if defined(TC12)
      {TC12, TC12_IRQn, TC12_GCLK_ID},
#endif
  };
#define NUM_TIMERS (sizeof timer / sizeof timer[0])

  Tc *tc = (Tc *)core->timer; // Cast peripheral address passed in

  uint8_t timerNum = 0;
  while ((timerNum < NUM_TIMERS) && (timer[timerNum].tc != tc)) {
    timerNum++;
  }
  if (timerNum >= NUM_TIMERS)
    return;

  // Feed timer/counter off GCLK1 (already set 48 MHz by Arduino core).
  // Sure, SAMD51 can run timers up to F_CPU (e.g. 120 MHz or up to
  // 200 MHz with overclocking), but on higher bitplanes (which have
  // progressively longer timer periods) I could see this possibly
  // exceeding a 16-bit timer, and would have to switch prescalers.
  // We don't actually need atomic precision on the timer -- point is
  // simply that the period doubles with each bitplane, and this can
  // work fine at 48 MHz.
  GCLK->PCHCTRL[timer[timerNum].GCLK_ID].bit.CHEN = 0; // Disable
  while (GCLK->PCHCTRL[timer[timerNum].GCLK_ID].bit.CHEN)
    ;                        // Wait for it
  GCLK_PCHCTRL_Type pchctrl; // Read-modify-store
  pchctrl.reg = GCLK->PCHCTRL[timer[timerNum].GCLK_ID].reg;
  pchctrl.bit.GEN = GCLK_PCHCTRL_GEN_GCLK1_Val;
  pchctrl.bit.CHEN = 1;
  GCLK->PCHCTRL[timer[timerNum].GCLK_ID].reg = pchctrl.reg;
  while (!GCLK->PCHCTRL[timer[timerNum].GCLK_ID].bit.CHEN)
    ;

  // Disable timer before configuring it
  tc->COUNT16.CTRLA.bit.ENABLE = 0;
  while (tc->COUNT16.SYNCBUSY.bit.ENABLE)
    ;

  // 16-bit counter mode, 1:1 prescale
  tc->COUNT16.CTRLA.bit.MODE = TC_CTRLA_MODE_COUNT16;
  tc->COUNT16.CTRLA.bit.PRESCALER = TC_CTRLA_PRESCALER_DIV1_Val;

  tc->COUNT16.WAVE.bit.WAVEGEN =
      TC_WAVE_WAVEGEN_MFRQ_Val; // Match frequency generation mode (MFRQ)

  tc->COUNT16.CTRLBCLR.reg = TC_CTRLBCLR_DIR; // Count up
  while (tc->COUNT16.SYNCBUSY.bit.CTRLB)
    ;

  // Overflow interrupt
  tc->COUNT16.INTENSET.reg = TC_INTENSET_OVF;

  NVIC_DisableIRQ(timer[timerNum].IRQn);
  NVIC_ClearPendingIRQ(timer[timerNum].IRQn);
  NVIC_SetPriority(timer[timerNum].IRQn, 0); // Top priority
  NVIC_EnableIRQ(timer[timerNum].IRQn);

  // Timer is configured but NOT enabled by default

#if defined(CIRCUITPY) // See notes earlier; Arduino doesn't need this.
  // Enable high drive strength on all Protomatter pins. TBH this is kind
  // of a jerky place to do this (it's not actually related to the timer
  // peripheral) but Protomatter doesn't really have a spot for it.
  uint8_t i;
  for (i = 0; i < core->parallel * 6; i++)
    _hi_drive(core->rgbPins[i]);
  for (i = 0; i < core->numAddressLines; i++)
    _hi_drive(core->addr[i].pin);
  _hi_drive(core->clockPin);
  _hi_drive(core->latch.pin);
  _hi_drive(core->oe.pin);
#endif
}

// Set timer period, initialize count value to zero, enable timer.
// Timer must be initialized to 16-bit mode using the init function
// above, but must be inactive before calling this.
inline void _PM_timerStart(Protomatter_core *core, uint32_t period) {
  Tc *tc = (Tc *)core->timer; // Cast peripheral address passed in
  tc->COUNT16.COUNT.reg = 0;
  while (tc->COUNT16.SYNCBUSY.bit.COUNT)
    ;
  tc->COUNT16.CC[0].reg = period;
  while (tc->COUNT16.SYNCBUSY.bit.CC0)
    ;
  tc->COUNT16.CTRLA.bit.ENABLE = 1;
  while (tc->COUNT16.SYNCBUSY.bit.STATUS)
    ;
}

// Return current count value (timer enabled or not).
// Timer must be previously initialized.
inline uint32_t _PM_timerGetCount(Protomatter_core *core) {
  Tc *tc = (Tc *)core->timer;         // Cast peripheral address passed in
  tc->COUNT16.CTRLBSET.bit.CMD = 0x4; // Sync COUNT
  while (tc->COUNT16.CTRLBSET.bit.CMD)
    ; // Wait for command
  return tc->COUNT16.COUNT.reg;
}

// Disable timer and return current count value.
// Timer must be previously initialized.
uint32_t _PM_timerStop(Protomatter_core *core) {
  Tc *tc = (Tc *)core->timer; // Cast peripheral address passed in
  uint32_t count = _PM_timerGetCount(core);
  tc->COUNT16.CTRLA.bit.ENABLE = 0;
  while (tc->COUNT16.SYNCBUSY.bit.STATUS)
    ;
  return count;
}

// SAMD51 takes a WEIRD TURN here, in an attempt to make the HUB75 clock
// waveform slightly adjustable. Old vs new matrices seem to have different
// preferences, and this tries to address that. If this works well then the
// approach might be applied to other architectures (which are all fixed
// duty cycle right now). THE CHALLENGE is that Protomatter works in a bit-
// bangingly way (this is on purpose and by design, avoiding peripherals
// that might work only on certain pins, for better compatibility with
// existing shields and wings from the AVR era), we're aiming for nearly a
// 20 MHz signal, and the SAMD51 cycle clock is ostensibly 120 MHz. With
// just a few cycles to toggle the data and clock lines, that doesn't even
// leave enough time for a counter loop.

#define _PM_CUSTOM_BLAST ///< Disable blast_*() functions in core.c

#define _PM_chunkSize 8 ///< Data-stuffing loop is unrolled to this size

extern uint8_t _PM_duty; // In core.c

// The approach is to use a small list of pointers, with a clock-toggling
// value written to each one in succession. Most of the pointers are aimed
// on a nonsense "bit bucket" variable, effectively becoming NOPs, and just
// one is set to the PORT toggle register, raising the clock. A couple of
// actual traditional NOPs are also present because concurrent PORT writes
// on SAMD51 incur a 1-cycle delay, so the NOPs keep the clock frequency
// constant (tradeoff is that the clock is now 7 rather than 6 cycles --
// ~17.1 MHz rather than 20 with F_CPU at 120 MHz). The NOPs could be
// removed and duty range increased by one, but the tradeoff then is
// inconsistent timing at different duty settings. That 1-cycle delay is
// also why this uses a list of pointers with a common value, rather than
// a common pointer (the PORT reg) with a list of values -- because those
// writes would all take 2 cycles, way too slow. A counter loop would also
// take 2 cycles/count, because of the branch.

#if F_CPU >= 200000000 // 200 MHz; 10 cycles/bit; 20 MHz, 6 duty settings

#define _PM_maxDuty 5     ///< Allow duty settings 0-5
#define _PM_defaultDuty 2 ///< ~60%

#define PEW                                                                    \
  asm("nop");                                                                  \
  *toggle = *data++;                                                           \
  asm("nop");                                                                  \
  *ptr0 = clock;                                                               \
  *ptr1 = clock;                                                               \
  *ptr2 = clock;                                                               \
  *ptr3 = clock;                                                               \
  *ptr4 = clock;                                                               \
  *ptr5 = clock;

#elif F_CPU >= 180000000 // 180 MHz; 9 cycles/bit; 20 MHz, 5 duty settings

#define _PM_maxDuty 4     ///< Allow duty settings 0-4
#define _PM_defaultDuty 1 ///< ~50%

#define PEW                                                                    \
  asm("nop");                                                                  \
  *toggle = *data++;                                                           \
  asm("nop");                                                                  \
  *ptr0 = clock;                                                               \
  *ptr1 = clock;                                                               \
  *ptr2 = clock;                                                               \
  *ptr3 = clock;                                                               \
  *ptr4 = clock;

#elif F_CPU >= 150000000 // 150 MHz; 8 cycles/bit; 18.75 MHz, 4 duty settings

#define _PM_maxDuty 3     ///< Allow duty settings 0-3
#define _PM_defaultDuty 1 ///< ~55%

#define PEW                                                                    \
  asm("nop");                                                                  \
  *toggle = *data++;                                                           \
  asm("nop");                                                                  \
  *ptr0 = clock;                                                               \
  *ptr1 = clock;                                                               \
  *ptr2 = clock;                                                               \
  *ptr3 = clock;

#else // 120 MHz; 7 cycles/bit; 17.1 MHz, 3 duty settings

#define _PM_maxDuty 2     ///< Allow duty settings 0-2
#define _PM_defaultDuty 0 ///< ~50%

#define PEW                                                                    \
  asm("nop");                                                                  \
  *toggle = *data++;                                                           \
  asm("nop");                                                                  \
  *ptr0 = clock;                                                               \
  *ptr1 = clock;                                                               \
  *ptr2 = clock;

#endif

static void blast_byte(Protomatter_core *core, uint8_t *data) {
  // If here, it was established in begin() that the RGB data bits and
  // clock are all within the same byte of a PORT register, else we'd be
  // in the word- or long-blasting functions now. So we just need an
  // 8-bit pointer to the PORT:
  volatile uint8_t *toggle =
      (volatile uint8_t *)core->toggleReg + core->portOffset;
  uint8_t bucket, clock = core->clockMask;
  // Pointer list must be distinct vars, not an array, else slow.
  volatile uint8_t *ptr0 =
      (_PM_duty == _PM_maxDuty) ? toggle : (volatile uint8_t *)&bucket;
  volatile uint8_t *ptr1 =
      (_PM_duty == (_PM_maxDuty - 1)) ? toggle : (volatile uint8_t *)&bucket;
  volatile uint8_t *ptr2 =
      (_PM_duty == (_PM_maxDuty - 2)) ? toggle : (volatile uint8_t *)&bucket;
#if _PM_maxDuty >= 3
  volatile uint8_t *ptr3 =
      (_PM_duty == (_PM_maxDuty - 3)) ? toggle : (volatile uint8_t *)&bucket;
#endif
#if _PM_maxDuty >= 4
  volatile uint8_t *ptr4 =
      (_PM_duty == (_PM_maxDuty - 4)) ? toggle : (volatile uint8_t *)&bucket;
#endif
#if _PM_maxDuty >= 5
  volatile uint8_t *ptr5 =
      (_PM_duty == (_PM_maxDuty - 5)) ? toggle : (volatile uint8_t *)&bucket;
#endif
  uint16_t chunks = core->chainBits / 8;

  // PORT has already been initialized with RGB data + clock bits
  // all LOW, so we don't need to initialize that state here.

  do {
    PEW PEW PEW PEW PEW PEW PEW PEW
  } while (--chunks);

  // Want the PORT left with RGB data and clock LOW on function exit
  // (so it's easier to see on 'scope, and to prime it for the next call).
  // This is implicit in the no-toggle case (due to how the PEW macro
  // works), but toggle case requires explicitly clearing those bits.
  // rgbAndClockMask is an 8-bit value when toggling, hence offset here.
  *((volatile uint8_t *)core->clearReg + core->portOffset) =
      core->rgbAndClockMask;
}

// This is a copypasta of blast_byte() with types changed to uint16_t.
static void blast_word(Protomatter_core *core, uint16_t *data) {
  volatile uint16_t *toggle = (uint16_t *)core->toggleReg + core->portOffset;
  uint16_t bucket, clock = core->clockMask;
  volatile uint16_t *ptr0 =
      (_PM_duty == _PM_maxDuty) ? toggle : (volatile uint16_t *)&bucket;
  volatile uint16_t *ptr1 =
      (_PM_duty == (_PM_maxDuty - 1)) ? toggle : (volatile uint16_t *)&bucket;
  volatile uint16_t *ptr2 =
      (_PM_duty == (_PM_maxDuty - 2)) ? toggle : (volatile uint16_t *)&bucket;
#if _PM_maxDuty >= 3
  volatile uint16_t *ptr3 =
      (_PM_duty == (_PM_maxDuty - 3)) ? toggle : (volatile uint16_t *)&bucket;
#endif
#if _PM_maxDuty >= 4
  volatile uint16_t *ptr4 =
      (_PM_duty == (_PM_maxDuty - 4)) ? toggle : (volatile uint16_t *)&bucket;
#endif
#if _PM_maxDuty >= 5
  volatile uint16_t *ptr5 =
      (_PM_duty == (_PM_maxDuty - 5)) ? toggle : (volatile uint16_t *)&bucket;
#endif
  uint16_t chunks = core->chainBits / 8;
  do {
    PEW PEW PEW PEW PEW PEW PEW PEW
  } while (--chunks);
  *((volatile uint16_t *)core->clearReg + core->portOffset) =
      core->rgbAndClockMask;
}

// This is a copypasta of blast_byte() with types changed to uint32_t.
static void blast_long(Protomatter_core *core, uint32_t *data) {
  volatile uint32_t *toggle = (uint32_t *)core->toggleReg;
  uint32_t bucket, clock = core->clockMask;
  volatile uint32_t *ptr0 =
      (_PM_duty == _PM_maxDuty) ? toggle : (volatile uint32_t *)&bucket;
  volatile uint32_t *ptr1 =
      (_PM_duty == (_PM_maxDuty - 1)) ? toggle : (volatile uint32_t *)&bucket;
  volatile uint32_t *ptr2 =
      (_PM_duty == (_PM_maxDuty - 2)) ? toggle : (volatile uint32_t *)&bucket;
#if _PM_maxDuty >= 3
  volatile uint32_t *ptr3 =
      (_PM_duty == (_PM_maxDuty - 3)) ? toggle : (volatile uint32_t *)&bucket;
#endif
#if _PM_maxDuty >= 4
  volatile uint32_t *ptr4 =
      (_PM_duty == (_PM_maxDuty - 4)) ? toggle : (volatile uint32_t *)&bucket;
#endif
#if _PM_maxDuty >= 5
  volatile uint32_t *ptr5 =
      (_PM_duty == (_PM_maxDuty - 5)) ? toggle : (volatile uint32_t *)&bucket;
#endif
  uint16_t chunks = core->chainBits / 8;
  do {
    PEW PEW PEW PEW PEW PEW PEW PEW
  } while (--chunks);
  *((volatile uint32_t *)core->clearReg + core->portOffset) =
      core->rgbAndClockMask;
}

#define _PM_minMinPeriod 160

#endif // END __SAMD51__ || SAM_D5X_E5X
