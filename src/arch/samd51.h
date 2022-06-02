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

// See notes in core.c before the "blast" functions
#if F_CPU >= 200000000
#define _PM_clockHoldHigh asm("nop; nop; nop; nop; nop");
#define _PM_clockHoldLow asm("nop; nop");
#elif F_CPU >= 180000000
#define _PM_clockHoldHigh asm("nop; nop; nop; nop");
#define _PM_clockHoldLow asm("nop");
#elif F_CPU >= 150000000
#define _PM_clockHoldHigh asm("nop; nop; nop");
#define _PM_clockHoldLow asm("nop");
#else
#define _PM_clockHoldHigh asm("nop; nop; nop");
#define _PM_clockHoldLow asm("nop");
#endif

#define _PM_minMinPeriod 160

#endif // END __SAMD51__ || SAM_D5X_E5X
