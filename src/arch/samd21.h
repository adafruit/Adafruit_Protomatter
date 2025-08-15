/*!
 * @file samd21.h
 *
 * Part of Adafruit's Protomatter library for HUB75-style RGB LED matrices.
 * This file contains SAMD21-SPECIFIC CODE.
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

#if defined(_SAMD21_) || defined(SAMD21) // Arduino, Circuitpy SAMD21 defs

#if defined(ARDUINO) // COMPILING FOR ARDUINO ------------------------------

// g_APinDescription[] table and pin indices are Arduino specific:
#define _PM_portOutRegister(pin)                                               \
  &PORT_IOBUS->Group[g_APinDescription[pin].ulPort].OUT.reg

#define _PM_portSetRegister(pin)                                               \
  &PORT_IOBUS->Group[g_APinDescription[pin].ulPort].OUTSET.reg

#define _PM_portClearRegister(pin)                                             \
  &PORT_IOBUS->Group[g_APinDescription[pin].ulPort].OUTCLR.reg

#define _PM_portToggleRegister(pin)                                            \
  &PORT_IOBUS->Group[g_APinDescription[pin].ulPort].OUTTGL.reg

#else // END ARDUINO -------------------------------------------------------

// Non-Arduino port register lookups go here, if not already declared
// in samd-common.h.

#endif

// CODE COMMON TO ALL ENVIRONMENTS -----------------------------------------

// Initialize, but do not start, timer
void _PM_timerInit(Protomatter_core *core) {
  static const struct {
    Tc *tc;         // -> Timer/counter peripheral base address
    IRQn_Type IRQn; // Interrupt number
    uint8_t GCM_ID; // GCLK selection ID
  } timer[] = {
#if defined(TC0)
      {TC0, TC0_IRQn, GCM_TCC0_TCC1},
#endif
#if defined(TC1)
      {TC1, TC1_IRQn, GCM_TCC0_TCC1},
#endif
#if defined(TC2)
      {TC2, TC2_IRQn, GCM_TCC2_TC3},
#endif
#if defined(TC3)
      {TC3, TC3_IRQn, GCM_TCC2_TC3},
#endif
#if defined(TC4)
      {TC4, TC4_IRQn, GCM_TC4_TC5},
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

  // Enable GCLK for timer/counter
  GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 |
                                 GCLK_CLKCTRL_ID(timer[timerNum].GCM_ID));
  while (GCLK->STATUS.bit.SYNCBUSY == 1)
    ;

  // Counter must first be disabled to configure it
  tc->COUNT16.CTRLA.bit.ENABLE = 0;
  while (tc->COUNT16.STATUS.bit.SYNCBUSY)
    ;

  tc->COUNT16.CTRLA.reg =       // Configure timer counter
      TC_CTRLA_PRESCALER_DIV1 | // 1:1 Prescale
      TC_CTRLA_WAVEGEN_MFRQ |   // Match frequency generation mode (MFRQ)
      TC_CTRLA_MODE_COUNT16;    // 16-bit counter mode
  while (tc->COUNT16.STATUS.bit.SYNCBUSY)
    ;

  tc->COUNT16.CTRLBCLR.reg = TCC_CTRLBCLR_DIR; // Count up
  while (tc->COUNT16.STATUS.bit.SYNCBUSY)
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
  while (tc->COUNT16.STATUS.bit.SYNCBUSY)
    ;
  tc->COUNT16.CC[0].reg = period;
  while (tc->COUNT16.STATUS.bit.SYNCBUSY)
    ;
  tc->COUNT16.CTRLA.bit.ENABLE = 1;
  while (tc->COUNT16.STATUS.bit.SYNCBUSY)
    ;
}

// Return current count value (timer enabled or not).
// Timer must be previously initialized.
inline uint32_t _PM_timerGetCount(Protomatter_core *core) {
  Tc *tc = (Tc *)core->timer; // Cast peripheral address passed in
  tc->COUNT16.READREQ.reg = TC_READREQ_RCONT | TC_READREQ_ADDR(0x10);
  while (tc->COUNT16.STATUS.bit.SYNCBUSY)
    ;
  return tc->COUNT16.COUNT.reg;
}

// Disable timer and return current count value.
// Timer must be previously initialized.
inline uint32_t _PM_timerStop(Protomatter_core *core) {
  Tc *tc = (Tc *)core->timer; // Cast peripheral address passed in
  uint32_t count = _PM_timerGetCount(core);
  tc->COUNT16.CTRLA.bit.ENABLE = 0;
  while (tc->COUNT16.STATUS.bit.SYNCBUSY)
    ;
  return count;
}

#endif // END _SAMD21_ || SAMD21
