// Strictly for library use. User code should NOT include this header.

#if !defined(_PM_ARCH_H_)
#define _PM_ARCH_H_

#include <Arduino.h>

/*
Each architecture defines the following macros and/or functions (the _PM_
prefix on each is to reduce likelihood of naming collisions...especially on
ESP32, which has some similarly-named timer functions...though note that
this library is not currently ported to ESP32):

_PM_portOutRegister(pin):    Get address of PORT out register. Code calling
                             this can cast it to whatever type's needed.
_PM_portSetRegister(pin):    Get address of PORT set-bits register.
_PM_portClearRegister(pin):  Get address of PORT clear-bits register.
_PM_portToggleRegister(pin): Get address of PORT toggle-bits register.
                             Not all devices support this, in which case
                             it must be left undefined.
_PM_portBitMask(pin):        Get bit mask within PORT register corresponding
                             to an Arduino pin number.
_PM_byteOffset(pin):         Get index of byte (0 to 3) within 32-bit PORT
                             corresponding to an Arduino pin number.
_PM_wordOffset(pin):         Get index of word (0 or 1) within 32-bit PORT
                             corresponding to an Arduino pin number.

_PM_timerFreq:               A numerical constant - the source clock rate
                             (in Hz) that's fed to the timer peripheral.
_PM_timerInit(void):         Initialize (but do not start) timer.
_PM_timerStart(count):       (Re)start timer for a given timer-tick interval.
_PM_timerStop(void):         Stop timer, return current timer counter value.
_PM_timerGetCount(void):     Get current timer counter value (whether timer
                             is running or stopped).
A timer interrupt service routine is also required, syntax for which varies
among architectures.

_PM_chunkSize:               Matrix bitmap width (both in RAM and as issued
                             to the device) is rounded up (if necessary) to
                             a multiple of this value as a way of explicitly
                             unrolling the innermost data-stuffing loops.
                             So far all HUB75 displays I've encountered are
                             a multiple of 32 pixels wide, but in case
                             something new comes along, or if a larger
                             unroll actually decreases performance due to
                             cache size, this can be set to whatever works
                             best (any additional data is simply shifted
                             out the other end of the matrix). Default if
                             unspecified is 8 (e.g. four loop passes on a
                             32-pixel matrix, eight if 64-pixel). Only
                             certain chunkSizes are actually implemented,
                             see .cpp code (avoiding GCC-specific tricks
                             that would handle arbitrary chunk sizes).
*/


// SAMD --------------------------------------------------------------------

#if defined(ARDUINO_ARCH_SAMD)

  // Code common to both SAMD51 and SAMD21 ---------------------------------

  #define _PM_TIMER            TC4
  #define _PM_IRQN             TC4_IRQn
  #define _PM_IRQ_HANDLER      TC4_Handler
  #define _PM_TIMER_GCLK_ID    TC4_GCLK_ID
  #define _PM_GCM_ID           GCM_TC4_TC5

  #define _PM_portBitMask(pin) digitalPinToBitMask(pin)
  #define _PM_timerFreq        48000000

  // Return current count value (timer enabled or not)
  inline uint32_t _PM_timerGetCount(void) {
      return _PM_TIMER->COUNT16.COUNT.reg;
  }


  // Timer interrupt service routine
  void _PM_IRQ_HANDLER(void) {
      extern void _PM_row_handler(void); // In .cpp
      _PM_row_handler();
      _PM_TIMER->COUNT16.INTFLAG.reg = TC_INTFLAG_OVF; // Clear overflow flag
  }

  // Code below diverges for SAMD51 vs SAMD21, but is still very similar...
  // If making a change or bug fix in one, check to see if an equivalent
  // change should be made in the other!

  #if defined(__SAMD51__)

    // SAMD51 --------------------------------------------------------------

    #define _PM_portOutRegister(pin) \
      &PORT->Group[g_APinDescription[pin].ulPort].OUT.reg

    #define _PM_portSetRegister(pin) \
      &PORT->Group[g_APinDescription[pin].ulPort].OUTSET.reg

    #define _PM_portClearRegister(pin) \
      &PORT->Group[g_APinDescription[pin].ulPort].OUTCLR.reg

    #define _PM_portToggleRegister(pin) \
      &PORT->Group[g_APinDescription[pin].ulPort].OUTTGL.reg

    // Initialize, but do not start, timer
    void _PM_timerInit(void) {
        // Feed _PM_TIMER off GCLK1 (already set to 48 MHz by Arduino core).
        // Sure, SAMD51 can run timers up to F_CPU (e.g. 120 MHz or up to
        // 200 MHz with overclocking), but on higher bitplanes (which have
        // progressively longer timer periods) I could see this possibly
        // exceeding a 16-bit timer, and would have to switch prescalers.
        // We don't actually need atomic precision on the timer -- point is
        // simply that the period doubles with each bitplane, and this can
        // work fine at 48 MHz.
        GCLK->PCHCTRL[_PM_TIMER_GCLK_ID].bit.CHEN = 0;      // Disable
        while(GCLK->PCHCTRL[_PM_TIMER_GCLK_ID].bit.CHEN);   // Wait for it
        GCLK_PCHCTRL_Type pchctrl; // Read-modify-store
        pchctrl.reg      = GCLK->PCHCTRL[_PM_TIMER_GCLK_ID].reg;
        pchctrl.bit.GEN  = GCLK_PCHCTRL_GEN_GCLK1_Val;
        pchctrl.bit.CHEN = 1;
        GCLK->PCHCTRL[_PM_TIMER_GCLK_ID].reg = pchctrl.reg; // Atomic write
        while(!GCLK->PCHCTRL[_PM_TIMER_GCLK_ID].bit.CHEN);  // Wait for enable

        // Disable timer before configuring it
        _PM_TIMER->COUNT16.CTRLA.bit.ENABLE = 0;
        while(_PM_TIMER->COUNT16.SYNCBUSY.bit.ENABLE);

        // 16-bit counter mode, 1:1 prescale
        _PM_TIMER->COUNT16.CTRLA.bit.MODE      = TC_CTRLA_MODE_COUNT16;
        _PM_TIMER->COUNT16.CTRLA.bit.PRESCALER = TC_CTRLA_PRESCALER_DIV1_Val;

        _PM_TIMER->COUNT16.WAVE.bit.WAVEGEN =
          TC_WAVE_WAVEGEN_MFRQ_Val; // Match frequency generation mode (MFRQ)

        _PM_TIMER->COUNT16.CTRLBCLR.reg = TC_CTRLBCLR_DIR; // Count up
        while(_PM_TIMER->COUNT16.SYNCBUSY.bit.CTRLB);

        // Overflow interrupt
        _PM_TIMER->COUNT16.INTENSET.reg = TC_INTENSET_OVF;

        NVIC_DisableIRQ(_PM_IRQN);
        NVIC_ClearPendingIRQ(_PM_IRQN);
        NVIC_SetPriority(_PM_IRQN, 0); // Top priority
        NVIC_EnableIRQ(_PM_IRQN);

        // Timer is configured but NOT enabled by default
    }

    // Set timer period, initialize count value to zero, enable timer.
    // Timer must be inactive before calling this.
    inline void _PM_timerStart(uint32_t period) {
        _PM_TIMER->COUNT16.CC[0].reg = period;
        while(_PM_TIMER->COUNT16.SYNCBUSY.bit.CC0);
        _PM_TIMER->COUNT16.COUNT.reg = 0;
        while(_PM_TIMER->COUNT16.SYNCBUSY.bit.COUNT);
        _PM_TIMER->COUNT16.CTRLA.bit.ENABLE = 1;
        while(_PM_TIMER->COUNT16.SYNCBUSY.bit.STATUS);
    }

    // Disable timer and return current count value
    inline uint32_t _PM_timerStop(void) {
        _PM_TIMER->COUNT16.CTRLA.bit.ENABLE = 0;
        while(_PM_TIMER->COUNT16.SYNCBUSY.bit.STATUS);
        return timerGetCount();
    }

  #else

    // SAMD21 (presumably) -------------------------------------------------

    #define _PM_portOutRegister(pin) \
      &PORT_IOBUS->Group[g_APinDescription[pin].ulPort].OUT.reg

    #define _PM_portSetRegister(pin) \
      &PORT_IOBUS->Group[g_APinDescription[pin].ulPort].OUTSET.reg

    #define _PM_portClearRegister(pin) \
      &PORT_IOBUS->Group[g_APinDescription[pin].ulPort].OUTCLR.reg

    #define _PM_portToggleRegister(pin) \
      &PORT_IOBUS->Group[g_APinDescription[pin].ulPort].OUTTGL.reg

    // Initialize, but do not start, timer
    void _PM_timerInit(void) {
        // Enable GCLK for timer/counter
        GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN |
          GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(_PM_GCM_ID));
        while(GCLK->STATUS.bit.SYNCBUSY == 1);

        // Counter must first be disabled to configure it
        _PM_TIMER->COUNT16.CTRLA.bit.ENABLE = 0;
        while(_PM_TIMER->COUNT16.STATUS.bit.SYNCBUSY);

        _PM_TIMER->COUNT16.CTRLA.reg =  // Configure timer counter
          TC_CTRLA_PRESCALER_DIV1 | // 1:1 Prescale
          TC_CTRLA_WAVEGEN_MFRQ   | // Match frequency generation mode (MFRQ)
          TC_CTRLA_MODE_COUNT16;    // 16-bit counter mode
        while(_PM_TIMER->COUNT16.STATUS.bit.SYNCBUSY);

        _PM_TIMER->COUNT16.CTRLBCLR.reg = TCC_CTRLBCLR_DIR; // Count up
        while(_PM_TIMER->COUNT16.STATUS.bit.SYNCBUSY);

        // Overflow interrupt
        _PM_TIMER->COUNT16.INTENSET.reg = TC_INTENSET_OVF;

        NVIC_DisableIRQ(_PM_IRQN);
        NVIC_ClearPendingIRQ(_PM_IRQN);
        NVIC_SetPriority(_PM_IRQN, 0); // Top priority
        NVIC_EnableIRQ(_PM_IRQN);

        // Timer is configured but NOT enabled by default
    }

    // Set timer period, initialize count value to zero, enable timer
    // Timer must be inactive before calling this.
    inline void _PM_timerStart(uint32_t period) {
        _PM_TIMER->COUNT16.CC[0].reg = period;
        while(_PM_TIMER->COUNT16.STATUS.bit.SYNCBUSY);
        _PM_TIMER->COUNT16.COUNT.reg = 0;
        while(_PM_TIMER->COUNT16.STATUS.bit.SYNCBUSY);
        _PM_TIMER->COUNT16.CTRLA.bit.ENABLE = 1;
        while(_PM_TIMER->COUNT16.STATUS.bit.SYNCBUSY);
    }

    // Disable timer and return current count value
    inline uint32_t _PM_timerStop(void) {
        _PM_TIMER->COUNT16.CTRLA.bit.ENABLE = 0;
        while(_PM_TIMER->COUNT16.STATUS.bit.SYNCBUSY);
        return _PM_timerGetCount();
    }

  #endif // !__SAMD51__

#endif // ARDUINO_ARCH_SAMD


// NRF52 -------------------------------------------------------------------

#if defined(NRF52_SERIES)
#endif // NRF52_SERIES


// ESP32 -------------------------------------------------------------------

#if defined(ARDUINO_ARCH_ESP32)
#endif // ARDUINO_ARCH_ESP32


// DEFAULTS IF NOT DEFINED ABOVE -------------------------------------------

#if !defined(_PM_byteOffset)
  #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    #define _PM_byteOffset(pin) (g_APinDescription[pin].ulPin / 8)
  #else
    #define _PM_byteOffset(pin) (3 - (g_APinDescription[pin].ulPin / 8))
  #endif
#endif

#if !defined(_PM_wordOffset)
  #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    #define _PM_wordOffset(pin) (g_APinDescription[pin].ulPin / 16)
  #else
    #define _PM_wordOffset(pin) (1 - (g_APinDescription[pin].ulPin / 16))
  #endif
#endif

#if !defined(_PM_chunkSize)
  #define _PM_chunkSize 8
#endif

#endif // _PM_ARCH_H_
