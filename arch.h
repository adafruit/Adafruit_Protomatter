// Establishes some very low-level things specific to each supported device.
// This should ONLY be included by core.c, nowhere else. Ever.

#if !defined(_PROTOMATTER_ARCH_H_)
#define _PROTOMATTER_ARCH_H_

/*
Common ground for architectures to support this library:

- 32-bit device (e.g. ARM core, but potentially ESP32 and others in future)
- One or more 32-bit GPIO PORTs with atomic bitmask SET and CLEAR registers.
  A TOGGLE register, if present, may improve performance but is NOT required.
- Tolerate 8-bit or word-aligned 16-bit accesses within the 32-bit PORT
  registers (e.g. writing just one of four bytes, rather than the whole
  32 bits). The library does not use any unaligned accesses (i.e. the
  "middle word" of a 32-bit register), even if a device tolerates such.

"Pin" as used in this code is always a uint8_t value, but the semantics
of what it means may vary between Arduino and non-Arduino situations.
In Arduino, it's the pin index one would pass to functions such as
digitalWrite(), and doesn't necessarily correspond to physical hardware
pins or any other arrangement. Some may have names like 'A0' that really
just map to higher indices.
In non-Arduino settings (CircuitPython, other languages, etc.), how a
pin index relates to hardware is entirely implementation dependent, and
how to get from one to the other is what must be implemented in this file.
Quite often an environment will follow the Arduino pin designations
(since the numbers are on a board's silkscreen) and will have an internal
table mapping those indices to registers and bitmasks...but probably not
an identically-named and -structured table to the Arduino code, hence the
reason for many "else" situations in this code.

Each architecture defines the following macros and/or functions (the _PM_
prefix on each is to reduce likelihood of naming collisions...especially
on ESP32, which has some similarly-named timer functions...though note
that this library is NOT CURRENTLY PORTED to ESP32):

GPIO-related macros/functions:

_PM_portOutRegister(pin):    Get address of PORT out register. Code calling
                             this can cast it to whatever type's needed.
_PM_portSetRegister(pin):    Get address of PORT set-bits register.
_PM_portClearRegister(pin):  Get address of PORT clear-bits register.
_PM_portToggleRegister(pin): Get address of PORT toggle-bits register.
                             Not all devices support this, in which case
                             it must be left undefined.
_PM_portBitMask(pin):        Get bit mask within PORT register corresponding
                             to a pin number. When compiling for Arduino,
                             this just maps to digitalPinToBitMask(), other
                             environments will need an equivalent.
_PM_byteOffset(pin):         Get index of byte (0 to 3) within 32-bit PORT
                             corresponding to a pin number.
_PM_wordOffset(pin):         Get index of word (0 or 1) within 32-bit PORT
                             corresponding to a pin number.
_PM_pinOutput(pin):          Set a pin to output mode. In Arduino this maps
                             to pinMode(pin, OUTPUT). Other environments
                             will need an equivalent.
_PM_pinInput(pin):           Set a pin to input mode, no pullup. In Arduino
                             this maps to pinMode(pin, INPUT).
_PM_pinHigh(pin):            Set an output pin to a high or 1 state. In
                             Arduino this maps to digitalWrite(pin, HIGH).
_PM_pinLow(pin):             Set an output pin to a low or 0 state. In
                             Arduino this maps to digitalWrite(pin, LOW).

Timer-related macros/functions:

_PM_timerFreq:               A numerical constant - the source clock rate
                             (in Hz) that's fed to the timer peripheral.
_PM_timerInit(void*):        Initialize (but do not start) timer.
_PM_timerStart(void*,count): (Re)start timer for a given timer-tick interval.
_PM_timerStop(void*):        Stop timer, return current timer counter value.
_PM_timerGetCount(void*):    Get current timer counter value (whether timer
                             is running or stopped).
A timer interrupt service routine is also required, syntax for which varies
between architectures.
The void* argument passed to the timer functions is some indeterminate type
used to uniquely identify a timer peripheral within a given environment. For
example, in the Arduino wrapper for this library, compiling for SAMD chips,
it's just a pointer directly to a timer/counter peripheral base address. If
an implementation needs more data associated alongside a peripheral, this
could instead be a pointer to a struct, or an integer index.

Other macros/functions:

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
_PM_delayMicroseconds(us):   Function or macro to delay some number of
                             microseconds. For Arduino, this just maps to
                             delayMicroseconds(). Other environments will
                             need to provide their own or map to an
                             an equivalent function.
_PM_clockHoldHigh:           Additional code (typically some number of NOPs)
                             needed to delay the clock fall after RGB data is
                             written to PORT. Only required on fast devices.
                             If left undefined, no delay happens.
_PM_clockHoldLow:            Additional code (e.g. NOPs) needed to delay
                             clock rise after writing RGB data to PORT.
                             No delay if left undefined.
_PM_minMinPeriod:            Mininum value for the "minPeriod" class member,
                             so bit-angle-modulation time always doubles with
                             each bitplane (else lower bits may be the same).
*/

#if defined(ARDUINO)   // If compiling in Arduino IDE...
  #include <Arduino.h> // pull in all that stuff.

  #define _PM_delayMicroseconds(us) delayMicroseconds(us)
  #define _PM_pinOutput(pin)        pinMode(pin, OUTPUT)
  #define _PM_pinInput(pin)         pinMode(pin, INPUT)
  #define _PM_pinHigh(pin)          digitalWrite(pin, HIGH)
  #define _PM_pinLow(pin)           digitalWrite(pin, LOW)
  #define _PM_portBitMask(pin)      digitalPinToBitMask(pin)

// No #else here. In non-Arduino case, declare things in the arch-specific
// sections below...unless other environments provide device-neutral
// functions as above, in which case those could go here (w/#elif).

#endif // end defined(ARDUINO)


// CODE COMMON TO BOTH SAMD51 AND SAMD21 -----------------------------------

#if defined(__SAMD51__) || defined(_SAMD21_)
  #if defined(ARDUINO)

    // g_APinDescription[] table and pin indices are Arduino specific:
    #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
      #define _PM_byteOffset(pin) (g_APinDescription[pin].ulPin / 8)
    #else
      #define _PM_byteOffset(pin) (3 - (g_APinDescription[pin].ulPin / 8))
    #endif

    #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
      #define _PM_wordOffset(pin) (g_APinDescription[pin].ulPin / 16)
    #else
      #define _PM_wordOffset(pin) (1 - (g_APinDescription[pin].ulPin / 16))
    #endif

    // Arduino implementation is tied to a specific timer/counter & freq:
    #define _PM_TIMER_DEFAULT TC4
    #define _PM_IRQ_HANDLER   TC4_Handler
    #define _PM_timerFreq     48000000
    // Partly because IRQs must be declared at compile-time, and partly
    // because we know Arduino's already set up one of the GCLK sources
    // for 48 MHz.

    // Because it's tied to a specific timer right now, there can be only
    // one instance of the Protomatter_core struct. The Arduino library
    // sets up this pointer when calling begin().
    void *_PM_protoPtr = NULL;

    // Timer interrupt service routine
    void _PM_IRQ_HANDLER(void) {
        // Clear overflow flag:
        _PM_TIMER_DEFAULT->COUNT16.INTFLAG.reg = TC_INTFLAG_OVF;
        _PM_row_handler(_PM_protoPtr); // In core.c
    }

  #else

    // Non-arduino byte offset macros, timer and ISR work go here.

  #endif

  // Code below diverges for SAMD51 vs SAMD21, but is still very similar...
  // If making a change or bug fix in one, check to see if an equivalent
  // change should be made in the other!

#endif // __SAMD51__ || _SAMD21_


// SAMD51-SPECIFIC CODE ----------------------------------------------------

#if defined(__SAMD51__)

  #if defined(ARDUINO)

    // g_APinDescription[] table and pin indices are Arduino specific:
    #define _PM_portOutRegister(pin) \
      &PORT->Group[g_APinDescription[pin].ulPort].OUT.reg

    #define _PM_portSetRegister(pin) \
      &PORT->Group[g_APinDescription[pin].ulPort].OUTSET.reg

    #define _PM_portClearRegister(pin) \
      &PORT->Group[g_APinDescription[pin].ulPort].OUTCLR.reg

    #define _PM_portToggleRegister(pin) \
      &PORT->Group[g_APinDescription[pin].ulPort].OUTTGL.reg

  #else

    // Non-Arduino port register lookups go here

  #endif

  // Initialize, but do not start, timer
  void _PM_timerInit(void *tptr) {
      static const struct {
          Tc       *tc;      // -> Timer/counter peripheral base address
          IRQn_Type IRQn;    // Interrupt number
          uint8_t   GCLK_ID; // Peripheral channel # for clock source
      } timer[] = {
          TC0, TC0_IRQn, TC0_GCLK_ID,
          TC1, TC1_IRQn, TC1_GCLK_ID,
          TC2, TC2_IRQn, TC2_GCLK_ID,
          TC3, TC3_IRQn, TC3_GCLK_ID,
          TC4, TC4_IRQn, TC4_GCLK_ID,
          TC5, TC5_IRQn, TC5_GCLK_ID,
  #if defined(TC6)
          TC6, TC6_IRQn, TC6_GCLK_ID,
  #endif
  #if defined(TC7)
          TC7, TC7_IRQn, TC7_GCLK_ID,
  #endif
  #if defined(TC8)
          TC8, TC8_IRQn, TC8_GCLK_ID,
  #endif
  #if defined(TC9)
          TC9, TC9_IRQn, TC9_GCLK_ID,
  #endif
  #if defined(TC10)
          TC10, TC10_IRQn, TC10_GCLK_ID,
  #endif
  #if defined(TC11)
          TC11, TC11_IRQn, TC11_GCLK_ID,
  #endif
  #if defined(TC12)
          TC12, TC12_IRQn, TC12_GCLK_ID,
  #endif
      };
      #define NUM_TIMERS (sizeof timer / sizeof timer[0])

      Tc *tc = (Tc *)tptr; // Cast peripheral address passed in

      uint8_t timerNum = 0;
      while((timerNum < NUM_TIMERS) && (timer[timerNum].tc != tc)) {
          timerNum++;
      }
      if(timerNum >= NUM_TIMERS) return;

      // Feed timer/counter off GCLK1 (already set 48 MHz by Arduino core).
      // Sure, SAMD51 can run timers up to F_CPU (e.g. 120 MHz or up to
      // 200 MHz with overclocking), but on higher bitplanes (which have
      // progressively longer timer periods) I could see this possibly
      // exceeding a 16-bit timer, and would have to switch prescalers.
      // We don't actually need atomic precision on the timer -- point is
      // simply that the period doubles with each bitplane, and this can
      // work fine at 48 MHz.
      GCLK->PCHCTRL[timer[timerNum].GCLK_ID].bit.CHEN = 0;    // Disable
      while(GCLK->PCHCTRL[timer[timerNum].GCLK_ID].bit.CHEN); // Wait for it
      GCLK_PCHCTRL_Type pchctrl; // Read-modify-store
      pchctrl.reg      = GCLK->PCHCTRL[timer[timerNum].GCLK_ID].reg;
      pchctrl.bit.GEN  = GCLK_PCHCTRL_GEN_GCLK1_Val;
      pchctrl.bit.CHEN = 1;
      GCLK->PCHCTRL[timer[timerNum].GCLK_ID].reg = pchctrl.reg;
      while(!GCLK->PCHCTRL[timer[timerNum].GCLK_ID].bit.CHEN);

      // Disable timer before configuring it
      tc->COUNT16.CTRLA.bit.ENABLE = 0;
      while(tc->COUNT16.SYNCBUSY.bit.ENABLE);

      // 16-bit counter mode, 1:1 prescale
      tc->COUNT16.CTRLA.bit.MODE      = TC_CTRLA_MODE_COUNT16;
      tc->COUNT16.CTRLA.bit.PRESCALER = TC_CTRLA_PRESCALER_DIV1_Val;

      tc->COUNT16.WAVE.bit.WAVEGEN =
        TC_WAVE_WAVEGEN_MFRQ_Val; // Match frequency generation mode (MFRQ)

      tc->COUNT16.CTRLBCLR.reg = TC_CTRLBCLR_DIR; // Count up
      while(tc->COUNT16.SYNCBUSY.bit.CTRLB);

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
  inline void _PM_timerStart(void *tptr, uint32_t period) {
      Tc *tc = (Tc *)tptr; // Cast peripheral address passed in
      tc->COUNT16.COUNT.reg = 0;
      while(tc->COUNT16.SYNCBUSY.bit.COUNT);
      tc->COUNT16.CC[0].reg = period;
      while(tc->COUNT16.SYNCBUSY.bit.CC0);
      tc->COUNT16.CTRLA.bit.ENABLE = 1;
      while(tc->COUNT16.SYNCBUSY.bit.STATUS);
  }

  // Return current count value (timer enabled or not).
  // Timer must be previously initialized.
  inline uint32_t _PM_timerGetCount(void *tptr) {
      Tc *tc = (Tc *)tptr; // Cast peripheral address passed in
      tc->COUNT16.CTRLBSET.bit.CMD = 0x4;  // Sync COUNT
      while(tc->COUNT16.CTRLBSET.bit.CMD); // Wait for command
      return tc->COUNT16.COUNT.reg;
  }

  // Disable timer and return current count value.
  // Timer must be previously initialized.
  uint32_t _PM_timerStop(void *tptr) {
      Tc *tc = (Tc *)tptr; // Cast peripheral address passed in
      uint32_t count = _PM_timerGetCount(tptr);
      tc->COUNT16.CTRLA.bit.ENABLE = 0;
      while(tc->COUNT16.SYNCBUSY.bit.STATUS);
      return count;
  }

  // See notes in core.c before the "blast" functions
  #if F_CPU >= 200000000
    #define _PM_clockHoldHigh asm("nop; nop; nop; nop; nop");
    #define _PM_clockHoldLow  asm("nop; nop");
  #elif F_CPU >= 180000000
    #define _PM_clockHoldHigh asm("nop; nop; nop; nop");
    #define _PM_clockHoldLow  asm("nop");
  #elif F_CPU >= 150000000
    #define _PM_clockHoldHigh asm("nop; nop; nop");
    #define _PM_clockHoldLow  asm("nop");
  #else
    #define _PM_clockHoldHigh asm("nop; nop; nop");
    #define _PM_clockHoldLow  asm("nop");
  #endif

  #define _PM_minMinPeriod 160

#endif // end __SAMD51__


// SAMD21-SPECIFIC CODE ----------------------------------------------------

#if defined(_SAMD21_)

  #if defined(ARDUINO)

    // g_APinDescription[] table and pin indices are Arduino specific:
    #define _PM_portOutRegister(pin) \
      &PORT_IOBUS->Group[g_APinDescription[pin].ulPort].OUT.reg

    #define _PM_portSetRegister(pin) \
      &PORT_IOBUS->Group[g_APinDescription[pin].ulPort].OUTSET.reg

    #define _PM_portClearRegister(pin) \
      &PORT_IOBUS->Group[g_APinDescription[pin].ulPort].OUTCLR.reg

    #define _PM_portToggleRegister(pin) \
      &PORT_IOBUS->Group[g_APinDescription[pin].ulPort].OUTTGL.reg

  #else

    // Non-Arduino port register lookups go here

  #endif


    // Initialize, but do not start, timer
    void _PM_timerInit(void *tptr) {
        static const struct {
            Tc       *tc;     // -> Timer/counter peripheral base address
            IRQn_Type IRQn;   // Interrupt number
            uint8_t   GCM_ID; // GCLK selection ID
        } timer[] = {
            TC0, TC0_IRQn, GCM_TCC0_TCC1,
            TC1, TC1_IRQn, GCM_TCC0_TCC1,
#if defined(TC2)
            TC2, TC2_IRQn, GCM_TCC2_TC3,
#endif
#if defined(TC3)
            TC3, TC3_IRQn, GCM_TCC2_TC3,
#endif
#if defined(TC4)
            TC4, TC4_IRQn, GCM_TC4_TC5,
#endif
        };
        #define NUM_TIMERS (sizeof timer / sizeof timer[0])

        Tc *tc = (Tc *)tptr; // Cast peripheral address passed in

        uint8_t timerNum = 0;
        while((timerNum < NUM_TIMERS) && (timer[timerNum].tc != tc)) {
            timerNum++;
        }
        if(timerNum >= NUM_TIMERS) return;

        // Enable GCLK for timer/counter
        GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN |
          GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(timer[timerNum].GCM_ID));
        while(GCLK->STATUS.bit.SYNCBUSY == 1);

        // Counter must first be disabled to configure it
        tc->COUNT16.CTRLA.bit.ENABLE = 0;
        while(tc->COUNT16.STATUS.bit.SYNCBUSY);

        tc->COUNT16.CTRLA.reg =     // Configure timer counter
          TC_CTRLA_PRESCALER_DIV1 | // 1:1 Prescale
          TC_CTRLA_WAVEGEN_MFRQ   | // Match frequency generation mode (MFRQ)
          TC_CTRLA_MODE_COUNT16;    // 16-bit counter mode
        while(tc->COUNT16.STATUS.bit.SYNCBUSY);

        tc->COUNT16.CTRLBCLR.reg = TCC_CTRLBCLR_DIR; // Count up
        while(tc->COUNT16.STATUS.bit.SYNCBUSY);

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
    inline void _PM_timerStart(void *tptr, uint32_t period) {
        Tc *tc = (Tc *)tptr; // Cast peripheral address passed in
        tc->COUNT16.COUNT.reg = 0;
        while(tc->COUNT16.STATUS.bit.SYNCBUSY);
        tc->COUNT16.CC[0].reg = period;
        while(tc->COUNT16.STATUS.bit.SYNCBUSY);
        tc->COUNT16.CTRLA.bit.ENABLE = 1;
        while(tc->COUNT16.STATUS.bit.SYNCBUSY);
    }

    // Return current count value (timer enabled or not).
    // Timer must be previously initialized.
    inline uint32_t _PM_timerGetCount(void *tptr) {
        Tc *tc = (Tc *)tptr; // Cast peripheral address passed in
        tc->COUNT16.READREQ.reg = TC_READREQ_RCONT | TC_READREQ_ADDR(0x10);
        while(tc->COUNT16.STATUS.bit.SYNCBUSY);
        return tc->COUNT16.COUNT.reg;
    }

    // Disable timer and return current count value.
    // Timer must be previously initialized.
    inline uint32_t _PM_timerStop(void *tptr) {
        Tc      *tc    = (Tc *)tptr; // Cast peripheral address passed in
        uint32_t count = _PM_timerGetCount(tptr);
        tc->COUNT16.CTRLA.bit.ENABLE = 0;
        while(tc->COUNT16.STATUS.bit.SYNCBUSY);
        return count;
    }

#endif // _SAMD21_


// NRF52-SPECIFIC CODE -----------------------------------------------------

#if defined(NRF52_SERIES)
#endif // NRF52_SERIES


// ESP32-SPECIFIC CODE -----------------------------------------------------

#if defined(ARDUINO_ARCH_ESP32)
#endif // ARDUINO_ARCH_ESP32


// DEFAULTS IF NOT DEFINED ABOVE -------------------------------------------

#if !defined(_PM_chunkSize)
  #define _PM_chunkSize 8
#endif

#if !defined(_PM_clockHoldHigh)
  #define _PM_clockHoldHigh
#endif

#if !defined(_PM_clockHoldLow)
  #define _PM_clockHoldLow
#endif

#if !defined(_PM_minMinPeriod)
  #define _PM_minMinPeriod 100
#endif

// ARDUINO SPECIFIC CODE ---------------------------------------------------

#if defined(ARDUINO)

// 16-bit (565) color conversion functions go here (rather than in the
// Arduino lib .cpp) because knowledge is required of chunksize and the
// toggle register (or lack thereof), which are only known to this file,
// not the .cpp or anywhere else
// However...this file knows nothing of the GFXcanvas16 type (from
// Adafruit_GFX...another C++ lib), so the .cpp just passes down some
// pointers and minimal info about the canvas buffer.
// It's probably not ideal but this is my life now, oh well.

// Different runtime environments (which might not use the 565 canvas
// format) will need their own conversion functions.

// There are THREE COPIES of the following function -- one each for byte,
// word and long. If changes are made in any one of them, the others MUST
// be updated to match! Note that they are not simple duplicates of each
// other. The byte case, for example, doesn't need to handle parallel
// matrix chains (matrix data can only be byte-sized if one chain).

// width argument comes from GFX canvas width, which may be less than
// core's bitWidth (due to padding). height isn't needed, it can be
// inferred from core->numRowPairs.
void _PM_convert_565_byte(Protomatter_core *core, uint16_t *source,
  uint16_t width) {
    uint16_t *upperSrc = source;                             // Canvas top half
    uint16_t *lowerSrc = source + width * core->numRowPairs; // " bottom half
    uint8_t  *pinMask  = (uint8_t *)core->rgbMask;           // Pin bitmasks
    uint8_t  *dest     = (uint8_t *)core->screenData;
    if(core->doubleBuffer) {
        dest += core->bufferSize * (1 - core->activeBuffer);
    }

    // No need to clear matrix buffer, loops below do a full overwrite
    // (except for any scanline pad, which was already initialized in the
    // begin() function and won't be touched here).

    // Determine matrix bytes per bitplane & row (row pair really):

    uint32_t bitplaneSize = _PM_chunkSize *
      ((width + (_PM_chunkSize - 1)) / _PM_chunkSize); // 1 plane of row pair
    uint8_t  pad          = bitplaneSize - width;      // Start-of-plane pad

    // Skip initial scanline padding if present (HUB75 matrices shift data
    // in from right-to-left, so if we need scanline padding it occurs at
    // the start of a line, rather than the usual end). Destination pointer
    // passed in already handles double-buffer math, so we don't need to
    // handle that here, just the pad...
    dest += pad;

    uint32_t initialRedBit, initialGreenBit, initialBlueBit;
    if(core->numPlanes == 6) {
        // If numPlanes is 6, red and blue are expanded from 5 to 6 bits.
        // This involves duplicating the MSB of the 5-bit value to the LSB
        // of its corresponding 6-bit value...or in this case, bitmasks for
        // red and blue are initially assigned to canvas MSBs, while green
        // starts at LSB (because it's already 6-bit). Inner loop below then
        // wraps red & blue after the first bitplane.
        initialRedBit   = 0b1000000000000000; // MSB red
        initialGreenBit = 0b0000000000100000; // LSB green
        initialBlueBit  = 0b0000000000010000; // MSB blue
    } else {
        // If numPlanes is 1 to 5, no expansion is needed, and one or all
        // three color components might be decimated by some number of bits.
        // The initial bitmasks are set to the components' numPlanesth bit
        // (e.g. for 5 planes, start at red & blue bit #0, green bit #1,
        // for 4 planes, everything starts at the next bit up, etc.).
        uint8_t shiftLeft = 5 - core->numPlanes;
        initialRedBit   = 0b0000100000000000 << shiftLeft;
        initialGreenBit = 0b0000000001000000 << shiftLeft;
        initialBlueBit  = 0b0000000000000001 << shiftLeft;
    }

    // This works sequentially-ish through the destination buffer,
    // reading from the canvas source pixels in repeated passes,
    // beginning from the least bit.
    for(uint8_t row=0; row<core->numRowPairs; row++) {
        uint32_t redBit   = initialRedBit;
        uint32_t greenBit = initialGreenBit;
        uint32_t blueBit  = initialBlueBit;
        for(uint8_t plane=0; plane<core->numPlanes; plane++) {
#if defined(_PM_portToggleRegister)
            uint8_t prior = core->clockMask; // Set clock bit on 1st out
#endif
            for(uint16_t x=0; x<width; x++) {
                uint16_t upperRGB = upperSrc[x]; // Pixel in upper half
                uint16_t lowerRGB = lowerSrc[x]; // Pixel in lower half
                uint8_t  result   = 0;
                if(upperRGB & redBit)   result |= pinMask[0];
                if(upperRGB & greenBit) result |= pinMask[1];
                if(upperRGB & blueBit)  result |= pinMask[2];
                if(lowerRGB & redBit)   result |= pinMask[3];
                if(lowerRGB & greenBit) result |= pinMask[4];
                if(lowerRGB & blueBit)  result |= pinMask[5];
#if defined(_PM_portToggleRegister)
                dest[x] = result ^ prior;
                prior   = result | core->clockMask; // Set clock bit on next out
#else
                dest[x] = result;
#endif
            } // end x
            greenBit <<= 1;
            if(plane || (core->numPlanes < 6)) {
                // In most cases red & blue bit scoot 1 left...
                redBit  <<= 1;
                blueBit <<= 1;
            } else {
                // Exception being after bit 0 with 6-plane display,
                // in which case they're reset to red & blue LSBs
                // (so 5-bit colors are expanded to 6 bits).
                redBit  = 0b0000100000000000;
                blueBit = 0b0000000000000001;
            }
#if defined(_PM_portToggleRegister)
            // If using bit-toggle register, erase the toggle bit on the
            // first element of each bitplane & row pair. The matrix-driving
            // interrupt functions correspondingly set the clock low before
            // finishing. This is all done for legibility on oscilloscope --
            // so idle clock appears LOW -- but really the matrix samples on
            // a rising edge and we could leave it high, but at this stage
            // in development just want the scope "readable."
            dest[-pad] &= ~core->clockMask; // Negative index is legal & intentional
#endif
            dest += bitplaneSize; // Advance one scanline in dest buffer
        } // end plane
        upperSrc += width; // Advance one scanline in source buffer
        lowerSrc += width;
    } // end row
}

// Corresponding function for word output -- either 12 RGB bits (2 parallel
// matrix chains), or 1 chain with RGB bits not in the same byte (but in the
// same 16-bit word). Some of the comments have been stripped out since it's
// largely the same operation, but changes are noted.
void _PM_convert_565_word(Protomatter_core *core, uint16_t *source,
  uint16_t width) {
    uint16_t *upperSrc = source;                             // Matrix top half
    uint16_t *lowerSrc = source + width * core->numRowPairs; // " bottom half
    uint16_t *pinMask  = (uint16_t *)core->rgbMask;          // Pin bitmasks
    uint16_t *dest     = (uint16_t *)core->screenData;
    if(core->doubleBuffer) {
        dest += core->bufferSize / core->bytesPerElement *
          (1 - core->activeBuffer);
    }

    uint32_t bitplaneSize = _PM_chunkSize *
      ((width + (_PM_chunkSize - 1)) / _PM_chunkSize); // 1 plane of row pair
    uint8_t  pad          = bitplaneSize - width;      // Start-of-plane pad

    uint32_t initialRedBit, initialGreenBit, initialBlueBit;
    if(core->numPlanes == 6) {
        initialRedBit   = 0b1000000000000000; // MSB red
        initialGreenBit = 0b0000000000100000; // LSB green
        initialBlueBit  = 0b0000000000010000; // MSB blue
    } else {
        uint8_t shiftLeft = 5 - core->numPlanes;
        initialRedBit   = 0b0000100000000000 << shiftLeft;
        initialGreenBit = 0b0000000001000000 << shiftLeft;
        initialBlueBit  = 0b0000000000000001 << shiftLeft;
    }

    // Unlike the 565 byte converter, the word converter DOES clear out the
    // matrix buffer (because each chain is OR'd into place). If a toggle
    // register exists, "clear" really means the clock mask is set in all
    // but the first element on a scanline (per bitplane). If no toggle
    // register, can just zero everything out.
#if defined(_PM_portToggleRegister)
    // No per-chain loop is required; one clock bit handles all chains
    uint32_t offset = 0; // Current position in the 'dest' buffer
    for(uint8_t row=0; row<core->numRowPairs; row++) {
        for(uint8_t plane=0; plane<core->numPlanes; plane++) {
            dest[offset++] = 0; // First element of each plane
            for(uint16_t x=1; x<bitplaneSize; x++) { // All subsequent items
                dest[offset++] = core->clockMask;
            }
        }
    }
#else
    memset(dest, 0, core->bufferSize);
#endif

    dest += pad; // Pad value is in 'elements,' not bytes, so this is OK

    // After a set of rows+bitplanes are processed, upperSrc and lowerSrc
    // have advanced halfway down one matrix. This offset is used after
    // each chain to advance them to the start/middle of the next matrix.
    uint32_t halfMatrixOffset = width * core->numPlanes * core->numRowPairs;

    for(uint8_t chain=0; chain<core->parallel; chain++) {
        for(uint8_t row=0; row<core->numRowPairs; row++) {
            uint32_t redBit   = initialRedBit;
            uint32_t greenBit = initialGreenBit;
            uint32_t blueBit  = initialBlueBit;
            for(uint8_t plane=0; plane<core->numPlanes; plane++) {
#if defined(_PM_portToggleRegister)
                // Since we're ORing in bits over an existing clock bit,
                // prior is 0 rather than clockMask as in the byte case.
                uint16_t prior = 0;
#endif
                for(uint16_t x=0; x<width; x++) {
                    uint16_t upperRGB = upperSrc[x]; // Pixel in upper half
                    uint16_t lowerRGB = lowerSrc[x]; // Pixel in lower half
                    uint16_t result   = 0;
                    if(upperRGB & redBit)   result |= pinMask[0];
                    if(upperRGB & greenBit) result |= pinMask[1];
                    if(upperRGB & blueBit)  result |= pinMask[2];
                    if(lowerRGB & redBit)   result |= pinMask[3];
                    if(lowerRGB & greenBit) result |= pinMask[4];
                    if(lowerRGB & blueBit)  result |= pinMask[5];
                    // Main difference here vs byte converter is each chain
                    // ORs new bits into place (vs single-pass overwrite).
#if defined(_PM_portToggleRegister)
                    dest[x] |= result ^ prior; // Bitwise OR
                    prior    = result;
#else
                    dest[x] |= result;         // Bitwise OR
#endif
                } // end x
                greenBit <<= 1;
                if(plane || (core->numPlanes < 6)) {
                    redBit  <<= 1;
                    blueBit <<= 1;
                } else {
                    redBit  = 0b0000100000000000;
                    blueBit = 0b0000000000000001;
                }
                dest += bitplaneSize; // Advance one scanline in dest buffer
            } // end plane
            upperSrc += width; // Advance one scanline in source buffer
            lowerSrc += width;
        } // end row
        pinMask  += 6;                // Next chain's RGB pin masks
        upperSrc += halfMatrixOffset; // Advance to next matrix start pos
        lowerSrc += halfMatrixOffset;
    }
}

// Corresponding function for long output -- either several parallel chains
// (up to 5), or 1 chain with RGB bits scattered widely about the PORT.
// Same deal, comments are pared back, see above functions for explanations.
void _PM_convert_565_long(Protomatter_core *core, uint16_t *source,
  uint16_t width) {
    uint16_t *upperSrc = source;                             // Matrix top half
    uint16_t *lowerSrc = source + width * core->numRowPairs; // " bottom half
    uint32_t *pinMask  = (uint32_t *)core->rgbMask;          // Pin bitmasks
    uint32_t *dest     = (uint32_t *)core->screenData;
    if(core->doubleBuffer) {
        dest += core->bufferSize / core->bytesPerElement *
          (1 - core->activeBuffer);
    }

    uint32_t bitplaneSize = _PM_chunkSize *
      ((width + (_PM_chunkSize - 1)) / _PM_chunkSize); // 1 plane of row pair
    uint8_t  pad          = bitplaneSize - width;      // Start-of-plane pad

    uint32_t initialRedBit, initialGreenBit, initialBlueBit;
    if(core->numPlanes == 6) {
        initialRedBit   = 0b1000000000000000; // MSB red
        initialGreenBit = 0b0000000000100000; // LSB green
        initialBlueBit  = 0b0000000000010000; // MSB blue
    } else {
        uint8_t shiftLeft = 5 - core->numPlanes;
        initialRedBit   = 0b0000100000000000 << shiftLeft;
        initialGreenBit = 0b0000000001000000 << shiftLeft;
        initialBlueBit  = 0b0000000000000001 << shiftLeft;
    }

#if defined(_PM_portToggleRegister)
    // No per-chain loop is required; one clock bit handles all chains
    uint32_t offset = 0; // Current position in the 'dest' buffer
    for(uint8_t row=0; row<core->numRowPairs; row++) {
        for(uint8_t plane=0; plane<core->numPlanes; plane++) {
            dest[offset++] = 0; // First element of each plane
            for(uint16_t x=1; x<bitplaneSize; x++) { // All subsequent items
                dest[offset++] = core->clockMask;
            }
        }
    }
#else
    memset(dest, 0, core->bufferSize);
#endif

    dest += pad; // Pad value is in 'elements,' not bytes, so this is OK

    uint32_t halfMatrixOffset = width * core->numPlanes * core->numRowPairs;

    for(uint8_t chain=0; chain<core->parallel; chain++) {
        for(uint8_t row=0; row<core->numRowPairs; row++) {
            uint32_t redBit   = initialRedBit;
            uint32_t greenBit = initialGreenBit;
            uint32_t blueBit  = initialBlueBit;
            for(uint8_t plane=0; plane<core->numPlanes; plane++) {
#if defined(_PM_portToggleRegister)
                uint32_t prior = 0;
#endif
                for(uint16_t x=0; x<width; x++) {
                    uint16_t upperRGB = upperSrc[x]; // Pixel in upper half
                    uint16_t lowerRGB = lowerSrc[x]; // Pixel in lower half
                    uint32_t result   = 0;
                    if(upperRGB & redBit)   result |= pinMask[0];
                    if(upperRGB & greenBit) result |= pinMask[1];
                    if(upperRGB & blueBit)  result |= pinMask[2];
                    if(lowerRGB & redBit)   result |= pinMask[3];
                    if(lowerRGB & greenBit) result |= pinMask[4];
                    if(lowerRGB & blueBit)  result |= pinMask[5];
                    // Main difference here vs byte converter is each chain
                    // ORs new bits into place (vs single-pass overwrite).
#if defined(_PM_portToggleRegister)
                    dest[x] |= result ^ prior; // Bitwise OR
                    prior    = result;
#else
                    dest[x] |= result;         // Bitwise OR
#endif
                } // end x
                greenBit <<= 1;
                if(plane || (core->numPlanes < 6)) {
                    redBit  <<= 1;
                    blueBit <<= 1;
                } else {
                    redBit  = 0b0000100000000000;
                    blueBit = 0b0000000000000001;
                }
                dest += bitplaneSize; // Advance one scanline in dest buffer
            } // end plane
            upperSrc += width; // Advance one scanline in source buffer
            lowerSrc += width;
        } // end row
        pinMask  += 6;                // Next chain's RGB pin masks
        upperSrc += halfMatrixOffset; // Advance to next matrix start pos
        lowerSrc += halfMatrixOffset;
    }
}

#endif // ARDUINO

#endif // _PROTOMATTER_ARCH_H_
