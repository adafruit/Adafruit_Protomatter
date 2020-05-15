/*!
 * @file arch.h
 *
 * Part of Adafruit's Protomatter library for HUB75-style RGB LED matrices.
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

// Establishes some very low-level things specific to each supported device.
// This should ONLY be included by core.c, nowhere else. Ever.

#if !defined(_PROTOMATTER_ARCH_H_)
#define _PROTOMATTER_ARCH_H_

#include <string.h>

/*
Common ground for architectures to support this library:

- 32-bit device (e.g. ARM core, but potentially others in the future)
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
on ESP32, which has some similarly-named timer functions:

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

#if defined(ARDUINO) // If compiling in Arduino IDE...
#include <Arduino.h> // pull in all that stuff.

#define _PM_delayMicroseconds(us) delayMicroseconds(us)
#define _PM_pinOutput(pin) pinMode(pin, OUTPUT)
#define _PM_pinInput(pin) pinMode(pin, INPUT)
#define _PM_pinHigh(pin) digitalWrite(pin, HIGH)
#define _PM_pinLow(pin) digitalWrite(pin, LOW)
#define _PM_portBitMask(pin) digitalPinToBitMask(pin)

#elif defined(CIRCUITPY)
#include "py/mphal.h"
#include "shared-bindings/microcontroller/Pin.h"

#define _PM_delayMicroseconds(us) mp_hal_delay_us(us)

#ifdef SAMD51
#define __SAMD51__
#define F_CPU (120000000)
#endif
#ifdef SAMD21
#define _SAMD21_
#endif

#ifdef STM32F405xx
#define STM32F4_SERIES (1)
#endif

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
#define _PM_wordOffset(pin) (g_APinDescription[pin].ulPin / 16)
#else
#define _PM_byteOffset(pin) (3 - (g_APinDescription[pin].ulPin / 8))
#define _PM_wordOffset(pin) (1 - (g_APinDescription[pin].ulPin / 16))
#endif

// Arduino implementation is tied to a specific timer/counter & freq:
#if defined(TC4)
#define _PM_TIMER_DEFAULT TC4
#define _PM_IRQ_HANDLER TC4_Handler
#else // No TC4 on some M4's
#define _PM_TIMER_DEFAULT TC3
#define _PM_IRQ_HANDLER TC3_Handler
#endif
#define _PM_timerFreq 48000000
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

#elif defined(CIRCUITPY)

#include "hal_gpio.h"

#define _PM_pinOutput(pin) gpio_set_pin_direction(pin, GPIO_DIRECTION_OUT)
#define _PM_pinInput(pin) gpio_set_pin_direction(pin, GPIO_DIRECTION_IN)
#define _PM_pinHigh(pin) gpio_set_pin_level(pin, 1)
#define _PM_pinLow(pin) gpio_set_pin_level(pin, 0)
#define _PM_portBitMask(pin) (1u << ((pin) % 32))

#define _PM_byteOffset(pin) ((pin % 32) / 8)
#define _PM_wordOffset(pin) ((pin % 32) / 16)

#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error SRSLY
#endif

// CircuitPython implementation is tied to a specific freq (but the counter
// is dynamically allocated):
#define _PM_timerFreq 48000000
// Partly because IRQs must be declared at compile-time, and partly
// because we know Arduino's already set up one of the GCLK sources
// for 48 MHz.

// Because it's tied to a specific timer right now, there can be only
// one instance of the Protomatter_core struct. The Arduino library
// sets up this pointer when calling begin().
void *_PM_protoPtr = NULL;

// Timer interrupt service routine
void _PM_IRQ_HANDLER(void) {
  ((Tc *)(((Protomatter_core *)_PM_protoPtr)->timer))->COUNT16.INTFLAG.reg =
      TC_INTFLAG_OVF;
  _PM_row_handler(_PM_protoPtr); // In core.c
}

#else

// Other port byte offset macros, timer and ISR work go here.

#endif

// Code below diverges for SAMD51 vs SAMD21, but is still very similar...
// If making a change or bug fix in one, check to see if an equivalent
// change should be made in the other!

#endif // __SAMD51__ || _SAMD21_

// SAMD51-SPECIFIC CODE ----------------------------------------------------

#if defined(__SAMD51__)

#if defined(ARDUINO)

// g_APinDescription[] table and pin indices are Arduino specific:
#define _PM_portOutRegister(pin)                                               \
  &PORT->Group[g_APinDescription[pin].ulPort].OUT.reg

#define _PM_portSetRegister(pin)                                               \
  &PORT->Group[g_APinDescription[pin].ulPort].OUTSET.reg

#define _PM_portClearRegister(pin)                                             \
  &PORT->Group[g_APinDescription[pin].ulPort].OUTCLR.reg

#define _PM_portToggleRegister(pin)                                            \
  &PORT->Group[g_APinDescription[pin].ulPort].OUTTGL.reg

#elif defined(CIRCUITPY)

#include "hal_gpio.h"

#define _PM_portOutRegister(pin) (&PORT->Group[(pin / 32)].OUT.reg)

#define _PM_portSetRegister(pin) (&PORT->Group[(pin / 32)].OUTSET.reg)

#define _PM_portClearRegister(pin) (&PORT->Group[(pin / 32)].OUTCLR.reg)

#define _PM_portToggleRegister(pin) (&PORT->Group[(pin / 32)].OUTTGL.reg)

#else

// Other port register lookups go here

#endif

// Initialize, but do not start, timer
void _PM_timerInit(void *tptr) {
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

  Tc *tc = (Tc *)tptr; // Cast peripheral address passed in

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
inline void _PM_timerStart(void *tptr, uint32_t period) {
  Tc *tc = (Tc *)tptr; // Cast peripheral address passed in
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
inline uint32_t _PM_timerGetCount(void *tptr) {
  Tc *tc = (Tc *)tptr;                // Cast peripheral address passed in
  tc->COUNT16.CTRLBSET.bit.CMD = 0x4; // Sync COUNT
  while (tc->COUNT16.CTRLBSET.bit.CMD)
    ; // Wait for command
  return tc->COUNT16.COUNT.reg;
}

// Disable timer and return current count value.
// Timer must be previously initialized.
uint32_t _PM_timerStop(void *tptr) {
  Tc *tc = (Tc *)tptr; // Cast peripheral address passed in
  uint32_t count = _PM_timerGetCount(tptr);
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

#endif // end __SAMD51__

// SAMD21-SPECIFIC CODE ----------------------------------------------------

#if defined(_SAMD21_)

#if defined(ARDUINO)

// g_APinDescription[] table and pin indices are Arduino specific:
#define _PM_portOutRegister(pin)                                               \
  &PORT_IOBUS->Group[g_APinDescription[pin].ulPort].OUT.reg

#define _PM_portSetRegister(pin)                                               \
  &PORT_IOBUS->Group[g_APinDescription[pin].ulPort].OUTSET.reg

#define _PM_portClearRegister(pin)                                             \
  &PORT_IOBUS->Group[g_APinDescription[pin].ulPort].OUTCLR.reg

#define _PM_portToggleRegister(pin)                                            \
  &PORT_IOBUS->Group[g_APinDescription[pin].ulPort].OUTTGL.reg

#else

// Non-Arduino port register lookups go here

#endif

// Initialize, but do not start, timer
void _PM_timerInit(void *tptr) {
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

  Tc *tc = (Tc *)tptr; // Cast peripheral address passed in

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
inline void _PM_timerStart(void *tptr, uint32_t period) {
  Tc *tc = (Tc *)tptr; // Cast peripheral address passed in
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
inline uint32_t _PM_timerGetCount(void *tptr) {
  Tc *tc = (Tc *)tptr; // Cast peripheral address passed in
  tc->COUNT16.READREQ.reg = TC_READREQ_RCONT | TC_READREQ_ADDR(0x10);
  while (tc->COUNT16.STATUS.bit.SYNCBUSY)
    ;
  return tc->COUNT16.COUNT.reg;
}

// Disable timer and return current count value.
// Timer must be previously initialized.
inline uint32_t _PM_timerStop(void *tptr) {
  Tc *tc = (Tc *)tptr; // Cast peripheral address passed in
  uint32_t count = _PM_timerGetCount(tptr);
  tc->COUNT16.CTRLA.bit.ENABLE = 0;
  while (tc->COUNT16.STATUS.bit.SYNCBUSY)
    ;
  return count;
}

#endif // _SAMD21_

// NRF52-SPECIFIC CODE -----------------------------------------------------

#if defined(NRF52_SERIES)

#if defined(ARDUINO)

// digitalPinToPort, g_ADigitalPinMap[] are Arduino specific:

void *_PM_portOutRegister(uint32_t pin) {
  NRF_GPIO_Type *port = digitalPinToPort(pin);
  return &port->OUT;
}

void *_PM_portSetRegister(uint32_t pin) {
  NRF_GPIO_Type *port = digitalPinToPort(pin);
  return &port->OUTSET;
}

void *_PM_portClearRegister(uint32_t pin) {
  NRF_GPIO_Type *port = digitalPinToPort(pin);
  return &port->OUTCLR;
}

// Leave _PM_portToggleRegister(pin) undefined on nRF!

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define _PM_byteOffset(pin) ((g_ADigitalPinMap[pin] & 0x1F) / 8)
#define _PM_wordOffset(pin) ((g_ADigitalPinMap[pin] & 0x1F) / 16)
#else
#define _PM_byteOffset(pin) (3 - ((g_ADigitalPinMap[pin] & 0x1F) / 8))
#define _PM_wordOffset(pin) (1 - ((g_ADigitalPinMap[pin] & 0x1F) / 16))
#endif

// Because it's tied to a specific timer right now, there can be only
// one instance of the Protomatter_core struct. The Arduino library
// sets up this pointer when calling begin().
void *_PM_protoPtr = NULL;

// Arduino implementation is tied to a specific timer/counter,
// Partly because IRQs must be declared at compile-time.
#define _PM_IRQ_HANDLER TIMER4_IRQHandler
#define _PM_timerFreq 16000000
#define _PM_TIMER_DEFAULT NRF_TIMER4

#ifdef __cplusplus
extern "C" {
#endif

// Timer interrupt service routine
void _PM_IRQ_HANDLER(void) {
  if (_PM_TIMER_DEFAULT->EVENTS_COMPARE[0]) {
    _PM_TIMER_DEFAULT->EVENTS_COMPARE[0] = 0;
  }
  _PM_row_handler(_PM_protoPtr); // In core.c
}

#ifdef __cplusplus
}
#endif

#elif defined(CIRCUITPY)

#include "nrf_gpio.h"

volatile uint32_t *_PM_portOutRegister(uint32_t pin) {
  NRF_GPIO_Type *port = nrf_gpio_pin_port_decode(&pin);
  return &port->OUT;
}

volatile uint32_t *_PM_portSetRegister(uint32_t pin) {
  NRF_GPIO_Type *port = nrf_gpio_pin_port_decode(&pin);
  return &port->OUTSET;
}

volatile uint32_t *_PM_portClearRegister(uint32_t pin) {
  NRF_GPIO_Type *port = nrf_gpio_pin_port_decode(&pin);
  return &port->OUTCLR;
}
#define _PM_pinOutput(pin)                                                     \
  nrf_gpio_cfg(pin, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT,    \
               NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE)
#define _PM_pinInput(pin) nrf_gpio_cfg_input(pin)
#define _PM_pinHigh(pin) nrf_gpio_pin_set(pin)
#define _PM_pinLow(pin) nrf_gpio_pin_clear(pin)
#define _PM_portBitMask(pin) (1u << ((pin) % 32))

#define _PM_byteOffset(pin) ((pin % 32) / 8)
#define _PM_wordOffset(pin) ((pin % 32) / 16)

#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error SRSLY
#endif

// CircuitPython implementation is tied to a specific freq (but the counter
// is dynamically allocated):
#define _PM_timerFreq 16000000

// Because it's tied to a specific timer right now, there can be only
// one instance of the Protomatter_core struct. The Arduino library
// sets up this pointer when calling begin().
void *_PM_protoPtr = NULL;

// Timer interrupt service routine
void _PM_IRQ_HANDLER(void) {
  NRF_TIMER_Type *timer = (((Protomatter_core *)_PM_protoPtr)->timer);
  if (timer->EVENTS_COMPARE[0]) {
    timer->EVENTS_COMPARE[0] = 0;
  }

  _PM_row_handler(_PM_protoPtr); // In core.c
}

#else

// Non-arduino byte offset macros, timer and ISR work go here.

#endif

void _PM_timerInit(void *tptr) {
  static const struct {
    NRF_TIMER_Type *tc; // -> Timer peripheral base address
    IRQn_Type IRQn;     // Interrupt number
  } timer[] = {
#if defined(NRF_TIMER0)
    {NRF_TIMER0, TIMER0_IRQn},
#endif
#if defined(NRF_TIMER1)
    {NRF_TIMER1, TIMER1_IRQn},
#endif
#if defined(NRF_TIMER2)
    {NRF_TIMER2, TIMER2_IRQn},
#endif
#if defined(NRF_TIMER3)
    {NRF_TIMER3, TIMER3_IRQn},
#endif
#if defined(NRF_TIMER4)
    {NRF_TIMER4, TIMER4_IRQn},
#endif
  };
#define NUM_TIMERS (sizeof timer / sizeof timer[0])

  // Determine IRQn from timer address
  uint8_t timerNum = 0;
  while ((timerNum < NUM_TIMERS) && (timer[timerNum].tc != tptr)) {
    timerNum++;
  }
  if (timerNum >= NUM_TIMERS)
    return;

  NRF_TIMER_Type *tc = timer[timerNum].tc;

  tc->TASKS_STOP = 1;               // Stop timer
  tc->MODE = TIMER_MODE_MODE_Timer; // Timer (not counter) mode
  tc->TASKS_CLEAR = 1;
  tc->BITMODE = TIMER_BITMODE_BITMODE_16Bit
                << TIMER_BITMODE_BITMODE_Pos; // 16-bit timer res
  tc->PRESCALER = 0;                          // 1:1 prescale (16 MHz)
  tc->INTENSET = TIMER_INTENSET_COMPARE0_Enabled
                 << TIMER_INTENSET_COMPARE0_Pos; // Event 0 interrupt
  // NVIC_DisableIRQ(timer[timerNum].IRQn);
  // NVIC_ClearPendingIRQ(timer[timerNum].IRQn);
  // NVIC_SetPriority(timer[timerNum].IRQn, 0); // Top priority
  NVIC_EnableIRQ(timer[timerNum].IRQn);
}

inline void _PM_timerStart(void *tptr, uint32_t period) {
  volatile NRF_TIMER_Type *tc = (volatile NRF_TIMER_Type *)tptr;
  tc->TASKS_STOP = 1;  // Stop timer
  tc->TASKS_CLEAR = 1; // Reset to 0
  tc->CC[0] = period;
  tc->TASKS_START = 1; // Start timer
}

inline uint32_t _PM_timerGetCount(void *tptr) {
  volatile NRF_TIMER_Type *tc = (volatile NRF_TIMER_Type *)tptr;
  tc->TASKS_CAPTURE[0] = 1; // Capture timer to CC[n] register
  return tc->CC[0];
}

uint32_t _PM_timerStop(void *tptr) {
  volatile NRF_TIMER_Type *tc = (volatile NRF_TIMER_Type *)tptr;
  tc->TASKS_STOP = 1; // Stop timer
  __attribute__((unused)) uint32_t count = _PM_timerGetCount(tptr);
  // NOTE TO FUTURE SELF: I don't know why the GetCount code isn't
  // working. It does the expected thing in a small test program but
  // not here. I need to get on with testing on an actual matrix, so
  // this is just a nonsense fudge value for now:
  return 100;
  // return count;
}

#define _PM_clockHoldHigh asm("nop; nop");

#define _PM_minMinPeriod 100

#endif // NRF52_SERIES

// STM32F4xx SPECIFIC CODE -------------------------------------------------
#if defined(STM32F4_SERIES)
#if defined(ARDUINO)
// Arduino port register lookups go here
#elif defined(CIRCUITPY)

#undef _PM_portBitMask
#define _PM_portBitMask(pin) (1u << ((pin) % 16))
#define _PM_byteOffset(pin) ((pin % 16) / 8)
#define _PM_wordOffset(pin) ((pin % 16) / 16)

#define _PM_pinOutput(pin_)                                                    \
  do {                                                                         \
    int8_t pin = (pin_);                                                       \
    GPIO_InitTypeDef GPIO_InitStruct = {0};                                    \
    GPIO_InitStruct.Pin = 1 << (pin % 16);                                     \
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;                                \
    GPIO_InitStruct.Pull = GPIO_NOPULL;                                        \
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;                         \
    HAL_GPIO_Init(pin_port(pin / 16), &GPIO_InitStruct);                       \
  } while (0)
#define _PM_pinInput(pin_)                                                     \
  do {                                                                         \
    int8_t pin = (pin_);                                                       \
    GPIO_InitTypeDef GPIO_InitStruct = {0};                                    \
    GPIO_InitStruct.Pin = 1 << (pin % 16);                                     \
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;                                    \
    GPIO_InitStruct.Pull = GPIO_NOPULL;                                        \
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;                         \
    HAL_GPIO_Init(pin_port(pin / 16), &GPIO_InitStruct);                       \
  } while (0)
#define _PM_pinHigh(pin)                                                       \
  HAL_GPIO_WritePin(pin_port(pin / 16), 1 << (pin % 16), GPIO_PIN_SET)
#define _PM_pinLow(pin)                                                        \
  HAL_GPIO_WritePin(pin_port(pin / 16), 1 << (pin % 16), GPIO_PIN_RESET)

#define _PM_PORT_TYPE uint16_t

volatile uint16_t *_PM_portOutRegister(uint32_t pin) {
  return (uint16_t *)&pin_port(pin / 16)->ODR;
}

volatile uint16_t *_PM_portSetRegister(uint32_t pin) {
  return (uint16_t *)&pin_port(pin / 16)->BSRR;
}

// To make things interesting, STM32F4xx places the set and clear
// GPIO bits within a single register.  The "clear" bits are upper, so
// offset by 1 in uint16_ts
volatile uint16_t *_PM_portClearRegister(uint32_t pin) {
  return 1 + (uint16_t *)&pin_port(pin / 16)->BSRR;
}

// Use hard-coded TIM6 (TIM7 is used by PulseOut, other TIM by PWMOut)
#define _PM_timerFreq 42000000

// Because it's tied to a specific timer right now, there can be only
// one instance of the Protomatter_core struct. The Arduino library
// sets up this pointer when calling begin().
void *_PM_protoPtr = NULL;

STATIC TIM_HandleTypeDef t6_handle;

#define _PM_IRQ_HANDLER TIM6_DAC_IRQHandler

// Timer interrupt service routine
void _PM_IRQ_HANDLER(void) {
  // Clear overflow flag:
  //_PM_TIMER_DEFAULT->COUNT16.INTFLAG.reg = TC_INTFLAG_OVF;
  _PM_row_handler(_PM_protoPtr); // In core.c
}

// Initialize, but do not start, timer
void _PM_timerInit(void *tptr) {
  __HAL_RCC_TIM6_CLK_ENABLE();

  t6_handle.Instance = TIM6;
  t6_handle.Init.Period = 1000; // immediately replaced.
  t6_handle.Init.Prescaler = 0;
  t6_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  t6_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  t6_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  HAL_TIM_Base_Init(&t6_handle);

  HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
  NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
  NVIC_SetPriority(TIM6_DAC_IRQn, 0); // Top priority
}

inline void _PM_timerStart(void *tptr, uint32_t period) {
  TIM_TypeDef *tim = tptr;
  tim->SR = 0;
  tim->ARR = period;
  tim->CR1 |= TIM_CR1_CEN;
  tim->DIER |= TIM_DIER_UIE;
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

uint32_t _PM_timerStop(void *tptr) {
  HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
  TIM_TypeDef *tim = tptr;
  tim->CR1 &= ~TIM_CR1_CEN;
  tim->DIER &= ~TIM_DIER_UIE;
  return tim->CNT;
}
// settings from M4 for >= 150MHz, we use this part at 168MHz
#define _PM_clockHoldHigh asm("nop; nop; nop");
#define _PM_clockHoldLow asm("nop");

#define _PM_minMinPeriod 140

#endif
#endif // STM32F4_SERIES

// ESP32-SPECIFIC CODE -----------------------------------------------------

#if defined(ESP32)

#if defined(ARDUINO)

#include "driver/timer.h"

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

// ESP32 requires a custom PEW declaration (issues one set of RGB color bits
// followed by clock pulse). Turns out the bit set/clear registers are not
// actually atomic. If two writes are made in quick succession, the second
// has no effect. One option is NOPs, other is to write a 0 (no effect) to
// the opposing register (set vs clear) to synchronize the next write.
#define PEW                                                                    \
  *set = *data++;         /* Set RGB data high */                              \
  *clear_full = 0;        /* ESP32 MUST sync before 2nd 'set' */               \
  *set_full = clock;      /* Set clock high */                                 \
  *clear_full = rgbclock; /* Clear RGB data + clock */                         \
  ///< Bitbang one set of RGB data bits to matrix

// As written, because it's tied to a specific timer right now, the
// Arduino lib only permits one instance of the Protomatter_core struct,
// which it sets up when calling begin().
void *_PM_protoPtr = NULL;

#define _PM_timerFreq 40000000 // 40 MHz (1:2 prescale)
#define _PM_timerNum 0         // Timer #0 (can be 0-3)

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

// Initialize, but do not start, timer.
void _PM_timerInit(void *tptr) {
  hw_timer_t **timer = (hw_timer_t **)tptr; // pointer-to-pointer
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

#elif defined(CIRCUITPY)

// ESP32 CircuitPython magic goes here. If any of the above Arduino-specific
// defines, structs or functions are useful as-is, don't copy them, just
// move them above the ARDUINO check so fixes/changes carry over, thx.

#endif

#endif // ESP32

// i.MX 1062-SPECIFIC CODE (Teensy 4.0, 4.1) -------------------------------

#if defined(__IMXRT1062__) // (Teensy 4)

// i.MX only allows full 32-bit aligned writes to GPIO.
#define _PM_STRICT_32BIT_IO ///< Change core.c behavior for long accesses only

#if defined(ARDUINO)

static const struct {
  volatile uint32_t *base; ///< GPIO base address for pin
  uint8_t            bit;  ///< GPIO bit number for pin (0-31)
} _PM_teensyPins[] = {
  {&CORE_PIN0_PORTREG, CORE_PIN0_BIT},
  {&CORE_PIN1_PORTREG, CORE_PIN1_BIT},
  {&CORE_PIN2_PORTREG, CORE_PIN2_BIT},
  {&CORE_PIN3_PORTREG, CORE_PIN3_BIT},
  {&CORE_PIN4_PORTREG, CORE_PIN4_BIT},
  {&CORE_PIN5_PORTREG, CORE_PIN5_BIT},
  {&CORE_PIN6_PORTREG, CORE_PIN6_BIT},
  {&CORE_PIN7_PORTREG, CORE_PIN7_BIT},
  {&CORE_PIN8_PORTREG, CORE_PIN8_BIT},
  {&CORE_PIN9_PORTREG, CORE_PIN9_BIT},
  {&CORE_PIN10_PORTREG, CORE_PIN10_BIT},
  {&CORE_PIN11_PORTREG, CORE_PIN11_BIT},
  {&CORE_PIN12_PORTREG, CORE_PIN12_BIT},
  {&CORE_PIN13_PORTREG, CORE_PIN13_BIT},
  {&CORE_PIN14_PORTREG, CORE_PIN14_BIT},
  {&CORE_PIN15_PORTREG, CORE_PIN15_BIT},
  {&CORE_PIN16_PORTREG, CORE_PIN16_BIT},
  {&CORE_PIN17_PORTREG, CORE_PIN17_BIT},
  {&CORE_PIN18_PORTREG, CORE_PIN18_BIT},
  {&CORE_PIN19_PORTREG, CORE_PIN19_BIT},
  {&CORE_PIN20_PORTREG, CORE_PIN20_BIT},
  {&CORE_PIN21_PORTREG, CORE_PIN21_BIT},
  {&CORE_PIN22_PORTREG, CORE_PIN22_BIT},
  {&CORE_PIN23_PORTREG, CORE_PIN23_BIT},
  {&CORE_PIN24_PORTREG, CORE_PIN24_BIT},
  {&CORE_PIN25_PORTREG, CORE_PIN25_BIT},
  {&CORE_PIN26_PORTREG, CORE_PIN26_BIT},
  {&CORE_PIN27_PORTREG, CORE_PIN27_BIT},
  {&CORE_PIN28_PORTREG, CORE_PIN28_BIT},
  {&CORE_PIN29_PORTREG, CORE_PIN29_BIT},
  {&CORE_PIN30_PORTREG, CORE_PIN30_BIT},
  {&CORE_PIN31_PORTREG, CORE_PIN31_BIT},
  {&CORE_PIN32_PORTREG, CORE_PIN32_BIT},
  {&CORE_PIN33_PORTREG, CORE_PIN33_BIT},
  {&CORE_PIN34_PORTREG, CORE_PIN34_BIT},
  {&CORE_PIN35_PORTREG, CORE_PIN35_BIT},
  {&CORE_PIN36_PORTREG, CORE_PIN36_BIT},
  {&CORE_PIN37_PORTREG, CORE_PIN37_BIT},
  {&CORE_PIN38_PORTREG, CORE_PIN38_BIT},
  {&CORE_PIN39_PORTREG, CORE_PIN39_BIT},
};

#define _PM_SET_OFFSET    33 ///< 0x84 byte offset = 33 longs
#define _PM_CLEAR_OFFSET  34 ///< 0x88 byte offset = 34 longs
#define _PM_TOGGLE_OFFSET 35 ///< 0x8C byte offset = 35 longs

#define _PM_portOutRegister(pin) (void *)_PM_teensyPins[pin].base

#define _PM_portSetRegister(pin) \
  ((volatile uint32_t *)_PM_teensyPins[pin].base + _PM_SET_OFFSET)

#define _PM_portClearRegister(pin) \
  ((volatile uint32_t *)_PM_teensyPins[pin].base + _PM_CLEAR_OFFSET)

#define _PM_portToggleRegister(pin) \
  ((volatile uint32_t *)_PM_teensyPins[pin].base + _PM_TOGGLE_OFFSET)

// Timer stuff here

#elif defined(CIRCUITPY)

// Teensy 4 CircuitPython magic goes here.

#endif

#endif // __IMXRT1062__ (Teensy 4)

// DEFAULTS IF NOT DEFINED ABOVE -------------------------------------------

#if !defined(_PM_chunkSize)
#define _PM_chunkSize 8 ///< Unroll data-stuffing loop to this size
#endif

#if !defined(_PM_clockHoldHigh)
#define _PM_clockHoldHigh ///< Extra cycles (if any) on clock HIGH signal
#endif

#if !defined(_PM_clockHoldLow)
#define _PM_clockHoldLow ///< Extra cycles (if any) on clock LOW signal
#endif

#if !defined(_PM_minMinPeriod)
#define _PM_minMinPeriod 100 ///< Minimum timer interval for least bit
#endif

#ifndef _PM_ALLOCATOR
#define _PM_ALLOCATOR(x) (malloc((x))) ///< Memory alloc call
#endif

#ifndef _PM_FREE
#define _PM_FREE(x) (free((x))) ///< Memory free call
#endif

#ifndef IRAM_ATTR
#define IRAM_ATTR ///< Neutralize ESP32-specific attribute in core.c
#endif

// ARDUINO SPECIFIC CODE ---------------------------------------------------

#if defined(ARDUINO) || defined(CIRCUITPY)

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
__attribute__((noinline)) void _PM_convert_565_byte(Protomatter_core *core,
                                                    const uint16_t *source,
                                                    uint16_t width) {
  const uint16_t *upperSrc = source; // Canvas top half
  const uint16_t *lowerSrc =
      source + width * core->numRowPairs;      // " bottom half
  uint8_t *pinMask = (uint8_t *)core->rgbMask; // Pin bitmasks
  uint8_t *dest = (uint8_t *)core->screenData;
  if (core->doubleBuffer) {
    dest += core->bufferSize * (1 - core->activeBuffer);
  }

  // No need to clear matrix buffer, loops below do a full overwrite
  // (except for any scanline pad, which was already initialized in the
  // begin() function and won't be touched here).

  // Determine matrix bytes per bitplane & row (row pair really):

  uint32_t bitplaneSize =
      _PM_chunkSize *
      ((width + (_PM_chunkSize - 1)) / _PM_chunkSize); // 1 plane of row pair
  uint8_t pad = bitplaneSize - width;                  // Start-of-plane pad

  // Skip initial scanline padding if present (HUB75 matrices shift data
  // in from right-to-left, so if we need scanline padding it occurs at
  // the start of a line, rather than the usual end). Destination pointer
  // passed in already handles double-buffer math, so we don't need to
  // handle that here, just the pad...
  dest += pad;

  uint32_t initialRedBit, initialGreenBit, initialBlueBit;
  if (core->numPlanes == 6) {
    // If numPlanes is 6, red and blue are expanded from 5 to 6 bits.
    // This involves duplicating the MSB of the 5-bit value to the LSB
    // of its corresponding 6-bit value...or in this case, bitmasks for
    // red and blue are initially assigned to canvas MSBs, while green
    // starts at LSB (because it's already 6-bit). Inner loop below then
    // wraps red & blue after the first bitplane.
    initialRedBit = 0b1000000000000000;   // MSB red
    initialGreenBit = 0b0000000000100000; // LSB green
    initialBlueBit = 0b0000000000010000;  // MSB blue
  } else {
    // If numPlanes is 1 to 5, no expansion is needed, and one or all
    // three color components might be decimated by some number of bits.
    // The initial bitmasks are set to the components' numPlanesth bit
    // (e.g. for 5 planes, start at red & blue bit #0, green bit #1,
    // for 4 planes, everything starts at the next bit up, etc.).
    uint8_t shiftLeft = 5 - core->numPlanes;
    initialRedBit = 0b0000100000000000 << shiftLeft;
    initialGreenBit = 0b0000000001000000 << shiftLeft;
    initialBlueBit = 0b0000000000000001 << shiftLeft;
  }

  // This works sequentially-ish through the destination buffer,
  // reading from the canvas source pixels in repeated passes,
  // beginning from the least bit.
  for (uint8_t row = 0; row < core->numRowPairs; row++) {
    uint32_t redBit = initialRedBit;
    uint32_t greenBit = initialGreenBit;
    uint32_t blueBit = initialBlueBit;
    for (uint8_t plane = 0; plane < core->numPlanes; plane++) {
#if defined(_PM_portToggleRegister)
      uint8_t prior = core->clockMask; // Set clock bit on 1st out
#endif
      for (uint16_t x = 0; x < width; x++) {
        uint16_t upperRGB = upperSrc[x]; // Pixel in upper half
        uint16_t lowerRGB = lowerSrc[x]; // Pixel in lower half
        uint8_t result = 0;
        if (upperRGB & redBit)
          result |= pinMask[0];
        if (upperRGB & greenBit)
          result |= pinMask[1];
        if (upperRGB & blueBit)
          result |= pinMask[2];
        if (lowerRGB & redBit)
          result |= pinMask[3];
        if (lowerRGB & greenBit)
          result |= pinMask[4];
        if (lowerRGB & blueBit)
          result |= pinMask[5];
#if defined(_PM_portToggleRegister)
        dest[x] = result ^ prior;
        prior = result | core->clockMask; // Set clock bit on next out
#else
        dest[x] = result;
#endif
      } // end x
      greenBit <<= 1;
      if (plane || (core->numPlanes < 6)) {
        // In most cases red & blue bit scoot 1 left...
        redBit <<= 1;
        blueBit <<= 1;
      } else {
        // Exception being after bit 0 with 6-plane display,
        // in which case they're reset to red & blue LSBs
        // (so 5-bit colors are expanded to 6 bits).
        redBit = 0b0000100000000000;
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
    }                       // end plane
    upperSrc += width;      // Advance one scanline in source buffer
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
  uint16_t *pinMask = (uint16_t *)core->rgbMask;           // Pin bitmasks
  uint16_t *dest = (uint16_t *)core->screenData;
  if (core->doubleBuffer) {
    dest += core->bufferSize / core->bytesPerElement * (1 - core->activeBuffer);
  }

  uint32_t bitplaneSize =
      _PM_chunkSize *
      ((width + (_PM_chunkSize - 1)) / _PM_chunkSize); // 1 plane of row pair
  uint8_t pad = bitplaneSize - width;                  // Start-of-plane pad

  uint32_t initialRedBit, initialGreenBit, initialBlueBit;
  if (core->numPlanes == 6) {
    initialRedBit = 0b1000000000000000;   // MSB red
    initialGreenBit = 0b0000000000100000; // LSB green
    initialBlueBit = 0b0000000000010000;  // MSB blue
  } else {
    uint8_t shiftLeft = 5 - core->numPlanes;
    initialRedBit = 0b0000100000000000 << shiftLeft;
    initialGreenBit = 0b0000000001000000 << shiftLeft;
    initialBlueBit = 0b0000000000000001 << shiftLeft;
  }

  // Unlike the 565 byte converter, the word converter DOES clear out the
  // matrix buffer (because each chain is OR'd into place). If a toggle
  // register exists, "clear" really means the clock mask is set in all
  // but the first element on a scanline (per bitplane). If no toggle
  // register, can just zero everything out.
#if defined(_PM_portToggleRegister)
  // No per-chain loop is required; one clock bit handles all chains
  uint32_t offset = 0; // Current position in the 'dest' buffer
  for (uint8_t row = 0; row < core->numRowPairs; row++) {
    for (uint8_t plane = 0; plane < core->numPlanes; plane++) {
      dest[offset++] = 0; // First element of each plane
      for (uint16_t x = 1; x < bitplaneSize; x++) { // All subsequent items
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

  for (uint8_t chain = 0; chain < core->parallel; chain++) {
    for (uint8_t row = 0; row < core->numRowPairs; row++) {
      uint32_t redBit = initialRedBit;
      uint32_t greenBit = initialGreenBit;
      uint32_t blueBit = initialBlueBit;
      for (uint8_t plane = 0; plane < core->numPlanes; plane++) {
#if defined(_PM_portToggleRegister)
        // Since we're ORing in bits over an existing clock bit,
        // prior is 0 rather than clockMask as in the byte case.
        uint16_t prior = 0;
#endif
        for (uint16_t x = 0; x < width; x++) {
          uint16_t upperRGB = upperSrc[x]; // Pixel in upper half
          uint16_t lowerRGB = lowerSrc[x]; // Pixel in lower half
          uint16_t result = 0;
          if (upperRGB & redBit)
            result |= pinMask[0];
          if (upperRGB & greenBit)
            result |= pinMask[1];
          if (upperRGB & blueBit)
            result |= pinMask[2];
          if (lowerRGB & redBit)
            result |= pinMask[3];
          if (lowerRGB & greenBit)
            result |= pinMask[4];
          if (lowerRGB & blueBit)
            result |= pinMask[5];
            // Main difference here vs byte converter is each chain
            // ORs new bits into place (vs single-pass overwrite).
#if defined(_PM_portToggleRegister)
          dest[x] |= result ^ prior; // Bitwise OR
          prior = result;
#else
          dest[x] |= result; // Bitwise OR
#endif
        } // end x
        greenBit <<= 1;
        if (plane || (core->numPlanes < 6)) {
          redBit <<= 1;
          blueBit <<= 1;
        } else {
          redBit = 0b0000100000000000;
          blueBit = 0b0000000000000001;
        }
        dest += bitplaneSize; // Advance one scanline in dest buffer
      }                       // end plane
      upperSrc += width;      // Advance one scanline in source buffer
      lowerSrc += width;
    }                             // end row
    pinMask += 6;                 // Next chain's RGB pin masks
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
  uint32_t *pinMask = (uint32_t *)core->rgbMask;           // Pin bitmasks
  uint32_t *dest = (uint32_t *)core->screenData;
  if (core->doubleBuffer) {
    dest += core->bufferSize / core->bytesPerElement * (1 - core->activeBuffer);
  }

  uint32_t bitplaneSize =
      _PM_chunkSize *
      ((width + (_PM_chunkSize - 1)) / _PM_chunkSize); // 1 plane of row pair
  uint8_t pad = bitplaneSize - width;                  // Start-of-plane pad

  uint32_t initialRedBit, initialGreenBit, initialBlueBit;
  if (core->numPlanes == 6) {
    initialRedBit = 0b1000000000000000;   // MSB red
    initialGreenBit = 0b0000000000100000; // LSB green
    initialBlueBit = 0b0000000000010000;  // MSB blue
  } else {
    uint8_t shiftLeft = 5 - core->numPlanes;
    initialRedBit = 0b0000100000000000 << shiftLeft;
    initialGreenBit = 0b0000000001000000 << shiftLeft;
    initialBlueBit = 0b0000000000000001 << shiftLeft;
  }

#if defined(_PM_portToggleRegister)
  // No per-chain loop is required; one clock bit handles all chains
  uint32_t offset = 0; // Current position in the 'dest' buffer
  for (uint8_t row = 0; row < core->numRowPairs; row++) {
    for (uint8_t plane = 0; plane < core->numPlanes; plane++) {
      dest[offset++] = 0; // First element of each plane
      for (uint16_t x = 1; x < bitplaneSize; x++) { // All subsequent items
        dest[offset++] = core->clockMask;
      }
    }
  }
#else
  memset(dest, 0, core->bufferSize);
#endif

  dest += pad; // Pad value is in 'elements,' not bytes, so this is OK

  uint32_t halfMatrixOffset = width * core->numPlanes * core->numRowPairs;

  for (uint8_t chain = 0; chain < core->parallel; chain++) {
    for (uint8_t row = 0; row < core->numRowPairs; row++) {
      uint32_t redBit = initialRedBit;
      uint32_t greenBit = initialGreenBit;
      uint32_t blueBit = initialBlueBit;
      for (uint8_t plane = 0; plane < core->numPlanes; plane++) {
#if defined(_PM_portToggleRegister)
        uint32_t prior = 0;
#endif
        for (uint16_t x = 0; x < width; x++) {
          uint16_t upperRGB = upperSrc[x]; // Pixel in upper half
          uint16_t lowerRGB = lowerSrc[x]; // Pixel in lower half
          uint32_t result = 0;
          if (upperRGB & redBit)
            result |= pinMask[0];
          if (upperRGB & greenBit)
            result |= pinMask[1];
          if (upperRGB & blueBit)
            result |= pinMask[2];
          if (lowerRGB & redBit)
            result |= pinMask[3];
          if (lowerRGB & greenBit)
            result |= pinMask[4];
          if (lowerRGB & blueBit)
            result |= pinMask[5];
            // Main difference here vs byte converter is each chain
            // ORs new bits into place (vs single-pass overwrite).
#if defined(_PM_portToggleRegister)
          dest[x] |= result ^ prior; // Bitwise OR
          prior = result;
#else
          dest[x] |= result; // Bitwise OR
#endif
        } // end x
        greenBit <<= 1;
        if (plane || (core->numPlanes < 6)) {
          redBit <<= 1;
          blueBit <<= 1;
        } else {
          redBit = 0b0000100000000000;
          blueBit = 0b0000000000000001;
        }
        dest += bitplaneSize; // Advance one scanline in dest buffer
      }                       // end plane
      upperSrc += width;      // Advance one scanline in source buffer
      lowerSrc += width;
    }                             // end row
    pinMask += 6;                 // Next chain's RGB pin masks
    upperSrc += halfMatrixOffset; // Advance to next matrix start pos
    lowerSrc += halfMatrixOffset;
  }
}

void _PM_convert_565(Protomatter_core *core, uint16_t *source, uint16_t width) {
  // Destination address is computed in convert function
  // (based on active buffer value, if double-buffering),
  // just need to pass in the canvas buffer address and
  // width in pixels.
  if (core->bytesPerElement == 1) {
    _PM_convert_565_byte(core, source, width);
  } else if (core->bytesPerElement == 2) {
    _PM_convert_565_word(core, source, width);
  } else {
    _PM_convert_565_long(core, source, width);
  }
}

void _PM_swapbuffer_maybe(Protomatter_core *core) {
  if (core->doubleBuffer) {
    core->swapBuffers = 1;
    // To avoid overwriting data on the matrix, don't return
    // until the timer ISR has performed the swap at the right time.
    while (core->swapBuffers)
      ;
  }
}

#endif // ARDUINO || CIRCUITPYTHON

#ifndef _PM_PORT_TYPE
#define _PM_PORT_TYPE uint32_t ///< PORT register size/type
#endif

#endif // _PROTOMATTER_ARCH_H_
