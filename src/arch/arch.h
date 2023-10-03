/*!
 * @file arch.h
 *
 * Part of Adafruit's Protomatter library for HUB75-style RGB LED matrices.
 * This file establishes some very low-level things and includes headers
 * specific to each supported device. This should ONLY be included by
 * core.c, nowhere else. Ever.
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

#include <string.h>

/*
Common ground for architectures to support this library:

- 32-bit device (e.g. ARM core, ESP32, potentially others in the future)
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
_PM_timerInit(Protomatter_core*):        Initialize (but do not start) timer.
_PM_timerStart(Protomatter_core*,count): (Re)start timer for a given
                                         timer-tick interval.
_PM_timerStop(Protomatter_core*):        Stop timer, return current timer
                                         counter value.
_PM_timerGetCount(Protomatter_core*):    Get current timer counter value
                                         (whether timer is running or stopped).
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
_PM_allocate:                Memory allocation function, should return a
                             pointer to a buffer of requested size, aligned
                             to the architecture's largest native type.
                             If not defined, malloc() is used.
_PM_free:                    Corresponding deallocator for _PM_allocate().
                             If not defined, free() is used.
_PM_bytesPerElement          If defined, this allows an arch-specific source
                             file to override core's data size that's based
                             on pin selections. Reasonable values would be 1,
                             2 or 4. This came about during ESP32-S2
                             development; GPIO or I2S/LCD peripherals there
                             allows super flexible pin MUXing, so one byte
                             could be used even w/pins spread all over.
_PM_USE_TOGGLE_FORMAT        If defined, this instructs the core code to
                             format pixel data for GPIO bit-toggling, even
                             if _PM_portToggleRegister is not defined.
_PM_CUSTOM_BLAST             If defined, instructs core code to not compile
                             the blast_byte(), blast_word() or blast_long()
                             functions; these will be declared in the arch-
                             specific file instead. This might benefit
                             architectures, where DMA, PIO or other
                             specialized peripherals could be set up to
                             issue data independent of the CPU. This goes
                             against's Protomatter's normal design of using
                             the most baseline peripherals for a given
                             architecture, but time marches on, y'know?
*/

// ENVIRONMENT-SPECIFIC DECLARATIONS ---------------------------------------

#if defined(ARDUINO) // COMPILING FOR ARDUINO ------------------------------

#include <Arduino.h> // Pull in all that stuff.

#define _PM_delayMicroseconds(us) delayMicroseconds(us)
#define _PM_pinOutput(pin) pinMode(pin, OUTPUT)
#define _PM_pinInput(pin) pinMode(pin, INPUT)
#define _PM_pinHigh(pin) digitalWrite(pin, HIGH)
#define _PM_pinLow(pin) digitalWrite(pin, LOW)

#elif defined(CIRCUITPY) // COMPILING FOR CIRCUITPYTHON --------------------

#include "py/mphal.h"
#include "shared-bindings/microcontroller/Pin.h"

#define _PM_delayMicroseconds(us) mp_hal_delay_us(us)

// No #else here. In non-Arduino case, declare things in the arch-specific
// files below...unless other environments provide device-neutral functions
// as above, in which case those could go here (w/#elif).

#endif // END CIRCUITPYTHON ------------------------------------------------

// ARCHITECTURE-SPECIFIC HEADERS -------------------------------------------

// clang-format off
#include "esp32-common.h"
#include "esp32.h" // Original ESP32
#include "esp32-s2.h"
#include "esp32-s3.h"
#include "esp32-c3.h"
#include "esp32-c6.h"
#include "nrf52.h"
#include "rp2040.h"
#include "samd-common.h"
#include "samd21.h"
#include "samd51.h"
#include "stm32.h"
#include "teensy4.h"
// clang-format on

// DEFAULTS IF NOT DEFINED ABOVE -------------------------------------------

#if defined(_PM_portToggleRegister)
#define _PM_USE_TOGGLE_FORMAT
#endif

#if !defined(_PM_portBitMask)
#define _PM_portBitMask(pin) digitalPinToBitMask(pin)
#endif

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

#if !defined(_PM_allocate)
#define _PM_allocate(x) (malloc((x))) ///< Memory alloc call
#endif

#if !defined(_PM_free)
#define _PM_free(x) (free((x))) ///< Corresponding memory free call
#endif

#if !defined(IRAM_ATTR)
#define IRAM_ATTR ///< Neutralize ESP32-specific attribute in core.c
#endif

#if !defined(_PM_PORT_TYPE)
#define _PM_PORT_TYPE uint32_t ///< PORT register size/type
#endif

#if !defined(_PM_maxDuty)
#define _PM_maxDuty 0 ///< Max duty cycle setting (where supported)
#endif

#if !defined(_PM_defaultDuty)
#define _PM_defaultDuty 0 ///< Default duty cycle setting (where supported)
#endif
