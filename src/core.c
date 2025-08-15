/*!
 * @file core.c
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

// Device- and environment-neutral core matrix-driving functionality.
// See notes near top of arch/arch.h regarding assumptions of hardware
// "common ground." If you find yourself doing an "#ifdef ARDUINO" or
// "#ifdef _SAMD21_" in this file, STOP. Idea is that the code in this
// file is neutral and portable (within aforementioned assumptions).
// Nonportable elements should appear in arch.h. If arch.h functionality
// is lacking, extend it there, do not go making device- or environment-
// specific cases within this file.

// Function names are intentionally a little obtuse, idea is that one writes
// a more sensible wrapper around this for specific environments (e.g. the
// Arduino stuff in Adafruit_Protomatter.cpp). The "_PM_" prefix on most
// things hopefully makes function and variable name collisions much less
// likely with one's own code.

#include "core.h"      // enums and structs
#include "arch/arch.h" // Do NOT include this in any other source files
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

// Overall matrix refresh rate (frames/second) is a function of matrix width
// and chain length, number of address lines, number of bit planes, CPU speed
// and whether or not a GPIO toggle register is available. There is no "this
// will run at X-frames-per-second" constant figure. You typically just have
// to try it out and perhaps trade off some bit planes for refresh rate until
// the image looks good and stable. Anything over 100 Hz is usually passable,
// around 250 Hz is where things firm up. And while this could proceed higher
// in some situations, the tradeoff is that faster rates use progressively
// more CPU time (because it's timer interrupt based and not using DMA or
// special peripherals). So a throttle is set here, an approximate maximum
// frame rate which the software will attempt to avoid exceeding (but may
// refresh slower than this, and in many cases will...just need to set an
// upper limit to avoid excessive CPU load). An incredibly long comment block
// for a single constant, thank you for coming to my TED talk!
#define _PM_MAX_REFRESH_HZ 250 ///< Max matrix refresh rate

// Time (in microseconds) to pause following any change in address lines
// (individually or collectively). Some matrices respond slowly there...
// must pause on change for matrix to catch up. Defined here (rather than
// arch.h) because it's not architecture-specific.
#define _PM_ROW_DELAY 8 ///< Delay time between row address line changes (ms)

// These are the lowest-level functions for issing data to matrices.
// There are three versions because it depends on how the six RGB data bits
// (and clock bit) are arranged within a 32-bit PORT register. If all six
// (seven) fit within one byte or word of the PORT, the library's memory
// use (and corresponding data-issuing function) change. This will also have
// an impact on parallel chains in the future, where the number of concurrent
// RGB data bits isn't always six, but some multiple thereof (i.e. up to five
// parallel outputs -- 30 RGB bits + clock -- on a 32-bit PORT, though that's
// largely hypothetical as the chance of finding a PORT with that many bits
// exposed and NOT interfering with other peripherals on a board is highly
// improbable. But I could see four happening, maybe on a Grand Central or
// other kitchen-sink board.
static void blast_byte(Protomatter_core *core, uint8_t *data);
static void blast_word(Protomatter_core *core, uint16_t *data);
static void blast_long(Protomatter_core *core, uint32_t *data);

// Needed only for panels with FM6126A chipset
static void _PM_resetFM6126A(Protomatter_core *core);

#if !defined(_PM_clearReg)
#define _PM_clearReg(x)                                                        \
  (*(volatile _PM_PORT_TYPE *)((x).clearReg) =                                 \
       ((x).bit)) ///< Clear non-RGB-data-or-clock control line (_PM_pin type)
#endif
#if !defined(_PM_setReg)
#define _PM_setReg(x)                                                          \
  (*(volatile _PM_PORT_TYPE *)((x).setReg) =                                   \
       ((x).bit)) ///< Set non-RGB-data-or-clock control line (_PM_pin type)
#endif

// Validate and populate vital elements of core structure.
// Does NOT allocate core struct -- calling function must provide that.
// (In the Arduino C++ library, it’s part of the Protomatter class.)
ProtomatterStatus _PM_init(Protomatter_core *core, uint16_t bitWidth,
                           uint8_t bitDepth, uint8_t rgbCount, uint8_t *rgbList,
                           uint8_t addrCount, uint8_t *addrList,
                           uint8_t clockPin, uint8_t latchPin, uint8_t oePin,
                           bool doubleBuffer, int8_t tile, void *timer) {
  if (!core) {
    return PROTOMATTER_ERR_ARG;
  }

  // bitDepth is NOT constrained here, handle in calling function
  // (varies with implementation, e.g. GFX lib is max 6 bitplanes,
  // but might be more or less elsewhere)
#if defined(_PM_bytesPerElement)
#if _PM_bytesPerElement == 1
  if (rgbCount > 1)
    rgbCount = 1; // Parallel output not supported if only 8-bit PORT
#elif _PM_bytesPerElement == 2
  if (rgbCount > 2)
    rgbCount = 2; // Max 2 in parallel (13 bits in 16-bit PORT)
#else
  if (rgbCount > 5)
    rgbCount = 5; // Max 5 in parallel (31 bits in 32-bit PORT)
#endif
#else
  if (rgbCount > 5)
    rgbCount = 5; // Max 5 in parallel (31 bits in 32-bit PORT)
#endif // end _PM_bytesPerElement
  if (addrCount > 5)
    addrCount = 5; // Max 5 address lines (A-E)
  if (!tile)
    tile = 1; // Can't have zero vertical tiling. Single matrix is 1.

#if defined(_PM_TIMER_DEFAULT)
  // If NULL timer was passed in (the default case for the constructor),
  // use default value from arch.h. For example, in the Arduino case it's
  // tied to TC4 specifically.
  if (timer == NULL)
    timer = _PM_TIMER_DEFAULT;
#else
  if (timer == NULL)
    return PROTOMATTER_ERR_ARG;
#endif

  core->timer = timer;
  core->width = bitWidth; // Matrix chain width in bits (NOT including V tile)
  core->tile = tile;      // Matrix chain vertical tiling
  core->chainBits = bitWidth * abs(tile); // Total matrix chain bits
  core->numPlanes = bitDepth;
  core->parallel = rgbCount;
  core->numAddressLines = addrCount;
  core->clockPin = clockPin;
  core->latch.pin = latchPin;
  core->oe.pin = oePin;
  core->doubleBuffer = doubleBuffer;
  core->addr = NULL;
  core->screenData = NULL;

  // Make a copy of the rgbList and addrList tables in case they're
  // passed from local vars on the stack or some other non-persistent
  // source. screenData is NOT allocated here because data size (byte,
  // word, long) is not known until the begin function evaluates all
  // the pin bitmasks.

  rgbCount *= 6; // Convert parallel count to pin count
  if ((core->rgbPins = (uint8_t *)_PM_allocate(rgbCount * sizeof(uint8_t)))) {
    if ((core->addr = (_PM_pin *)_PM_allocate(addrCount * sizeof(_PM_pin)))) {
      memcpy(core->rgbPins, rgbList, rgbCount * sizeof(uint8_t));
      for (uint8_t i = 0; i < addrCount; i++) {
        core->addr[i].pin = addrList[i];
      }
      return PROTOMATTER_OK;
    }
    _PM_free(core->rgbPins);
    core->rgbPins = NULL;
  }
  return PROTOMATTER_ERR_MALLOC;
}

// Allocate display buffers and populate additional elements.
ProtomatterStatus _PM_begin(Protomatter_core *core) {
  if (!core)
    return PROTOMATTER_ERR_ARG;

  if (!core->rgbPins) { // NULL if copy failed to allocate
    return PROTOMATTER_ERR_MALLOC;
  }

#if defined(_PM_bytesPerElement)
  // Some chips (e.g. ESP32S2 & S3) have potent pin MUX capabilities and
  // arch-specific code might use special peripherals. The usual rules about
  // RGB+clock on one PORT, and the size of the internal data representation,
  // can be overridden because they're much simplified.
  core->bytesPerElement = _PM_bytesPerElement;
  uint32_t bitMask = 0;
#else
  // Verify that rgbPins and clockPin are all on the same PORT. If not,
  // return an error. Pin list is not freed; please call dealloc function.
  // Also get bitmask of which bits within 32-bit PORT register are
  // referenced.
  uint8_t *port = (uint8_t *)_PM_portOutRegister(core->clockPin);
#if defined(_PM_portToggleRegister)
  // If a bit-toggle register is present, the clock pin is included
  // in determining which bytes of the PORT register are used (and thus
  // the data storage efficiency).
  uint32_t bitMask = _PM_portBitMask(core->clockPin);
#else
  // If no bit-toggle register, clock pin can be on any bit, doesn't
  // affect storage efficiency.
  uint32_t bitMask = 0;
#endif

  for (uint8_t i = 0; i < core->parallel * 6; i++) {
    uint8_t *p2 = (uint8_t *)_PM_portOutRegister(core->rgbPins[i]);
    if (p2 != port) {
      return PROTOMATTER_ERR_PINS;
    }
    bitMask |= _PM_portBitMask(core->rgbPins[i]);
  }

  // RGB + clock are on same port, we can proceed...

  // Determine data type for internal representation. If all the data
  // bitmasks (and possibly clock bitmask, depending whether toggle-bits
  // register is present) are in the same byte, this can be stored more
  // compact than if they're spread across a word or long.
  uint8_t byteMask = 0;
  if (bitMask & 0xFF000000)
    byteMask |= 0b1000;
  if (bitMask & 0x00FF0000)
    byteMask |= 0b0100;
  if (bitMask & 0x0000FF00)
    byteMask |= 0b0010;
  if (bitMask & 0x000000FF)
    byteMask |= 0b0001;
  switch (byteMask) {
  case 0b0001: // If all PORT bits are in the same byte...
  case 0b0010:
  case 0b0100:
  case 0b1000:
    core->bytesPerElement = 1; // Use 8-bit PORT accesses.
    break;
  case 0b0011: // If all PORT bits in upper/lower word...
  case 0b1100:
    core->bytesPerElement = 2; // Use 16-bit PORT accesses.
    // Although some devices might tolerate unaligned 16-bit accesses
    // ('middle' word of 32-bit PORT), that is NOT handled here.
    // It's a portability liability.
    break;
  default:                     // Any other situation...
    core->bytesPerElement = 4; // Use 32-bit PORT accesses.
    break;
  }
#endif // end RGB+clock PORT check & bytesPerElement calc

  // Planning for screen data allocation...
  core->numRowPairs = 1 << core->numAddressLines;
  uint8_t chunks = (core->chainBits + (_PM_chunkSize - 1)) / _PM_chunkSize;
  uint16_t columns = chunks * _PM_chunkSize; // Padded matrix width
  uint32_t screenBytes =
      columns * core->numRowPairs * core->numPlanes * core->bytesPerElement;

  core->bufferSize = screenBytes; // Bytes per matrix buffer (1 or 2)
  if (core->doubleBuffer)
    screenBytes *= 2; // Total for matrix buffer(s)
  uint32_t rgbMaskBytes = core->parallel * 6 * core->bytesPerElement;

  // Allocate matrix buffer(s). Don't worry about the return type...
  // though we might be using words or longs for certain pin configs,
  // _PM_allocate() by definition always aligns to the longest type.
  if (!(core->screenData =
            (uint8_t *)_PM_allocate(screenBytes + rgbMaskBytes))) {
    return PROTOMATTER_ERR_MALLOC;
  }

  // rgbMask data follows the matrix buffer(s)
  core->rgbMask = core->screenData + screenBytes;

#if !defined(_PM_USE_TOGGLE_FORMAT)
  // Clear entire screenData buffer so there's no cruft in any pad bytes
  // (if using toggle register, each is set to clockMask below instead).
  memset(core->screenData, 0, screenBytes);
#endif

  // Figure out clockMask and rgbAndClockMask, clear matrix buffers
  if (core->bytesPerElement == 1) {
    core->portOffset = _PM_byteOffset(core->rgbPins[0]);
#if defined(_PM_USE_TOGGLE_FORMAT) && !defined(_PM_STRICT_32BIT_IO)
    // Clock and rgbAndClockMask are 8-bit values
    core->clockMask = _PM_portBitMask(core->clockPin) >> (core->portOffset * 8);
    core->rgbAndClockMask =
        (bitMask >> (core->portOffset * 8)) | core->clockMask;
    memset(core->screenData, core->clockMask, screenBytes);
#else
    // Clock and rgbAndClockMask are 32-bit values
    core->clockMask = _PM_portBitMask(core->clockPin);
    core->rgbAndClockMask = bitMask | core->clockMask;
#endif
    for (uint8_t i = 0; i < core->parallel * 6; i++) {
      ((uint8_t *)core->rgbMask)[i] = // Pin bitmasks are 8-bit
          _PM_portBitMask(core->rgbPins[i]) >> (core->portOffset * 8);
    }
  } else if (core->bytesPerElement == 2) {
    core->portOffset = _PM_wordOffset(core->rgbPins[0]);
#if defined(_PM_USE_TOGGLE_FORMAT) && !defined(_PM_STRICT_32BIT_IO)
    // Clock and rgbAndClockMask are 16-bit values
    core->clockMask =
        _PM_portBitMask(core->clockPin) >> (core->portOffset * 16);
    core->rgbAndClockMask =
        (bitMask >> (core->portOffset * 16)) | core->clockMask;
    uint32_t elements = screenBytes / 2;
    for (uint32_t i = 0; i < elements; i++) {
      ((uint16_t *)core->screenData)[i] = core->clockMask;
    }
#else
    // Clock and rgbAndClockMask are 32-bit values
    core->clockMask = _PM_portBitMask(core->clockPin);
    core->rgbAndClockMask = bitMask | core->clockMask;
#if defined(_PM_USE_TOGGLE_FORMAT)
    // TO DO: this ifdef and the one above can probably be wrapped up
    // in a more cohesive case. Think something similar will be needed
    // for the byte case. Will need Teensy 4.1 to test.
    uint32_t elements = screenBytes / 2;
    uint16_t mask = core->clockMask >> (core->portOffset * 16);
    for (uint32_t i = 0; i < elements; i++) {
      ((uint16_t *)core->screenData)[i] = mask;
    }
#endif
#endif
    for (uint8_t i = 0; i < core->parallel * 6; i++) {
      ((uint16_t *)core->rgbMask)[i] = // Pin bitmasks are 16-bit
          _PM_portBitMask(core->rgbPins[i]) >> (core->portOffset * 16);
    }
  } else {
    core->portOffset = 0;
    core->clockMask = _PM_portBitMask(core->clockPin);
    core->rgbAndClockMask = bitMask | core->clockMask;
#if defined(_PM_USE_TOGGLE_FORMAT)
    uint32_t elements = screenBytes / 4;
    for (uint32_t i = 0; i < elements; i++) {
      ((uint32_t *)core->screenData)[i] = core->clockMask;
    }
#endif
    for (uint8_t i = 0; i < core->parallel * 6; i++) {
      ((uint32_t *)core->rgbMask)[i] = // Pin bitmasks are 32-bit
          _PM_portBitMask(core->rgbPins[i]);
    }
  }

  // Estimate minimum bitplane #0 period for _PM_MAX_REFRESH_HZ rate.
  uint32_t minPeriodPerFrame = _PM_timerFreq / _PM_MAX_REFRESH_HZ;
  uint32_t minPeriodPerLine = minPeriodPerFrame / core->numRowPairs;
  core->minPeriod = minPeriodPerLine / ((1 << core->numPlanes) - 1);
  if (core->minPeriod < _PM_minMinPeriod) {
    core->minPeriod = _PM_minMinPeriod;
  }
  core->bitZeroPeriod = core->minPeriod;
  // Actual frame rate may be lower than this...it's only an estimate
  // and does not factor in things like address line selection delays
  // or interrupt overhead. That's OK, just don't want to exceed this
  // rate, as it'll eat all the CPU cycles.

  core->activeBuffer = 0;

  // Configure pins as outputs and initialize their states.

  core->latch.setReg = _PM_portSetRegister(core->latch.pin);
  core->latch.clearReg = _PM_portClearRegister(core->latch.pin);
  core->latch.bit = _PM_portBitMask(core->latch.pin);
  core->oe.setReg = _PM_portSetRegister(core->oe.pin);
  core->oe.clearReg = _PM_portClearRegister(core->oe.pin);
  core->oe.bit = _PM_portBitMask(core->oe.pin);

  _PM_pinOutput(core->clockPin);
  _PM_pinLow(core->clockPin); // Init clock LOW
  _PM_pinOutput(core->latch.pin);
  _PM_pinLow(core->latch.pin); // Init latch LOW
  _PM_pinOutput(core->oe.pin);
  _PM_pinHigh(core->oe.pin); // Init OE HIGH (disable output)

  for (uint8_t i = 0; i < core->parallel * 6; i++) {
    _PM_pinOutput(core->rgbPins[i]);
    _PM_pinLow(core->rgbPins[i]);
  }

  _PM_resetFM6126A(core);

#if defined(_PM_portToggleRegister)
  core->addrPortToggle = _PM_portToggleRegister(core->addr[0].pin);
  core->singleAddrPort = 1;
#endif
  core->prevRow = (1 << core->numAddressLines) - 2;
  for (uint8_t line = 0, bit = 1; line < core->numAddressLines;
       line++, bit <<= 1) {
    core->addr[line].setReg = _PM_portSetRegister(core->addr[line].pin);
    core->addr[line].clearReg = _PM_portClearRegister(core->addr[line].pin);
    core->addr[line].bit = _PM_portBitMask(core->addr[line].pin);
    _PM_pinOutput(core->addr[line].pin);
    if (core->prevRow & bit) {
      _PM_pinHigh(core->addr[line].pin);
    } else {
      _PM_pinLow(core->addr[line].pin);
    }
#if defined(_PM_portToggleRegister)
    // If address pin on different port than addr 0, no singleAddrPort.
    if (_PM_portToggleRegister(core->addr[line].pin) != core->addrPortToggle) {
      core->singleAddrPort = 0;
    }
#endif
  }

  // Get pointers to bit set and clear registers (and toggle, if present)
  core->setReg = (uint8_t *)_PM_portSetRegister(core->clockPin);
  core->clearReg = (uint8_t *)_PM_portClearRegister(core->clockPin);
#if defined(_PM_portToggleRegister)
  core->toggleReg = (uint8_t *)_PM_portToggleRegister(core->clockPin);
#endif

  // Reset plane/row counters, config and start timer
  _PM_resume(core);

  return PROTOMATTER_OK;
}

// Disable (but do not deallocate) a Protomatter matrix. Disables matrix by
// setting OE pin HIGH and writing all-zero data to matrix shift registers,
// so it won't halt with lit LEDs.
void _PM_stop(Protomatter_core *core) {
  if ((core)) {
    // If _PM_begin failed, this will be a NULL pointer.  Stop early,
    // none of the other "stop" operations make sense
    if (!core->screenData) {
      return;
    }
    while (core->swapBuffers)
      ;                   // Wait for any pending buffer swap
    _PM_timerStop(core);  // Halt timer
    _PM_setReg(core->oe); // Set OE HIGH (disable output)
    // So, in PRINCIPLE, setting OE high would be sufficient...
    // but in case that pin is shared with another function such
    // as the onloard LED (which pulses during bootloading) let's
    // also clear out the matrix shift registers for good measure.
    // Set all RGB pins LOW...
    for (uint8_t i = 0; i < core->parallel * 6; i++) {
      _PM_pinLow(core->rgbPins[i]);
    }
    // Clock out bits (just need to toggle clock with RGBs held low)
    for (uint32_t i = 0; i < core->chainBits; i++) {
      _PM_pinHigh(core->clockPin);
      _PM_clockHoldHigh;
      _PM_pinLow(core->clockPin);
      _PM_clockHoldLow;
    }
    // Latch data
    _PM_setReg(core->latch);
    _PM_clearReg(core->latch);
  }
}

void _PM_resume(Protomatter_core *core) {
  if ((core)) {
    // Init plane & row to max values so they roll over on 1st interrupt
    core->plane = core->numPlanes - 1;
    core->row = core->numRowPairs - 1;
    core->prevRow = (core->numRowPairs > 1) ? (core->row - 1) : 1;
    core->swapBuffers = 0;
    core->frameCount = 0;

    for (uint8_t line = 0, bit = 1; line < core->numAddressLines;
         line++, bit <<= 1) {
      _PM_pinOutput(core->addr[line].pin);
      if (core->prevRow & bit) {
        _PM_pinHigh(core->addr[line].pin);
      } else {
        _PM_pinLow(core->addr[line].pin);
      }
    }

    _PM_timerInit(core);        // Configure timer & any other periphs
    _PM_timerStart(core, 1000); // Start timer
  }
}

// Free memory associated with core structure. Does NOT dealloc struct.
void _PM_deallocate(Protomatter_core *core) {
  if ((core)) {
    _PM_stop(core);
    // TO DO: Set all pins back to inputs here?
    if (core->screenData)
      _PM_free(core->screenData);
    if (core->addr)
      _PM_free(core->addr);
    if (core->rgbPins) {
      _PM_free(core->rgbPins);
      core->rgbPins = NULL;
    }
  }
}

// ISR function (in arch.h) calls this function which it extern'd.
// Profuse apologies for the ESP32-specific IRAM_ATTR here -- the goal was
// for all architecture-specific detauls to be in arch.h -- but the need
// for one here caught me off guard. So, in arch.h, for all non-ESP32
// devices, IRAM_ATTR is defined to nothing and is ignored here. If any
// future architectures have their own attribute for making a function
// RAM-resident, #define IRAM_ATTR to that in the corresponding device-
// specific section of arch.h. Sorry. :/
// Any functions called by this function should also be IRAM_ATTR'd.
IRAM_ATTR void _PM_row_handler(Protomatter_core *core) {

  _PM_setReg(core->oe); // Disable LED output

  // ESP32 requires this next line, but not wanting to put arch-specific
  // ifdefs in this code...it's a trivial operation so just do it.
  // Latch is already clear at this point, but we go through the motions
  // to clear it again in order to sync up the setReg(OE) above with the
  // setReg(latch) that follows. Reason being, bit set/clear operations
  // on ESP32 aren't truly atomic, and if those two pins are on the same
  // port (quite common) the second setReg will be ignored. The nonsense
  // clearReg is used to sync up the two setReg operations. See also the
  // ESP32-specific PEW define in arch.h, same deal.
  _PM_clearReg(core->latch);

  _PM_setReg(core->latch);
  (void)_PM_timerStop(core);
  uint8_t prevPlane = core->plane; // Save that plane # for later timing
  _PM_clearReg(core->latch);       // (split to add a few cycles)

  if (prevPlane == 0) { // Plane 0 just finished loading
#if defined(_PM_portToggleRegister)
    // If all address lines are on a single PORT (and bit toggle is
    // available), do address line change all at once. Even doing all
    // this math takes MUCH less time than the delays required when
    // doing line-by-line changes.
    if (core->singleAddrPort) {
      // Make bitmasks of prior and new row bits
      uint32_t priorBits = 0, newBits = 0;
      for (uint8_t line = 0, bit = 1; line < core->numAddressLines;
           line++, bit <<= 1) {
        if (core->row & bit) {
          newBits |= core->addr[line].bit;
        }
        if (core->prevRow & bit) {
          priorBits |= core->addr[line].bit;
        }
      }
      *(volatile _PM_PORT_TYPE *)core->addrPortToggle = newBits ^ priorBits;
      _PM_delayMicroseconds(_PM_ROW_DELAY);
    } else {
#endif
      // Configure row address lines individually, making changes
      // (with delays) only where necessary.
      for (uint8_t line = 0, bit = 1; line < core->numAddressLines;
           line++, bit <<= 1) {
        if ((core->row & bit) != (core->prevRow & bit)) {
          if (core->row & bit) { // Set addr line high
            _PM_setReg(core->addr[line]);
          } else { // Set addr line low
            _PM_clearReg(core->addr[line]);
          }
          _PM_delayMicroseconds(_PM_ROW_DELAY);
        }
      }
#if defined(_PM_portToggleRegister)
    }
#endif
    core->prevRow = core->row;
  }

  // Advance bitplane index and/or row as necessary
  if (++core->plane >= core->numPlanes) {   // Next data bitplane, or
    core->plane = 0;                        // roll over bitplane to start
    if (++core->row >= core->numRowPairs) { // Next row, or
      core->row = 0;                        // roll over row to start
      // Switch matrix buffers if due (only if double-buffered)
      if (core->swapBuffers) {
        core->activeBuffer = 1 - core->activeBuffer;
        core->swapBuffers = 0; // Swapped!
      }
      core->frameCount++;
    }
  }

  // core->plane now is index of data to issue, NOT data to display.
  // 'prevPlane' is the previously-loaded data, which gets displayed
  // now while the next plane data is loaded.

  // Set timer and enable LED output for data loaded on PRIOR pass:
  _PM_timerStart(core, core->bitZeroPeriod << prevPlane);
  _PM_delayMicroseconds(1); // Appease Teensy4
  _PM_clearReg(core->oe);   // Enable LED output

  uint32_t elementsPerLine =
      _PM_chunkSize * ((core->chainBits + (_PM_chunkSize - 1)) / _PM_chunkSize);
  uint32_t srcOffset = elementsPerLine *
                       (core->numPlanes * core->row + core->plane) *
                       core->bytesPerElement;
  if (core->doubleBuffer) {
    srcOffset += core->bufferSize * core->activeBuffer;
  }

  if (core->bytesPerElement == 1) {
    blast_byte(core, (uint8_t *)(core->screenData + srcOffset));
  } else if (core->bytesPerElement == 2) {
    blast_word(core, (uint16_t *)(core->screenData + srcOffset));
  } else {
    blast_long(core, (uint32_t *)(core->screenData + srcOffset));
  }

  // core->plane data is now loaded, will be shown on NEXT pass

  // On the last (longest) bitplane (note that 'plane' has already wrapped
  // around earlier, so a value of 0 here indicates longest plane), take
  // note of the elapsed timer value at this point...that's the number of
  // cycles required to issue (not necessarily display) data for one plane,
  // and the bare minimum display duration allowed for plane 0.
  if ((core->numPlanes > 1) && (core->plane == 0)) {
    // Determine number of timer cycles taken to issue the data.
    // It can vary slightly if heavy interrupts happen, things like that.
    // Timer is still running and counting up at this point.
    uint32_t elapsed = _PM_timerGetCount(core);
    // Nudge the plane-zero time up or down (filtering to avoid jitter)
    core->bitZeroPeriod = ((core->bitZeroPeriod * 7) + elapsed + 4) / 8;
    // But don't allow it to drop below the minimum period calculated during
    // begin(), that's a hard limit and would just waste cycles.
    if (core->bitZeroPeriod < core->minPeriod) {
      core->bitZeroPeriod = core->minPeriod;
    }
  }
}

#if !defined _PM_CUSTOM_BLAST

// Innermost data-stuffing loop functions

// The presence of a bit-toggle register can make the data-stuffing loop a
// fair bit faster (2 PORT accesses per column vs 3). But ironically, some
// devices (e.g. SAMD51) can outpace the matrix max CLK speed, so we slow
// them down with a few NOPs. These are defined in arch.h as needed.
// _PM_clockHoldLow is whatever code necessary to delay the clock rise
// after data is placed on the PORT. _PM_clockHoldHigh is code for delay
// before setting the clock back low. If undefined, nothing goes there.

#if !defined(PEW) // arch.h can define a custom PEW if needed (e.g. ESP32)

#if !defined(_PM_STRICT_32BIT_IO) // Partial access to 32-bit GPIO OK

#if defined(_PM_portToggleRegister)
#define PEW                                                                    \
  *toggle = *data++; /* Toggle in new data + toggle clock low */               \
  _PM_clockHoldLow;                                                            \
  *toggle = clock; /* Toggle clock high */                                     \
  _PM_clockHoldHigh;
#else
#define PEW                                                                    \
  *set = *data++; /* Set RGB data high */                                      \
  _PM_clockHoldLow;                                                            \
  *set_full = clock; /* Set clock high */                                      \
  _PM_clockHoldHigh;                                                           \
  *clear_full = rgbclock; /* Clear RGB data + clock */                         \
  ///< Bitbang one set of RGB data bits to matrix
#endif

#else // ONLY 32-bit GPIO

#if defined(_PM_portToggleRegister)
#define PEW                                                                    \
  *toggle = *data++ << shift; /* Toggle in new data + toggle clock low */      \
  _PM_clockHoldLow;                                                            \
  *toggle = clock; /* Toggle clock high */                                     \
  _PM_clockHoldHigh;
#else
#define PEW                                                                    \
  *set = *data++ << shift; /* Set RGB data high */                             \
  _PM_clockHoldLow;                                                            \
  *set = clock; /* Set clock high */                                           \
  _PM_clockHoldHigh;                                                           \
  *clear_full = rgbclock; /* Clear RGB data + clock */                         \
  ///< Bitbang one set of RGB data bits to matrix
#endif

#endif // end 32-bit GPIO

#endif // end PEW

#if _PM_chunkSize == 1
#define PEW_UNROLL PEW
#elif _PM_chunkSize == 2
#define PEW_UNROLL PEW PEW ///< 2-way PEW unroll
#elif _PM_chunkSize == 4
#define PEW_UNROLL PEW PEW PEW PEW ///< 4-way PEW unroll
#elif _PM_chunkSize == 8
#define PEW_UNROLL PEW PEW PEW PEW PEW PEW PEW PEW ///< 8-way PEW unroll
#elif _PM_chunkSize == 16
#define PEW_UNROLL                                                             \
  PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW
#elif _PM_chunkSize == 32
#define PEW_UNROLL                                                             \
  PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW  \
      PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW
#elif _PM_chunkSize == 64
#define PEW_UNROLL                                                             \
  PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW  \
      PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW  \
          PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW  \
              PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW
#else
#error "Unimplemented _PM_chunkSize value"
#endif

// There are THREE COPIES of the following function -- one each for byte,
// word and long. If changes are made in any one of them, the others MUST
// be updated to match! (Decided against using macro tricks for the
// function, too often ends in disaster...but must be vigilant in the
// three-function maintenance then.)

IRAM_ATTR static void blast_byte(Protomatter_core *core, uint8_t *data) {
#if !defined(_PM_STRICT_32BIT_IO) // Partial access to 32-bit GPIO OK

#if defined(_PM_portToggleRegister)
  // If here, it was established in begin() that the RGB data bits and
  // clock are all within the same byte of a PORT register, else we'd be
  // in the word- or long-blasting functions now. So we just need an
  // 8-bit pointer to the PORT.
  volatile uint8_t *toggle =
      (volatile uint8_t *)core->toggleReg + core->portOffset;
#else
  // No-toggle version is a little different. If here, RGB data is all
  // in one byte of PORT register, clock can be any bit in 32-bit PORT.
  volatile uint8_t *set;              // For RGB data set
  volatile _PM_PORT_TYPE *set_full;   // For clock set
  volatile _PM_PORT_TYPE *clear_full; // For RGB data + clock clear
  set = (volatile uint8_t *)core->setReg + core->portOffset;
  set_full = (volatile _PM_PORT_TYPE *)core->setReg;
  clear_full = (volatile _PM_PORT_TYPE *)core->clearReg;
  _PM_PORT_TYPE rgbclock = core->rgbAndClockMask; // RGB + clock bit
#endif
  _PM_PORT_TYPE clock = core->clockMask; // Clock bit
  uint16_t chunks = (core->chainBits + (_PM_chunkSize - 1)) / _PM_chunkSize;

  // PORT has already been initialized with RGB data + clock bits
  // all LOW, so we don't need to initialize that state here.

  while (chunks--) {
    PEW_UNROLL // _PM_chunkSize RGB+clock writes
  }

#if defined(_PM_portToggleRegister)
  // Want the PORT left with RGB data and clock LOW on function exit
  // (so it's easier to see on 'scope, and to prime it for the next call).
  // This is implicit in the no-toggle case (due to how the PEW macro
  // works), but toggle case requires explicitly clearing those bits.
  // rgbAndClockMask is an 8-bit value when toggling, hence offset here.
  *((volatile uint8_t *)core->clearReg + core->portOffset) =
      core->rgbAndClockMask;
#endif

#else // ONLY 32-bit GPIO

#if defined(_PM_portToggleRegister)
  volatile _PM_PORT_TYPE *toggle = (volatile _PM_PORT_TYPE *)core->toggleReg;
#else
  volatile _PM_PORT_TYPE *set = (volatile _PM_PORT_TYPE *)core->setReg;
  volatile _PM_PORT_TYPE *clear_full = (volatile _PM_PORT_TYPE *)core->clearReg;
  _PM_PORT_TYPE rgbclock = core->rgbAndClockMask; // RGB + clock bit
#endif
  _PM_PORT_TYPE clock = core->clockMask; // Clock bit
  uint8_t shift = core->portOffset * 8;
  uint8_t chunks = (core->chainBits + (_PM_chunkSize - 1)) / _PM_chunkSize;

  // PORT has already been initialized with RGB data + clock bits
  // all LOW, so we don't need to initialize that state here.

  while (chunks--) {
    PEW_UNROLL // _PM_chunkSize RGB+clock writes
  }

#if defined(_PM_portToggleRegister)
  *((volatile uint32_t *)core->clearReg) = core->rgbAndClockMask;
#endif

#endif // 32-bit GPIO
}

IRAM_ATTR static void blast_word(Protomatter_core *core, uint16_t *data) {
#if !defined(_PM_STRICT_32BIT_IO) // Partial access to 32-bit GPIO OK

#if defined(_PM_portToggleRegister)
  // See notes above -- except now 16-bit word in PORT.
  volatile uint16_t *toggle =
      (volatile uint16_t *)core->toggleReg + core->portOffset;
#else
  volatile uint16_t *set;             // For RGB data set
  volatile _PM_PORT_TYPE *set_full;   // For clock set
  volatile _PM_PORT_TYPE *clear_full; // For RGB data + clock clear
  set = (volatile uint16_t *)core->setReg + core->portOffset;
  set_full = (volatile _PM_PORT_TYPE *)core->setReg;
  clear_full = (volatile _PM_PORT_TYPE *)core->clearReg;
  _PM_PORT_TYPE rgbclock = core->rgbAndClockMask; // RGB + clock bit
#endif
  _PM_PORT_TYPE clock = core->clockMask; // Clock bit
  uint8_t chunks = (core->chainBits + (_PM_chunkSize - 1)) / _PM_chunkSize;
  while (chunks--) {
    PEW_UNROLL // _PM_chunkSize RGB+clock writes
  }
#if defined(_PM_portToggleRegister)
  // rgbAndClockMask is a 16-bit value when toggling, hence offset here.
  *((volatile uint16_t *)core->clearReg + core->portOffset) =
      core->rgbAndClockMask;
#endif

#else // ONLY 32-bit GPIO

#if defined(_PM_portToggleRegister)
  volatile _PM_PORT_TYPE *toggle = (volatile _PM_PORT_TYPE *)core->toggleReg;
#else
  volatile _PM_PORT_TYPE *set = (volatile _PM_PORT_TYPE *)core->setReg;
  volatile _PM_PORT_TYPE *clear_full = (volatile _PM_PORT_TYPE *)core->clearReg;
  _PM_PORT_TYPE rgbclock = core->rgbAndClockMask; // RGB + clock bit
#endif
  _PM_PORT_TYPE clock = core->clockMask; // Clock bit
  uint8_t shift = core->portOffset * 16;
  uint8_t chunks = (core->chainBits + (_PM_chunkSize - 1)) / _PM_chunkSize;
  while (chunks--) {
    PEW_UNROLL // _PM_chunkSize RGB+clock writes
  }
#if defined(_PM_portToggleRegister)
  *((volatile _PM_PORT_TYPE *)core->clearReg) = core->rgbAndClockMask;
#endif

#endif // 32-bit GPIO
}

IRAM_ATTR static void blast_long(Protomatter_core *core, uint32_t *data) {
#if defined(_PM_portToggleRegister)
  // See notes above -- except now full 32-bit PORT.
  volatile uint32_t *toggle = (volatile uint32_t *)core->toggleReg;
#else
  // Note in this case two copies exist of the PORT set register.
  // The optimizer will most likely simplify this; leaving as-is, not
  // wanting a special case of the PEW macro due to divergence risk.
  volatile uint32_t *set; // For RGB data set
#if !defined(_PM_STRICT_32BIT_IO)
  volatile _PM_PORT_TYPE *set_full; // For clock set
  set_full = (volatile _PM_PORT_TYPE *)core->setReg;
#endif
  volatile _PM_PORT_TYPE *clear_full; // For RGB data + clock clear
  set = (volatile uint32_t *)core->setReg;
  clear_full = (volatile _PM_PORT_TYPE *)core->clearReg;
  _PM_PORT_TYPE rgbclock = core->rgbAndClockMask; // RGB + clock bit
#endif
  _PM_PORT_TYPE clock = core->clockMask; // Clock bit
#if defined(_PM_STRICT_32BIT_IO)
  uint8_t shift = 0;
#endif
  uint8_t chunks = (core->chainBits + (_PM_chunkSize - 1)) / _PM_chunkSize;
  while (chunks--) {
    PEW_UNROLL // _PM_chunkSize RGB+clock writes
  }
#if defined(_PM_portToggleRegister)
  *(volatile uint32_t *)core->clearReg = core->rgbAndClockMask;
#endif
}

#endif // end !_PM_CUSTOM_BLAST

// Returns current value of frame counter and resets its value to zero.
// Two calls to this, timed one second apart (or use math with other
// intervals), can be used to get a rough frames-per-second value for
// the matrix (since this is difficult to estimate beforehand).
uint32_t _PM_getFrameCount(Protomatter_core *core) {
  uint32_t count = 0;
  if ((core)) {
    count = core->frameCount;
    core->frameCount = 0;
  }
  return count;
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

// Set all RGB data bits high or low. Its done this way (rather than via
// setReg/clearReg members, which would be a single call) because the latter
// aren't configured on some architectures (e.g. ESP32-S3) where special
// peripherals are used. Nothing in FM6126A init needs to be super optimal
// as it's only called once briefly on startup...clocking takes more time!
static void _PM_rgbState(Protomatter_core *core, bool state) {
  for (uint8_t p = 0; p < core->parallel * 6; p++) {
    if (state)
      _PM_pinHigh(core->rgbPins[p]);
    else
      _PM_pinLow(core->rgbPins[p]);
  }
}

// Configure one register of FM6126A. Latch assumed LOW on entry.
static void _PM_FM6126A_reg(Protomatter_core *core, uint8_t reg,
                            uint16_t dataMask) {
  for (uint16_t i = 0; i < core->chainBits; i++) {
    _PM_rgbState(core, dataMask & (0x8000 >> (i & 15)));

    if (i > (core->chainBits - reg))
      _PM_pinHigh(core->latch.pin);

    _PM_pinHigh(core->clockPin);
    _PM_delayMicroseconds(1);
    _PM_pinLow(core->clockPin);
    _PM_delayMicroseconds(1);
  }

  _PM_pinLow(core->latch.pin);
}

// Adapted from SmartMatrix: FM6126A chipset reset sequence,
// harmless and ignored by other chips. Thanks to Bob Davis:
// bobdavis321.blogspot.com/2019/02/p3-64x32-hub75e-led-matrix-panels-with.html
static void _PM_resetFM6126A(Protomatter_core *core) {
  // On arrival here, clock and latch are low, OE is high, no need to config,
  // but they must be in the same states when finished.

  _PM_FM6126A_reg(core, 12, 0b0111111111111111);
  _PM_FM6126A_reg(core, 13, 0b0000000001000000);

  _PM_rgbState(core, 0); // Set all RGB low so port toggle can work
}

uint8_t _PM_duty = _PM_defaultDuty;

void _PM_setDuty(uint8_t d) { _PM_duty = (d > _PM_maxDuty) ? _PM_maxDuty : d; }

#if defined(ARDUINO) || defined(CIRCUITPY)

// Arduino and CircuitPython happen to use the same internal canvas
// representation.

// 16-bit (565) color conversion functions go here (rather than in the
// Arduino lib .cpp) because knowledge is required of chunksize and the
// toggle register (or lack thereof), which are only known to this file,
// not the .cpp or anywhere else. However...this file knows nothing of
// the GFXcanvas16 type (from Adafruit_GFX...another C++ lib), so the
// .cpp just passes down some pointers and minimal info about the canvas
// buffer. It's probably not ideal but this is my life now, oh well.

// Different runtime environments (which might not use the 565 canvas
// format) will need their own conversion functions.

// There are THREE COPIES of the following function -- one each for byte,
// word and long. If changes are made in any one of them, the others MUST
// be updated to match! Note that they are not simple duplicates of each
// other. The byte case, for example, doesn't need to handle parallel
// matrix chains (matrix data can only be byte-sized if one chain).

// width argument comes from GFX canvas width, which may be less than
// core's bitWidth (due to padding). height isn't needed, it can be
// inferred from core->numRowPairs and core->tile.
__attribute__((noinline)) void _PM_convert_565_byte(Protomatter_core *core,
                                                    const uint16_t *source,
                                                    uint16_t width) {
  uint8_t *pinMask = (uint8_t *)core->rgbMask; // Pin bitmasks
  uint8_t *dest = (uint8_t *)core->screenData;
  if (core->doubleBuffer) {
    dest += core->bufferSize * (1 - core->activeBuffer);
  }

// #if defined(_PM_portToggleRegister)
#if defined(_PM_USE_TOGGLE_FORMAT)
#if !defined(_PM_STRICT_32BIT_IO)
  // core->clockMask mask is already an 8-bit value
  uint8_t clockMask = core->clockMask;
#else
  // core->clockMask mask is 32-bit, shift down to 8-bit for this func.
  uint8_t clockMask = core->clockMask >> (core->portOffset * 8);
#endif
#endif

  // No need to clear matrix buffer, loops below do a full overwrite
  // (except for any scanline pad, which was already initialized in the
  // begin() function and won't be touched here).

  // Determine matrix bytes per bitplane & row (row pair really):

  // Size of 1 plane of row pair (across full chain / tile set)
  uint32_t bitplaneSize =
      _PM_chunkSize * ((core->chainBits + (_PM_chunkSize - 1)) / _PM_chunkSize);
  uint8_t pad = bitplaneSize - core->chainBits; // Plane-start pad

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
// #if defined(_PM_portToggleRegister)
#if defined(_PM_USE_TOGGLE_FORMAT)
      uint8_t prior = clockMask; // Set clock bit on 1st out
#endif
      uint8_t *d2 = dest; // Incremented per-pixel across all tiles

      // Work from bottom tile to top, because data is issued in that order
      for (int8_t tile = abs(core->tile) - 1; tile >= 0; tile--) {
        const uint16_t *upperSrc, *lowerSrc; // Canvas scanline pointers
        int16_t srcIdx;
        int8_t srcInc;

        // Source pointer to tile's upper-left pixel
        const uint16_t *srcTileUL =
            source + tile * width * core->numRowPairs * 2;
        if ((tile & 1) && (core->tile < 0)) {
          // Special handling for serpentine tiles
          lowerSrc = srcTileUL + width * (core->numRowPairs - 1 - row);
          upperSrc = lowerSrc + width * core->numRowPairs;
          srcIdx = width - 1; // Work right to left
          srcInc = -1;
        } else {
          // Progressive tile
          upperSrc = srcTileUL + width * row;              // Top row
          lowerSrc = upperSrc + width * core->numRowPairs; // Bottom row
          srcIdx = 0;                                      // Left to right
          srcInc = 1;
        }

        for (uint16_t x = 0; x < width; x++, srcIdx += srcInc) {
          uint16_t upperRGB = upperSrc[srcIdx]; // Pixel in upper half
          uint16_t lowerRGB = lowerSrc[srcIdx]; // Pixel in lower half
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
// THIS is where toggle format (without toggle reg.) messes up
// #if defined(_PM_portToggleRegister)
#if defined(_PM_USE_TOGGLE_FORMAT)
          *d2++ = result ^ prior;
          prior = result | clockMask; // Set clock bit on next out
#else
          *d2++ = result;
#endif
        } // end x
      } // end tile

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
// #if defined(_PM_portToggleRegister)
#if defined(_PM_USE_TOGGLE_FORMAT)
      // If using bit-toggle register, erase the toggle bit on the
      // first element of each bitplane & row pair. The matrix-driving
      // interrupt functions correspondingly set the clock low before
      // finishing. This is all done for legibility on oscilloscope --
      // so idle clock appears LOW -- but really the matrix samples on
      // a rising edge and we could leave it high, but at this stage
      // in development just want the scope "readable."
      dest[-pad] &= ~clockMask; // Negative index is legal & intentional
#endif
      dest += bitplaneSize; // Advance one scanline in dest buffer
    } // end plane
  } // end row
}

// Corresponding function for word output -- either 12 RGB bits (2 parallel
// matrix chains), or 1 chain with RGB bits not in the same byte (but in the
// same 16-bit word). Some of the comments have been stripped out since it's
// largely the same operation, but changes are noted.
// WORD OUTPUT IS UNTESTED AND ROW TILING MAY ESPECIALLY PRESENT ISSUES.
void _PM_convert_565_word(Protomatter_core *core, uint16_t *source,
                          uint16_t width) {
  uint16_t *pinMask = (uint16_t *)core->rgbMask; // Pin bitmasks
  uint16_t *dest = (uint16_t *)core->screenData;
  if (core->doubleBuffer) {
    dest += core->bufferSize / core->bytesPerElement * (1 - core->activeBuffer);
  }

  // Size of 1 plane of row pair (across full chain / tile set)
  uint32_t bitplaneSize =
      _PM_chunkSize * ((core->chainBits + (_PM_chunkSize - 1)) / _PM_chunkSize);
  uint8_t pad = bitplaneSize - core->chainBits; // Plane-start pad

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
// #if defined(_PM_portToggleRegister)
#if defined(_PM_USE_TOGGLE_FORMAT)
  // No per-chain loop is required; one clock bit handles all chains
  uint32_t offset = 0; // Current position in the 'dest' buffer
  uint16_t mask = core->clockMask >> (core->portOffset * 16);
  for (uint8_t row = 0; row < core->numRowPairs; row++) {
    for (uint8_t plane = 0; plane < core->numPlanes; plane++) {
      dest[offset++] = 0; // First element of each plane
      for (uint16_t x = 1; x < bitplaneSize; x++) { // All subsequent items
        dest[offset++] = mask;
      }
    }
  }
#else
  memset(dest, 0, core->bufferSize);
#endif

  dest += pad; // Pad value is in 'elements,' not bytes, so this is OK

  for (uint8_t chain = 0; chain < core->parallel; chain++) {
    for (uint8_t row = 0; row < core->numRowPairs; row++) {
      uint32_t redBit = initialRedBit;
      uint32_t greenBit = initialGreenBit;
      uint32_t blueBit = initialBlueBit;
      for (uint8_t plane = 0; plane < core->numPlanes; plane++) {
// #if defined(_PM_portToggleRegister)
#if defined(_PM_USE_TOGGLE_FORMAT)
        // Since we're ORing in bits over an existing clock bit,
        // prior is 0 rather than clockMask as in the byte case.
        uint16_t prior = 0;
#endif
        uint16_t *d2 = dest; // Incremented per-pixel across all tiles

        // Work from bottom tile to top, because data is issued in that order
        for (int8_t tile = abs(core->tile) - 1; tile >= 0; tile--) {
          uint16_t *upperSrc, *lowerSrc; // Canvas scanline pointers
          int16_t srcIdx;
          int8_t srcInc;

          // Source pointer to tile's upper-left pixel
          uint16_t *srcTileUL = source + (chain * abs(core->tile) + tile) *
                                             width * core->numRowPairs * 2;
          if ((tile & 1) && (core->tile < 0)) {
            // Special handling for serpentine tiles
            lowerSrc = srcTileUL + width * (core->numRowPairs - 1 - row);
            upperSrc = lowerSrc + width * core->numRowPairs;
            srcIdx = width - 1; // Work right to left
            srcInc = -1;
          } else {
            // Progressive tile
            upperSrc = srcTileUL + width * row;              // Top row
            lowerSrc = upperSrc + width * core->numRowPairs; // Bottom row
            srcIdx = 0;                                      // Left to right
            srcInc = 1;
          }

          for (uint16_t x = 0; x < width; x++, srcIdx += srcInc) {
            uint16_t upperRGB = upperSrc[srcIdx]; // Pixel in upper half
            uint16_t lowerRGB = lowerSrc[srcIdx]; // Pixel in lower half
            uint16_t result = 0;
            if (upperRGB & redBit) {
              result |= pinMask[0];
            }
            if (upperRGB & greenBit) {
              result |= pinMask[1];
            }
            if (upperRGB & blueBit) {
              result |= pinMask[2];
            }
            if (lowerRGB & redBit) {
              result |= pinMask[3];
            }
            if (lowerRGB & greenBit) {
              result |= pinMask[4];
            }
            if (lowerRGB & blueBit) {
              result |= pinMask[5];
            }
            // Main difference here vs byte converter is each chain
            // ORs new bits into place (vs single-pass overwrite).
// #if defined(_PM_portToggleRegister)
#if defined(_PM_USE_TOGGLE_FORMAT)
            *d2++ |= result ^ prior; // Bitwise OR
            prior = result;
#else
            *d2++ |= result; // Bitwise OR
#endif
          } // end x
        } // end tile
        greenBit <<= 1;
        if (plane || (core->numPlanes < 6)) {
          redBit <<= 1;
          blueBit <<= 1;
        } else {
          redBit = 0b0000100000000000;
          blueBit = 0b0000000000000001;
        }
        dest += bitplaneSize; // Advance one scanline in dest buffer
      } // end plane
    } // end row
    pinMask += 6; // Next chain's RGB pin masks
  }
}

// Corresponding function for long output -- either several parallel chains
// (up to 5), or 1 chain with RGB bits scattered widely about the PORT.
// Same deal, comments are pared back, see above functions for explanations.
// LONG OUTPUT IS UNTESTED AND ROW TILING MAY ESPECIALLY PRESENT ISSUES.
void _PM_convert_565_long(Protomatter_core *core, uint16_t *source,
                          uint16_t width) {
  uint32_t *pinMask = (uint32_t *)core->rgbMask; // Pin bitmasks
  uint32_t *dest = (uint32_t *)core->screenData;
  if (core->doubleBuffer) {
    dest += core->bufferSize / core->bytesPerElement * (1 - core->activeBuffer);
  }

  // Size of 1 plane of row pair (across full chain / tile set)
  uint32_t bitplaneSize =
      _PM_chunkSize * ((core->chainBits + (_PM_chunkSize - 1)) / _PM_chunkSize);
  uint8_t pad = bitplaneSize - core->chainBits; // Plane-start pad

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

// #if defined(_PM_portToggleRegister)
#if defined(_PM_USE_TOGGLE_FORMAT)
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

  for (uint8_t chain = 0; chain < core->parallel; chain++) {
    for (uint8_t row = 0; row < core->numRowPairs; row++) {
      uint32_t redBit = initialRedBit;
      uint32_t greenBit = initialGreenBit;
      uint32_t blueBit = initialBlueBit;
      for (uint8_t plane = 0; plane < core->numPlanes; plane++) {
// #if defined(_PM_portToggleRegister)
#if defined(_PM_USE_TOGGLE_FORMAT)
        uint32_t prior = 0;
#endif
        uint32_t *d2 = dest; // Incremented per-pixel across all tiles

        // Work from bottom tile to top, because data is issued in that order
        for (int8_t tile = abs(core->tile) - 1; tile >= 0; tile--) {
          uint16_t *upperSrc, *lowerSrc; // Canvas scanline pointers
          int16_t srcIdx;
          int8_t srcInc;

          // Source pointer to tile's upper-left pixel
          uint16_t *srcTileUL = source + (chain * abs(core->tile) + tile) *
                                             width * core->numRowPairs * 2;
          if ((tile & 1) && (core->tile < 0)) {
            // Special handling for serpentine tiles
            lowerSrc = srcTileUL + width * (core->numRowPairs - 1 - row);
            upperSrc = lowerSrc + width * core->numRowPairs;
            srcIdx = width - 1; // Work right to left
            srcInc = -1;
          } else {
            // Progressive tile
            upperSrc = srcTileUL + width * row;              // Top row
            lowerSrc = upperSrc + width * core->numRowPairs; // Bottom row
            srcIdx = 0;                                      // Left to right
            srcInc = 1;
          }

          for (uint16_t x = 0; x < width; x++, srcIdx += srcInc) {
            uint16_t upperRGB = upperSrc[srcIdx]; // Pixel in upper half
            uint16_t lowerRGB = lowerSrc[srcIdx]; // Pixel in lower half
            uint32_t result = 0;
            if (upperRGB & redBit) {
              result |= pinMask[0];
            }
            if (upperRGB & greenBit) {
              result |= pinMask[1];
            }
            if (upperRGB & blueBit) {
              result |= pinMask[2];
            }
            if (lowerRGB & redBit) {
              result |= pinMask[3];
            }
            if (lowerRGB & greenBit) {
              result |= pinMask[4];
            }
            if (lowerRGB & blueBit) {
              result |= pinMask[5];
            }
            // Main difference here vs byte converter is each chain
            // ORs new bits into place (vs single-pass overwrite).
// #if defined(_PM_portToggleRegister)
#if defined(_PM_USE_TOGGLE_FORMAT)
            *d2++ |= result ^ prior; // Bitwise OR
            prior = result;
#else
            *d2++ |= result; // Bitwise OR
#endif
          } // end x
        } // end tile
        greenBit <<= 1;
        if (plane || (core->numPlanes < 6)) {
          redBit <<= 1;
          blueBit <<= 1;
        } else {
          redBit = 0b0000100000000000;
          blueBit = 0b0000000000000001;
        }
        dest += bitplaneSize; // Advance one scanline in dest buffer
      } // end plane
    } // end row
    pinMask += 6; // Next chain's RGB pin masks
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

#endif // END ARDUINO || CIRCUITPY

/* NOTES TO FUTURE SELF ----------------------------------------------------

ON BYTES, WORDS and LONGS:
I've gone back and forth between implementing all this either as it
currently is (with byte, word and long cases for various steps), or using
a uint32_t[64] table for expanding RGB bit combos to PORT bit combos.
The latter would certainly simplify the code a ton, and the additional
table lookup step wouldn't significantly impact performance, especially
going forward with faster processors (several devices already require a
few NOPs in the innermost loop to avoid outpacing the matrix).
BUT, the reason this is NOT currently done is that it only allows for a
single matrix chain (doing parallel chains would require either an
impractically large lookup table, or adding together multiple tables'
worth of bitmasks, which would slow things down in the vital inner loop).
Although parallel matrix chains aren't yet 100% implemented in this code
right now, I wanted to leave that possibility for the future, as a way to
handle larger matrix combos, because long chains will slow down the
refresh rate.
*/
