/*!
 * @file core.h
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

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/** Status type returned by some functions. */
typedef enum {
  PROTOMATTER_OK,         // Everything is hunky-dory!
  PROTOMATTER_ERR_PINS,   // Clock and/or data pins on different PORTs
  PROTOMATTER_ERR_MALLOC, // Couldn't allocate memory for display
  PROTOMATTER_ERR_ARG,    // Bad input to function
} ProtomatterStatus;

/** Struct for matrix control lines NOT related to RGB data or clock, i.e.
    latch, OE and address lines. RGB data and clock ("RGBC") are handled
    differently as they have specific requirements (and might use a toggle
    register if present). The data conversion functions need bitmasks for
    RGB data but do NOT need the set or clear registers, so those items are
    also declared as separate things in the core structure that follows. */
typedef struct {
  volatile void *setReg;   ///< GPIO bit set register
  volatile void *clearReg; ///< GPIO bit clear register
  uint32_t bit;            ///< GPIO bitmask
  uint8_t pin;             ///< Some unique ID, e.g. Arduino pin #
} _PM_pin;

/** Struct with info about an RGB matrix chain and lots of state and buffer
    details for the library. Toggle-related items in this structure MUST be
    declared even if the device lacks GPIO bit-toggle registers (i.e. don't
    do an ifdef check around these). All hardware-specific details (including
    the presence or lack of toggle registers) are isolated to a single
    file -- arch.h -- which should ONLY be included by core.c, and ifdef'ing
    them would result in differing representations of this structure which
    must be shared between the library and calling code. (An alternative is
    to put any toggle-specific stuff at the end of the struct with an ifdef
    check, but that's just dirty pool and asking for trouble.) */
typedef struct {
  void *timer;                   ///< Arch-specific timer/counter info
  void *setReg;                  ///< RGBC bit set register (cast to use)
  void *clearReg;                ///< RGBC bit clear register "
  void *toggleReg;               ///< RGBC bit toggle register "
  uint8_t *rgbPins;              ///< Array of RGB data pins (mult of 6)
  void *rgbMask;                 ///< PORT bit mask for each RGB pin
  uint32_t clockMask;            ///< PORT bit mask for RGB clock
  uint32_t rgbAndClockMask;      ///< PORT bit mask for RGB data + clock
  volatile void *addrPortToggle; ///< See singleAddrPort below
  void *screenData;              ///< Per-bitplane RGB data for matrix
  _PM_pin latch;                 ///< RGB data latch
  _PM_pin oe;                    ///< !OE (LOW out enable)
  _PM_pin *addr;                 ///< Array of address pins
  uint32_t bufferSize;           ///< Bytes per matrix buffer
  uint32_t bitZeroPeriod;        ///< Bitplane 0 timer period
  uint32_t minPeriod;            ///< Plane 0 timer period for ~250Hz
  volatile uint32_t frameCount;  ///< For estimating refresh rate
  uint16_t width;                ///< Matrix chain width only in bits
  uint16_t chainBits;            ///< Matrix chain width*tiling in bits
  uint8_t bytesPerElement;       ///< Using 8, 16 or 32 bits of PORT?
  uint8_t clockPin;              ///< RGB clock pin identifier
  uint8_t parallel;              ///< Number of concurrent matrix outs
  uint8_t numAddressLines;       ///< Number of address line pins
  uint8_t portOffset;            ///< Active 8- or 16-bit pos. in PORT
  uint8_t numPlanes;             ///< Display bitplanes (1 to 6)
  uint8_t numRowPairs;           ///< Addressable row pairs
  int8_t tile;                   ///< Vertical tiling repetitions
  bool doubleBuffer;             ///< 2X buffers for clean switchover
  bool singleAddrPort;           ///< If 1, all addr lines on same PORT
  volatile uint8_t activeBuffer; ///< Index of currently-displayed buf
  volatile uint8_t plane;        ///< Current bitplane (changes in ISR)
  volatile uint8_t row;          ///< Current scanline (changes in ISR)
  volatile uint8_t prevRow;      ///< Scanline from prior ISR
  volatile bool swapBuffers;     ///< If 1, awaiting double-buf switch
} Protomatter_core;

// Protomatter core function prototypes. Environment-specific code (like the
// Adafruit_Protomatter class for Arduino) calls on these underlying things,
// and has to provide a few extras of its own (interrupt handlers and such).
// User code shouldn't need to invoke any of them directly.

/*!
  @brief  Initialize values in Protomatter_core structure.
  @param  core  Pointer to Protomatter_core structure.
  @param  bitWidth      Total width of RGB matrix chain, in pixels.
                        Usu. some multiple of 32, but maybe exceptions.
  @param  bitDepth      Color "depth" in bitplanes, determines range of
                        shades of red, green and blue. e.g. passing 4
                        bits = 16 shades ea. R,G,B = 16x16x16 = 4096
                        colors.
  @param  rgbCount      Number of "sets" of RGB data pins, each set
                        containing 6 pins (2 ea. R,G,B). Typically 1,
                        indicating a single matrix (or matrix chain).
                        In theory (but not yet extensively tested),
                        multiple sets of pins can be driven in parallel,
                        up to 5 on some devices (if the hardware design
                        provides all those bits on one PORT).
  @param  rgbList       A uint8_t array of pins (values are platform-
                        dependent), 6X the prior rgbCount value,
                        corresponding to the 6 output color bits for a
                        matrix (or chain). Order is upper-half red, green,
                        blue, lower-half red, green blue (repeat for each
                        add'l chain). All the RGB pins (plus the clock pin
                        below on some architectures) MUST be on the same
                        PORT register. It's recommended (but not required)
                        that all RGB pins (and clock depending on arch) be
                        within the same byte of a PORT (but do not need to
                        be sequential or contiguous within that byte) for
                        more efficient RAM utilization. For two concurrent
                        chains, same principle but 16-bit word.
  @param  addrCount     Number of row address lines required of matrix.
                        Total pixel height is then 2 x 2^addrCount, e.g.
                        32-pixel-tall matrices have 4 row address lines.
  @param  addrList      A uint8_t array of pins (platform-dependent pin
                        numbering), one per row address line.
  @param  clockPin      RGB clock pin (platform-dependent pin #).
  @param  latchPin      RGB data latch pin (platform-dependent pin #).
  @param  oePin         Output enable pin (platform-dependent pin #),
                        active low.
  @param  doubleBuffer  If true, two matrix buffers are allocated,
                        so changing display contents doesn't introduce
                        artifacts mid-conversion. Requires ~2X RAM.
  @param  tile          If multiple matrices are chained and stacked
                        vertically (rather than or in addition to
                        horizontally), the number of vertical tiles is
                        specified here. Positive values indicate a
                        "progressive" arrangement (always left-to-right),
                        negative for a "serpentine" arrangement (alternating
                        180 degree orientation). Horizontal tiles are implied
                        in the 'bitWidth' argument.
  @param  timer         Pointer to timer peripheral or timer-related
                        struct (architecture-dependent), or NULL to
                        use a default timer ID (also arch-dependent).
  @return A ProtomatterStatus status, one of:
          PROTOMATTER_OK if everything is good.
          PROTOMATTER_ERR_PINS if data and/or clock pins are split across
          different PORTs.
          PROTOMATTER_ERR_MALLOC if insufficient RAM to allocate display
          memory.
          PROTOMATTER_ERR_ARG if a bad value (core or timer pointer) was
          passed in.
*/
extern ProtomatterStatus _PM_init(Protomatter_core *core, uint16_t bitWidth,
                                  uint8_t bitDepth, uint8_t rgbCount,
                                  uint8_t *rgbList, uint8_t addrCount,
                                  uint8_t *addrList, uint8_t clockPin,
                                  uint8_t latchPin, uint8_t oePin,
                                  bool doubleBuffer, int8_t tile, void *timer);

/*!
  @brief  Allocate display buffers and populate additional elements of a
          Protomatter matrix.
  @param  core  Pointer to Protomatter_core structure.
  @return A ProtomatterStatus status, one of:
          PROTOMATTER_OK if everything is good.
          PROTOMATTER_ERR_PINS if data and/or clock pins are split across
          different PORTs.
          PROTOMATTER_ERR_MALLOC if insufficient RAM to allocate display
          memory.
          PROTOMATTER_ERR_ARG if a bad value.
*/
extern ProtomatterStatus _PM_begin(Protomatter_core *core);

/*!
  @brief  Disable (but do not deallocate) a Protomatter matrix. Disables
          matrix by setting OE pin HIGH and writing all-zero data to
          matrix shift registers, so it won't halt with lit LEDs.
  @param  core  Pointer to Protomatter_core structure.
*/
extern void _PM_stop(Protomatter_core *core);

/*!
  @brief  Start or restart a matrix. Initialize counters, configure and
          start timer.
  @param  core  Pointer to Protomatter_core structure.
*/
extern void _PM_resume(Protomatter_core *core);

/*!
  @brief  Deallocate memory associated with Protomatter_core structure
          (e.g. screen data, pin lists for data and rows). Does not
          deallocate the structure itself.
  @param  core  Pointer to Protomatter_core structure.
*/
extern void _PM_deallocate(Protomatter_core *core);

/*!
  @brief  Matrix "row handler" that's called by the timer interrupt.
          Handles row address lines and issuing data to matrix.
  @param  core  Pointer to Protomatter_core structure.
*/
extern void _PM_row_handler(Protomatter_core *core);

// *********************************************************************
// NOTE: AS OF 1.3.0, TIMER-RELATED FUNCTIONS REQUIRE A Protomatter_core
// STRUCT POINTER, RATHER THAN A void* TIMER-RELATED POINTER.
// *********************************************************************

/*!
  @brief  Returns current value of frame counter and resets its value to
          zero. Two calls to this, timed one second apart (or use math with
          other intervals), can be used to get a rough frames-per-second
          value for the matrix (since this is difficult to estimate
          beforehand).
  @param  core  Pointer to Protomatter_core structure.
  @return Frame count since previous call to function, as a uint32_t.
*/
extern uint32_t _PM_getFrameCount(Protomatter_core *core);

/*!
  @brief  Start (or restart) a timer/counter peripheral.
  @param  core    Pointer to Protomatter core structure, from which timer
                  details can be derived.
  @param  period  Timer 'top' / rollover value.
*/
extern void _PM_timerStart(Protomatter_core *core, uint32_t period);

/*!
  @brief  Stop timer/counter peripheral.
  @param  core    Pointer to Protomatter core structure, from which timer
                  details can be derived.
  @return Counter value when timer was stopped.
*/
extern uint32_t _PM_timerStop(Protomatter_core *core);

/*!
  @brief  Query a timer/counter peripheral's current count.
  @param  core    Pointer to Protomatter core structure, from which timer
                  details can be derived.
  @return Counter value.
*/
extern uint32_t _PM_timerGetCount(Protomatter_core *core);

/*!
  @brief  Pauses until the next vertical blank to avoid 'tearing' animation
          (if display is double-buffered). If single-buffered, has no effect.
  @param  core  Pointer to Protomatter_core structure.
*/
extern void _PM_swapbuffer_maybe(Protomatter_core *core);

/*!
  @brief  Adjust duty cycle of HUB75 clock signal. This is not supported on
          all architectures.
  @param  d  Duty setting, 0 minimum. Increasing values generate higher clock
             duty cycles at the same frequency. Arbitrary granular units, max
             varies by architecture and CPU speed, if supported at all.
             e.g. SAMD51 @ 120 MHz supports 0 (~50% duty) through 2 (~75%).
*/
extern void _PM_setDuty(uint8_t d);

#if defined(ARDUINO) || defined(CIRCUITPY)

/*!
  @brief  Converts image data from GFX16 canvas to the matrices weird
          internal format.
  @param  core    Pointer to Protomatter_core structure.
  @param  source  Pointer to source image data (see Adafruit_GFX 16-bit
                  canvas type for format).
  @param  width   Width of canvas in pixels, as this may be different than
                  the matrix pixel width due to row padding.
*/
extern void _PM_convert_565(Protomatter_core *core, uint16_t *source,
                            uint16_t width);

#endif // END ARDUINO || CIRCUITPY

#ifdef __cplusplus
} // extern "C"
#endif
