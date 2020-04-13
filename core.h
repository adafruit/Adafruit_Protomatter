#ifndef _PROTOMATTER_CORE_H_
#define _PROTOMATTER_CORE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// Status type returned by some functions.
typedef enum {
    PROTOMATTER_OK,         // Everything is hunky-dory!
    PROTOMATTER_ERR_PINS,   // Clock and/or data pins on different PORTs
    PROTOMATTER_ERR_MALLOC, // Couldn't allocate memory for display
    PROTOMATTER_ERR_ARG,    // Bad input to function
} ProtomatterStatus;

// Struct for matrix control lines NOT related to RGB data or clock, i.e.
// latch, OE and address lines. RGB data and clock ("RGBC") are handled
// differently as they have specific requirements (and might use a toggle
// register if present). The data conversion functions need bitmasks for
// RGB data but do NOT need the set or clear registers, so those items
// are also declared as separate things in the core structure that follows.
typedef struct {
    volatile uint32_t *setReg;          // GPIO bit set register
    volatile uint32_t *clearReg;        // GPIO bit clear register
    uint32_t           bit;             // GPIO bitmask
    uint8_t            pin;             // Some identifier, e.g. Arduino pin #
} _PM_pin;

// Struct with info about an RGB matrix chain and lots of state and buffer
// details for the library. Toggle-related items in this structure MUST be
// declared even if the device lacks GPIO bit-toggle registers (i.e. don't
// do an ifdef check around these). All hardware-specific details (including
// the presence or lack of toggle registers) are isolated to a single
// file -- arch.h -- which should ONLY be included by core.c, and ifdef'ing
// them would result in differing representations of this structure which
// must be shared between the library and calling code. (An alternative is
// to put any toggle-specific stuff at the end of the struct with an ifdef
// check, but that's just dirty pool and asking for trouble.)
typedef struct {
    void              *timer;           // Arch-specific timer/counter info
    void              *setReg;          // RGBC bit set register (cast to use)
    void              *clearReg;        // RGBC bit clear register "
    void              *toggleReg;       // RGBC bit toggle register "
    uint8_t           *rgbPins;         // Array of RGB data pins (mult of 6)
    void              *rgbMask;         // PORT bit mask for each RGB pin
    uint32_t           clockMask;       // PORT bit mask for RGB clock
    uint32_t           rgbAndClockMask; // PORT bit mask for RGB data + clock
    volatile uint32_t *addrPortToggle;  // See singleAddrPort below
    void              *screenData;      // Per-bitplane RGB data for matrix
    _PM_pin            latch;           // RGB data latch
    _PM_pin            oe;              // !OE (LOW out enable)
    _PM_pin           *addr;            // Array of address pins
    uint32_t           bufferSize;      // Bytes per matrix buffer
    uint32_t           bitZeroPeriod;   // Bitplane 0 timer period
    uint32_t           minPeriod;       // Plane 0 timer period for ~250Hz
    volatile uint32_t  frameCount;      // For estimating refresh rate
    uint16_t           width;           // Matrix chain width in bits
    uint8_t            bytesPerElement; // Using 8, 16 or 32 bits of PORT?
    uint8_t            clockPin;        // RGB clock pin identifier
    uint8_t            parallel;        // Number of concurrent matrix outs
    uint8_t            numAddressLines; // Number of address line pins
    uint8_t            portOffset;      // Active 8- or 16-bit pos. in PORT
    uint8_t            numPlanes;       // Display bitplanes (1 to 6)
    uint8_t            numRowPairs;     // Addressable row pairs
    bool               doubleBuffer;    // 2X buffers for clean switchover
    bool               singleAddrPort;  // If 1, all addr lines on same PORT
    volatile uint8_t   activeBuffer;    // Index of currently-displayed buf
    volatile uint8_t   plane;           // Current bitplane (changes in ISR)
    volatile uint8_t   row;             // Current scanline (changes in ISR)
    volatile uint8_t   prevRow;         // Scanline from prior ISR
    volatile bool      swapBuffers;     // If 1, awaiting double-buf switch
} Protomatter_core;

// Protomatter core function prototypes. Environment-specific code (like the
// Adafruit_Protomatter class for Arduino) calls on these underlying things,
// and has to provide a few extras of its own (interrupt handlers and such).
// User code shouldn't need to invoke any of them directly.
extern ProtomatterStatus _PM_init(Protomatter_core *core,
  uint16_t bitWidth, uint8_t bitDepth,
  uint8_t rgbCount, uint8_t *rgbList,
  uint8_t addrCount, uint8_t *addrList,
  uint8_t clockPin, uint8_t latchPin, uint8_t oePin,
  bool doubleBuffer, void *timer);
extern ProtomatterStatus _PM_begin(Protomatter_core *core);
extern void              _PM_stop(Protomatter_core *core);
extern void              _PM_resume(Protomatter_core *core);
extern void              _PM_free(Protomatter_core *core);
extern void              _PM_row_handler(Protomatter_core *core);
extern uint32_t          _PM_getFrameCount(Protomatter_core *core);
extern void              _PM_timerStart(void *tptr, uint32_t period);
extern uint32_t          _PM_timerStop(void *tptr);
extern uint32_t          _PM_timerGetCount(void *tptr);
extern void              _PM_convert_565(Protomatter_core *core,
  uint16_t *source, uint16_t width);
extern void              _PM_swapbuffer_maybe(Protomatter_core *core);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // _PROTOMATTER_CORE_H_
