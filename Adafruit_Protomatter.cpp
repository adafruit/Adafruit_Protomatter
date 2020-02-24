#include "arch.h"
#include "Adafruit_Protomatter.h"
static Adafruit_Protomatter *protoPtr = NULL;

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
#define _PM_MAX_REFRESH_HZ 250

Adafruit_Protomatter::Adafruit_Protomatter(
  uint16_t bitWidth, uint8_t bitDepth,
  uint8_t rgbCount, uint8_t *rgbList, uint8_t addrCount,
  uint8_t *addrList, uint8_t clockPin, uint8_t latchPin,
  uint8_t oePin, bool doubleBuffer) :
  GFXcanvas16(bitWidth, (2 << addrCount) * rgbCount), parallel(rgbCount),
  numAddressLines(addrCount), clockPin(clockPin), latchPin(latchPin),
  oePin(oePin), doubleBuffer(doubleBuffer), swapBuffers(0), screenData(NULL),
  rgbPins(NULL), bitZeroPeriod(bitWidth * 5), frameCount(0) {

    if(bitDepth > 6) bitDepth = 6;
    numPlanes   = bitDepth;
    numRowPairs = 1 << addrCount;

    // Make a copy of the rgbList and addrList tables in case
    // they're passed from local vars on the stack.
    rgbCount *= 6; // Convert # of parallel chains to # of RGB pins
    if((rgbPins = (uint8_t *)malloc(rgbCount + addrCount)) != NULL) {
        memcpy(rgbPins, rgbList, rgbCount);
        addrPins = &rgbPins[rgbCount];
        memcpy(addrPins, addrList, addrCount);
    }

    // screenData is NOT allocated here because data size (byte, word, long)
    // is not known until begin() evaluates all the pin bitmasks.
}

Adafruit_Protomatter::~Adafruit_Protomatter(void) {
    _PM_timerStop();
    protoPtr = NULL;
    if(screenData) free(screenData);
    if(rgbPins)    free(rgbPins);
}

ProtomatterStatus Adafruit_Protomatter::begin(void) {
    if(!rgbPins) {
        // rgbPins will be NULL if copy failed to allocate.
        return PROTOMATTER_ERR_MALLOC;
    }

    // Verify that rgbPins and clockPin are all on the same PORT. If not,
    // return an error. Pin list is not freed; please invoke destructor.
    // Also get bitmask of which bits within 32-bit PORT register are
    // referenced.
    uint8_t *port = (uint8_t *)_PM_portOutRegister(clockPin);
#if defined(_PM_portToggleRegister)
    // If a bit-toggle register is present, the clock pin is included
    // in determining which bytes of the PORT register are used (and thus
    // the data storage efficiency).
    uint32_t bitMask = _PM_portBitMask(clockPin);
#else
    // If no bit-toggle register, clock pin can be on any bit, doesn't
    // affect storage efficiency.
    uint32_t bitMask = 0;
#endif

    for(uint8_t i=0; i<parallel * 6; i++) {
        uint8_t *p2 = (uint8_t *)_PM_portOutRegister(rgbPins[i]);
        if(p2 != port) {
            return PROTOMATTER_ERR_PINS;
        }
        bitMask |= _PM_portBitMask(rgbPins[i]);
    }

    // Start planning for screen data allocation (happens later)
    uint8_t  chunks      = (WIDTH + (_PM_chunkSize - 1)) / _PM_chunkSize;
    uint16_t columns     = chunks * _PM_chunkSize; // Padded matrix width
    uint8_t  rowPairs    = 1 << numAddressLines;   // 2 rows per byte/word/etc.
    uint32_t screenBytes = columns * rowPairs * numPlanes;

    // Determine data type for internal representation. If all the data
    // bitmasks (and possibly clock bitmask, depending whether toggle-bits
    // register is present) are in the same byte, this can be stored more
    // compact than if they're spread across a word or long.
    uint8_t byteMask = 0;
    if(bitMask & 0xFF000000) byteMask |= 0b1000;
    if(bitMask & 0x00FF0000) byteMask |= 0b0100;
    if(bitMask & 0x0000FF00) byteMask |= 0b0010;
    if(bitMask & 0x000000FF) byteMask |= 0b0001;
    switch(byteMask) {
      case 0b0001:           // If all PORT bits are in the same byte...
      case 0b0010:
      case 0b0100:
      case 0b1000:
        bytesPerElement = 1; // Use 8-bit PORT accesses.
        break;
      case 0b0011:           // If all PORT bits are in upper or lower word...
      case 0b1100:
        bytesPerElement = 2; // Use 16-bit PORT accesses.
        // Although some devices might tolerate unaligned 16-bit accesses
        // ('middle' word of 32-bit PORT), that is NOT handled here.
        // It's a portability liability.
        break;
      default:               // Any other situation...
        bytesPerElement = 4; // Use 32-bit PORT accesses.
        break;
    }

    screenBytes *= bytesPerElement;
    bufferSize   = screenBytes;        // Bytes per matrix buffer (1 or 2)
    if(doubleBuffer) screenBytes *= 2; // Bytes total for matrix buffer(s)
    uint32_t rgbMaskBytes = parallel * 6 * bytesPerElement;

    // Allocate matrix buffer(s). Don't worry about the return type...
    // though we might be using words or longs for certain pin configs,
    // malloc() by definition always aligns to the longest type.
    if((screenData = (uint8_t *)malloc(screenBytes + rgbMaskBytes)) == NULL) {
        return PROTOMATTER_ERR_MALLOC;
    }

    // rgbMask data follows the matrix buffer(s)
    rgbMask = screenData + screenBytes;

#if !defined(_PM_portToggleRegister)
    // Clear entire screenData buffer so there's no cruft in any pad bytes
    // (if using toggle register, each is set to clockMask below instead).
    memset(screenData, 0, screenBytes);
#endif

    if(bytesPerElement == 1) {
        portOffset      = _PM_byteOffset(rgbPins[0]);
#if defined(_PM_portToggleRegister)
        // Clock and rgbAndClockMask are 8-bit values
        clockMask       = _PM_portBitMask(clockPin) >> (portOffset * 8);
        rgbAndClockMask = (bitMask >> (portOffset * 8)) | clockMask;
        memset(screenData, clockMask, screenBytes);
#else
        // Clock and rgbAndClockMask are 32-bit values
        clockMask       = _PM_portBitMask(clockPin);
        rgbAndClockMask = bitMask | clockMask;
#endif
        for(uint8_t i=0; i<parallel * 6; i++) {
            ((uint8_t *)rgbMask)[i] =  // Pin bitmasks are stored 8-bit
              _PM_portBitMask(rgbPins[i]) >> (portOffset * 8);
        }
    } else if(bytesPerElement == 2) {
        portOffset      = _PM_wordOffset(rgbPins[0]);
#if defined(_PM_portToggleRegister)
        // Clock and rgbAndClockMask are 16-bit values
        clockMask       = _PM_portBitMask(clockPin) >> (portOffset * 16);
        rgbAndClockMask = (bitMask >> (portOffset * 16)) | clockMask;
        uint32_t elements = screenBytes / 2;
        for(uint32_t i=0; i<elements; i++) {
            ((uint16_t *)screenData)[i] = clockMask;
        }
#else
        // Clock and rgbAndClockMask are 32-bit values
        clockMask       = _PM_portBitMask(clockPin);
        rgbAndClockMask = bitMask | clockMask;
#endif
        for(uint8_t i=0; i<parallel * 6; i++) {
            ((uint16_t *)rgbMask)[i] =  // Pin bitmasks are stored 16-bit
              _PM_portBitMask(rgbPins[i]) >> (portOffset * 16);
        }
    } else {
        portOffset      = 0;
        clockMask       = _PM_portBitMask(clockPin);
        rgbAndClockMask = bitMask | clockMask;
#if defined(_PM_portToggleRegister)
        uint32_t elements = screenBytes / 4;
        for(uint32_t i=0; i<elements; i++) {
            ((uint32_t *)screenData)[i] = clockMask;
        }
#endif
        for(uint8_t i=0; i<parallel * 6; i++) {
            ((uint32_t *)rgbMask)[i] = // Pin bitmasks are stored 32-bit
              _PM_portBitMask(rgbPins[i]);
        }
    }

    activeBuffer = 0;

    // Estimate minimum bitplane #0 period for _PM_MAX_REFRESH_HZ rate.
    uint32_t minPeriodPerFrame = _PM_timerFreq / _PM_MAX_REFRESH_HZ;
    uint32_t minPeriodPerLine  = minPeriodPerFrame / rowPairs;
    minPeriod = minPeriodPerLine / ((1 << numPlanes) - 1);
    // Actual frame rate may be lower than this. That's OK, just
    // don't want to exceed this, as it'll eat all the CPU cycles.

    // Once allocation is set up, configure all the
    // pins as outputs and initialize their states.
    pinMode(clockPin, OUTPUT); digitalWrite(clockPin, LOW);
    pinMode(latchPin, OUTPUT); digitalWrite(latchPin, LOW);
    pinMode(oePin   , OUTPUT); digitalWrite(oePin   , HIGH); // Disable output
    for(uint8_t i=0; i<parallel * 6; i++) {
        pinMode(rgbPins[i], OUTPUT); digitalWrite(rgbPins[i], LOW);
    }
    for(uint8_t i=0; i<numAddressLines; i++) {
        pinMode(addrPins[i], OUTPUT); digitalWrite(addrPins[i], LOW);
    }

    // Get pointers to bit set and clear registers (and toggle, if present)
    setReg    = (uint8_t *)_PM_portSetRegister(clockPin);
    clearReg  = (uint8_t *)_PM_portClearRegister(clockPin);
#if defined(_PM_portToggleRegister)
    toggleReg = (uint8_t *)_PM_portToggleRegister(clockPin);
#endif

    protoPtr = this;            // Only one active Adafruit_Protomatter object!
    plane    = numPlanes   - 1; // Initialize plane & row to their max values
    row      = numRowPairs - 1; // so they roll over to start on 1st interrupt.
    _PM_timerInit();            // Configure timer
    _PM_timerStart(1000);       // Start timer

    return PROTOMATTER_OK;
}

// Convert GFXcanvas16 framebuffer representation to weird internal format
// used by the matrix-driving loop.
void Adafruit_Protomatter::show(void) {

    uint8_t *dest = screenData;
    if(doubleBuffer) {
        dest += bufferSize * (1 - activeBuffer);
    }

    if(bytesPerElement == 1) {
        convert_byte((uint8_t *)dest);
    } else if(bytesPerElement == 2) {
        convert_word((uint16_t *)dest);
    } else {
        convert_long((uint32_t *)dest);
    }

    if(doubleBuffer) {
        swapBuffers = 1;
        // To avoid overwriting data on the matrix, don't return
        // until the timer ISR has performed the swap at the right time.
        while(swapBuffers);
    }
}

// Process data from GFXcanvas16 into screenData buffer

// There are THREE COPIES of the following function -- one each for byte,
// word and long. If changes are made in any one of them, the others MUST
// be updated to match! Note that they are not simple duplicates of each
// other. The byte case, for example, doesn't need to handle parallel
// matrix chains (matrix data can only be byte-sized if one chain).

void Adafruit_Protomatter::convert_byte(uint8_t *dest) {
    uint16_t *upperSrc = getBuffer();                    // Canvas top half
    uint16_t *lowerSrc = upperSrc + WIDTH * numRowPairs; // " bottom half
    uint8_t  *pinMask  = (uint8_t *)rgbMask;             // Pin bitmasks

    // No need to clear matrix buffer, loops below do a full overwrite
    // (except for any scanline pad, which was already initialized in the
    // begin() function and won't be touched here).

    // Determine matrix bytes per bitplane & row (row pair really):
    uint32_t bitplaneSize = _PM_chunkSize *
        ((WIDTH + (_PM_chunkSize - 1)) / _PM_chunkSize); // 1 plane of row pair
    uint32_t rowSize      = bitplaneSize * numPlanes; // All planes of row pair
    uint8_t  pad          = bitplaneSize - WIDTH;     // Start-of-plane pad

    // Skip initial scanline padding if present (HUB75 matrices shift data
    // in from right-to-left, so if we need scanline padding it occurs at
    // the start of a line, rather than the usual end). Destination pointer
    // passed in already handles double-buffer math, so we don't need to
    // handle that here, just the pad...
    dest += pad;

    uint32_t initialRedBit, initialGreenBit, initialBlueBit;
    if(numPlanes == 6) {
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
        uint8_t shiftLeft = 5 - numPlanes;
        initialRedBit   = 0b0000100000000000 << shiftLeft;
        initialGreenBit = 0b0000000001000000 << shiftLeft;
        initialBlueBit  = 0b0000000000000001 << shiftLeft;
    }

    // This works sequentially-ish through the destination buffer,
    // reading from the canvas source pixels in repeated passes,
    // beginning from the least bit.
    for(uint8_t row=0; row<numRowPairs; row++) {
        for(uint8_t plane=0; plane<numPlanes; plane++) {
            uint32_t redBit   = initialRedBit;
            uint32_t greenBit = initialGreenBit;
            uint32_t blueBit  = initialBlueBit;
            for(uint16_t x=0; x<WIDTH; x++) {
                uint16_t upperRGB = upperSrc[x]; // Pixel in upper half
                uint16_t lowerRGB = lowerSrc[x]; // Pixel in lower half
#if defined(_PM_portToggleRegister)
                uint8_t result = clockMask;
#else
                uint8_t result = 0;
#endif
                if(upperRGB & redBit)   result |= pinMask[0];
                if(upperRGB & greenBit) result |= pinMask[1];
                if(upperRGB & blueBit)  result |= pinMask[2];
                if(lowerRGB & redBit)   result |= pinMask[3];
                if(lowerRGB & greenBit) result |= pinMask[4];
                if(lowerRGB & blueBit)  result |= pinMask[5];
                dest[x] = result;
            } // end x
            greenBit <<= 1;
            if(plane || (numPlanes < 6)) {
                // In mose cases red & blue bit scoot 1 left...
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
            dest[-pad] &= ~clockMask; // Negative index is legal & intentional
#endif
            dest += bitplaneSize; // Advance one scanline in dest buffer
        } // end plane
        upperSrc += WIDTH; // Advance one scanline in source buffer
        lowerSrc += WIDTH;
    } // end row
}

void Adafruit_Protomatter::convert_word(uint16_t *dest) {
    // TO DO
}

void Adafruit_Protomatter::convert_long(uint32_t *dest) {
    // TO DO
}

// ISR function (in arch.h) calls this function which it's extern'd.
// It can't call protoPtr->row_handler() directly, because the Protomatter
// class isn't known within arch.h (because it needs to come first -- the
// class header file references things in it).
// ISR -> _PM_row_handler() -> protoPtr->row_handler
void _PM_row_handler(void) {
    protoPtr->row_handler();
}

// Invoked by timer interrupt service routine (via protoPtr). User code
// should NOT call this function directly, even though it's declared public.
void Adafruit_Protomatter::row_handler(void) {

    for(;;) { // Function MIGHT make multiple passes in certain situations...

        digitalWrite(oePin, HIGH); // Disable LED output

        // Stop timer, save count value at stop
        uint32_t elapsed = _PM_timerStop();

        digitalWrite(latchPin, HIGH); // Latch data loaded on PRIOR pass
        digitalWrite(latchPin, LOW);
        uint8_t timePlane = plane;    // Save that plane # for later timing

        // If plane 0 just finished being displayed (plane 1 was loaded on
        // prior pass, or there's only one plane...I know, it's confusing),
        // take note of the elapsed timer value, for subsequent bitplane
        // timing (each plane period is double the previous). Value is
        // filtered slightly to avoid jitter.
        if((plane == 1) || (numPlanes == 1)) {
            bitZeroPeriod = ((bitZeroPeriod * 7) + elapsed) / 8;
            if(bitZeroPeriod < minPeriod) {
                bitZeroPeriod = minPeriod;
            }
        }

        if(timePlane == 0) {
            // Configure row address lines:
            for(uint8_t line=0,bit=1; line<numAddressLines; line++, bit<<=1) {
                digitalWrite(addrPins[line], row & bit);
                delayMicroseconds(10);
            }
            // Optimization opportunity: if device has a toggle register,
            // and if all address lines are on same PORT, can do in a single
            // operation and not need delays for each address bit.
            // Also consider even in non-same-PORT case, each delay is
            // probably only required if address line value has changed.
        }

        // Advance bitplane index and/or row as necessary
        if(++plane >= numPlanes) {     // Next data bitplane, or
            plane = 0;                 // roll over bitplane to start
            if(++row >= numRowPairs) { // Next row, or
                row = 0;               // roll over row to start
                // Switch matrix buffers if due (only if double-buffered)
                if(swapBuffers) {
                    activeBuffer = 1 - activeBuffer;
                    swapBuffers  = 0; // Swapped!
                }
                frameCount++;
            }
        }

        // 'plane' now is index of data to load, NOT data to display.
        // 'timePlane' is the data to display (and configure timer for)
        // concurrent with the plane data being loaded.

        // Set timer and enable LED output for data loaded on PRIOR pass:
        _PM_timerStart(bitZeroPeriod << timePlane);
        digitalWrite(oePin, LOW);

        uint32_t elementsPerLine = _PM_chunkSize *
            ((WIDTH + (_PM_chunkSize - 1)) / _PM_chunkSize);
        uint32_t srcOffset = elementsPerLine * (numPlanes * row + plane);

        if(bytesPerElement == 1) {
            blast_byte((uint8_t *)screenData + srcOffset);
        } else if(bytesPerElement == 2) {
            blast_word((uint16_t *)screenData + srcOffset * 2);
        } else {
            blast_long((uint32_t *)screenData + srcOffset * 4);
        }

        // 'plane' data is now loaded, will be shown on NEXT pass

        // If bitplane 0 data took more than 7/8 minPeriod timer cycles to
        // issue, don't return, make a second pass through the function right
        // now. This is to make the plane-zero interval as short as possible
        // (for a more stable image), as there's some overhead in the
        // interrupt and function calls (that it might be around 1/8 is just
        // a quick guess and not based on science). In all other cases,
        // return now, don't make the second pass. Await next timer ISR.
        if((plane > 0) || (_PM_timerGetCount() < (minPeriod * 7 / 8))) {
            return;
        }
    }
}

// Innermost data-stuffing loop functions

#if defined(_PM_portToggleRegister)
  #define PEW \
    *toggle  = *data++; /* Toggle in new data + toggle clock low */ \
    *toggle  =  clock;  /* Toggle clock high */
#else
  #define PEW \
    *set     = *data++;   /* Set RGB data high */ \
    *set32   =  clock;    /* Set clock high */ \
    *clear32 =  rgbclock; /* Clear RGB data + clock */
#endif

#if _PM_chunkSize == 1
  #define PEW_UNROLL PEW
#elif _PM_chunkSize == 8
  #define PEW_UNROLL PEW PEW PEW PEW PEW PEW PEW PEW
#elif _PM_chunkSize == 16
  #define PEW_UNROLL \
    PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW
#elif _PM_chunkSize == 32
  #define PEW_UNROLL \
    PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW \
    PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW
#elif _PM_chunkSize == 64
  #define PEW_UNROLL \
    PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW \
    PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW \
    PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW \
    PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW PEW
#else
  #error "Unimplemented _PM_chunkSize value"
#endif

// There are THREE COPIES of the following function -- one each for byte,
// word and long. If changes are made in any one of them, the others MUST
// be updated to match! (Decided against using macro tricks for the
// function, too often ends in disaster...but must be vigilant in the
// three-function maintenance then.)

void Adafruit_Protomatter::blast_byte(uint8_t *data) {
#if defined(_PM_portToggleRegister)
    // If here, it was established in begin() that the RGB data bits and
    // clock are all within the same byte of a PORT register, else we'd be
    // in the word- or long-blasting functions now. So we just need an
    // 8-bit pointer to the PORT.
    volatile uint8_t *toggle = (volatile uint8_t *)toggleReg + portOffset;
#else
    // No-toggle version is a little different. If here, RGB data is all
    // in one byte of PORT register, clock can be any bit in 32-bit PORT.
    volatile uint8_t  *set;     // For RGB data set
    volatile uint32_t *set32;   // For clock set
    volatile uint32_t *clear32; // For RGB data + clock clear
    set     = (volatile uint8_t *)setReg + portOffset;
    set32   = (volatile uint32_t *)setReg;
    clear32 = (volatile uint32_t *)clearReg;
    uint32_t rgbclock = rgbAndClockMask; // RGB + clock bit
#endif
    uint32_t clock  = clockMask; // Clock bit
    uint8_t  chunks = (WIDTH + (_PM_chunkSize - 1)) / _PM_chunkSize;

    // PORT has already been initialized with RGB data + clock bits
    // all LOW, so we don't need to initialize that state here.

    while(chunks--) {
        PEW_UNROLL // _PM_chunkSize RGB+clock writes
    }

#if defined(_PM_portToggleRegister)
    // Want the PORT left with RGB data and clock LOW on function exit
    // (so it's easier to see on 'scope, and to prime it for the next call).
    // This is implicit in the no-toggle case (due to how the PEW macro
    // works), but toggle case requires explicitly clearing those bits.
    // rgbAndClockMask is an 8-bit value when toggling, hence offset here.
    *((volatile uint8_t *)clearReg + portOffset) = rgbAndClockMask;
#endif
}

void Adafruit_Protomatter::blast_word(uint16_t *data) {
#if defined(_PM_portToggleRegister)
    // See notes above -- except now 16-bit word in PORT.
    volatile uint16_t *toggle = (volatile uint16_t *)toggleReg + portOffset;
#else
    volatile uint16_t *set;     // For RGB data set
    volatile uint32_t *set32;   // For clock set
    volatile uint32_t *clear32; // For RGB data + clock clear
    set     = (volatile uint16_t *)setReg + portOffset;
    set32   = (volatile uint32_t *)setReg;
    clear32 = (volatile uint32_t *)clearReg;
    uint32_t rgbclock = rgbAndClockMask; // RGB + clock bit
#endif
    uint32_t clock  = clockMask; // Clock bit
    uint8_t  chunks = (WIDTH + (_PM_chunkSize - 1)) / _PM_chunkSize;
    while(chunks--) {
        PEW_UNROLL // _PM_chunkSize RGB+clock writes
    }
#if defined(_PM_portToggleRegister)
    // rgbAndClockMask is a 16-bit value when toggling, hence offset here.
    *((volatile uint16_t *)clearReg + portOffset) = rgbAndClockMask;
#endif
}

void Adafruit_Protomatter::blast_long(uint32_t *data) {
#if defined(_PM_portToggleRegister)
    // See notes above -- except now full 32-bit PORT.
    volatile uint32_t *toggle = (volatile uint32_t *)toggleReg;
#else
    // Note in this case two copies exist of the PORT set register.
    // The optimizer will most likely simplify this; leaving as-is, not
    // wanting a special case of the PEW macro due to divergence risk.
    volatile uint32_t *set;     // For RGB data set
    volatile uint32_t *set32;   // For clock set
    volatile uint32_t *clear32; // For RGB data + clock clear
    set     = (volatile uint32_t *)setReg;
    set32   = (volatile uint32_t *)setReg;
    clear32 = (volatile uint32_t *)clearReg;
    uint32_t rgbclock = rgbAndClockMask; // RGB + clock bit
#endif
    uint32_t clock  = clockMask; // Clock bit
    uint8_t  chunks = (WIDTH + (_PM_chunkSize - 1)) / _PM_chunkSize;
    while(chunks--) {
        PEW_UNROLL // _PM_chunkSize RGB+clock writes
    }
#if defined(_PM_portToggleRegister)
    *(volatile uint32_t *)clearReg = rgbAndClockMask;
#endif
}

// Returns current value of frame counter and resets its value to zero.
// Two calls to this, timed one second apart (or use math with other
// intervals), can be used to get a rough frames-per-second value for
// the matrix (since this is difficult to estimate beforehand).
uint32_t Adafruit_Protomatter::getFrameCount(void) {
    uint32_t count = frameCount;
    frameCount = 0;
    return count;
}
