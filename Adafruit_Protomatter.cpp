// Arduino-specific wrapper for the Protomatter C library (provides
// constructor and so forth, builds on Adafruit_GFX). There should
// not be any device-specific #ifdefs here. See notes in core.c and
// arch.h regarding portability.

#include "Adafruit_Protomatter.h" // Also includes core.h & Adafruit_GFX.h

extern Protomatter_core *_PM_protoPtr; // In core.c (via arch.h)

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

// Time (in milliseconds) to pause following any change in address lines
// (individually or collectively). Some matrices respond slowly there...
// must pause on change for matrix to catch up. Defined here (rather than
// arch.h) because it's not architecture-specific.
#define _PM_ROW_DELAY 8


// Need to pass in Tc here

Adafruit_Protomatter::Adafruit_Protomatter(
  uint16_t bitWidth, uint8_t bitDepth,
  uint8_t rgbCount, uint8_t *rgbList,
  uint8_t addrCount, uint8_t *addrList,
  uint8_t clockPin, uint8_t latchPin, uint8_t oePin,
  bool doubleBuffer, void *timer) :
  GFXcanvas16(bitWidth, (2 << min(addrCount, 5)) * min(rgbCount, 5)) {
    if(bitDepth > 6)  bitDepth = 6;   // GFXcanvas16 color limit (565)

    // Arguments are passed through to the C _PM_init() function which does
    // some input validation and minor allocation. Return value is ignored
    // because we can't really do anything about it in a C++ constructor.
    // The class begin() function checks rgbPins for NULL to determine
    // whether to proceed or indicate an error.
    (void)_PM_init(&core, bitWidth, bitDepth, rgbCount, rgbList,
      addrCount, addrList, clockPin, latchPin, oePin, doubleBuffer, timer);
}

Adafruit_Protomatter::~Adafruit_Protomatter(void) {
    _PM_free(&core);
}

ProtomatterStatus Adafruit_Protomatter::begin(void) {
// make this call underlying lib
    return PROTOMATTER_OK;
}

// Convert GFXcanvas16 framebuffer representation to weird internal format
// used by the matrix-driving loop.
void Adafruit_Protomatter::show(void) {

    uint8_t *dest = (uint8_t *)core.screenData;
    if(core.doubleBuffer) {
        dest += core.bufferSize * (1 - core.activeBuffer);
    }

    if(core.bytesPerElement == 1) {
        convert_byte((uint8_t *)dest);
    } else if(core.bytesPerElement == 2) {
        convert_word((uint16_t *)dest);
    } else {
        convert_long((uint32_t *)dest);
    }

    if(core.doubleBuffer) {
        core.swapBuffers = 1;
        // To avoid overwriting data on the matrix, don't return
        // until the timer ISR has performed the swap at the right time.
        while(core.swapBuffers);
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
    uint16_t *lowerSrc = upperSrc + WIDTH * core.numRowPairs; // " bottom half
    uint8_t  *pinMask  = (uint8_t *)core.rgbMask;             // Pin bitmasks

    // No need to clear matrix buffer, loops below do a full overwrite
    // (except for any scanline pad, which was already initialized in the
    // begin() function and won't be touched here).

    // Determine matrix bytes per bitplane & row (row pair really):

    uint8_t  chunkSize    = _PM_getChunkSize();
    uint32_t bitplaneSize = chunkSize *
        ((WIDTH + (chunkSize - 1)) / chunkSize);  // 1 plane of row pair
    uint8_t  pad          = bitplaneSize - WIDTH; // Start-of-plane pad

    // Skip initial scanline padding if present (HUB75 matrices shift data
    // in from right-to-left, so if we need scanline padding it occurs at
    // the start of a line, rather than the usual end). Destination pointer
    // passed in already handles double-buffer math, so we don't need to
    // handle that here, just the pad...
    dest += pad;

    uint32_t initialRedBit, initialGreenBit, initialBlueBit;
    if(core.numPlanes == 6) {
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
        uint8_t shiftLeft = 5 - core.numPlanes;
        initialRedBit   = 0b0000100000000000 << shiftLeft;
        initialGreenBit = 0b0000000001000000 << shiftLeft;
        initialBlueBit  = 0b0000000000000001 << shiftLeft;
    }

    // This works sequentially-ish through the destination buffer,
    // reading from the canvas source pixels in repeated passes,
    // beginning from the least bit.
    for(uint8_t row=0; row<core.numRowPairs; row++) {
        uint32_t redBit   = initialRedBit;
        uint32_t greenBit = initialGreenBit;
        uint32_t blueBit  = initialBlueBit;
        for(uint8_t plane=0; plane<core.numPlanes; plane++) {
#if defined(_PM_portToggleRegister)
            uint8_t prior = clockMask; // Set clock bit on 1st out
#endif
            for(uint16_t x=0; x<WIDTH; x++) {
                uint16_t upperRGB = upperSrc[x]; // Pixel in upper half
                uint16_t lowerRGB = lowerSrc[x]; // Pixel in lower half
                uint8_t result = 0;
                if(upperRGB & redBit)   result |= pinMask[0];
                if(upperRGB & greenBit) result |= pinMask[1];
                if(upperRGB & blueBit)  result |= pinMask[2];
                if(lowerRGB & redBit)   result |= pinMask[3];
                if(lowerRGB & greenBit) result |= pinMask[4];
                if(lowerRGB & blueBit)  result |= pinMask[5];
#if defined(_PM_portToggleRegister)
                dest[x] = result ^ prior;
                prior   = result | clockMask; // Set clock bit on next out
#else
                dest[x] = result;
#endif
            } // end x
            greenBit <<= 1;
            if(plane || (core.numPlanes < 6)) {
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

// Returns current value of frame counter and resets its value to zero.
// Two calls to this, timed one second apart (or use math with other
// intervals), can be used to get a rough frames-per-second value for
// the matrix (since this is difficult to estimate beforehand).
uint32_t Adafruit_Protomatter::getFrameCount(void) {
    return _PM_getFrameCount(_PM_protoPtr);
}
