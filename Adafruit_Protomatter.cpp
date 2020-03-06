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
    _PM_protoPtr = NULL;
}

ProtomatterStatus Adafruit_Protomatter::begin(void) {
    _PM_protoPtr = &core;
    _PM_begin(&core);
    return PROTOMATTER_OK;
}

// Transfer data from GFXcanvas16 to the matrix framebuffer's weird
// internal format. The actual conversion functions referenced below
// are in core.c, reasoning is explained there.
void Adafruit_Protomatter::show(void) {

    // Destination address is computed in convert function
    // (based on active buffer value, if double-buffering),
    // just need to pass in the canvas buffer address and
    // width in pixels.
    if(core.bytesPerElement == 1) {
        _PM_convert_565_byte(&core, getBuffer(), WIDTH);
    } else if(core.bytesPerElement == 2) {
        _PM_convert_565_word(&core, getBuffer(), WIDTH);
    } else {
        _PM_convert_565_long(&core, getBuffer(), WIDTH);
    }

    if(core.doubleBuffer) {
        core.swapBuffers = 1;
        // To avoid overwriting data on the matrix, don't return
        // until the timer ISR has performed the swap at the right time.
        while(core.swapBuffers);
    }
}

// Returns current value of frame counter and resets its value to zero.
// Two calls to this, timed one second apart (or use math with other
// intervals), can be used to get a rough frames-per-second value for
// the matrix (since this is difficult to estimate beforehand).
uint32_t Adafruit_Protomatter::getFrameCount(void) {
    return _PM_getFrameCount(_PM_protoPtr);
}
