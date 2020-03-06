// Arduino-specific header, accompanies Adafruit_Protomatter.cpp.
// There should not be any device-specific #ifdefs here.

#ifndef _ADAFRUIT_PROTOMATTER_H_
#define _ADAFRUIT_PROTOMATTER_H_

#include "core.h"
#include <Adafruit_GFX.h>

class Adafruit_Protomatter : public GFXcanvas16 {
  public:
    Adafruit_Protomatter(uint16_t bitWidth, uint8_t bitDepth,
      uint8_t rgbCount, uint8_t *rgbList,
      uint8_t addrCount, uint8_t *addrList,
      uint8_t clockPin, uint8_t latchPin, uint8_t oePin,
      bool doubleBuffer, void *timer=NULL);
    ~Adafruit_Protomatter(void);
    ProtomatterStatus begin(void);
    void              show(void);
    uint32_t          getFrameCount(void);
  private:
    Protomatter_core  core;                         // Underlying C struct
    void              convert_byte(uint8_t  *dest); // GFXcanvas16-to-matrix
    void              convert_word(uint16_t *dest); // conversion functions
    void              convert_long(uint32_t *dest); // for 8/16/32 bit bufs
};

#endif // _ADAFRUIT_PROTOMATTER_H_
