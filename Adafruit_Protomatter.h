#ifndef _ADAFRUIT_PROTOMATTER_H_
#define _ADAFRUIT_PROTOMATTER_H_

#include <Adafruit_GFX.h>

enum ProtomatterStatus {
    PROTOMATTER_OK,         // Everything is hunky-dory!
    PROTOMATTER_ERR_PINS,   // Clock and/or data pins on different PORTs
    PROTOMATTER_ERR_MALLOC, // Couldn't allocate memory for display
};

class Adafruit_Protomatter : public GFXcanvas16 {
  public:
    Adafruit_Protomatter(uint16_t bitWidth, uint8_t bitDepth,
      uint8_t rgbCount, uint8_t *rgbList, uint8_t addrCount,
      uint8_t *addrList, uint8_t clockPin, uint8_t latchPin,
      uint8_t oePin, bool doubleBuffer);
    ~Adafruit_Protomatter(void);
    ProtomatterStatus begin(void);
    void              show(void);
    uint32_t          getFrameCount(void);
    // This function needs to be declared public for protoPtr (in .cpp)
    // to access it, but should NOT be invoked by user code:
    void              row_handler(void);
  private:
    void              blast_byte(uint8_t  *data); // Data-writing functions
    void              blast_word(uint16_t *data); // for 8/16/32 bit output
    void              blast_long(uint32_t *data); // pin arrangements.
    void              convert_byte(uint8_t  *dest); // Canvas-to-matrix
    void              convert_word(uint16_t *dest); // conversion functions
    void              convert_long(uint32_t *dest); // for 8/16/32 bit bufs
    uint8_t          *setReg;           // GPIO bit set register
    uint8_t          *clearReg;         // GPIO bit clear register
    uint8_t          *toggleReg;        // GPIO bit toggle register
    // IMPORTANT NOTE: toggleReg member MUST be present even if the device
    // doesn't support it, we CAN'T #ifdef around this. Reason being that
    // arch.h is ONLY included by the .cpp code (not this header, nor the
    // user code) to avoid duplicate instances of the timer ISR and other
    // functions, and if the member is ifdef'd then the .cpp and user code
    // definition of this structure would diverge (values would be stored in
    // the wrong places). Could play games by putting that member at the end
    // (with ifdef check), but that's just dirty pool and asking for trouble.
    // So on some devices (with no bit-toggle register) 4 bytes are unused.
    uint8_t          *screenData;       // Processed from GFXcanvas16
    uint8_t          *rgbPins;          // Array of RGB data pins (mult of 6)
    uint8_t          *addrPins;         // Array of matrix address line pins
    uint8_t          *rgbMask;          // PORT bit mask for each RGB pin
    uint32_t          clockMask;        // PORT bit mask for data clock
    uint32_t          rgbAndClockMask;  // PORT bit mask for RGB data + clock
    uint32_t          bufferSize;       // Bytes per matrix buffer (1 or 2)
    uint32_t          bitZeroPeriod;    // Bitplane 0 timer period
    uint32_t          minPeriod;        // Plane 0 timer period for ~400Hz
    volatile uint32_t *addrPortToggle;  // See singleAddrPort below
    volatile uint32_t frameCount;       // For estimating refresh rate
    uint8_t           bytesPerElement;  // Using 8, 16 or 32 bits of PORT?
    uint8_t           clockPin;         // Data clock pin (Arduino pin #)
    uint8_t           latchPin;         // Data latch pin (Arduino pin #)
    uint8_t           oePin;            // !OE (LOW out enable) (Arduino pin #)
    uint8_t           parallel;         // Number of concurrent matrix outputs
    uint8_t           numAddressLines;  // Number of address line pins
    uint8_t           portOffset;       // Active 8- or 16-bit pos. in PORT
    uint8_t           numPlanes;        // Display bitplanes (1 to 6)
    uint8_t           numRowPairs;      // Addressable row pairs
    bool              doubleBuffer;     // 2X buffers for clean switchover
    bool              singleAddrPort;   // If 1, all addr lines on same PORT
    volatile uint8_t  activeBuffer;     // Index of currently-displayed buf
    volatile uint8_t  plane;            // Current bitplane (changes in ISR)
    volatile uint8_t  row;              // Current scanline (changes in ISR)
    volatile uint8_t  prevRow;          // Scanline from prior ISR
    volatile bool     swapBuffers;      // If 1, awaiting double-buf switch
};

#endif // _ADAFRUIT_PROTOMATTER_H_
