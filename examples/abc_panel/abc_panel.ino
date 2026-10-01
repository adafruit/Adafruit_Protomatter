/*
  Serial ABC row selection on a 128x64 FM6126A panel.

  Some panels marked HUB75E have only A, B and C connected. On this type,
  A clocks a row shift register, B enables shifting, and C supplies row data.
  This is different from ordinary binary A/B/C/D/E row addressing. The
  FM6126A pixel driver alone does not identify which row protocol is used.

  Select your board in the Arduino board menu. Pin mappings below match
  the simple example; only A, B and C are needed for row selection. Connect
  the panel's INPUT connector and use a suitable 5V panel supply. This
  example uses dim text; it is not a current limiter.
*/

#include <Adafruit_Protomatter.h>

// Board wiring. addrPins contains A (row clock), B (enable), C (row data).
#if defined(_VARIANT_MATRIXPORTAL_M4_) // MatrixPortal M4
  uint8_t rgbPins[]  = {7, 8, 9, 10, 11, 12};
  uint8_t addrPins[] = {17, 18, 19};
  uint8_t clockPin   = 14;
  uint8_t latchPin   = 15;
  uint8_t oePin      = 16;
#elif defined(ARDUINO_ADAFRUIT_MATRIXPORTAL_ESP32S3) // MatrixPortal ESP32-S3
  uint8_t rgbPins[]  = {42, 41, 40, 38, 39, 37};
  uint8_t addrPins[] = {45, 36, 48};
  uint8_t clockPin   = 2;
  uint8_t latchPin   = 47;
  uint8_t oePin      = 14;
#elif defined(_VARIANT_FEATHER_M4_) // Feather M4 + RGB Matrix FeatherWing
  uint8_t rgbPins[]  = {6, 5, 9, 11, 10, 12};
  uint8_t addrPins[] = {A5, A4, A3};
  uint8_t clockPin   = 13;
  uint8_t latchPin   = 0;
  uint8_t oePin      = 1;
#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32C6) // Feather ESP32-C6
  // not featherwing compatible, but can 'hand wire' if desired
  uint8_t rgbPins[]  = {6, A3, A1, A0, A2, 0};
  uint8_t addrPins[] = {8, 5, 15};
  uint8_t clockPin   = 14;
  uint8_t latchPin   = RX;
  uint8_t oePin      = TX;
#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) // Feather ESP32-S2
  // M0/M4/RP2040 Matrix FeatherWing compatible:
  uint8_t rgbPins[]  = {6, 5, 9, 11, 10, 12};
  uint8_t addrPins[] = {A5, A4, A3};
  uint8_t clockPin   = 13; // Must be on same port as rgbPins
  uint8_t latchPin   = RX;
  uint8_t oePin      = TX;
#elif defined(ARDUINO_METRO_ESP32S2) // Metro ESP32-S2
  // Matrix Shield compatible:
  uint8_t rgbPins[]  = {7, 8, 9, 10, 11, 12};
  uint8_t addrPins[] = {A0, A1, A2};
  uint8_t clockPin   = 13; // Must be on same port as rgbPins
  uint8_t latchPin   = 15;
  uint8_t oePin      = 14;
#elif defined(__SAMD51__) // M4 Metro Variants (Express, AirLift)
  uint8_t rgbPins[]  = {6, 5, 9, 11, 10, 12};
  uint8_t addrPins[] = {A5, A4, A3};
  uint8_t clockPin   = 13;
  uint8_t latchPin   = 0;
  uint8_t oePin      = 1;
#elif defined(_SAMD21_) // Feather M0 variants
  uint8_t rgbPins[]  = {6, 5, 9, 11, 10, 12};
  uint8_t addrPins[] = {A5, A4, A3};
  uint8_t clockPin   = 13;
  uint8_t latchPin   = 0;
  uint8_t oePin      = 1;
#elif defined(NRF52_SERIES) // Special nRF52840 FeatherWing pinout
  uint8_t rgbPins[]  = {6, A5, A1, A0, A4, 11};
  uint8_t addrPins[] = {10, 5, 13};
  uint8_t clockPin   = 12;
  uint8_t latchPin   = PIN_SERIAL1_RX;
  uint8_t oePin      = PIN_SERIAL1_TX;
#elif USB_VID == 0x239A && USB_PID == 0x8113 // Feather ESP32-S3 No PSRAM
  // M0/M4/RP2040 Matrix FeatherWing compatible:
  uint8_t rgbPins[]  = {6, 5, 9, 11, 10, 12};
  uint8_t addrPins[] = {A5, A4, A3};
  uint8_t clockPin   = 13; // Must be on same port as rgbPins
  uint8_t latchPin   = RX;
  uint8_t oePin      = TX;
#elif defined(ESP32)
  // 'Safe' pins, not overlapping any peripherals:
  // GPIO.out: 4, 12, 13, 14, 15, 21, 27, GPIO.out1: 32, 33
  // Peripheral-overlapping pins, sorted from 'most expendible':
  // 16, 17 (RX, TX)
  // 25, 26 (A0, A1)
  // 18, 5, 9 (MOSI, SCK, MISO)
  // 22, 23 (SCL, SDA)
  uint8_t rgbPins[]  = {4, 12, 13, 14, 15, 21};
  uint8_t addrPins[] = {16, 17, 25};
  uint8_t clockPin   = 27; // Must be on same port as rgbPins
  uint8_t latchPin   = 32;
  uint8_t oePin      = 33;
#elif defined(ARDUINO_TEENSY40)
  uint8_t rgbPins[]  = {15, 16, 17, 20, 21, 22}; // A1-A3, A6-A8, skip SDA,SCL
  uint8_t addrPins[] = {2, 3, 4};
  uint8_t clockPin   = 23; // A9
  uint8_t latchPin   = 6;
  uint8_t oePin      = 9;
#elif defined(ARDUINO_TEENSY41)
  uint8_t rgbPins[]  = {26, 27, 38, 20, 21, 22}; // A12-14, A6-A8
  uint8_t addrPins[] = {2, 3, 4};
  uint8_t clockPin   = 23; // A9
  uint8_t latchPin   = 6;
  uint8_t oePin      = 9;
#elif defined(ARDUINO_ADAFRUIT_FEATHER_RP2040)
  // RP2040 support requires the Earle Philhower board support package;
  // will not compile with the Arduino Mbed OS board package.
  // The following pinout works with the Adafruit Feather RP2040 and
  // original RGB Matrix FeatherWing (M0/M4/RP2040, not nRF version).
  // Pin numbers here are GP## numbers, which may be different than
  // the pins printed on some boards' top silkscreen.
  uint8_t rgbPins[]  = {8, 7, 9, 11, 10, 12};
  uint8_t addrPins[] = {25, 24, 29};
  uint8_t clockPin   = 13;
  uint8_t latchPin   = 1;
  uint8_t oePin      = 0;
#else
#error "Add a pin mapping for your board, as described in the simple example."
#endif

// The tested panel swaps red and blue in both halves. For a standard RGB
// panel, pass rgbPins instead of bgrPins to the constructor below.
uint8_t bgrPins[] = {rgbPins[2], rgbPins[1], rgbPins[0],
                     rgbPins[5], rgbPins[4], rgbPins[3]};

Adafruit_Protomatter matrix(
  128, 4, 1, bgrPins, // Width, bit depth, one RGB pin set
  5, addrPins,       // 2 * 2^5 = 64 rows, but only THREE pins in ABC mode
  clockPin, latchPin, oePin,
  true,             // Double buffering for scrolling
  1, NULL,          // One vertical tile, default timer
  PROTOMATTER_ROW_ADDRESS_ABC);

int16_t textX = 128;

void setup() {
  Serial.begin(115200);
  ProtomatterStatus status = matrix.begin();
  if (status != PROTOMATTER_OK) {
    Serial.print("Matrix initialization failed: ");
    Serial.println((int)status);
    while (true) {
      delay(10);
    }
  }
  matrix.setTextWrap(false);
  matrix.setTextSize(1);
}

void loop() {
  matrix.fillScreen(0);
  matrix.setTextColor(matrix.color565(16, 0, 0));
  matrix.setCursor(textX, 12);
  matrix.print("ABC ROW SELECT");
  matrix.setTextColor(matrix.color565(0, 16, 0));
  matrix.setCursor(textX, 44);
  matrix.print("128x64 FM6126A");
  matrix.show();

  if (--textX < -84) { // 14 characters, six pixels each
    textX = matrix.width();
  }
  delay(30);
}
