/* ----------------------------------------------------------------------
"Tiled" Protomatter library example sketch. Demonstrates use of multiple
RGB LED matrices as a single larger drawing surface. This example is
written for two 64x32 matrices (tiled into a 64x64 display) but can be
adapted to others. If using MatrixPortal, larger multi-panel tilings like
this should be powered from a separate 5V DC supply, not the USB port
(this example works OK because the graphics are very minimal).

PLEASE SEE THE "simple" EXAMPLE FOR AN INTRODUCTORY SKETCH.
------------------------------------------------------------------------- */

#include <Adafruit_Protomatter.h>

/* ----------------------------------------------------------------------
The RGB matrix must be wired to VERY SPECIFIC pins, different for each
microcontroller board. This first section sets that up for a number of
supported boards.
------------------------------------------------------------------------- */

#if defined(_VARIANT_MATRIXPORTAL_M4_) // MatrixPortal M4
  uint8_t rgbPins[]  = {7, 8, 9, 10, 11, 12};
  uint8_t addrPins[] = {17, 18, 19, 20, 21};
  uint8_t clockPin   = 14;
  uint8_t latchPin   = 15;
  uint8_t oePin      = 16;
#elif defined(ARDUINO_ADAFRUIT_MATRIXPORTAL_ESP32S3) // MatrixPortal ESP32-S3
  uint8_t rgbPins[]  = {42, 41, 40, 38, 39, 37};
  uint8_t addrPins[] = {45, 36, 48, 35, 21};
  uint8_t clockPin   = 2;
  uint8_t latchPin   = 47;
  uint8_t oePin      = 14;
#elif defined(_VARIANT_FEATHER_M4_) // Feather M4 + RGB Matrix FeatherWing
  uint8_t rgbPins[]  = {6, 5, 9, 11, 10, 12};
  uint8_t addrPins[] = {A5, A4, A3, A2};
  uint8_t clockPin   = 13;
  uint8_t latchPin   = 0;
  uint8_t oePin      = 1;
#elif defined(__SAMD51__) // M4 Metro Variants (Express, AirLift)
  uint8_t rgbPins[]  = {6, 5, 9, 11, 10, 12};
  uint8_t addrPins[] = {A5, A4, A3, A2};
  uint8_t clockPin   = 13;
  uint8_t latchPin   = 0;
  uint8_t oePin      = 1;
#elif defined(_SAMD21_) // Feather M0 variants
  uint8_t rgbPins[]  = {6, 7, 10, 11, 12, 13};
  uint8_t addrPins[] = {0, 1, 2, 3};
  uint8_t clockPin   = SDA;
  uint8_t latchPin   = 4;
  uint8_t oePin      = 5;
#elif defined(NRF52_SERIES) // Special nRF52840 FeatherWing pinout
  uint8_t rgbPins[]  = {6, A5, A1, A0, A4, 11};
  uint8_t addrPins[] = {10, 5, 13, 9};
  uint8_t clockPin   = 12;
  uint8_t latchPin   = PIN_SERIAL1_RX;
  uint8_t oePin      = PIN_SERIAL1_TX;
#elif USB_VID == 0x239A && USB_PID == 0x8113 // Feather ESP32-S3 No PSRAM
  // M0/M4/RP2040 Matrix FeatherWing compatible:
  uint8_t rgbPins[]  = {6, 5, 9, 11, 10, 12};
  uint8_t addrPins[] = {A5, A4, A3, A2};
  uint8_t clockPin   = 13; // Must be on same port as rgbPins
  uint8_t latchPin   = RX;
  uint8_t oePin      = TX;
#elif USB_VID == 0x239A && USB_PID == 0x80EB // Feather ESP32-S2
  // M0/M4/RP2040 Matrix FeatherWing compatible:
  uint8_t rgbPins[]  = {6, 5, 9, 11, 10, 12};
  uint8_t addrPins[] = {A5, A4, A3, A2};
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
  uint8_t addrPins[] = {16, 17, 25, 26};
  uint8_t clockPin   = 27; // Must be on same port as rgbPins
  uint8_t latchPin   = 32;
  uint8_t oePin      = 33;
#elif defined(ARDUINO_TEENSY40)
  uint8_t rgbPins[]  = {15, 16, 17, 20, 21, 22}; // A1-A3, A6-A8, skip SDA,SCL
  uint8_t addrPins[] = {2, 3, 4, 5};
  uint8_t clockPin   = 23; // A9
  uint8_t latchPin   = 6;
  uint8_t oePin      = 9;
#elif defined(ARDUINO_TEENSY41)
  uint8_t rgbPins[]  = {26, 27, 38, 20, 21, 22}; // A12-14, A6-A8
  uint8_t addrPins[] = {2, 3, 4, 5};
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
  uint8_t addrPins[] = {25, 24, 29, 28};
  uint8_t clockPin   = 13;
  uint8_t latchPin   = 1;
  uint8_t oePin      = 0;
#endif

/* ----------------------------------------------------------------------
Matrix initialization is explained EXTENSIVELY in "simple" example sketch!
It's very similar here, but we're passing an extra argument to define the
matrix tiling along the vertical axis: -2 means there are two matrices
(or rows of matrices) arranged in a "serpentine" path (the second matrix
is rotated 180 degrees relative to the first, and positioned below).
A positive 2 would indicate a "progressive" path (both matrices are
oriented the same way), but usually requires longer cables.
------------------------------------------------------------------------- */

Adafruit_Protomatter matrix(
  64,          // Width of matrix (or matrices, if tiled horizontally)
  6,           // Bit depth, 1-6
  1, rgbPins,  // # of matrix chains, array of 6 RGB pins for each
  4, addrPins, // # of address pins (height is inferred), array of pins
  clockPin, latchPin, oePin, // Other matrix control pins
  false,       // No double-buffering here (see "doublebuffer" example)
  -2);         // Row tiling: two rows in "serpentine" path
  
// SETUP - RUNS ONCE AT PROGRAM START --------------------------------------

void setup(void) {
  Serial.begin(9600);

  // Initialize matrix...
  ProtomatterStatus status = matrix.begin();
  Serial.print("Protomatter begin() status: ");
  Serial.println((int)status);
  if(status != PROTOMATTER_OK) {
    // DO NOT CONTINUE if matrix setup encountered an error.
    for(;;);
  }

  // Since this program has no animation, all the drawing can be done
  // here in setup() rather than loop(). It's just a few basic shapes
  // that span across the matrices...nothing showy, the goal of this
  // sketch is just to demonstrate tiling basics.

  matrix.drawLine(0, 0, matrix.width() - 1, matrix.height() - 1,
    matrix.color565(255, 0, 0)); // Red line
  matrix.drawLine(matrix.width() - 1, 0, 0, matrix.height() - 1,
    matrix.color565(0, 0, 255)); // Blue line
  int radius = min(matrix.width(), matrix.height()) / 2;
  matrix.drawCircle(matrix.width() / 2, matrix.height() / 2, radius,
    matrix.color565(0, 255, 0)); // Green circle

  // AFTER DRAWING, A show() CALL IS REQUIRED TO UPDATE THE MATRIX!

  matrix.show(); // Copy data to matrix buffers
}

// LOOP - RUNS REPEATEDLY AFTER SETUP --------------------------------------

void loop(void) {
  // Since there's nothing more to be drawn, this loop() function just
  // prints the approximate refresh rate of the matrix at current settings.
  Serial.print("Refresh FPS = ~");
  Serial.println(matrix.getFrameCount());
  delay(1000);
}
