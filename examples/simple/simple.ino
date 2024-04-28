/* ----------------------------------------------------------------------
"Simple" Protomatter library example sketch (once you get past all
the various pin configurations at the top, and all the comments).
Shows basic use of Adafruit_Protomatter library with different devices.

This example is written for a 64x32 matrix but can be adapted to others.

Once the RGB matrix is initialized, most functions of the Adafruit_GFX
library are available for drawing -- code from other projects that use
LCDs or OLEDs can be easily adapted, or may be insightful for reference.
GFX library is documented here:
https://learn.adafruit.com/adafruit-gfx-graphics-library
------------------------------------------------------------------------- */

#include <Adafruit_Protomatter.h>

/* ----------------------------------------------------------------------
The RGB matrix must be wired to VERY SPECIFIC pins, different for each
microcontroller board. This first section sets that up for a number of
supported boards. Notes have been moved to the bottom of the code.
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
#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32C6) // Feather ESP32-C6
  // not featherwing compatible, but can 'hand wire' if desired
  uint8_t rgbPins[]  = {6, A3, A1, A0, A2, 0};
  uint8_t addrPins[] = {8, 5, 15, 7};
  uint8_t clockPin   = 14;
  uint8_t latchPin   = RX;
  uint8_t oePin      = TX;
#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) // Feather ESP32-S2
  // M0/M4/RP2040 Matrix FeatherWing compatible:
  uint8_t rgbPins[]  = {6, 5, 9, 11, 10, 12};
  uint8_t addrPins[] = {A5, A4, A3, A2};
  uint8_t clockPin   = 13; // Must be on same port as rgbPins
  uint8_t latchPin   = RX;
  uint8_t oePin      = TX;
#elif defined(ARDUINO_METRO_ESP32S2) // Metro ESP32-S2
  // Matrix Shield compatible:
  uint8_t rgbPins[]  = {7, 8, 9, 10, 11, 12};
  uint8_t addrPins[] = {A0, A1, A2, A3};
  uint8_t clockPin   = 13; // Must be on same port as rgbPins
  uint8_t latchPin   = 15;
  uint8_t oePin      = 14;
#elif defined(__SAMD51__) // M4 Metro Variants (Express, AirLift)
  uint8_t rgbPins[]  = {6, 5, 9, 11, 10, 12};
  uint8_t addrPins[] = {A5, A4, A3, A2};
  uint8_t clockPin   = 13;
  uint8_t latchPin   = 0;
  uint8_t oePin      = 1;
#elif defined(_SAMD21_) // Feather M0 variants
  uint8_t rgbPins[]  = {6, 5, 9, 11, 10, 12};
  uint8_t addrPins[] = {A5, A4, A3, A2};
  uint8_t clockPin   = 13;
  uint8_t latchPin   = 0;
  uint8_t oePin      = 1;
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
Okay, here's where the RGB LED matrix is actually declared...

First argument is the matrix width, in pixels. Usually 32 or
64, but might go larger if you're chaining multiple matrices.

Second argument is the "bit depth," which determines color
fidelity, applied to red, green and blue (e.g. "4" here means
4 bits red, 4 green, 4 blue = 2^4 x 2^4 x 2^4 = 4096 colors).
There is a trade-off between bit depth and RAM usage. Most
programs will tend to use either 1 (R,G,B on/off, 8 colors,
best for text, LED sand, etc.) or the maximum of 6 (best for
shaded images...though, because the GFX library was designed
for LCDs, only 5 of those bits are available for red and blue.

Third argument is the number of concurrent (parallel) matrix
outputs. THIS SHOULD ALWAYS BE "1" FOR NOW. Fourth is a uint8_t
array listing six pins: red, green and blue data out for the
top half of the display, and same for bottom half. There are
hard constraints as to which pins can be used -- they must all
be on the same PORT register, ideally all within the same byte
of that PORT.

Fifth argument is the number of "address" (aka row select) pins,
from which the matrix height is inferred. "4" here means four
address lines, matrix height is then (2 x 2^4) = 32 pixels.
16-pixel-tall matrices will have 3 pins here, 32-pixel will have
4, 64-pixel will have 5. Sixth argument is a uint8_t array
listing those pin numbers. No PORT constraints here.

Next three arguments are pin numbers for other RGB matrix
control lines: clock, latch and output enable (active low).
Clock pin MUST be on the same PORT register as RGB data pins
(and ideally in same byte). Other pins have no special rules.

Last argument is a boolean (true/false) to enable double-
buffering for smooth animation (requires 2X the RAM). See the
"doublebuffer" example for a demonstration.
------------------------------------------------------------------------- */

Adafruit_Protomatter matrix(
  64,          // Width of matrix (or matrix chain) in pixels
  4,           // Bit depth, 1-6
  1, rgbPins,  // # of matrix chains, array of 6 RGB pins for each
  4, addrPins, // # of address pins (height is inferred), array of pins
  clockPin, latchPin, oePin, // Other matrix control pins
  false);      // No double-buffering here (see "doublebuffer" example)

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

  // Since this is a simple program with no animation, all the
  // drawing can be done here in setup() rather than loop():

  // Make four color bars (red, green, blue, white) with brightness ramp:
  for(int x=0; x<matrix.width(); x++) {
    uint8_t level = x * 256 / matrix.width(); // 0-255 brightness
    matrix.drawPixel(x, matrix.height() - 4, matrix.color565(level, 0, 0));
    matrix.drawPixel(x, matrix.height() - 3, matrix.color565(0, level, 0));
    matrix.drawPixel(x, matrix.height() - 2, matrix.color565(0, 0, level));
    matrix.drawPixel(x, matrix.height() - 1,
                     matrix.color565(level, level, level));
  }
  // You'll notice the ramp looks smoother as bit depth increases
  // (second argument to the matrix constructor call above setup()).

  // Simple shapes and text, showing GFX library calls:
  matrix.drawCircle(12, 10, 9, matrix.color565(255, 0, 0));
  matrix.drawRect(14, 6, 17, 17, matrix.color565(0, 255, 0));
  matrix.drawTriangle(32, 9, 41, 27, 23, 27, matrix.color565(0, 0, 255));
  matrix.println("ADAFRUIT"); // Default text color is white
  if (matrix.height() > 32) {
    matrix.setCursor(0, 32);
    matrix.println("64 pixel"); // Default text color is white
    matrix.println("matrix"); // Default text color is white
  }

  // AFTER DRAWING, A show() CALL IS REQUIRED TO UPDATE THE MATRIX!

  matrix.show(); // Copy data to matrix buffers
}

// LOOP - RUNS REPEATEDLY AFTER SETUP --------------------------------------

void loop(void) {
  // Since there's nothing more to be drawn, this loop() function just
  // shows the approximate refresh rate of the matrix at current settings.
  Serial.print("Refresh FPS = ~");
  Serial.println(matrix.getFrameCount());
  delay(1000);
}

// MORE NOTES --------------------------------------------------------------

/*
The "RGB and clock bits on same PORT register" constraint requires
considerable planning and knowledge of the underlying microcontroller
hardware. These are some earlier notes on various devices' PORT registers
and bits and their corresponding Arduino pin numbers. You probably won't
need this -- it's all codified in the #if defined() sections at the top
of this sketch now -- but keeping it around for reference if needed.

METRO M0 PORT-TO-PIN ASSIGNMENTS BY BYTE:
PA00       PA08 D4    PA16 D11   PB00       PB08 A1
PA01       PA09 D3    PA17 D13   PB01       PB09 A2
PA02 A0    PA10 D1    PA18 D10   PB02 A5    PB10 MOSI
PA03       PA11 D0    PA19 D12   PB03       PB11 SCK
PA04 A3    PA12 MISO  PA20 D6    PB04       PB12
PA05 A4    PA13       PA21 D7    PB05       PB13
PA06 D8    PA14 D2    PA22 SDA   PB06       PB14
PA07 D9    PA15 D5    PA23 SCL   PB07       PB15

SAME, METRO M4:
PA00       PA08       PA16 D13   PB00       PB08 A4    PB16 D3
PA01       PA09       PA17 D12   PB01       PB09 A5    PB17 D2
PA02 A0    PA10       PA18 D10   PB02 SDA   PB10       PB18
PA03       PA11       PA19 D11   PB03 SCL   PB11       PB19
PA04 A3    PA12 MISO  PA20 D9    PB04       PB12 D7    PB20
PA05 A1    PA13 SCK   PA21 D8    PB05       PB13 D4    PB21
PA06 A2    PA14 MISO  PA22 D1    PB06       PB14 D5    PB22
PA07       PA15       PA23 D0    PB07       PB15 D6    PB23

FEATHER M4:
PA00       PA08       PA16 D5    PB08 A2    PB16 D1/TX
PA01       PA09       PA17 SCK   PB09 A3    PB17 D0/RX
PA02 A0    PA10       PA18 D6    PB10       PB18
PA03       PA11       PA19 D9    PB11       PB19
PA04 A4    PA12 SDA   PA20 D10   PB12       PB20
PA05 A1    PA13 SCL   PA21 D11   PB13       PB21
PA06 A5    PA14 D4    PA22 D12   PB14       PB22 MISO
PA07       PA15       PA23 D13   PB15       PB23 MOSI

FEATHER M0:
PA00       PA08       PA16 D11   PB00       PB08 A1
PA01       PA09       PA17 D13   PB01       PB09 A2
PA02 A0    PA10 TX/D1 PA18 D10   PB02 A5    PB10 MOSI
PA03       PA11 RX/D0 PA19 D12   PB03       PB11 SCK
PA04 A3    PA12 MISO  PA20 D6    PB04       PB12
PA05 A4    PA13       PA21 D7    PB05       PB13
PA06       PA14       PA22 SDA   PB06       PB14
PA07 D9    PA15 D5    PA23 SCL   PB07       PB15

FEATHER nRF52840:
P0.00      P0.08 D12       P0.24 RXD  P1.08 D5
P0.01      P0.09           P0.25 TXD  P1.09 D13
P0.02 A4   P0.10 D2 (NFC)  P0.26 D9   P1.10
P0.03 A5   P0.11 SCL       P0.27 D10  P1.11
P0.04 A0   P0.12 SDA       P0.28 A3   P1.12
P0.05 A1   P0.13 MOSI      P0.29      P1.13
P0.06 D11  P0.14 SCK       P0.30 A2   P1.14
P0.07 D6   P0.15 MISO      P0.31      P1.15

FEATHER ESP32:
P0.00          P0.08          P0.16 16/RX    P0.24          P1.00 32/A7
P0.01          P0.09          P0.17 17/TX    P0.25 25/A1    P1.01 33/A9/SS
P0.02          P0.10          P0.18 18/MOSI  P0.26 26/A0    P1.02 34/A2 (in)
P0.03          P0.11          P0.19 19/MISO  P0.27 27/A10   P1.03
P0.04 4/A5     P0.12 12/A11   P0.20          P0.28          P1.04 36/A4 (in)
P0.05 5/SCK    P0.13 13/A12   P0.21 21       P0.29          P1.05
P0.06          P0.14 14/A6    P0.22 22/SCL   P0.30          P1.06
P0.07          P0.15 15/A8    P0.23 23/SDA   P0.31          P1.07 39/A3 (in)

GRAND CENTRAL M4: (___ = byte boundaries)
PA00            PB00 D12        PC00 A3        PD00
PA01            PB01 D13 (LED)  PC01 A4        PD01
PA02 A0         PB02 D9         PC02 A5        PD02
PA03 84 (AREF)  PB03 A2         PC03 A6        PD03
PA04 A13        PB04 A7         PC04 D48       PD04
PA05 A1         PB05 A8         PC05 D49       PD05
PA06 A14        PB06 A9         PC06 D46       PD06
PA07 A15 ______ PB07 A10 ______ PC07 D47 _____ PD07 __________
PA08            PB08 A11        PC08           PD08 D51 (SCK)
PA09            PB09 A12        PC09           PD09 D52 (MOSI)
PA10            PB10            PC10 D45       PD10 D53
PA11            PB11            PC11 D44       PD11 D50 (MISO)
PA12 D26        PB12 D18        PC12 D41       PD12 D22
PA13 D27        PB13 D19        PC13 D40       PD13
PA14 D28        PB14 D39        PC14 D43       PD14
PA15 D23 ______ PB15 D38 ______ PC15 D42 _____ PD15 __________
PA16 D37        PB16 D14        PC16 D25       PD16
PA17 D36        PB17 D15        PC17 D24       PD17
PA18 D35        PB18 D8         PC18 D2        PD18
PA19 D34        PB19 D29        PC19 D3        PD19
PA20 D33        PB20 D20 (SDA)  PC20 D4        PD20 D6
PA21 D32        PB21 D21 (SCL)  PC21 D5        PD21 D7
PA22 D31        PB22 D10        PC22 D16       PD22
PA23 D30 ______ PB23 D11 ______ PC23 D17 _____ PD23 __________
PA24            PB24 D1
PA25            PB25 D0
PA26            PB26
PA27            PB27
PA28            PB28
PA29            PB29
PA30            PB30 96 (SWO)
PA31 __________ PB31 95 (SD CD) ______________________________

RGB MATRIX FEATHERWING NOTES:
R1  D6    A   A5
G1  D5    B   A4
B1  D9    C   A3
R2  D11   D   A2
G2  D10   LAT D0/RX
B2  D12   OE  D1/TX
CLK D13
RGB+clock fit in one PORT byte on Feather M4!
RGB+clock are on same PORT but not within same byte on Feather M0 --
the code could run there, but would be super RAM-inefficient. Avoid.
Should be fine on other M0 devices like a Metro, if wiring manually
so one can pick a contiguous byte of PORT bits.
Original RGB Matrix FeatherWing will NOT work on Feather nRF52840
because RGB+clock are on different PORTs. This was resolved by making
a unique version of the FeatherWing that works with that board!
*/
