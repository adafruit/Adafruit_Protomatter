#include "Adafruit_Protomatter.h"

/*
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

RGB Matrix FeatherWing:
R1  D6    A   A5
G1  D5    B   A4
B1  D9    C   A3
R2  D11   D   A2
G2  D10   LAT D0/RX
B2  D12   OE  D1/TX
CLK D13
RGB+clock in one PORT byte on Feather M4!
RGB+clock are on same PORT but not within same byte on Feather M0 --
the code could run there (with some work to be done in the convert_*
functions), but would be super RAM-inefficient. Should be fine on other
M0 devices like a Metro, if wiring manually so one can pick a contiguous
byte of PORT bits.
RGB+clock are on different PORTs on nRF52840.
*/

#if defined(__SAMD51__)
  // Use FeatherWing pinout
  uint8_t rgbPins[]  = {6, 5, 9, 11, 10, 12};
  uint8_t addrPins[] = {A5, A4, A3, A2};
  uint8_t clockPin   = 13;
  uint8_t latchPin   = 0;
  uint8_t oePin      = 1;
#elif defined(_SAMD21_)
  uint8_t rgbPins[]  = {6, 7, 10, 11, 12, 13};
  uint8_t addrPins[] = {0, 1, 2, 3};
  uint8_t clockPin   = SDA;
  uint8_t latchPin   = 4;
  uint8_t oePin      = 5;
#elif defined(NRF52_SERIES)
  // Special nRF52840 FeatherWing pinout
  uint8_t rgbPins[]  = {6, A5, A1, A0, A4, 11};
  uint8_t addrPins[] = {10, 5, 13, 9};
  uint8_t clockPin   = 12;
  uint8_t latchPin   = PIN_SERIAL1_RX;
  uint8_t oePin      = PIN_SERIAL1_TX;
#elif defined(ESP32)
  // 'Safe' pins (not overlapping any peripherals):
  // GPIO.out: 4, 12, 13, 14, 15, 21, 27, GPIO.out1: 32, 33
  // Peripheral-overlapping pins, sorted from 'most expendible':
  // 16, 17 (RX, TX), 25, 26 (A0, A1), 18, 5, 9 (MOSI, SCK, MISO), 22, 23 (SCL, SDA)
  uint8_t rgbPins[]  = {4, 12, 13, 14, 15, 21};
  uint8_t addrPins[] = {16, 17, 25, 26};
  uint8_t clockPin   = 27; // Must be on same port as rgbPins
  uint8_t latchPin   = 32;
  uint8_t oePin      = 33;
#elif defined(ARDUINO_TEENSY40)
  uint8_t rgbPins[]  = {15, 16, 17, 20, 21, 22}; // A1-A3, A6-A8, skips SDA,SCL
  uint8_t addrPins[] = {2, 3, 4, 5};
  uint8_t clockPin   = 23; // A9
  uint8_t latchPin   = 6;
  uint8_t oePin      = 9;
#elif defined(ARDUINO_TEENSY41)
  uint8_t rgbPins[]  = {26, 27, 38, 20, 21, 22}; // A12-14, A6-A8 (yes that's a 38, NOT 28!)
  uint8_t addrPins[] = {2, 3, 4, 5};
  uint8_t clockPin   = 23; // A9
  uint8_t latchPin   = 6;
  uint8_t oePin      = 9;
#endif

// Last arg here enables double-buffering
Adafruit_Protomatter matrix(
  64, 6, 1, rgbPins, 4, addrPins, clockPin, latchPin, oePin, true);

int16_t textMin,
        textX      = matrix.width(),
        hue        = 0;
char    str[40];
int8_t  ball[3][4] = {
  {  3,  0,  1,  1 }, // Initial X,Y pos & velocity for 3 bouncy balls
  { 17, 15,  1, -1 },
  { 27,  4, -1,  1 }
};

const uint16_t ballcolor[3] = {
  0b0000000001000000, // Dark green
  0b0000000000000001, // Dark blue
  0b0000100000000000  // Dark red
};

void setup(void) {
  Serial.begin(9600);

  ProtomatterStatus status = matrix.begin();
  Serial.print("Protomatter begin() status: ");
  Serial.println((int)status);

  sprintf(str, "Adafruit %dx%d RGB LED Matrix",
    matrix.width(), matrix.height());
  textMin = strlen(str) * -12;
  matrix.setTextWrap(false);
  matrix.setTextSize(2);
  matrix.setTextColor(0xFFFF); // White
}

void loop(void) {
  byte i;

  // Clear background
  matrix.fillScreen(0);

  // Bounce three balls around
  for(i=0; i<3; i++) {
    // Draw 'ball'
    matrix.fillCircle(ball[i][0], ball[i][1], 5, ballcolor[i]);
    // Update X, Y position
    ball[i][0] += ball[i][2];
    ball[i][1] += ball[i][3];
    // Bounce off edges
    if((ball[i][0] == 0) || (ball[i][0] == (matrix.width() - 1)))
      ball[i][2] *= -1;
    if((ball[i][1] == 0) || (ball[i][1] == (matrix.height() - 1)))
      ball[i][3] *= -1;
  }

  // Draw big scrolly text on top
  matrix.setCursor(textX, 1);
  matrix.print(str);

  // Move text left (w/wrap), increase hue
  if((--textX) < textMin) textX = matrix.width();
  hue += 7;
  if(hue >= 1536) hue -= 1536;

  matrix.show();

  delay(20);
}
