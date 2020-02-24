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

RGB Matrix FeatherWing:
R1  D6    A   A5
G1  D5    B   A4
B1  D9    C   A3
R2  D11   D   A2
G2  D10   LAT D0/RX
B2  D12   OE  D1/TX
CLK D13
RGB+clock in one PORT byte on Feather M4!
*/

#if defined(__SAMD51__)
  uint8_t rgbPins[]  = {6, 5, 9, 11, 10, 12};
  uint8_t addrPins[] = {A5, A4, A3, A2};
  uint8_t clockPin   = 13;
  uint8_t latchPin   = 0;
  uint8_t oePin      = 1;
#else // SAMD21
  uint8_t rgbPins[]  = {6, 7, 10, 11, 12, 13};
  uint8_t addrPins[] = {0, 1, 2, 3};
  uint8_t clockPin   = SDA;
  uint8_t latchPin   = 4;
  uint8_t oePin      = 5;
#endif

Adafruit_Protomatter matter(
  64, 4, 1, rgbPins, 4, addrPins, clockPin, latchPin, oePin, false);

void setup(void) {
  Serial.begin(9600);

  ProtomatterStatus status = matter.begin();
  Serial.print("Protomatter begin() status: ");
  Serial.println((int)status);

  matter.fillRect( 1, 1, 8, 8, 0b1111100000000000); // Red
  matter.fillRect( 7, 4, 8, 8, 0b0000011111100000); // Green
  matter.fillRect(13, 7, 8, 8, 0b0000000000011111); // Blue
  matter.show(); // Push data to matrix
}

void loop(void) {
  Serial.print("Refresh FPS = ~");
  Serial.println(matter.getFrameCount());
  delay(1000);
}
