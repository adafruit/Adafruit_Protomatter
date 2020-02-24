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
*/

uint8_t rgbPins[]  = {6, 7, 10, 11, 12, 13};
uint8_t addrPins[] = {0, 1, 2, 3};
uint8_t clockPin   = SDA;
uint8_t latchPin   = 4;
uint8_t oePin      = 5;
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
