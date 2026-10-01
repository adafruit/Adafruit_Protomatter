/*
  Serial ABC row selection on a 128x64 FM6126A panel, using MatrixPortal S3.

  Some panels marked HUB75E have only A, B and C connected. On this type,
  A clocks a row shift register, B enables shifting, and C supplies row data.
  This is different from ordinary binary A/B/C/D/E row addressing. The
  FM6126A pixel driver alone does not identify which row protocol is used.

  Select Adafruit MatrixPortal ESP32-S3 in the Arduino board menu. Connect
  the panel's INPUT connector to the MatrixPortal and use a suitable 5V
  panel supply. This example uses dim text; it is not a current limiter.
*/

#include <Adafruit_Protomatter.h>

#if !defined(ARDUINO_ADAFRUIT_MATRIXPORTAL_ESP32S3)
#error "Select Adafruit MatrixPortal ESP32-S3 for this example."
#endif

// This panel swaps red and blue in both halves. For standard RGB order,
// use {42, 41, 40, 38, 39, 37} instead.
uint8_t rgbPins[] = {40, 41, 42, 37, 39, 38};
uint8_t addrPins[] = {45, 36, 48}; // A: row clock, B: enable, C: row data
uint8_t clockPin = 2;
uint8_t latchPin = 47;
uint8_t oePin = 14;

Adafruit_Protomatter matrix(
  128, 4, 1, rgbPins, // Width, bit depth, one RGB pin set
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
