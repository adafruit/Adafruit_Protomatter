/* ----------------------------------------------------------------------
Refresh-rate test for Protomatter (written for Adafruit Matrix Portal S3).

Draws color gradients, including a dim ramp of the darkest values, and
once per second prints to Serial:
  - refresh:  matrix frames per second (each frame = all rows, all planes)
  - loop:     how often loop() could run per second, i.e. CPU time that's
              left over for the sketch (compare builds, not absolute)

Set WIDTH, HEIGHT and BIT_DEPTH below to match the setup being tested.
Default: three chained 64x32 panels (192x32) at 6 bits.

To compare with the previous ESP32-S3 timing, build with the flag
-DPROTOMATTER_NO_SHORT_PLANES (arduino-cli: --build-property
"compiler.c.extra_flags=-DPROTOMATTER_NO_SHORT_PLANES").
------------------------------------------------------------------------- */

#include <Adafruit_Protomatter.h>

#define WIDTH 192   // Total chain width in pixels (3 x 64)
#define HEIGHT 32   // Panel height: 32 -> 4 address lines, 64 -> 5
#define BIT_DEPTH 6 // 1-6

#if defined(ARDUINO_ADAFRUIT_MATRIXPORTAL_ESP32S3)
uint8_t rgbPins[] = {42, 41, 40, 38, 39, 37};
uint8_t addrPins[] = {45, 36, 48, 35, 21};
uint8_t clockPin = 2;
uint8_t latchPin = 47;
uint8_t oePin = 14;
#elif defined(_VARIANT_MATRIXPORTAL_M4_)
uint8_t rgbPins[] = {7, 8, 9, 10, 11, 12};
uint8_t addrPins[] = {17, 18, 19, 20, 21};
uint8_t clockPin = 14;
uint8_t latchPin = 15;
uint8_t oePin = 16;
#else
#error "Set the pins for this board"
#endif

Adafruit_Protomatter matrix(WIDTH, BIT_DEPTH, 1, rgbPins, HEIGHT == 64 ? 5 : 4,
                            addrPins, clockPin, latchPin, oePin, false);

void drawTestImage(void) {
  int16_t w = matrix.width(), h = matrix.height();
  int16_t band = h / 5;
  for (int16_t x = 0; x < w; x++) {
    uint8_t v = x * 255 / (w - 1); // Full ramp 0-255
    uint8_t d = x * 32 / w;        // Dim ramp 0-31 (lowest bits only)
    matrix.drawFastVLine(x, 0 * band, band, matrix.color565(v, 0, 0));
    matrix.drawFastVLine(x, 1 * band, band, matrix.color565(0, v, 0));
    matrix.drawFastVLine(x, 2 * band, band, matrix.color565(0, 0, v));
    matrix.drawFastVLine(x, 3 * band, band, matrix.color565(v, v, v));
    matrix.drawFastVLine(x, 4 * band, h - 4 * band, matrix.color565(d, d, d));
  }
  matrix.show();
}

void setup(void) {
  Serial.begin(115200);
  ProtomatterStatus status = matrix.begin();
  Serial.printf("Protomatter begin() status: %d\n", (int)status);
  if (status != PROTOMATTER_OK) {
    for (;;)
      ;
  }
  drawTestImage();
  Serial.printf("%dx%d, %d bits\n", matrix.width(), matrix.height(), BIT_DEPTH);
}

void loop(void) {
  static uint32_t lastTime = millis(), loops = 0;
  loops++;
  uint32_t now = millis();
  if (now - lastTime >= 1000) {
    uint32_t frames = matrix.getFrameCount(); // Frames since last call
    Serial.printf("refresh: %lu Hz   loop: %lu /s\n",
                  (unsigned long)(frames * 1000 / (now - lastTime)),
                  (unsigned long)(loops * 1000 / (now - lastTime)));
    lastTime = now;
    loops = 0;
  }
}
