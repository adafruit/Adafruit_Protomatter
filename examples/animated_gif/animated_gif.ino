// Play GIFs from CIRCUITPY drive (USB-accessible filesystem) to LED matrix.
// ***DESIGNED FOR ADAFRUIT MATRIXPORTAL***, but may run on some other M4,
// M0, ESP32S3 and nRF52 boards (relies on TinyUSB stack). As written, runs
// on 64x32 pixel matrix, this can be changed by editing the WIDTH and HEIGHT
// definitions. See the "simple" example for a run-down on matrix config.
// Adapted from examples from Larry Bank's AnimatedGIF library and
// msc_external_flash example in Adafruit_TinyUSB_Arduino.
// Prerequisite libraries:
//   - Adafruit_Protomatter
//   - Adafruit_SPIFlash
//   - Adafruit_TinyUSB
//   - SdFat (Adafruit fork)
//   - AnimatedGIF
// Set ENABLE_EXTENDED_TRANSFER_CLASS and FAT12_SUPPORT in SdFatConfig.h.
// Select Tools->USB Stack->TinyUSB before compiling.

#include <Adafruit_Protomatter.h>
#include <Adafruit_SPIFlash.h>
#include <Adafruit_TinyUSB.h>
#include <AnimatedGIF.h>
#include <SPI.h>
#include <SdFat.h>

// CONFIGURABLE SETTINGS ---------------------------------------------------

char GIFpath[] = "/gifs";     // Absolute path to GIFs on CIRCUITPY drive
uint16_t GIFminimumTime = 10; // Min. repeat time (seconds) until next GIF
#define WIDTH  64             // Matrix width in pixels
#define HEIGHT 32             // Matrix height in pixels
// Maximim matrix height is 32px on most boards, 64 on MatrixPortal if the
// 'E' jumper is set.

// FLASH FILESYSTEM STUFF --------------------------------------------------

// External flash macros for QSPI or SPI are defined in board variant file.
#if defined(ARDUINO_ARCH_ESP32)
static Adafruit_FlashTransport_ESP32 flashTransport;
#elif defined(EXTERNAL_FLASH_USE_QSPI)
Adafruit_FlashTransport_QSPI flashTransport;
#elif defined(EXTERNAL_FLASH_USE_SPI)
Adafruit_FlashTransport_SPI flashTransport(EXTERNAL_FLASH_USE_CS,
                                           EXTERNAL_FLASH_USE_SPI);
#else
#error No QSPI/SPI flash are defined in your board variant.h!
#endif

Adafruit_SPIFlash flash(&flashTransport);
FatFileSystem filesys;     // Filesystem object from SdFat
Adafruit_USBD_MSC usb_msc; // USB mass storage object

// RGB MATRIX (PROTOMATTER) LIBRARY STUFF ----------------------------------

#if defined(_VARIANT_MATRIXPORTAL_M4_)
uint8_t rgbPins[] = {7, 8, 9, 10, 11, 12};
uint8_t addrPins[] = {17, 18, 19, 20, 21}; // 16/32/64 pixels tall
uint8_t clockPin = 14;
uint8_t latchPin = 15;
uint8_t oePin = 16;
#define BACK_BUTTON 2
#define NEXT_BUTTON 3
#elif defined(ARDUINO_ADAFRUIT_MATRIXPORTAL_ESP32S3)
uint8_t rgbPins[] = {42, 41, 40, 38, 39, 37};
uint8_t addrPins[] = {45, 36, 48, 35, 21}; // 16/32/64 pixels tall
uint8_t clockPin = 2;
uint8_t latchPin = 47;
uint8_t oePin = 14;
#define BACK_BUTTON 6
#define NEXT_BUTTON 7
#elif defined(_VARIANT_METRO_M4_)
uint8_t rgbPins[] = {2, 3, 4, 5, 6, 7};
uint8_t addrPins[] = {A0, A1, A2, A3}; // 16 or 32 pixels tall
uint8_t clockPin = A4;
uint8_t latchPin = 10;
uint8_t oePin = 9;
#elif defined(_VARIANT_FEATHER_M4_)
uint8_t rgbPins[] = {6, 5, 9, 11, 10, 12};
uint8_t addrPins[] = {A5, A4, A3, A2}; // 16 or 32 pixels tall
uint8_t clockPin = 13;
uint8_t latchPin = 0;
uint8_t oePin = 1;
#endif
#if HEIGHT == 16
#define NUM_ADDR_PINS 3
#elif HEIGHT == 32
#define NUM_ADDR_PINS 4
#elif HEIGHT == 64
#define NUM_ADDR_PINS 5
#endif

Adafruit_Protomatter matrix(WIDTH, 6, 1, rgbPins, NUM_ADDR_PINS, addrPins,
                            clockPin, latchPin, oePin, true);

// ANIMATEDGIF LIBRARY STUFF -----------------------------------------------

AnimatedGIF GIF;
File GIFfile;
int16_t xPos = 0, yPos = 0; // Top-left pixel coord of GIF in matrix space

// FILE ACCESS FUNCTIONS REQUIRED BY ANIMATED GIF LIB ----------------------

// Pass in ABSOLUTE PATH of GIF file to open
void *GIFOpenFile(const char *filename, int32_t *pSize) {
  GIFfile = filesys.open(filename);
  if (GIFfile) {
    *pSize = GIFfile.size();
    return (void *)&GIFfile;
  }
  return NULL;
}

void GIFCloseFile(void *pHandle) {
  File *f = static_cast<File *>(pHandle);
  if (f) f->close();
}

int32_t GIFReadFile(GIFFILE *pFile, uint8_t *pBuf, int32_t iLen) {
  int32_t iBytesRead = iLen;
  File *f = static_cast<File *>(pFile->fHandle);
  // If a file is read all the way to last byte, seek() stops working
  if ((pFile->iSize - pFile->iPos) < iLen)
    iBytesRead = pFile->iSize - pFile->iPos - 1; // ugly work-around
  if (iBytesRead <= 0) return 0;
  iBytesRead = (int32_t)f->read(pBuf, iBytesRead);
  pFile->iPos = f->position();
  return iBytesRead;
}

int32_t GIFSeekFile(GIFFILE *pFile, int32_t iPosition) {
  File *f = static_cast<File *>(pFile->fHandle);
  f->seek(iPosition);
  pFile->iPos = (int32_t)f->position();
  return pFile->iPos;
}

// Draw one line of image to matrix back buffer
void GIFDraw(GIFDRAW *pDraw) {
  uint8_t *s;
  uint16_t *d, *usPalette, usTemp[320];
  int x, y;

  y = pDraw->iY + pDraw->y; // current line in image

  // Vertical clip
  int16_t screenY = yPos + y; // current row on matrix
  if ((screenY < 0) || (screenY >= matrix.height())) return;

  usPalette = pDraw->pPalette;

  s = pDraw->pPixels;
  // Apply the new pixels to the main image
  if (pDraw->ucHasTransparency) { // if transparency used
    uint8_t *pEnd, c, ucTransparent = pDraw->ucTransparent;
    int x, iCount;
    pEnd = s + pDraw->iWidth;
    x = 0;
    iCount = 0; // count non-transparent pixels
    while (x < pDraw->iWidth) {
      c = ucTransparent - 1;
      d = usTemp;
      while (c != ucTransparent && s < pEnd) {
        c = *s++;
        if (c == ucTransparent) { // done, stop
          s--;                    // back up to treat it like transparent
        } else {                  // opaque
          *d++ = usPalette[c];
          iCount++;
        }
      }             // while looking for opaque pixels
      if (iCount) { // any opaque pixels?
        span(usTemp, xPos + pDraw->iX + x, screenY, iCount);
        x += iCount;
        iCount = 0;
      }
      // no, look for a run of transparent pixels
      c = ucTransparent;
      while (c == ucTransparent && s < pEnd) {
        c = *s++;
        if (c == ucTransparent)
          iCount++;
        else
          s--;
      }
      if (iCount) {
        x += iCount; // skip these
        iCount = 0;
      }
    }
  } else {
    s = pDraw->pPixels;
    // Translate 8-bit pixels through RGB565 palette (already byte reversed)
    for (x = 0; x < pDraw->iWidth; x++)
      usTemp[x] = usPalette[*s++];
    span(usTemp, xPos + pDraw->iX, screenY, pDraw->iWidth);
  }
}

// Copy a horizontal span of pixels from a source buffer to an X,Y position
// in matrix back buffer, applying horizontal clipping. Vertical clipping is
// handled in GIFDraw() above -- y can safely be assumed valid here.
void span(uint16_t *src, int16_t x, int16_t y, int16_t width) {
  if (x >= matrix.width()) return; // Span entirely off right of matrix
  int16_t x2 = x + width - 1;      // Rightmost pixel
  if (x2 < 0) return;              // Span entirely off left of matrix
  if (x < 0) {                     // Span partially off left of matrix
    width += x;                    // Decrease span width
    src -= x;                      // Increment source pointer to new start
    x = 0;                         // Leftmost pixel is first column
  }
  if (x2 >= matrix.width()) {      // Span partially off right of matrix
    width -= (x2 - matrix.width() + 1);
  }
  if(matrix.getRotation() == 0) {
    memcpy(matrix.getBuffer() + y * matrix.width() + x, src, width * 2);
  } else {
    while(x <= x2) {
      matrix.drawPixel(x++, y, *src++);
    }
  }
}

// FUNCTIONS REQUIRED FOR USB MASS STORAGE ---------------------------------

static bool msc_changed = true; // Is set true on filesystem changes

// Callback on READ10 command.
int32_t msc_read_cb(uint32_t lba, void *buffer, uint32_t bufsize) {
  return flash.readBlocks(lba, (uint8_t *)buffer, bufsize / 512) ? bufsize : -1;
}

// Callback on WRITE10 command.
int32_t msc_write_cb(uint32_t lba, uint8_t *buffer, uint32_t bufsize) {
  digitalWrite(LED_BUILTIN, HIGH);
  return flash.writeBlocks(lba, buffer, bufsize / 512) ? bufsize : -1;
}

// Callback on WRITE10 completion.
void msc_flush_cb(void) {
  flash.syncBlocks();   // Sync with flash
  filesys.cacheClear(); // Clear filesystem cache to force refresh
  digitalWrite(LED_BUILTIN, LOW);
  msc_changed = true;
}

// Get number of files in a specified path that match extension ('filter').
// Pass in absolute path (e.g. "/" or "/gifs") and extension WITHOUT period
// (e.g. "gif", NOT ".gif").
int16_t numFiles(const char *path, const char *filter) {
  File dir = filesys.open(path);
  if (!dir) return -1;
  char filename[256];
  for(int16_t num_files = 0;;) {
    File entry = dir.openNextFile();
    if (!entry) return num_files; // No more files
    entry.getName(filename, sizeof(filename) - 1);
    entry.close();
    if (!entry.isDirectory() &&       // Skip directories
        strncmp(filename, "._", 2)) { // and Mac junk files
      char *extension = strrchr(filename, '.');
      if (extension && !strcasecmp(&extension[1], filter)) num_files++;
    }
  }
  return -1;
}

// Return name of file (matching extension) by index (0 to numFiles()-1)
char *filenameByIndex(const char *path, const char *filter, int16_t index) {
  static char filename[256]; // Must be static, we return a pointer to this!
  File entry, dir = filesys.open(path);
  if (!dir) return NULL;
  while(entry = dir.openNextFile()) {
    entry.getName(filename, sizeof(filename) - 1);
    entry.close();
    if(!entry.isDirectory() &&       // Skip directories
       strncmp(filename, "._", 2)) { // and Mac junk files
      char *extension = strrchr(filename, '.');
      if (extension  && !strcasecmp(&extension[1], filter)) {
        if(!index--) {
          return filename;
        }
      }
    }
  }
  return NULL;
}

// SETUP FUNCTION - RUNS ONCE AT STARTUP -----------------------------------

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
#if defined(BACK_BUTTON)
  pinMode(BACK_BUTTON, INPUT_PULLUP);
#endif
#if defined(NEXT_BUTTON)
  pinMode(NEXT_BUTTON, INPUT_PULLUP);
#endif

  // USB mass storage / filesystem setup (do BEFORE Serial init)
  flash.begin();
  // Set disk vendor id, product id and revision
  usb_msc.setID("Adafruit", "External Flash", "1.0");
  // Set disk size, block size is 512 regardless of spi flash page size
  usb_msc.setCapacity(flash.pageSize() * flash.numPages() / 512, 512);
  usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);
  usb_msc.setUnitReady(true); // MSC is ready for read/write
  usb_msc.begin();
  filesys.begin(&flash); // Start filesystem on the flash

  Serial.begin(115200);
  //while (!Serial);

  // Protomatter (RGB matrix) setup
  ProtomatterStatus status = matrix.begin();
  Serial.print("Protomatter begin() status: ");
  Serial.println((int)status);
  matrix.fillScreen(0);
  matrix.show();

  // GIF setup
  GIF.begin(LITTLE_ENDIAN_PIXELS);
}

// LOOP FUNCTION - RUNS REPEATEDLY UNTIL RESET / POWER OFF -----------------

int16_t GIFindex = -1;     // Current file index in GIFpath
int8_t GIFincrement = 1;   // +1 = next GIF, -1 = prev, 0 = same
uint32_t GIFstartTime = 0; // When current GIF started playing
bool GIFisOpen = false;    // True if GIF is currently open

void loop() {
  if (msc_changed) {     // If filesystem has changed...
    msc_changed = false; // Clear flag
    GIFincrement = 1;    // Set index to next file when we resume here
    return;              // Prioritize USB, handled in calling func
  }

#if defined(BACK_BUTTON)
  if(!digitalRead(BACK_BUTTON)) {
    GIFincrement = -1;                // Back
    while(!digitalRead(BACK_BUTTON)); // Wait for release
  }
#endif
#if defined(NEXT_BUTTON)
  if(!digitalRead(NEXT_BUTTON)) {
    GIFincrement = 1;                 // Forward
    while(!digitalRead(NEXT_BUTTON)); // Wait for release
  }
#endif

  if (GIFincrement) { // Change file?
    if (GIFisOpen) {  // If currently playing,
      GIF.close();    // stop it
      GIFisOpen = false;
    }
    GIFindex += GIFincrement; // Fwd or back 1 file
    int num_files = numFiles(GIFpath, "GIF");
    if(GIFindex >= num_files) GIFindex = 0;         // 'Wrap around' file index
    else if(GIFindex < 0) GIFindex = num_files - 1; // both directions

    char *filename = filenameByIndex(GIFpath, "GIF", GIFindex);
    if (filename) {
      char fullname[sizeof GIFpath + 256];
      sprintf(fullname, "%s/%s", GIFpath, filename); // Absolute path to GIF
      Serial.printf("Opening file '%s'\n", fullname);
      if (GIF.open(fullname, GIFOpenFile, GIFCloseFile,
                   GIFReadFile, GIFSeekFile, GIFDraw)) {
        matrix.fillScreen(0);
        Serial.printf("GIF dimensions: %d x %d\n",
                      GIF.getCanvasWidth(), GIF.getCanvasHeight());
        xPos = (matrix.width() - GIF.getCanvasWidth()) / 2; // Center on matrix
        yPos = (matrix.height() - GIF.getCanvasHeight()) / 2;
        GIFisOpen = true;
        GIFstartTime = millis();
        GIFincrement = 0; // Reset increment flag
      }
    }
  } else if(GIFisOpen) {
    if (GIF.playFrame(true, NULL) >= 0) { // Auto resets to start if needed
      matrix.show();
      if ((millis() - GIFstartTime) >= (GIFminimumTime * 1000)) {
        GIFincrement = 1; // Minimum time has elapsed, proceed to next GIF
      }
    } else {
      GIFincrement = 1; // Decode error, proceed to next GIF
    }
  }
}
