/*!
 * @file Adafruit_SSD1306.cpp
 *
 * @mainpage Arduino library for monochrome OLEDs based on SSD1306 drivers.
 *
 * @section intro_sec Introduction
 *
 * This is documentation for Adafruit's SSD1306 library for monochrome
 * OLED displays: http://www.adafruit.com/category/63_98
 *
 * These displays use I2C or SPI to communicate. I2C requires 2 pins
 * (SCL+SDA) and optionally a RESET pin. SPI requires 4 pins (MOSI, SCK,
 * select, data/command) and optionally a reset pin. Hardware SPI or
 * 'bitbang' software SPI are both supported.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section dependencies Dependencies
 *
 * This library depends on <a
 * href="https://github.com/adafruit/Adafruit-GFX-Library"> Adafruit_GFX</a>
 * being present on your system. Please make sure you have installed the latest
 * version before using this library.
 *
 * @section author Author
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries, with
 * contributions from the open source community.
 *
 * @section license License
 *
 * BSD license, all text above, and the splash screen included below,
 * must be included in any redistribution.
 *
 */

#ifdef __AVR__
#include <avr/pgmspace.h>
#elif defined(ESP8266) || defined(ESP32)
#include <pgmspace.h>
#else
#define pgm_read_byte(addr)                                                    \
  (*(const unsigned char *)(addr)) ///< PROGMEM workaround for non-AVR
#endif

#if !defined(__ARM_ARCH) && !defined(ENERGIA) && !defined(ESP8266) &&          \
    !defined(ESP32) && !defined(__arc__)
#include <util/delay.h>
#endif

#include "GP1287Lib.h"
#include "splash.h"
#include <Adafruit_GFX.h>

// SOME DEFINES AND STATIC VARIABLES USED INTERNALLY -----------------------

#ifdef HAVE_PORTREG
#define GP1287_SELECT *csPort &= ~csPinMask;       ///< Device select
#define GP1287_DESELECT *csPort |= csPinMask;      ///< Device deselect
#else
#define GP1287_SELECT digitalWrite(csPin, LOW);       ///< Device select
#define GP1287_DESELECT digitalWrite(csPin, HIGH);    ///< Device deselect
#endif

#if defined(SPI_HAS_TRANSACTION)
#define SPI_TRANSACTION_START spi->beginTransaction(spiSettings) ///< Pre-SPI
#define SPI_TRANSACTION_END spi->endTransaction()                ///< Post-SPI
#else // SPI transactions likewise not present in older Arduino SPI lib
#define SPI_TRANSACTION_START ///< Dummy stand-in define
#define SPI_TRANSACTION_END   ///< keeps compiler happy
#endif

// The definition of 'transaction' is broadened a bit in the context of
// this library -- referring not just to SPI transactions (if supported
// in the version of the SPI library being used), but also chip select
// (if SPI is being used, whether hardware or soft), and also to the
// beginning and end of I2C transfers (the Wire clock may be sped up before
// issuing data to the display, then restored to the default rate afterward
// so other I2C device types still work).  All of these are encapsulated
// in the TRANSACTION_* macros.

// Check first if Wire, then hardware SPI, then soft SPI:
#define TRANSACTION_START                                                     \                                                               
    SPI_TRANSACTION_START;                                                     \                                                                     
    GP1287_SELECT;                                                            \
  
#define TRANSACTION_END                                                        \
    GP1287_DESELECT;                                                          \
    SPI_TRANSACTION_END;                                                     \
 
 

// CONSTRUCTORS, DESTRUCTOR ------------------------------------------------

/*!
    @brief  Constructor for SPI GP1287VFD displays, using software (bitbang)
            SPI.
    @param  w
            Display width in pixels
    @param  h
            Display height in pixels
    @param  mosi_pin
            MOSI (master out, slave in) pin (using Arduino pin numbering).
            This transfers serial data from microcontroller to display.
    @param  sclk_pin
            SCLK (serial clock) pin (using Arduino pin numbering).
            This clocks each bit from MOSI.
    @param  dc_pin
            Data/command pin (using Arduino pin numbering), selects whether
            display is receiving commands (low) or data (high).
    @param  rst_pin
            Reset pin (using Arduino pin numbering), or -1 if not used
            (some displays might be wired to share the microcontroller's
            reset pin).
    @param  cs_pin
            Chip-select pin (using Arduino pin numbering) for sharing the
            bus with other devices. Active low.
    @return GP1287VFD object.
    @note   Call the object's begin() function before use -- buffer
            allocation is performed there!
*/
GP1287VFD::GP1287VFD(uint16_t w, uint8_t h, int8_t mosi_pin,
                                   int8_t sclk_pin, 
                                   int8_t rst_pin, int8_t cs_pin)
    : Adafruit_GFX(w, h), spi(NULL), buffer(NULL),
      mosiPin(mosi_pin), clkPin(sclk_pin), csPin(cs_pin),
      rstPin(rst_pin) {}

/*!
    @brief  Constructor for SPI GP1287VFD displays, using native hardware SPI.
    @param  w
            Display width in pixels
    @param  h
            Display height in pixels
    @param  spi
            Pointer to an existing SPIClass instance (e.g. &SPI, the
            microcontroller's primary SPI bus).
    @param  dc_pin
            Data/command pin (using Arduino pin numbering), selects whether
            display is receiving commands (low) or data (high).
    @param  rst_pin
            Reset pin (using Arduino pin numbering), or -1 if not used
            (some displays might be wired to share the microcontroller's
            reset pin).
    @param  cs_pin
            Chip-select pin (using Arduino pin numbering) for sharing the
            bus with other devices. Active low.
    @param  bitrate
            SPI clock rate for transfers to this display. Default if
            unspecified is 8000000UL (8 MHz).
    @return GP1287VFD object.
    @note   Call the object's begin() function before use -- buffer
            allocation is performed there!
*/
GP1287VFD::GP1287VFD(uint16_t w, uint8_t h, SPIClass *spi,
                                   int8_t rst_pin, int8_t cs_pin,
                                   uint32_t bitrate)
    : Adafruit_GFX(w, h), spi(spi ? spi : &SPI), buffer(NULL),
      mosiPin(-1), clkPin(-1), csPin(cs_pin), rstPin(rst_pin) {
#ifdef SPI_HAS_TRANSACTION
  spiSettings = SPISettings(bitrate, LSBFIRST, SPI_MODE0);
#endif
}

/*!
    @brief  Destructor for GP1287VFD object.
*/
GP1287VFD::~GP1287VFD(void) {
  if (buffer) {
    free(buffer);
    buffer = NULL;
  }
}

// LOW-LEVEL UTILS ---------------------------------------------------------

// Issue single byte out SPI, either soft or hardware as appropriate.
// SPI transaction/selection must be performed in calling function.
inline void GP1287VFD::SPIwrite(uint8_t d) {
  if (spi) {
    (void)spi->transfer(d);
  } else {
    for (uint8_t bit = 0x80; bit; bit >>= 1) {
#ifdef HAVE_PORTREG
      if (d & bit)
        *mosiPort |= mosiPinMask;
      else
        *mosiPort &= ~mosiPinMask;
      *clkPort |= clkPinMask;  // Clock high
      *clkPort &= ~clkPinMask; // Clock low
#else
      digitalWrite(mosiPin, d & bit);
      digitalWrite(clkPin, HIGH);
      digitalWrite(clkPin, LOW);
#endif
    }
  }
}

// Issue single command to GP1287VFD, using I2C or hard/soft SPI as needed.
// Because command calls are often grouped, SPI transaction and selection
// must be started/ended in calling function for efficiency.
// This is a private function, not exposed (see ssd1306_command() instead).
void GP1287VFD::gp1287_command(uint8_t c) {
    SPIwrite(c);
}


// ALLOCATE & INIT DISPLAY -------------------------------------------------

/*!
    @brief  Allocate RAM for image buffer, initialize peripherals and pins.
    @param  vcs
            VCC selection. Pass SSD1306_SWITCHCAPVCC to generate the display
            voltage (step up) from the 3.3V source, or SSD1306_EXTERNALVCC
            otherwise. Most situations with Adafruit SSD1306 breakouts will
            want SSD1306_SWITCHCAPVCC.
    @param  addr
            I2C address of corresponding SSD1306 display (or pass 0 to use
            default of 0x3C for 128x32 display, 0x3D for all others).
            SPI displays (hardware or software) do not use addresses, but
            this argument is still required (pass 0 or any value really,
            it will simply be ignored). Default if unspecified is 0.
    @param  reset
            If true, and if the reset pin passed to the constructor is
            valid, a hard reset will be performed before initializing the
            display. If using multiple SSD1306 displays on the same bus, and
            if they all share the same reset pin, you should only pass true
            on the first display being initialized, false on all others,
            else the already-initialized displays would be reset. Default if
            unspecified is true.
    @param  periphBegin
            If true, and if a hardware peripheral is being used (I2C or SPI,
            but not software SPI), call that peripheral's begin() function,
            else (false) it has already been done in one's sketch code.
            Cases where false might be used include multiple displays or
            other devices sharing a common bus, or situations on some
            platforms where a nonstandard begin() function is available
            (e.g. a TwoWire interface on non-default pins, as can be done
            on the ESP8266 and perhaps others).
    @return true on successful allocation/init, false otherwise.
            Well-behaved code should check the return value before
            proceeding.
    @note   MUST call this function before any drawing or updates!
*/
bool GP1287VFD::begin(bool reset,
                             bool periphBegin) {

  if ((!buffer) && !(buffer = (uint8_t *)malloc(WIDTH * ((HEIGHT + 7) / 8))))
    return false;

  clearDisplay();




  // Setup pin directions
  // Using one of the SPI modes, either soft or hardware
    pinMode(csPin, OUTPUT); // Same for chip select
#ifdef HAVE_PORTREG
    csPort = (PortReg *)portOutputRegister(digitalPinToPort(csPin));
    csPinMask = digitalPinToBitMask(csPin);
#endif
    GP1287_DESELECT
    if (spi) { // Hardware SPI
      // SPI peripheral begin same as wire check above.
      if (periphBegin)
        spi->begin();
    } else {                    // Soft SPI
      pinMode(mosiPin, OUTPUT); // MOSI and SCLK outputs
      pinMode(clkPin, OUTPUT);
#ifdef HAVE_PORTREG
      mosiPort = (PortReg *)portOutputRegister(digitalPinToPort(mosiPin));
      mosiPinMask = digitalPinToBitMask(mosiPin);
      clkPort = (PortReg *)portOutputRegister(digitalPinToPort(clkPin));
      clkPinMask = digitalPinToBitMask(clkPin);
      *clkPort &= ~clkPinMask; // Clock low
#else
      digitalWrite(clkPin, LOW); // Clock low
#endif
    }
  

  // Reset GP1287 if requested and reset pin specified in constructor
  if (reset && (rstPin >= 0)) {
    pinMode(rstPin, OUTPUT);
    digitalWrite(rstPin, HIGH);
    delay(1);                   // VDD goes high at start, pause for 1 ms
    digitalWrite(rstPin, LOW);  // Bring reset low
    delay(10);                  // Wait 10 ms
    digitalWrite(rstPin, HIGH); // Bring out of reset
  }

  // software reset
  TRANSACTION_START
  gp1287_command(GP1287VFD_SOFTWARERESET);
  TRANSACTION_END

  delay(10);
 
  //VFD mode setting
  TRANSACTION_START
  gp1287_command(GP1287VFD_SETVFDMODE);
  gp1287_command(0x02);
  gp1287_command(0x00);
  TRANSACTION_END  
 
  //Display area setting
  TRANSACTION_START
  gp1287_command(GP1287VFD_SETDISPLAYAREA);
  gp1287_command(0xFF);
  gp1287_command(0x31);
  gp1287_command(0x00);  
  gp1287_command(0x20);
  gp1287_command(0x00);
  gp1287_command(0x00);
  gp1287_command(0x80);    
  TRANSACTION_END 
 
  //Internal speed setting
  TRANSACTION_START
  gp1287_command(GP1287VFD_SETINTERNALSPEED);
  gp1287_command(0x20);
  gp1287_command(0x3F);
  gp1287_command(0x00);  
  gp1287_command(0x01);  
  TRANSACTION_END 

  delay(10);
 
  //Dimming level setting
  TRANSACTION_START
  gp1287_command(GP1287VFD_SETDIMMING);
  gp1287_command(0x03);
  gp1287_command(0xFF);
  TRANSACTION_END 
  
  //Clear GRAM
  TRANSACTION_START
  gp1287_command(GP1287VFD_CLEARGRAM);
  TRANSACTION_END

  delay(20);

  //Display position offset
  TRANSACTION_START
  gp1287_command(GP1287VFD_DISPLAYPOSITIONOFFSET);
  gp1287_command(0x00);
  gp1287_command(0x00);
  TRANSACTION_END  
 
  //Display mode setting
  TRANSACTION_START
  gp1287_command(GP1287VFD_SETDISPLAYMODE);
  gp1287_command(0x00);
  TRANSACTION_END 

  return true; // Success
}

// DRAWING FUNCTIONS -------------------------------------------------------

/*!
    @brief  Set/clear/invert a single pixel. This is also invoked by the
            Adafruit_GFX library in generating many higher-level graphics
            primitives.
    @param  x
            Column of display -- 0 at left to (screen width - 1) at right.
    @param  y
            Row of display -- 0 at top to (screen height -1) at bottom.
    @param  color
            Pixel color, one of: GP1287VFD_BLACK, GP1287VFD_WHITE or GP1287VFD_INVERT.
    @return None (void).
    @note   Changes buffer contents only, no immediate effect on display.
            Follow up with a call to display(), or with other graphics
            commands as needed by one's own application.
*/
void GP1287VFD::drawPixel(int16_t x, int16_t y, uint16_t color) {
  if ((x >= 0) && (x < width()) && (y >= 0) && (y < height())) {
    // Pixel is in-bounds. Rotate coordinates if needed.
    switch (getRotation()) {
    case 1:
      GP1287VFD_swap(x, y);
      x = WIDTH - x - 1;
      break;
    case 2:
      x = WIDTH - x - 1;
      y = HEIGHT - y - 1;
      break;
    case 3:
      GP1287VFD_swap(x, y);
      y = HEIGHT - y - 1;
      break;
    }
    switch (color) {
    case GP1287VFD_WHITE:
      buffer[x + (y / 8) * WIDTH] |= (1 << (y & 7));
      break;
    case GP1287VFD_BLACK:
      buffer[x + (y / 8) * WIDTH] &= ~(1 << (y & 7));
      break;
    case GP1287VFD_INVERSE:
      buffer[x + (y / 8) * WIDTH] ^= (1 << (y & 7));
      break;
    }
  }
}

/*!
    @brief  Clear contents of display buffer (set all pixels to off).
    @return None (void).
    @note   Changes buffer contents only, no immediate effect on display.
            Follow up with a call to display(), or with other graphics
            commands as needed by one's own application.
*/
void GP1287VFD::clearDisplay(void) {
  memset(buffer, 0, WIDTH * ((HEIGHT + 7) / 8));
}

/*!
    @brief  Draw a horizontal line. This is also invoked by the Adafruit_GFX
            library in generating many higher-level graphics primitives.
    @param  x
            Leftmost column -- 0 at left to (screen width - 1) at right.
    @param  y
            Row of display -- 0 at top to (screen height -1) at bottom.
    @param  w
            Width of line, in pixels.
    @param  color
            Line color, one of: GP1287VFD_BLACK, GP1287VFD_WHITE or GP1287VFD_INVERT.
    @return None (void).
    @note   Changes buffer contents only, no immediate effect on display.
            Follow up with a call to display(), or with other graphics
            commands as needed by one's own application.
*/
void GP1287VFD::drawFastHLine(int16_t x, int16_t y, int16_t w,
                                     uint16_t color) {
  bool bSwap = false;
  switch (rotation) {
  case 1:
    // 90 degree rotation, swap x & y for rotation, then invert x
    bSwap = true;
    GP1287VFD_swap(x, y);
    x = WIDTH - x - 1;
    break;
  case 2:
    // 180 degree rotation, invert x and y, then shift y around for height.
    x = WIDTH - x - 1;
    y = HEIGHT - y - 1;
    x -= (w - 1);
    break;
  case 3:
    // 270 degree rotation, swap x & y for rotation,
    // then invert y and adjust y for w (not to become h)
    bSwap = true;
    GP1287VFD_swap(x, y);
    y = HEIGHT - y - 1;
    y -= (w - 1);
    break;
  }

  if (bSwap)
    drawFastVLineInternal(x, y, w, color);
  else
    drawFastHLineInternal(x, y, w, color);
}

void GP1287VFD::drawFastHLineInternal(int16_t x, int16_t y, int16_t w,
                                             uint16_t color) {

  if ((y >= 0) && (y < HEIGHT)) { // Y coord in bounds?
    if (x < 0) {                  // Clip left
      w += x;
      x = 0;
    }
    if ((x + w) > WIDTH) { // Clip right
      w = (WIDTH - x);
    }
    if (w > 0) { // Proceed only if width is positive
      uint8_t *pBuf = &buffer[(y / 8) * WIDTH + x], mask = 1 << (y & 7);
      switch (color) {
      case GP1287VFD_WHITE:
        while (w--) {
          *pBuf++ |= mask;
        };
        break;
      case GP1287VFD_BLACK:
        mask = ~mask;
        while (w--) {
          *pBuf++ &= mask;
        };
        break;
      case GP1287VFD_INVERSE:
        while (w--) {
          *pBuf++ ^= mask;
        };
        break;
      }
    }
  }
}

/*!
    @brief  Draw a vertical line. This is also invoked by the Adafruit_GFX
            library in generating many higher-level graphics primitives.
    @param  x
            Column of display -- 0 at left to (screen width -1) at right.
    @param  y
            Topmost row -- 0 at top to (screen height - 1) at bottom.
    @param  h
            Height of line, in pixels.
    @param  color
            Line color, one of: GP1287VFD_BLACK, GP1287VFD_WHITE or GP1287VFD_INVERT.
    @return None (void).
    @note   Changes buffer contents only, no immediate effect on display.
            Follow up with a call to display(), or with other graphics
            commands as needed by one's own application.
*/
void GP1287VFD::drawFastVLine(int16_t x, int16_t y, int16_t h,
                                     uint16_t color) {
  bool bSwap = false;
  switch (rotation) {
  case 1:
    // 90 degree rotation, swap x & y for rotation,
    // then invert x and adjust x for h (now to become w)
    bSwap = true;
    GP1287VFD_swap(x, y);
    x = WIDTH - x - 1;
    x -= (h - 1);
    break;
  case 2:
    // 180 degree rotation, invert x and y, then shift y around for height.
    x = WIDTH - x - 1;
    y = HEIGHT - y - 1;
    y -= (h - 1);
    break;
  case 3:
    // 270 degree rotation, swap x & y for rotation, then invert y
    bSwap = true;
    GP1287VFD_swap(x, y);
    y = HEIGHT - y - 1;
    break;
  }

  if (bSwap)
    drawFastHLineInternal(x, y, h, color);
  else
    drawFastVLineInternal(x, y, h, color);
}

void GP1287VFD::drawFastVLineInternal(int16_t x, int16_t __y,
                                             int16_t __h, uint16_t color) {

  if ((x >= 0) && (x < WIDTH)) { // X coord in bounds?
    if (__y < 0) {               // Clip top
      __h += __y;
      __y = 0;
    }
    if ((__y + __h) > HEIGHT) { // Clip bottom
      __h = (HEIGHT - __y);
    }
    if (__h > 0) { // Proceed only if height is now positive
      // this display doesn't need ints for coordinates,
      // use local byte registers for faster juggling
      uint8_t y = __y, h = __h;
      uint8_t *pBuf = &buffer[(y / 8) * WIDTH + x];

      // do the first partial byte, if necessary - this requires some masking
      uint8_t mod = (y & 7);
      if (mod) {
        // mask off the high n bits we want to set
        mod = 8 - mod;
        // note - lookup table results in a nearly 10% performance
        // improvement in fill* functions
        // uint8_t mask = ~(0xFF >> mod);
        static const uint8_t PROGMEM premask[8] = {0x00, 0x80, 0xC0, 0xE0,
                                                   0xF0, 0xF8, 0xFC, 0xFE};
        uint8_t mask = pgm_read_byte(&premask[mod]);
        // adjust the mask if we're not going to reach the end of this byte
        if (h < mod)
          mask &= (0XFF >> (mod - h));

        switch (color) {
        case GP1287VFD_WHITE:
          *pBuf |= mask;
          break;
        case GP1287VFD_BLACK:
          *pBuf &= ~mask;
          break;
        case GP1287VFD_INVERSE:
          *pBuf ^= mask;
          break;
        }
        pBuf += WIDTH;
      }

      if (h >= mod) { // More to go?
        h -= mod;
        // Write solid bytes while we can - effectively 8 rows at a time
        if (h >= 8) {
          if (color == GP1287VFD_INVERSE) {
            // separate copy of the code so we don't impact performance of
            // black/white write version with an extra comparison per loop
            do {
              *pBuf ^= 0xFF; // Invert byte
              pBuf += WIDTH; // Advance pointer 8 rows
              h -= 8;        // Subtract 8 rows from height
            } while (h >= 8);
          } else {
            // store a local value to work with
            uint8_t val = (color != GP1287VFD_BLACK) ? 255 : 0;
            do {
              *pBuf = val;   // Set byte
              pBuf += WIDTH; // Advance pointer 8 rows
              h -= 8;        // Subtract 8 rows from height
            } while (h >= 8);
          }
        }

        if (h) { // Do the final partial byte, if necessary
          mod = h & 7;
          // this time we want to mask the low bits of the byte,
          // vs the high bits we did above
          // uint8_t mask = (1 << mod) - 1;
          // note - lookup table results in a nearly 10% performance
          // improvement in fill* functions
          static const uint8_t PROGMEM postmask[8] = {0x00, 0x01, 0x03, 0x07,
                                                      0x0F, 0x1F, 0x3F, 0x7F};
          uint8_t mask = pgm_read_byte(&postmask[mod]);
          switch (color) {
          case GP1287VFD_WHITE:
            *pBuf |= mask;
            break;
          case GP1287VFD_BLACK:
            *pBuf &= ~mask;
            break;
          case GP1287VFD_INVERSE:
            *pBuf ^= mask;
            break;
          }
        }
      }
    } // endif positive height
  }   // endif x in bounds
}

/*!
    @brief  Return color of a single pixel in display buffer.
    @param  x
            Column of display -- 0 at left to (screen width - 1) at right.
    @param  y
            Row of display -- 0 at top to (screen height -1) at bottom.
    @return true if pixel is set (usually GP1287VFD_WHITE, unless display invert
   mode is enabled), false if clear (GP1287VFD_BLACK).
    @note   Reads from buffer contents; may not reflect current contents of
            screen if display() has not been called.
*/
bool GP1287VFD::getPixel(int16_t x, int16_t y) {
  if ((x >= 0) && (x < width()) && (y >= 0) && (y < height())) {
    // Pixel is in-bounds. Rotate coordinates if needed.
    switch (getRotation()) {
    case 1:
      GP1287VFD_swap(x, y);
      x = WIDTH - x - 1;
      break;
    case 2:
      x = WIDTH - x - 1;
      y = HEIGHT - y - 1;
      break;
    case 3:
      GP1287VFD_swap(x, y);
      y = HEIGHT - y - 1;
      break;
    }
    return (buffer[x + (y / 8) * WIDTH] & (1 << (y & 7)));
  }
  return false; // Pixel out of bounds
}

/*!
    @brief  Get base address of display buffer for direct reading or writing.
    @return Pointer to an unsigned 8-bit array, column-major, columns padded
            to full byte boundary if needed.
*/
uint8_t *GP1287VFD::getBuffer(void) { return buffer; }

// REFRESH DISPLAY ---------------------------------------------------------

/*!
    @brief  Push data currently in RAM to GP1287VFD display.
    @return None (void).
    @note   Drawing operations are not visible until this function is
            called. Call after each graphics command, or after a whole set
            of graphics commands, as best needed by one's own application.
*/
void GP1287VFD::display(void) {

  uint8_t *ptr = buffer;
   
  for(int y_offs=0;y_offs<((HEIGHT+7)/8);y_offs++) {
      TRANSACTION_START
#if defined(ESP8266)
  // ESP8266 needs a periodic yield() call to avoid watchdog reset.
  // With the limited size of GP1287VFD displays, and the fast bitrate
  // being used (1 MHz or more), I think one yield() immediately before
  // a screen write and one immediately after should cover it.  But if
  // not, if this becomes a problem, yields() might be added in the
  // 32-byte transfer condition below.
  yield();
#endif	  

  gp1287_command(GP1287VFD_WRITEGRAM);
  gp1287_command(0x00);
  gp1287_command(y_offs*8);
 //   gp1287_command(0x00);
  gp1287_command(0x07);  
  
 for (int cnt=0;cnt<WIDTH;cnt++) {
	
	 SPIwrite(*ptr++);
 }

  TRANSACTION_END
  }
#if defined(ESP8266)
  yield();
#endif
}



// OTHER HARDWARE SETTINGS -------------------------------------------------

/*!
    @brief  Enable or disable display invert mode (white-on-black vs
            black-on-white).
    @param  i
            If true, switch to invert mode (black-on-white), else normal
            mode (white-on-black).
    @return None (void).
    @note   This has an immediate effect on the display, no need to call the
            display() function -- buffer contents are not changed, rather a
            different pixel mode of the display hardware is used. When
            enabled, drawing GP1287VFD_BLACK (value 0) pixels will actually draw
   white, GP1287VFD_WHITE (value 1) will draw black.
*/
void GP1287VFD::invertDisplay(bool i) {
  TRANSACTION_START
  gp1287_command(GP1287VFD_SETDISPLAYMODE);
  gp1287_command(i ? GP1287VFD_INVERTEDDISPLAY : GP1287VFD_NORMALDISPLAY);
  TRANSACTION_END
}

/*!
    @brief  Dim the display.
    @param  dim
            true to enable lower brightness mode, false for full brightness.
    @return None (void).
    @note   This has an immediate effect on the display, no need to call the
            display() function -- buffer contents are not changed.
*/
void GP1287VFD::dim(uint16_t dim) {
  dim = constrain(dim,0,1023);

  uint8_t dim_h;
  uint8_t dim_l;

  dim_h = dim/256;
  dim_l = dim%256;

  TRANSACTION_START
  gp1287_command(GP1287VFD_SETDIMMING);
  gp1287_command(dim_h);
  gp1287_command(dim_l);
  TRANSACTION_END
}
