/*!
 * @file GP1287Lib.h
 *
 * This is part of for Adafruit's SSD1306 library for monochrome
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
 * Written by Limor Fried/Ladyada for Adafruit Industries, with
 * contributions from the open source community.
 *
 * BSD license, all text above, and the splash screen header file,
 * must be included in any redistribution.
 *
 */

#ifndef _GP1287Lib_H_
#define _GP1287Lib_H_

#if defined(ARDUINO_STM32_FEATHER)
typedef class HardwareSPI SPIClass;
#endif

#include <Adafruit_GFX.h>
#include <SPI.h>

#if defined(__AVR__)
typedef volatile uint8_t PortReg;
typedef uint8_t PortMask;
#define HAVE_PORTREG
#elif defined(__SAM3X8E__)
typedef volatile RwReg PortReg;
typedef uint32_t PortMask;
#define HAVE_PORTREG
#elif (defined(__arm__) || defined(ARDUINO_FEATHER52)) &&                      \
    !defined(ARDUINO_ARCH_MBED)
typedef volatile uint32_t PortReg;
typedef uint32_t PortMask;
#define HAVE_PORTREG
#endif

/// The following "raw" color names are kept for backwards client compatability
/// They can be disabled by predefining this macro before including the Adafruit
/// header client code will then need to be modified to use the scoped enum
/// values directly
#ifndef NO_ADAFRUIT_SSD1306_COLOR_COMPATIBILITY
#define BLACK GP1287VFD_BLACK     ///< Draw 'off' pixels
#define WHITE GP1287VFD_WHITE     ///< Draw 'on' pixels
#define INVERSE GP1287VFD_INVERSE ///< Invert pixels
#endif
/// fit into the GP1287VFD_ naming scheme
#define GP1287VFD_BLACK 0   ///< Draw 'off' pixels
#define GP1287VFD_WHITE 1   ///< Draw 'on' pixels
#define GP1287VFD_INVERSE 2 ///< Invert pixels

#define GP1287VFD_SOFTWARERESET 0xAA          ///< See datasheet
#define GP1287VFD_SETVFDMODE 0xCC          ///< See datasheet
#define GP1287VFD_SETDISPLAYAREA 0xE0            ///< See datasheet
#define GP1287VFD_SETINTERNALSPEED 0xB1         ///< See datasheet
#define GP1287VFD_SETDIMMING 0xA0          ///< See datasheet
#define GP1287VFD_CLEARGRAM 0x55            ///< See datasheet
#define GP1287VFD_DISPLAYPOSITIONOFFSET 0xC0        ///< See datasheet
#define GP1287VFD_SETDISPLAYMODE 0x80       ///< See datasheet
#define GP1287VFD_WRITEGRAM 0xF0       ///< See datasheet

#define GP1287VFD_NORMALDISPLAY 0x00  ///< 
#define GP1287VFD_INVERTEDDISPLAY 0x01 ///< 

#define GP1287VFD_swap(a, b)                                                     \
  (((a) ^= (b)), ((b) ^= (a)), ((a) ^= (b))) ///< No-temp-var swap operation

/*!
    @brief  Class that stores state and functions for interacting with
            SSD1306 OLED displays.
*/
class GP1287VFD : public Adafruit_GFX {
public:
  // NEW CONSTRUCTORS -- recommended for new projects
  GP1287VFD(uint16_t w, uint8_t h, int8_t mosi_pin, int8_t sclk_pin,
                   int8_t rst_pin, int8_t cs_pin);
  GP1287VFD(uint16_t w, uint8_t h, SPIClass *spi, 
                   int8_t rst_pin, int8_t cs_pin, uint32_t bitrate = 8000000UL);

  ~GP1287VFD(void);

  bool begin(
             bool reset = true, bool periphBegin = true);
  void display(void);
  void clearDisplay(void);
  void invertDisplay(bool i);
  void dim(uint16_t dim);
  void drawPixel(int16_t x, int16_t y, uint16_t color);
  virtual void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
  virtual void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
  bool getPixel(int16_t x, int16_t y);
  uint8_t *getBuffer(void);

private:
  inline void SPIwrite(uint8_t d) __attribute__((always_inline));
  void drawFastHLineInternal(int16_t x, int16_t y, int16_t w, uint16_t color);
  void drawFastVLineInternal(int16_t x, int16_t y, int16_t h, uint16_t color);
  void gp1287_command(uint8_t c);

  SPIClass *spi;
  uint8_t *buffer;
  int8_t page_end;
  int8_t mosiPin, clkPin, dcPin, csPin, rstPin;
#ifdef HAVE_PORTREG
  PortReg *mosiPort, *clkPort, *dcPort, *csPort;
  PortMask mosiPinMask, clkPinMask, dcPinMask, csPinMask;
#endif

#if defined(SPI_HAS_TRANSACTION)
protected:
  // Allow sub-class to change
  SPISettings spiSettings;
#endif
};

#endif // _GP1287Lib_H_
