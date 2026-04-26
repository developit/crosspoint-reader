#pragma once

#include <Arduino.h>

// IT8951-based EInkDisplay implementation for M5Paper (540x960 panel).
// Provides the same public interface as the open-x4-sdk EInkDisplay so the
// HAL layer and GfxRenderer can use it without modification.
class EInkDisplay {
 public:
  // SPI pins: sclk/mosi/cs match M5Paper IT8951 wiring.
  // dc and rst are accepted for API compatibility but are unused (IT8951 does
  // not use a separate DC line).
  EInkDisplay(int8_t sclk, int8_t mosi, int8_t cs, int8_t dc, int8_t rst, int8_t busy);

  enum RefreshMode {
    FULL_REFRESH,  // GC16 waveform – best quality, slowest
    HALF_REFRESH,  // GL16 waveform – balanced
    FAST_REFRESH   // DU  waveform – fastest, 1-bit only
  };

  // No-op: M5Paper is a fixed hardware target, not selectable at runtime.
  void setDisplayX3() {}

  void begin();

  // Panel dimensions for the M5Paper 4.7" IT8951 e-ink display.
  // Physical panel: 540 px wide × 960 px tall (portrait orientation).
  // Row stride: ceil(540/8) = 68 bytes so the 1-bpp framebuffer rows are
  // byte-aligned; the final 4 bits of each row are unused padding.
  static constexpr uint16_t DISPLAY_WIDTH = 540;
  static constexpr uint16_t DISPLAY_HEIGHT = 960;
  static constexpr uint16_t DISPLAY_WIDTH_BYTES = 68;  // ceil(540/8)
  static constexpr uint32_t BUFFER_SIZE = DISPLAY_WIDTH_BYTES * DISPLAY_HEIGHT;

  // Runtime geometry getters (match open-x4-sdk EInkDisplay API).
  uint16_t getDisplayWidth() const { return DISPLAY_WIDTH; }
  uint16_t getDisplayHeight() const { return DISPLAY_HEIGHT; }
  uint16_t getDisplayWidthBytes() const { return DISPLAY_WIDTH_BYTES; }
  uint32_t getBufferSize() const { return BUFFER_SIZE; }

  // Framebuffer operations (write into the 1-bpp RAM buffer).
  void clearScreen(uint8_t color = 0xFF) const;
  void drawImage(const uint8_t* imageData, uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                 bool fromProgmem = false) const;
  void drawImageTransparent(const uint8_t* imageData, uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                             bool fromProgmem = false) const;

  // Display update: push the framebuffer to the IT8951 and trigger a refresh.
  void displayBuffer(RefreshMode mode = FAST_REFRESH, bool turnOffScreen = false);
  void refreshDisplay(RefreshMode mode = FAST_REFRESH, bool turnOffScreen = false);

  // Grayscale support: 2-bit (4-level) gray using LSB + MSB plane technique.
  void copyGrayscaleBuffers(const uint8_t* lsbBuffer, const uint8_t* msbBuffer);
  void copyGrayscaleLsbBuffers(const uint8_t* lsbBuffer);
  void copyGrayscaleMsbBuffers(const uint8_t* msbBuffer);
#ifdef EINK_DISPLAY_SINGLE_BUFFER_MODE
  void cleanupGrayscaleBuffers(const uint8_t* bwBuffer);
#endif
  void displayGrayBuffer(bool turnOffScreen = false);

  // No-op for IT8951: resync concept does not apply.
  void requestResync(uint8_t settlePasses = 0) {}

  // Power management.
  void deepSleep();

  // Access the 1-bpp framebuffer held in ESP32 SRAM.
  uint8_t* getFrameBuffer() const { return frameBuffer; }

 private:
  // IT8951 SPI protocol helpers.
  void waitBusy() const;
  void writeCommand(uint16_t cmd) const;
  void writeData(uint16_t data) const;
  uint16_t readData() const;
  void writeReg(uint16_t reg, uint16_t val) const;
  uint16_t readReg(uint16_t reg) const;

  // Image loading helpers.
  void setImageBufferAddress(uint32_t addr) const;
  void loadImageArea(uint16_t x, uint16_t y, uint16_t w, uint16_t h, RefreshMode mode) const;
  void sendPixels1bpp(uint16_t x, uint16_t y, uint16_t w, uint16_t h) const;
  void sendGrayPixels(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                      const uint8_t* lsb, const uint8_t* msb) const;
  void triggerDisplayArea(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t waveformMode) const;

  // SPI pin numbers.
  int8_t _sclk;
  int8_t _mosi;
  int8_t _cs;
  int8_t _busy;  // HRDY – active HIGH when IT8951 is ready

  // IT8951 internal image buffer address (read from DevInfo on begin()).
  uint32_t imgBufAddr = 0;

  // 1-bpp framebuffer: DISPLAY_WIDTH_BYTES × DISPLAY_HEIGHT bytes in SRAM.
  static uint8_t frameBuffer[BUFFER_SIZE];

  // Grayscale: temporary copies of LSB/MSB planes while displayGrayBuffer()
  // is pending. Allocated in copyGrayscale*Buffers(), freed after display.
  uint8_t* grayLsbBuf = nullptr;
  uint8_t* grayMsbBuf = nullptr;
};
