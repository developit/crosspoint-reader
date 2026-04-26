#include "EInkDisplay.h"

#include <SPI.h>
#include <esp_heap_caps.h>

// ---------------------------------------------------------------------------
// IT8951 host interface constants
// Based on the IT8951 Host Interface Command Specification.
// ---------------------------------------------------------------------------

// Preamble words sent before a command or data transfer.
static constexpr uint16_t IT8951_PREAMBLE_CMD  = 0x6000;  // write a command
static constexpr uint16_t IT8951_PREAMBLE_WR   = 0x0000;  // write data
static constexpr uint16_t IT8951_PREAMBLE_RD   = 0x1000;  // read data

// Tcon command codes.
static constexpr uint16_t IT8951_TCON_SYS_RUN      = 0x0001;
static constexpr uint16_t IT8951_TCON_STANDBY       = 0x0002;
static constexpr uint16_t IT8951_TCON_SLEEP         = 0x0003;
static constexpr uint16_t IT8951_TCON_REG_RD        = 0x0010;
static constexpr uint16_t IT8951_TCON_REG_WR        = 0x0011;
static constexpr uint16_t IT8951_TCON_LD_IMG        = 0x0020;
static constexpr uint16_t IT8951_TCON_LD_IMG_AREA   = 0x0021;
static constexpr uint16_t IT8951_TCON_LD_IMG_END    = 0x0022;
static constexpr uint16_t IT8951_TCON_DISP_AREA     = 0x0034;
static constexpr uint16_t IT8951_TCON_INQUIRY       = 0x0302;  // get DevInfo

// IT8951 register addresses (accessed via REG_RD / REG_WR commands).
static constexpr uint16_t IT8951_LISAR_LO  = 0x0008;  // Load Image Start Addr (lo)
static constexpr uint16_t IT8951_LISAR_HI  = 0x0009;  // Load Image Start Addr (hi)

// Waveform mode codes (passed to DISP_AREA command).
static constexpr uint16_t IT8951_WFM_DU   = 1;   // fast, 2-level
static constexpr uint16_t IT8951_WFM_GC16 = 2;   // high quality 16-gray
static constexpr uint16_t IT8951_WFM_GL16 = 3;   // ghost-less 16-gray

// LD_IMG_AREA first argument packing.
// Bit [15:12] rotate, bit [11:8] BPP, bit [7:0] endian/mem-mode.
// BPP: 0=2bpp, 1=3bpp, 2=4bpp, 3=8bpp.
static constexpr uint16_t IT8951_BPP_4    = 2;   // 4-bit per pixel
static constexpr uint16_t IT8951_ENDIAN_L = 0;   // little-endian
static constexpr uint16_t IT8951_ROTATE_0 = 0;   // no rotation

// Construct the first arg word for LD_IMG_AREA.
static constexpr uint16_t LD_IMG_ARG(uint16_t bpp, uint16_t endian = IT8951_ENDIAN_L,
                                     uint16_t rotate = IT8951_ROTATE_0) {
  return static_cast<uint16_t>((endian << 8) | (bpp << 4) | rotate);
}

// SPI speed (IT8951 supports up to 24 MHz).
static constexpr uint32_t IT8951_SPI_HZ = 24000000UL;

// HRDY busy-wait timeout (ms).
static constexpr unsigned long IT8951_BUSY_TIMEOUT_MS = 5000;

// ---------------------------------------------------------------------------
// Static storage for the 1-bpp framebuffer.
// Placed in DRAM (not PSRAM) for lowest latency during rendering.
// ---------------------------------------------------------------------------
uint8_t EInkDisplay::frameBuffer[EInkDisplay::BUFFER_SIZE];

// ---------------------------------------------------------------------------
// Constructor / begin
// ---------------------------------------------------------------------------

EInkDisplay::EInkDisplay(int8_t sclk, int8_t mosi, int8_t cs, int8_t /*dc*/, int8_t /*rst*/, int8_t busy)
    : _sclk(sclk), _mosi(mosi), _cs(cs), _busy(busy) {}

// ---------------------------------------------------------------------------
// Low-level IT8951 SPI protocol helpers
// ---------------------------------------------------------------------------

void EInkDisplay::waitBusy() const {
  const unsigned long deadline = millis() + IT8951_BUSY_TIMEOUT_MS;
  while (digitalRead(_busy) == LOW) {
    if (millis() > deadline) {
      if (Serial) Serial.printf("[EInk] HRDY timeout!\n");
      break;
    }
    delayMicroseconds(10);
  }
}

void EInkDisplay::writeCommand(uint16_t cmd) const {
  waitBusy();
  SPI.beginTransaction(SPISettings(IT8951_SPI_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, LOW);
  SPI.transfer16(IT8951_PREAMBLE_CMD);
  waitBusy();
  SPI.transfer16(cmd);
  digitalWrite(_cs, HIGH);
  SPI.endTransaction();
}

void EInkDisplay::writeData(uint16_t data) const {
  waitBusy();
  SPI.beginTransaction(SPISettings(IT8951_SPI_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, LOW);
  SPI.transfer16(IT8951_PREAMBLE_WR);
  waitBusy();
  SPI.transfer16(data);
  digitalWrite(_cs, HIGH);
  SPI.endTransaction();
}

uint16_t EInkDisplay::readData() const {
  waitBusy();
  SPI.beginTransaction(SPISettings(IT8951_SPI_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, LOW);
  SPI.transfer16(IT8951_PREAMBLE_RD);
  waitBusy();
  SPI.transfer16(0x0000);  // dummy word
  const uint16_t value = SPI.transfer16(0x0000);
  digitalWrite(_cs, HIGH);
  SPI.endTransaction();
  return value;
}

void EInkDisplay::writeReg(uint16_t reg, uint16_t val) const {
  writeCommand(IT8951_TCON_REG_WR);
  writeData(reg);
  writeData(val);
}

uint16_t EInkDisplay::readReg(uint16_t reg) const {
  writeCommand(IT8951_TCON_REG_RD);
  writeData(reg);
  return readData();
}

// ---------------------------------------------------------------------------
// Set the IT8951 load-image start address (LISAR register pair).
// ---------------------------------------------------------------------------
void EInkDisplay::setImageBufferAddress(uint32_t addr) const {
  writeReg(IT8951_LISAR_LO, static_cast<uint16_t>(addr & 0xFFFF));
  writeReg(IT8951_LISAR_HI, static_cast<uint16_t>(addr >> 16));
}

// ---------------------------------------------------------------------------
// begin(): initialise the IT8951 and allocate the framebuffer.
// ---------------------------------------------------------------------------
void EInkDisplay::begin() {
  if (Serial) Serial.printf("[EInk] IT8951 begin()\n");

  pinMode(_cs,   OUTPUT);
  pinMode(_busy, INPUT);
  digitalWrite(_cs, HIGH);

  // Allow the IT8951 power rails to stabilise.
  delay(100);
  waitBusy();

  // Wake the IT8951.
  writeCommand(IT8951_TCON_SYS_RUN);
  delay(10);

  // Read device info (command 0x0302).
  // DevInfo layout (all uint16_t words):
  //   [0] panelW, [1] panelH, [2] imgBufAddrL, [3] imgBufAddrH,
  //   [4..11] fwVersion (8 words), [12..19] lutVersion (8 words)
  writeCommand(IT8951_TCON_INQUIRY);
  const uint16_t panelW      = readData();
  const uint16_t panelH      = readData();
  const uint16_t imgAddrL    = readData();
  const uint16_t imgAddrH    = readData();
  // Consume firmware and LUT version strings (8 + 8 words).
  for (int i = 0; i < 16; i++) { readData(); }

  imgBufAddr = (static_cast<uint32_t>(imgAddrH) << 16) | imgAddrL;
  if (Serial)
    Serial.printf("[EInk] panel=%ux%u imgBuf=0x%08X\n", panelW, panelH, imgBufAddr);

  // Point the IT8951 at its internal image buffer.
  setImageBufferAddress(imgBufAddr);

  // Initialise the framebuffer to white (0xFF = all bits 1 = white).
  memset(frameBuffer, 0xFF, BUFFER_SIZE);

  // Perform one full INIT refresh to clear the panel.
  sendPixels1bpp(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT);
  writeCommand(IT8951_TCON_LD_IMG_END);
  triggerDisplayArea(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, 0 /* INIT mode */);
  if (Serial) Serial.printf("[EInk] init refresh done\n");
}

// ---------------------------------------------------------------------------
// Pixel streaming helpers
// ---------------------------------------------------------------------------

// Expand a single 1-bpp source byte into four 4-bpp output bytes.
// In 4-bpp IT8951 format: 0x00 = black, 0xFF = white; each nibble is one pixel.
static inline void expand1bpp_to_4x4bpp(uint8_t src, uint8_t* out) {
  out[0] = ((src & 0x80) ? 0xF0u : 0x00u) | ((src & 0x40) ? 0x0Fu : 0x00u);
  out[1] = ((src & 0x20) ? 0xF0u : 0x00u) | ((src & 0x10) ? 0x0Fu : 0x00u);
  out[2] = ((src & 0x08) ? 0xF0u : 0x00u) | ((src & 0x04) ? 0x0Fu : 0x00u);
  out[3] = ((src & 0x02) ? 0xF0u : 0x00u) | ((src & 0x01) ? 0x0Fu : 0x00u);
}

// Send all pixels from the 1-bpp framebuffer to the IT8951's internal RAM
// for the specified rectangle.  Converts on-the-fly: no extra heap needed.
//
// Column stride: DISPLAY_WIDTH_BYTES (68) bytes per row; only the first
// DISPLAY_WIDTH (540) pixels of each row are valid – the last 4 padding bits
// are excluded from the IT8951 transfer.
void EInkDisplay::sendPixels1bpp(uint16_t x, uint16_t y, uint16_t w, uint16_t h) const {
  // Begin loading image area.
  writeCommand(IT8951_TCON_LD_IMG_AREA);
  writeData(LD_IMG_ARG(IT8951_BPP_4));  // 4bpp, no rotation, little-endian
  writeData(x);
  writeData(y);
  writeData(w);
  writeData(h);

  // Open a single SPI transaction for the entire pixel stream to avoid
  // beginTransaction/endTransaction overhead per pixel.
  waitBusy();
  SPI.beginTransaction(SPISettings(IT8951_SPI_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, LOW);
  SPI.transfer16(IT8951_PREAMBLE_WR);
  waitBusy();

  // Iterate over the requested rows.  Each source row is DISPLAY_WIDTH_BYTES
  // (68) bytes wide.  The 540-pixel row maps to exactly 270 4-bpp output bytes.
  // 67 source bytes × 4 output bytes = 268 bytes, plus 2 bytes from the last
  // partial source byte (only bits 7-4 are valid pixels 536-539).
  const uint16_t rowStart = y;
  const uint16_t rowEnd   = y + h;
  for (uint16_t row = rowStart; row < rowEnd; row++) {
    const uint8_t* src = frameBuffer + static_cast<uint32_t>(row) * DISPLAY_WIDTH_BYTES;
    uint8_t out[4];

    // Full bytes (each covers 8 pixels → 4 output 4bpp bytes).
    for (int b = 0; b < 67; b++) {
      expand1bpp_to_4x4bpp(src[b], out);
      SPI.transfer(out[0]);
      SPI.transfer(out[1]);
      SPI.transfer(out[2]);
      SPI.transfer(out[3]);
    }
    // Last partial byte: pixels 536-539 live in bits 7-4 of src[67].
    // Bits 3-0 are padding and must not be sent.
    const uint8_t lastByte = src[67];
    SPI.transfer(((lastByte & 0x80) ? 0xF0u : 0x00u) | ((lastByte & 0x40) ? 0x0Fu : 0x00u));
    SPI.transfer(((lastByte & 0x20) ? 0xF0u : 0x00u) | ((lastByte & 0x10) ? 0x0Fu : 0x00u));
  }

  digitalWrite(_cs, HIGH);
  SPI.endTransaction();
}

// Send 2-bit (LSB+MSB) gray pixels to the IT8951 as 4-bpp.
// Gray levels: 00=black, 01=dark gray, 10=light gray, 11=white.
void EInkDisplay::sendGrayPixels(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                                  const uint8_t* lsb, const uint8_t* msb) const {
  writeCommand(IT8951_TCON_LD_IMG_AREA);
  writeData(LD_IMG_ARG(IT8951_BPP_4));
  writeData(x);
  writeData(y);
  writeData(w);
  writeData(h);

  waitBusy();
  SPI.beginTransaction(SPISettings(IT8951_SPI_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, LOW);
  SPI.transfer16(IT8951_PREAMBLE_WR);
  waitBusy();

  // Map 2-bit gray (lsb bit + msb bit) to 4-bit gray (0-15).
  // lsb=0 msb=0 → 0x00 (black)
  // lsb=1 msb=0 → 0x55 (dark gray, ~1/3)
  // lsb=0 msb=1 → 0xAA (light gray, ~2/3)
  // lsb=1 msb=1 → 0xFF (white)
  static constexpr uint8_t kGrayLUT[4] = {0x00, 0x55, 0xAA, 0xFF};

  const uint16_t rowStart = y;
  const uint16_t rowEnd   = y + h;

  for (uint16_t row = rowStart; row < rowEnd; row++) {
    const uint8_t* lsbRow = lsb + static_cast<uint32_t>(row) * DISPLAY_WIDTH_BYTES;
    const uint8_t* msbRow = msb + static_cast<uint32_t>(row) * DISPLAY_WIDTH_BYTES;

    for (int b = 0; b < 67; b++) {
      const uint8_t lsbByte = lsbRow[b];
      const uint8_t msbByte = msbRow[b];

      for (int bit = 7; bit >= 0; bit -= 2) {
        const uint8_t p0 = static_cast<uint8_t>(((lsbByte >> (bit    )) & 1) |
                                                 (((msbByte >> (bit    )) & 1) << 1));
        const uint8_t p1 = static_cast<uint8_t>(((lsbByte >> (bit - 1)) & 1) |
                                                 (((msbByte >> (bit - 1)) & 1) << 1));
        SPI.transfer(static_cast<uint8_t>((kGrayLUT[p0] & 0xF0u) | (kGrayLUT[p1] >> 4)));
      }
    }
    // Last partial byte (pixels 536-539, bits 7-4 only).
    const uint8_t lsbLast = lsbRow[67];
    const uint8_t msbLast = msbRow[67];
    for (int bit = 7; bit >= 4; bit -= 2) {
      const uint8_t p0 = static_cast<uint8_t>(((lsbLast >> (bit    )) & 1) |
                                               (((msbLast >> (bit    )) & 1) << 1));
      const uint8_t p1 = static_cast<uint8_t>(((lsbLast >> (bit - 1)) & 1) |
                                               (((msbLast >> (bit - 1)) & 1) << 1));
      SPI.transfer(static_cast<uint8_t>((kGrayLUT[p0] & 0xF0u) | (kGrayLUT[p1] >> 4)));
    }
  }

  digitalWrite(_cs, HIGH);
  SPI.endTransaction();
}

// ---------------------------------------------------------------------------
// Trigger a display area refresh (DISP_AREA command).
// ---------------------------------------------------------------------------
void EInkDisplay::triggerDisplayArea(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                                      uint16_t waveformMode) const {
  writeCommand(IT8951_TCON_DISP_AREA);
  writeData(x);
  writeData(y);
  writeData(w);
  writeData(h);
  writeData(waveformMode);

  // Wait for the refresh to complete (IT8951 holds HRDY low while applying
  // the waveform; GC16 can take up to ~2 seconds).
  waitBusy();
}

// ---------------------------------------------------------------------------
// Public display operations
// ---------------------------------------------------------------------------

void EInkDisplay::clearScreen(uint8_t color) const {
  memset(frameBuffer, color, BUFFER_SIZE);
}

void EInkDisplay::drawImage(const uint8_t* imageData, uint16_t x, uint16_t y, uint16_t w,
                             uint16_t h, bool fromProgmem) const {
  if (!imageData || w == 0 || h == 0) return;

  const uint16_t srcWidthBytes = (w + 7) / 8;
  for (uint16_t row = 0; row < h; row++) {
    const uint16_t destRow = y + row;
    if (destRow >= DISPLAY_HEIGHT) break;

    const uint8_t* src = imageData + static_cast<uint32_t>(row) * srcWidthBytes;
    uint8_t* dest = frameBuffer + static_cast<uint32_t>(destRow) * DISPLAY_WIDTH_BYTES + x / 8;

    if (fromProgmem) {
      for (uint16_t col = 0; col < srcWidthBytes; col++) {
        dest[col] = pgm_read_byte(&src[col]);
      }
    } else {
      memcpy(dest, src, srcWidthBytes);
    }
  }
}

void EInkDisplay::drawImageTransparent(const uint8_t* imageData, uint16_t x, uint16_t y,
                                        uint16_t w, uint16_t h, bool fromProgmem) const {
  if (!imageData || w == 0 || h == 0) return;

  const uint16_t srcWidthBytes = (w + 7) / 8;
  for (uint16_t row = 0; row < h; row++) {
    const uint16_t destRow = y + row;
    if (destRow >= DISPLAY_HEIGHT) break;

    const uint8_t* src  = imageData + static_cast<uint32_t>(row) * srcWidthBytes;
    uint8_t* dest = frameBuffer + static_cast<uint32_t>(destRow) * DISPLAY_WIDTH_BYTES + x / 8;

    for (uint16_t col = 0; col < srcWidthBytes; col++) {
      const uint8_t byte = fromProgmem ? pgm_read_byte(&src[col]) : src[col];
      dest[col] &= byte;  // AND to make black pixels transparent
    }
  }
}

void EInkDisplay::displayBuffer(RefreshMode mode, bool /*turnOffScreen*/) {
  setImageBufferAddress(imgBufAddr);
  sendPixels1bpp(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT);
  writeCommand(IT8951_TCON_LD_IMG_END);

  uint16_t wfm = IT8951_WFM_GC16;
  if (mode == FAST_REFRESH) wfm = IT8951_WFM_DU;
  else if (mode == HALF_REFRESH) wfm = IT8951_WFM_GL16;

  triggerDisplayArea(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, wfm);
}

void EInkDisplay::refreshDisplay(RefreshMode mode, bool turnOffScreen) {
  displayBuffer(mode, turnOffScreen);
}

// ---------------------------------------------------------------------------
// Grayscale support
// ---------------------------------------------------------------------------

void EInkDisplay::copyGrayscaleLsbBuffers(const uint8_t* lsbBuffer) {
  if (!lsbBuffer) {
    free(grayLsbBuf);
    grayLsbBuf = nullptr;
    return;
  }
  if (!grayLsbBuf) {
    grayLsbBuf = static_cast<uint8_t*>(malloc(BUFFER_SIZE));
    if (!grayLsbBuf) {
      if (Serial) Serial.printf("[EInk] grayLsb malloc failed\n");
      return;
    }
  }
  memcpy(grayLsbBuf, lsbBuffer, BUFFER_SIZE);
}

void EInkDisplay::copyGrayscaleMsbBuffers(const uint8_t* msbBuffer) {
  if (!msbBuffer) {
    free(grayMsbBuf);
    grayMsbBuf = nullptr;
    return;
  }
  if (!grayMsbBuf) {
    grayMsbBuf = static_cast<uint8_t*>(malloc(BUFFER_SIZE));
    if (!grayMsbBuf) {
      if (Serial) Serial.printf("[EInk] grayMsb malloc failed\n");
      return;
    }
  }
  memcpy(grayMsbBuf, msbBuffer, BUFFER_SIZE);
}

void EInkDisplay::copyGrayscaleBuffers(const uint8_t* lsbBuffer, const uint8_t* msbBuffer) {
  copyGrayscaleLsbBuffers(lsbBuffer);
  copyGrayscaleMsbBuffers(msbBuffer);
}

#ifdef EINK_DISPLAY_SINGLE_BUFFER_MODE
void EInkDisplay::cleanupGrayscaleBuffers(const uint8_t* /*bwBuffer*/) {
  free(grayLsbBuf);
  grayLsbBuf = nullptr;
  free(grayMsbBuf);
  grayMsbBuf = nullptr;
}
#endif

void EInkDisplay::displayGrayBuffer(bool /*turnOffScreen*/) {
  if (!grayLsbBuf || !grayMsbBuf) {
    // Fallback: plain BW refresh if gray planes are unavailable.
    displayBuffer(FULL_REFRESH, false);
    return;
  }

  setImageBufferAddress(imgBufAddr);
  sendGrayPixels(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, grayLsbBuf, grayMsbBuf);
  writeCommand(IT8951_TCON_LD_IMG_END);
  triggerDisplayArea(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, IT8951_WFM_GC16);

  // Free the temporary gray planes.
  free(grayLsbBuf);
  grayLsbBuf = nullptr;
  free(grayMsbBuf);
  grayMsbBuf = nullptr;
}

// ---------------------------------------------------------------------------
// Power management
// ---------------------------------------------------------------------------

void EInkDisplay::deepSleep() {
  writeCommand(IT8951_TCON_SLEEP);
}
