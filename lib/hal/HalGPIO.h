#pragma once

#include <Arduino.h>
#include <InputManager.h>

#ifdef TARGET_M5PAPER
// ---- M5Paper hardware (ESP32-D0WDQ6-V3) ----
// IT8951 e-ink controller connected via shared SPI bus (GPIO12/13/14).
#define EPD_SCLK 14   // SPI Clock
#define EPD_MOSI 12   // SPI MOSI
#define EPD_CS   15   // IT8951 Chip Select
#define EPD_DC   -1   // Unused (IT8951 has no DC line)
#define EPD_RST  -1   // Unused (IT8951 reset is handled by power circuit)
#define EPD_BUSY 27   // HRDY – active HIGH when IT8951 is ready

#define SPI_MISO 13   // SPI MISO, shared between IT8951 and SD card

// SD card uses the same SPI bus as the IT8951, CS on GPIO4.
#define SD_SPI_CS 4

// M5Paper ADC pin for battery voltage (via resistor divider).
#define BAT_ADC_PIN 35

// GPIO3 (U0RXD) is pulled HIGH by the CP2104 when USB is connected.
#define UART0_RXD 3

#else
// ---- Xteink X4 / X3 hardware (ESP32-C3) ----
// Display SPI pins (custom pins for XteinkX4, not hardware SPI defaults)
#define EPD_SCLK 8   // SPI Clock
#define EPD_MOSI 10  // SPI MOSI (Master Out Slave In)
#define EPD_CS 21    // Chip Select
#define EPD_DC 4     // Data/Command
#define EPD_RST 5    // Reset
#define EPD_BUSY 6   // Busy

#define SPI_MISO 7  // SPI MISO, shared between SD card and display (Master In Slave Out)

#define BAT_GPIO0 0  // Battery voltage

#define UART0_RXD 20  // Used for USB connection detection
#endif  // TARGET_M5PAPER

// Xteink X3 Hardware
#define X3_I2C_SDA 20
#define X3_I2C_SCL 0
#define X3_I2C_FREQ 400000

// TI BQ27220 Fuel gauge I2C
#define I2C_ADDR_BQ27220 0x55  // Fuel gauge I2C address
#define BQ27220_SOC_REG 0x2C   // StateOfCharge() command code (%)
#define BQ27220_CUR_REG 0x0C   // Current() command code (signed mA)
#define BQ27220_VOLT_REG 0x08  // Voltage() command code (mV)

// Analog DS3231 RTC I2C
#define I2C_ADDR_DS3231 0x68  // RTC I2C address
#define DS3231_SEC_REG 0x00   // Seconds command code (BCD)

// QST QMI8658 IMU I2C
#define I2C_ADDR_QMI8658 0x6B        // IMU I2C address
#define I2C_ADDR_QMI8658_ALT 0x6A    // IMU I2C fallback address
#define QMI8658_WHO_AM_I_REG 0x00    // WHO_AM_I command code
#define QMI8658_WHO_AM_I_VALUE 0x05  // WHO_AM_I expected value

class HalGPIO {
#if CROSSPOINT_EMULATED == 0
  InputManager inputMgr;
#endif

  bool lastUsbConnected = false;
  bool usbStateChanged = false;

 public:
#ifdef TARGET_M5PAPER
  enum class DeviceType : uint8_t { M5Paper };
#else
  enum class DeviceType : uint8_t { X4, X3 };
#endif

 private:
#ifdef TARGET_M5PAPER
  DeviceType _deviceType = DeviceType::M5Paper;
#else
  DeviceType _deviceType = DeviceType::X4;
#endif

 public:
  HalGPIO() = default;

  // Inline device type helpers for cleaner downstream checks
  inline bool deviceIsX3() const {
#ifdef TARGET_M5PAPER
    return false;
#else
    return _deviceType == DeviceType::X3;
#endif
  }
  inline bool deviceIsX4() const {
#ifdef TARGET_M5PAPER
    return false;
#else
    return _deviceType == DeviceType::X4;
#endif
  }
#ifdef TARGET_M5PAPER
  inline bool deviceIsM5Paper() const { return true; }
#endif

  // Start button GPIO and setup SPI for screen and SD card
  void begin();

  // Button input methods
  void update();
  bool isPressed(uint8_t buttonIndex) const;
  bool wasPressed(uint8_t buttonIndex) const;
  bool wasAnyPressed() const;
  bool wasReleased(uint8_t buttonIndex) const;
  bool wasAnyReleased() const;
  unsigned long getHeldTime() const;

  // Setup wake up GPIO and enter deep sleep
  void startDeepSleep();

  // Verify power button was held long enough after wakeup.
  // If verification fails, enters deep sleep and does not return.
  // Should only be called when wakeup reason is PowerButton.
  void verifyPowerButtonWakeup(uint16_t requiredDurationMs, bool shortPressAllowed);

  // Check if USB is connected
  bool isUsbConnected() const;

  // Returns true once per edge (plug or unplug) since the last update()
  bool wasUsbStateChanged() const;

  enum class WakeupReason { PowerButton, AfterFlash, AfterUSBPower, Other };

  WakeupReason getWakeupReason() const;

  // Button indices
  static constexpr uint8_t BTN_BACK = 0;
  static constexpr uint8_t BTN_CONFIRM = 1;
  static constexpr uint8_t BTN_LEFT = 2;
  static constexpr uint8_t BTN_RIGHT = 3;
  static constexpr uint8_t BTN_UP = 4;
  static constexpr uint8_t BTN_DOWN = 5;
  static constexpr uint8_t BTN_POWER = 6;
};

extern HalGPIO gpio;
