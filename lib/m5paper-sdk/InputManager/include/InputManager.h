#pragma once

#include <Arduino.h>

// InputManager for M5Paper hardware.
//
// M5Paper physical buttons (all active LOW with internal pull-up):
//   GPIO39 – Left  button
//   GPIO38 – Power/BTN (middle)
//   GPIO37 – Right button
//
// Logical button mapping: the Left button triggers Back/Left/Up simultaneously,
// the Right button triggers Confirm/Right/Down simultaneously, allowing basic
// UI navigation and EPUB page turning with just three physical buttons.
// This provides the same public interface as the open-x4-sdk InputManager.
class InputManager {
 public:
  InputManager();
  void begin();

  // Not used by M5Paper implementation; provided for API compatibility.
  uint8_t getState() { return currentState; }

  void update();

  bool isPressed(uint8_t buttonIndex) const;
  bool wasPressed(uint8_t buttonIndex) const;
  bool wasAnyPressed() const;
  bool wasReleased(uint8_t buttonIndex) const;
  bool wasAnyReleased() const;
  unsigned long getHeldTime() const;

  // Button indices – same values as the open-x4-sdk InputManager.
  static constexpr uint8_t BTN_BACK    = 0;
  static constexpr uint8_t BTN_CONFIRM = 1;
  static constexpr uint8_t BTN_LEFT    = 2;
  static constexpr uint8_t BTN_RIGHT   = 3;
  static constexpr uint8_t BTN_UP      = 4;
  static constexpr uint8_t BTN_DOWN    = 5;
  static constexpr uint8_t BTN_POWER   = 6;

  // Physical GPIO pins (needed by HalPowerManager for deep-sleep wakeup).
  static constexpr int POWER_BUTTON_PIN = 38;

  bool isPowerButtonPressed() const;

  static const char* getButtonName(uint8_t buttonIndex);

 private:
  // Reads the raw state of all three physical buttons into a 7-bit mask.
  // Bits set correspond to the BTN_* constants above.
  uint8_t readPhysical() const;

  uint8_t currentState  = 0;  // active bitmask this tick
  uint8_t lastState     = 0;  // active bitmask previous tick
  uint8_t pressedEvents = 0;  // bits that went 0→1 this tick
  uint8_t releasedEvents= 0;  // bits that went 1→0 this tick

  unsigned long buttonPressStart  = 0;
  unsigned long buttonPressFinish = 0;

  static const char* BUTTON_NAMES[];
};
