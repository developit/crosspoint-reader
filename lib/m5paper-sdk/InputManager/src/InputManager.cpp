#include "InputManager.h"

// M5Paper physical GPIO pin assignments.
static constexpr int M5PAPER_BTN_LEFT  = 39;  // Left  button (active LOW)
static constexpr int M5PAPER_BTN_PWR   = 38;  // Power button (active LOW)
static constexpr int M5PAPER_BTN_RIGHT = 37;  // Right button (active LOW)

// Debounce interval in milliseconds.
static constexpr unsigned long DEBOUNCE_MS = 20;

const char* InputManager::BUTTON_NAMES[] = {
    "Back", "Confirm", "Left", "Right", "Up", "Down", "Power"};

InputManager::InputManager() = default;

void InputManager::begin() {
  // All three buttons are active-LOW; use the internal pull-up.
  pinMode(M5PAPER_BTN_LEFT,  INPUT_PULLUP);
  pinMode(M5PAPER_BTN_PWR,   INPUT_PULLUP);
  pinMode(M5PAPER_BTN_RIGHT, INPUT_PULLUP);
}

// ---------------------------------------------------------------------------
// readPhysical()
//
// Map 3 physical buttons to 7 logical BTN_* bits so that the rest of the
// firmware (which expects up to 6 navigation buttons + power) functions with
// minimal changes.
//
// Mapping chosen so that:
//   • EPUB page navigation (BTN_UP / BTN_DOWN via MappedInputManager) works
//     with the Left and Right physical buttons respectively.
//   • UI menu navigation (BTN_BACK / BTN_LEFT and BTN_CONFIRM / BTN_RIGHT)
//     also works with the same two physical buttons.
//
// This means pressing Left sets bits for Back + Left + Up simultaneously, and
// pressing Right sets bits for Confirm + Right + Down simultaneously.
// ---------------------------------------------------------------------------
uint8_t InputManager::readPhysical() const {
  uint8_t state = 0;

  if (digitalRead(M5PAPER_BTN_LEFT) == LOW) {
    state |= (1u << BTN_BACK);
    state |= (1u << BTN_LEFT);
    state |= (1u << BTN_UP);
  }

  if (digitalRead(M5PAPER_BTN_RIGHT) == LOW) {
    state |= (1u << BTN_CONFIRM);
    state |= (1u << BTN_RIGHT);
    state |= (1u << BTN_DOWN);
  }

  if (digitalRead(M5PAPER_BTN_PWR) == LOW) {
    state |= (1u << BTN_POWER);
  }

  return state;
}

void InputManager::update() {
  // Simple debounce: require two consecutive identical readings.
  const uint8_t raw1 = readPhysical();
  delay(DEBOUNCE_MS);
  const uint8_t raw2 = readPhysical();
  const uint8_t stable = (raw1 == raw2) ? raw1 : currentState;

  lastState      = currentState;
  currentState   = stable;
  pressedEvents  = static_cast<uint8_t>( currentState & ~lastState);
  releasedEvents = static_cast<uint8_t>(~currentState &  lastState);

  const bool anyDown = (currentState != 0);
  const bool wasDown = (lastState    != 0);

  if (anyDown && !wasDown) {
    buttonPressStart  = millis();
    buttonPressFinish = 0;
  } else if (!anyDown && wasDown) {
    buttonPressFinish = millis();
  }
}

bool InputManager::isPressed(uint8_t buttonIndex) const {
  return (currentState >> buttonIndex) & 1u;
}

bool InputManager::wasPressed(uint8_t buttonIndex) const {
  return (pressedEvents >> buttonIndex) & 1u;
}

bool InputManager::wasAnyPressed() const {
  return pressedEvents != 0;
}

bool InputManager::wasReleased(uint8_t buttonIndex) const {
  return (releasedEvents >> buttonIndex) & 1u;
}

bool InputManager::wasAnyReleased() const {
  return releasedEvents != 0;
}

unsigned long InputManager::getHeldTime() const {
  if (currentState != 0) {
    return millis() - buttonPressStart;
  }
  if (buttonPressFinish > buttonPressStart) {
    return buttonPressFinish - buttonPressStart;
  }
  return 0;
}

bool InputManager::isPowerButtonPressed() const {
  return digitalRead(M5PAPER_BTN_PWR) == LOW;
}

const char* InputManager::getButtonName(uint8_t buttonIndex) {
  if (buttonIndex < 7) {
    return BUTTON_NAMES[buttonIndex];
  }
  return "Unknown";
}
