#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCA8418.h>

#include "KeyboardMatrix.h"

static constexpr uint8_t INT_STAT_K_INT_BIT = 0x01;            // K_INT is bit 0 in the INT_STAT register.
static constexpr uint8_t KEY_STATE_MASK = 0x80;                // Bit 7 in the key event indicates whether the button was pressed or released.
static constexpr uint8_t KEY_ID_MASK = 0x7F;                   // Bits 0-6 in the key event indicate which key was pressed.
static constexpr uint8_t CLR_KEY_ID = 54;                      // Key ID for the special-cased CLR/DEL key.
static constexpr uint8_t DEL_KEY_ID = 10;                      // Key ID for the virtual DEL key (CLR when long press).
static constexpr unsigned long PRESS_AND_HOLD_LENGTH_MS = 500; // Length of time a key must be held for a long press.

KeyboardMatrix::KeyboardMatrix(uint8_t interruptPin, KeyboardEvent interruptHandler, ButtonEvent buttonHandler)
{
  _interruptPin = interruptPin;
  _interruptHandler = interruptHandler;
  _buttonHandler = buttonHandler;
}

/**
 * @brief Interrupt handler for when the row changed interrupt fires.
 *
 */
void KeyboardMatrix::HandleInterrupt()

{
  if (_currentState == WaitingForKey)
  {
    _currentState = DetectionState::KeyDetected;
  }
}

/**
 * @brief Initializes the keyboard matrix.
 *
 */
void KeyboardMatrix::Init()
{
  _keyMatrix = new Adafruit_TCA8418();

  // Set up the matrix with the correct number of rows and columns.
  _keyMatrix->begin(TCA8418_DEFAULT_ADDR, &Wire);
  _keyMatrix->matrix(6, 10);

  // Attach the Arduino interrupt handler.
  pinMode(_interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(_interruptPin), _interruptHandler, CHANGE);

  // Flush any pending interrupts then enable interrupt sending
  _keyMatrix->flush();
  _keyMatrix->enableInterrupts();

  _currentState = DetectionState::WaitingForKey;
}

/**
 * @brief Determines the key and event (pressed or released) when an interrupt fires.
 *
 */
void KeyboardMatrix::ProcessKeys()
{
  int keyEvent;
  int keyId;
  ButtonState keyState;

  // Read the key press waiting in the buffer and get the ID of the key that fired.
  keyEvent = _keyMatrix->getEvent();
  keyId = keyEvent & KEY_ID_MASK;

  // The chip reports 1 for press and 0 for release. Since Arduinos
  // use inverse logic for key states invert the value from the chip
  // and cast it to a ButtonState.
  keyState = (ButtonState)(!(keyEvent & KEY_STATE_MASK));

#ifdef DEBUG
  Serial.print("Detected key: ");
  Serial.print(keyId);
  Serial.print(" State: ");
  Serial.println(keyState);
#endif

  // Try and clear the interrupt. If there are more events in the buffer the interrupt won't clear.
  _keyMatrix->writeRegister(TCA8418_REG_INT_STAT, 1);
  int interruptStatus = _keyMatrix->readRegister(TCA8418_REG_INT_STAT);
  if ((interruptStatus & INT_STAT_K_INT_BIT) == 0)
  {
    _currentState = DetectionState::WaitingForKey;
  }

  // The CLR/DEL key is special. To support press-and-hold and match actual aircraft behaviour it should only send release events
  // This check ensures the press event fires for all other keys.
  if (keyId == CLR_KEY_ID)
  {
    ProcessClrDel(keyState);
  }
  else
  {
    // All other keys send the key event
    _buttonHandler(keyId, keyState);
  }
}

/**
 * @brief Handles sending either CLR or DEL release events depending on how long the
 * key was held down.
 *
 * @param keyState Whether the key is pressed or released.
 */
void KeyboardMatrix::ProcessClrDel(ButtonState keyState)
{
  if (keyState == ButtonState::Pressed)
  {
    _lastPressEventTime = millis();
    return;
  }

  // At this point we know the key was released. Check how long it was held
  // down and send either DEL for long press or CLR for short press.
  if ((millis() - _lastPressEventTime) > PRESS_AND_HOLD_LENGTH_MS)
  {
    _buttonHandler(DEL_KEY_ID, keyState);
  }
  else
  {
    _buttonHandler(CLR_KEY_ID, keyState);
  }
}

void KeyboardMatrix::Loop()
{
  // Fininte state machine for button detection
  switch (_currentState)
  {
  case WaitingForKey:
    // Nothing to do here, interrupts will handle it
    break;
  case KeyDetected:
    ProcessKeys();
    break;
  }
}