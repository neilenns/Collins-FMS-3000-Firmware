#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCA8418.h>

#include "KeyboardMatrix.h"

static constexpr uint8_t INT_STAT_GPI_INT_BIT = 0x02;          // GPI_INT_BIT is bit 1 in the INT_STAT register.
static constexpr uint8_t INT_STAT_K_INT_BIT = 0x01;            // K_INT is bit 0 in the INT_STAT register.
static constexpr uint8_t KEY_STATE_MASK = 0x80;                // Bit 7 in the key event indicates whether the button was pressed or released.
static constexpr uint8_t KEY_ID_MASK = 0x7F;                   // Bits 0-6 in the key event indicate which key was pressed.
static constexpr uint8_t CLR_KEY_ID = 54;                      // Key ID for the special-cased CLR/DEL key.
static constexpr uint8_t DEL_KEY_ID = 10;                      // Key ID for the virtual DEL key (CLR when long press).
static constexpr uint8_t DIM_KEY_ID = 53;                      // Key ID for the special-cased DIM key.
static constexpr unsigned long PRESS_AND_HOLD_LENGTH_MS = 500; // Length of time a key must be held for a long press.

#ifdef DEBUG
// Helper function to write a 16 bit value out as bits for debugging purposes.
void write16AsBits(uint16_t value)
{
  for (int i = 0; i < 8; i++)
  {
    bool b = value & 0x8000;
    Serial.print(b);
    value = value << 1;
  }
  Serial.print(" ");
  for (int i = 0; i < 8; i++)
  {
    bool b = value & 0x8000;
    Serial.print(b);
    value = value << 1;
  }
}
// Helper function to write an 8 bit value out as bits for debugging purposes.
void write8AsBits(uint8_t value)
{
  for (int i = 0; i < 8; i++)
  {
    bool b = value & 0x80;
    Serial.print(b);
    value = value << 1;
  }
}
#endif

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
  _keyMatrix->matrix(7, 10);

  // Attach the Arduino interrupt handler.
  pinMode(_interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(_interruptPin), _interruptHandler, CHANGE);

  // Flush any pending interrupts then enable interrupt sending
  _keyMatrix->flush();
  _keyMatrix->enableInterrupts();

  _currentState = DetectionState::WaitingForKey;
}

/**
 * @brief Reads a single key event from the event queue and processes it as a MobiFlight
 * key press.
 *
 */
void KeyboardMatrix::ReadKeyEvent()
{
  // Read the key press waiting in the buffer and get the ID of the key that fired.
  int keyEvent = _keyMatrix->getEvent();
  int keyId = keyEvent & KEY_ID_MASK;

  // The chip reports 1 for press and 0 for release. Since Arduinos
  // use inverse logic for key states invert the value from the chip
  // and cast it to a ButtonState.
  ButtonState keyState = (ButtonState)(!(keyEvent & KEY_STATE_MASK));

#ifdef DEBUG
  Serial.print("Key event bits: ");
  write8AsBits(keyEvent);
  Serial.println();
  Serial.print("Detected key: ");
  Serial.print(keyId);
  Serial.print(" State: ");
  Serial.println(keyState);
#endif

  // Issue 31: For some reason the A key also sends a keyId 98 event.
  if (keyId == 98)
  {
    return;
  }

  // The CLR/DEL key is special. To support press-and-hold and match actual aircraft behaviour it should only send release events
  // This check ensures the press event fires for all other keys.
  if (keyId == CLR_KEY_ID)
  {
    ProcessClrDel(keyState);
  }
  else if (keyId == DIM_KEY_ID)
  {
    ProcessDim(keyState);
  }
  else
  {
    // All other keys send the key event
    _buttonHandler(keyId, keyState);
  }
}

/**
 * @brief Handles an interrupt from the TCA8418 and processes all keys in the event queue.
 *
 */
void KeyboardMatrix::ProcessKeys()
{
  // This flow comes from the TCA8418 datasheet, section 8.3.1.3: Key Event (FIFO) Reading.

  // Step 1: Find out what caused the interrupt. Anything other than K_INT gets ignored.
  int interruptStatus = _keyMatrix->readRegister(TCA8418_REG_INT_STAT);
  if ((interruptStatus && INT_STAT_K_INT_BIT) != INT_STAT_K_INT_BIT)
  {
    _currentState = DetectionState::WaitingForKey;
    return;
  }

  // Step 2: Read KEY_LCK_EC register to find out how many events are stored in the queue.
  int pendingKeyCount = _keyMatrix->readRegister(TCA8418_REG_KEY_LCK_EC);

  // Step 3: Read the pending keys in the FIFO queue.
  while (pendingKeyCount != 0)
  {
    ReadKeyEvent();

    // Step 4: Check the FIFO queue length again.
    pendingKeyCount = _keyMatrix->readRegister(TCA8418_REG_KEY_LCK_EC);
  }

  // Step 5: Reset the interrupt flag.
  _keyMatrix->writeRegister(TCA8418_REG_INT_STAT, INT_STAT_K_INT_BIT);

  // Set the state machine back to waiting for key.
  _currentState = DetectionState::WaitingForKey;
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

/**
 * @brief Handles the special case DIM key being used to reset the board. A long press
 * will trigger a board reboot.
 *
 * @param keyState Whether the key is pressed or released.
 */
void KeyboardMatrix::ProcessDim(ButtonState keyState)
{
  if (keyState == ButtonState::Pressed)
  {
    _lastPressEventTime = millis();
    _buttonHandler(DIM_KEY_ID, keyState); // send the press event
    return;
  }

  // At this point we know the key was released. If it was a long press
  // reboot the board. Otherwise send a release event
  if ((millis() - _lastPressEventTime) > PRESS_AND_HOLD_LENGTH_MS)
  {
    rp2040.reboot();
  }
  else
  {
    _buttonHandler(DIM_KEY_ID, keyState);
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