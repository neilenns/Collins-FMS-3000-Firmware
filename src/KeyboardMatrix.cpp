#include <Arduino.h>
#include <Wire.h>
#include <MCP23017.h>

#include "KeyboardMatrix.h"

// Number of milliseconds a key must be presed to be considered
// a press and hold event.
constexpr unsigned long PRESS_AND_HOLD_LENGTH = 500;

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

KeyboardMatrix::KeyboardMatrix(uint8_t rowAddress, uint8_t columnAddress, uint8_t interruptPin, KeyboardEvent interruptHandler, ButtonEvent buttonHandler)
{
  _rows = new MCP23017(rowAddress);
  _columns = new MCP23017(columnAddress);
  _interruptPin = interruptPin;
  _interruptHandler = interruptHandler;
  _buttonHandler = buttonHandler;
}

/**
 * @brief Get the bit position for a single low bit in a byte.
 * 
 * @param value The byte to check, with the active bit set to low and inactive bits set high
 * @return int The position of the single low bit in the byte
 */
int KeyboardMatrix::GetBitPosition(uint16_t value)
{
  // The entire key detection logic uses low for active so invert the bits
  // to ensure this magic works properly.
  value = ~value;
  // value is a power of two, returned values are {0, 1, ..., 15}
  // This is effectively calculating log2(n) since it's guaranteed there will only ever
  // be one bit set in the value. It's a variation of the method shown
  // at http://graphics.stanford.edu/~seander/bithacks.html#IntegerLog.
  return (((value & 0xAAAAAAAA) != 0) |
          (((value & 0xCCCCCCCC) != 0) << 1) |
          (((value & 0xF0F0F0F0) != 0) << 2) |
          (((value & 0xFF00FF00) != 0) << 3));
}

/**
 * @brief Interrupt handler for when the row changed interrupt fires.
 * 
 */
void KeyboardMatrix::HandleInterrupt()
{
  if (_currentState == WaitingForPress)
  {
    _currentState = DetectionState::PressDetected;
  }
}

void KeyboardMatrix::EnableRowInterrupts()
{
  // Turn on interrupts
  _rows->writeRegister(MCP23017Register::GPINTEN_A, 0xFF, 0xFF); // Turn on the interrupts for the rows
}

void KeyboardMatrix::DisableRowInterrupts()
{
  _rows->writeRegister(MCP23017Register::GPINTEN_A, 0x00, 0x00); // Disable row interrupts
}

/**
 * @brief Initializes the MCP23017 to detect interrupts on row changes.
 * 
 * @param setPullups True if the pullup resistors should get configured. This only needs to happen once when the chip
 * is first initialized at board startup.
 */
void KeyboardMatrix::InitForRowDetection(bool setPullups)
{
  _columns->writeRegister(MCP23017Register::IODIR_A, 0x00, 0x00); // Columns as output
  _columns->writeRegister(MCP23017Register::GPIO_A, 0x00, 0x00);  // Reset columns to 0s
  _rows->writeRegister(MCP23017Register::IODIR_A, 0xFF, 0xFF);    // Rows as input
  _rows->writeRegister(MCP23017Register::GPIO_A, 0xFF, 0xFF);     // Reset rows to 1s

  if (setPullups)
  {
    _columns->writeRegister(MCP23017Register::GPPU_A, 0xFF, 0xFF); // Columns have pull-up resistors on
    _rows->writeRegister(MCP23017Register::GPPU_A, 0xFF, 0xFF);    // Rows have pull-up resistors off for all connected lines, but on for the four unused rows
  }

  _columns->writeRegister(MCP23017Register::INTCON_A, 0x00, 0x00); // Turn interrupts off for columns
  _columns->writeRegister(MCP23017Register::DEFVAL_A, 0x00, 0x00); // Default value of 0 for columns
  _rows->writeRegister(MCP23017Register::INTCON_A, 0xFF, 0xFF);    // Turn interrupts on for rows
  _rows->writeRegister(MCP23017Register::DEFVAL_A, 0xFF, 0xFF);    // Default value of 1 for rows
}

/**
 * @brief initializes the keyboard matrix.
 * 
 */
void KeyboardMatrix::Init()
{
  _columns->init();
  _rows->init();

  InitForRowDetection(true);

  // Attach the Arduino interrupt handler.
  pinMode(_interruptPin, INPUT_PULLUP);
  attachInterrupt(
      digitalPinToInterrupt(_interruptPin), _interruptHandler,
      CHANGE);

  // The order of interrupt startup matters a lot. After the row is initialized for
  // interrupt detection then the state machine is set to WaitingForPress. This ensures
  // in the rare case that an interrupt fires immediately after initialization
  // that the state machine won't miss it.
  _rows->interruptMode(MCP23017InterruptMode::Or); // Interrupt on one line
  EnableRowInterrupts();
  _rows->clearInterrupts(); // Clear all interrupts which could come from initialization
  _currentState = DetectionState::WaitingForPress;
}

/**
 * @brief Determines which button is currently pressed when a row changed interrupt fires.
 * 
 */
void KeyboardMatrix::CheckForButton()
{
  uint8_t rowPortA, rowPortB;
  uint16_t rowStates;
  uint8_t columnPortA, columnPortB;
  uint16_t columnStates;

  // Read the current state of all 16 column pins. The board is wired with the low row numbers
  // connected to port B and the high row numbers connected to port A so read the two
  // ports separately then combine them in the right order in a 16-bit number.
  rowPortA = _rows->readPort(MCP23017Port::A);
  rowPortB = _rows->readPort(MCP23017Port::B);
  rowStates = (rowPortA << 8) | rowPortB;

  // Once the row is known reconfigure a bunch of registers to read the active column
  _columns->writeRegister(MCP23017Register::IODIR_A, 0xFF, 0xFF);  // Switch columns to input
  _rows->writeRegister(MCP23017Register::IODIR_A, 0x00, 0x00);     // Switch rows to output
  _columns->writeRegister(MCP23017Register::INTCON_A, 0xFF, 0xFF); // Turn interrupts on for columns
  _rows->writeRegister(MCP23017Register::INTCON_A, 0x00, 0x00);    // Turn interrupts off for rows
  _columns->writeRegister(MCP23017Register::DEFVAL_A, 0xFF, 0xFF); // Default value of 1 for columns
  _rows->writeRegister(MCP23017Register::DEFVAL_A, 0x00, 0x00);    // Default value of 0 for rows

  // Disable row interrupts until the key is released. Without this incorrect interrupts
  // will get fired while attempting to read the column to find the pressed button,
  // and interrupts can fire while waiting for a key release which leads to unhandled
  // interrupts blocking key detection forever.
  DisableRowInterrupts();

  // Write 0s to the rows then read the columns to find out what button is pressed.
  // This step is missing from the application note.
  _rows->write(0x0000);

  // Read the current state of all 16 column pins. The board is wired with the low column
  // numbers connected to port B and the high column numbers connected to port A so read the two
  // ports separately then combine them in the right order in a 16-bit number.
  columnPortA = _columns->readPort(MCP23017Port::A);
  columnPortB = _columns->readPort(MCP23017Port::B);
  columnStates = (columnPortA << 8) | columnPortB;

  _activeRow = KeyboardMatrix::GetBitPosition(rowStates);
  _activeColumn = KeyboardMatrix::GetBitPosition(columnStates);

#ifdef DEBUG
  Serial.print("Detected press at row: ");
  Serial.print(_activeRow);
  Serial.print(" column: ");
  Serial.println(_activeColumn);
#endif

  // The CLR/DEL key is in row 8 column 11 and is special. To support press-and-hold and
  // match actual aircraft behaviour it should only send release events. This check ensures
  // the press event only fires for all other keys.
  if ((_activeRow != 8) || (_activeColumn != 11))
  {
    _buttonHandler(ButtonState::Pressed, _activeRow, _activeColumn);
  }

  // Save when the press event happened so a test can be done on release
  // to look for a press-and-hold on the CLR/DEL key.
  _lastPressEventTime = millis();

  // Flip all the registers back to the default configuration to look for
  // when the row clears.
  _currentState = WaitingForRelease;
  InitForRowDetection(false);
}

/**
 * @brief Determines when a button was released by watching for the row port to reset
 * to all 1s.
 * 
 */
void KeyboardMatrix::CheckForRelease()
{
  uint16_t rowState;

  // Read the row state to see if the button was released. This has the side effect
  // of clearing the interrupt if the triggering pin reset as well.
  rowState = _rows->read();

  // If all the inputs for the row are back to 1s then the button was released
  if (rowState == 0xFFFF)
  {
#ifdef DEBUG
    Serial.print("Detected release at row: ");
    Serial.print(_activeRow);
    Serial.print(" column: ");
    Serial.println(_activeColumn);
#endif

    // The CLR/DEL key in row 8 column 11 is special and send a different row/column position
    // if the button was held down for press-and-hold behaviour. Row 0 column 1
    // is an unused position in the key matrix that acts as the position of
    // the DEL key when CLR/DEL is press and held.
    if ((_activeRow == 8) && (_activeColumn == 11) && ((millis() - _lastPressEventTime) > PRESS_AND_HOLD_LENGTH))
    {
      _activeRow = 0;
      _activeColumn = 1;
    }

    _buttonHandler(ButtonState::Released, _activeRow, _activeColumn);

    // Issue 7
    // The order of these two lines is very important. Interrupts get enabled
    // after the state machine is reset to waiting for an interrupt. Otherwise
    // a race condition can (and did!) occur where an interrupt fires and then
    // the state machine resets back to waiting for an interrupt, resulting
    // in the interrupt never getting handled and all further key detection
    // being blocked.
    _currentState = WaitingForPress;
    EnableRowInterrupts();
  }
}

void KeyboardMatrix::Loop()
{
  // Fininte state machine for button detection
  switch (_currentState)
  {
  case WaitingForPress:
    // Nothing to do here, interrupts will handle it
    break;
  case PressDetected:
    CheckForButton();
    break;
  case WaitingForRelease:
    CheckForRelease();
    break;
  }
}
