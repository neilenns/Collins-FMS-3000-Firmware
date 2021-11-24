#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <MCP23017.h>

enum DetectionState
{
  WaitingForPress,
  PressDetected,
  WaitingForRelease
};

enum ButtonState
{
  Pressed,
  Released,
};

extern "C"
{
  typedef void (*KeyboardEvent)();
  typedef void (*ButtonEvent)(ButtonState, const char *);
};

class KeyboardMatrix
{
private:
  uint8_t _interruptPin;
  volatile DetectionState currentState = DetectionState::WaitingForPress;
  uint8_t _activeRow = 0;
  uint8_t _activeColumn = 0;
  KeyboardEvent _interruptHandler;
  ButtonEvent _buttonHandler;

  MCP23017 *_rows;
  MCP23017 *_columns;

  static int GetBitPosition(uint16_t value);
  void InitForRowDetection(bool setPullups);
  void CheckForButton();
  void CheckForRelease();

public:
  KeyboardMatrix(uint8_t rowAddress, uint8_t columnAddress, uint8_t interruptPin, KeyboardEvent interruptHandler, ButtonEvent buttonHandler);
  void Init();
  void Loop();
  void HandleInterrupt();
};