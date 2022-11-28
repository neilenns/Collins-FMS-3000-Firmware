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
  typedef void (*ButtonEvent)(ButtonState, uint8_t, uint8_t);
};

class KeyboardMatrix
{
private:
  uint8_t _activeRow = 0;
  uint8_t _activeColumn = 0;
  ButtonEvent _buttonHandler;
  volatile DetectionState _currentState = DetectionState::WaitingForPress;
  unsigned long _lastPressEventTime;

  MCP23017 *_rows;
  MCP23017 *_columns;

  void CheckForButton();
  void CheckForRelease();
  void InitForRowDetection(bool setPullups);
  static int GetBitPosition(uint16_t value);

public:
  KeyboardMatrix(uint8_t rowAddress, uint8_t columnAddress, ButtonEvent buttonHandler);
  void Init();
  void Loop();
  void HandleInterrupt();
};