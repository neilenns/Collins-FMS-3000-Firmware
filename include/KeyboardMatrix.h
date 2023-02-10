#pragma once

#include <Arduino.h>
#include <Wire.h>

enum DetectionState
{
  WaitingForKey,
  KeyDetected
};

enum ButtonState
{
  Pressed,
  Released,
};

extern "C"
{
  typedef void (*KeyboardEvent)();
  typedef void (*ButtonEvent)(uint8_t, ButtonState);
};

class KeyboardMatrix
{
private:
  ButtonEvent _buttonHandler;
  volatile DetectionState _currentState = DetectionState::WaitingForKey;
  KeyboardEvent _interruptHandler;
  uint8_t _interruptPin;
  unsigned long _lastPressEventTime;

#ifdef DEBUG
  void DumpRegisters();
#endif

  void ProcessClrDel(ButtonState);
  void ProcessDim(ButtonState);
  void ProcessKeys();
  void ReadKeyEvent(int keyEvent);

public:
  KeyboardMatrix(uint8_t interruptPin, KeyboardEvent interruptHandler, ButtonEvent buttonHandler);
  void Init();
  void Loop();
  void HandleInterrupt();
};