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
  typedef void (*ButtonEvent)(const uint8_t, const ButtonState);
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

  void ProcessClrDel(const ButtonState);
  void ProcessDim(const ButtonState);
  void ProcessKeys();
  void ReadKeyEvent(int keyEvent);

public:
  KeyboardMatrix(const uint8_t, const KeyboardEvent, const ButtonEvent);
  void Init();
  void Loop();
  void HandleInterrupt();
};