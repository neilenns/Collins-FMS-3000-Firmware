#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCA8418.h>

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

  Adafruit_TCA8418 *_keyMatrix;

  void ProcessKeys();
  void ProcessClrDel(ButtonState);

public:
  KeyboardMatrix(uint8_t interruptPin, KeyboardEvent interruptHandler, ButtonEvent buttonHandler);
  void Init();
  void Loop();
  void HandleInterrupt();
};