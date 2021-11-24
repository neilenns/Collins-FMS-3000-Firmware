#pragma once
#include "is31fl3733.hpp"

using namespace IS31FL3733;

extern "C"
{
  typedef void (*LEDEvent)();
};

/**
 * @brief Finite state machine states for the LEDs.
 * 
 */
enum LedState
{
  ABMNotStarted, //< Before ABM starts running.
  ABMRunning,    //< While ABM is running.
  ABMComplete,   //< After the ABM complete interrupt fires.
  LEDOn          //< ABM is complete and LEDs are on.
};

class LEDMatrix
{
private:
  volatile LedState ledState = LedState::ABMNotStarted;
  IS31FL3733::IS31FL3733Driver *driver;
  LEDEvent _eventHandler;
  uint8_t _sdbPin;
  uint8_t _intbPin;

public:
  LEDMatrix(ADDR addr1, ADDR addr2, uint8_t sdbPin, uint8_t intbPin, LEDEvent eventHandler);

  void HandleInterrupt();
  void Init();
  void Loop();
  void SetBrightness(uint8_t brightness);
};