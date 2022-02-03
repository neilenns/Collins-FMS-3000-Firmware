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
  ABMNotStarted,    //< Before ABM starts running.
  ABMRunning,       //< While ABM is running.
  ABMComplete,      //< After the ABM complete interrupt fires.
  LEDOn,            //< ABM is complete and LEDs are on.
  LEDOff,           //< ABM is complete and LEDs are off.
  TurnOnPowerSave,  //< Before power save is turned on.
  TurnOffPowerSave, //< Before power save is turned off.
};

class LEDMatrix
{
private:
  IS31FL3733::IS31FL3733Driver *_driver;
  LEDEvent _eventHandler;
  uint8_t _intbPin;
  volatile LedState _ledState = LedState::ABMNotStarted;
  uint8_t _sdbPin;

public:
  LEDMatrix(ADDR addr1, ADDR addr2, uint8_t sdbPin, uint8_t intbPin, LEDEvent eventHandler);

  void HandleInterrupt();
  void Init();
  void Loop();
  void SetBrightness(uint8_t brightness);
  void SetPowerSaveMode(bool state);
};