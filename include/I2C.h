#pragma once

#include <Arduino.h>

namespace I2C
{
  uint8_t ReadRegister(const uint8_t, const uint8_t);
  uint8_t WriteRegister(const uint8_t, const uint8_t, const uint8_t);

}