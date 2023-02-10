#pragma once

#include <Arduino.h>

namespace I2C
{
  uint8_t ReadRegister(const uint8_t i2cAddress, const uint8_t registerAddress);
  uint8_t WriteRegister(const uint8_t i2CAddress, const uint8_t registerAddress, const uint8_t value);

}