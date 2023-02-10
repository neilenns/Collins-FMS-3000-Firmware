#include "I2C.h"

#include <Arduino.h>
#include <Wire.h>

namespace I2C
{
  /**
   * @brief Reads a byte from an I2C register.
   *
   * @param i2cAddress Address of the I2C device to read from.
   * @param registerAddress Address of the registery to read from.
   * @return uint8_t The byte read from the device's register.
   */
  uint8_t ReadRegister(const uint8_t i2cAddress, const uint8_t registerAddress)
  {
    Wire.beginTransmission(i2cAddress);
    Wire.write(registerAddress);
    Wire.endTransmission();

    Wire.requestFrom(i2cAddress, sizeof(uint8_t));
    return Wire.read();
  }

  /**
   * @brief Writes a byte to an I2C register.
   *
   * @param i2CAddress Address of the I2C device to write to.
   * @param registerAddress Address of the register to write to.
   * @param value Value to write.
   * @return uint8_t Number of bytes written.
   */
  uint8_t WriteRegister(const uint8_t i2CAddress, const uint8_t registerAddress, const uint8_t value)
  {
    Wire.beginTransmission(i2CAddress);
    Wire.write(registerAddress);
    byte bytesWritten = Wire.write(value);
    Wire.endTransmission();

    return bytesWritten;
  }
}