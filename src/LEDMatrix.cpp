
#include <Arduino.h>
#include <Wire.h>

#include "LEDMatrix.h"

using namespace IS31FL3733;

// Arduino pin for the SDB line which is set high to enable the IS31FL3733 chip.
const uint8_t SDB_PIN = 4;
// Arduino pin for the IS13FL3733 interrupt pin.
const uint8_t INTB_PIN = 7;

// Function prototypes for the read and write functions defined later in the file.
uint8_t i2c_read_reg(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *buffer, uint8_t length);
uint8_t i2c_write_reg(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *buffer, uint8_t count);

/**
 * @brief Read a buffer of data from the specified register.
 * 
 * @param i2c_addr I2C address of the device to read the data from.
 * @param reg_addr Address of the register to read from.
 * @param buffer Buffer to read the data into.
 * @param length Length of the buffer.
 * @return uint8_t 
 */
uint8_t i2c_read_reg(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *buffer, uint8_t length)
{
  Wire.beginTransmission(i2c_addr);
  Wire.write(reg_addr);
  Wire.endTransmission();
  byte bytesRead = Wire.requestFrom(i2c_addr, length);
  for (int i = 0; i < bytesRead && i < length; i++)
  {
    buffer[i] = Wire.read();
  }
  return bytesRead;
}

/**
 * @brief Writes a buffer to the specified register.
 * 
 * @param i2c_addr I2C address of the device to write the data to.
 * @param reg_addr Address of the register to write to.
 * @param buffer Pointer to an array of bytes to write.
 * @param count Number of bytes in the buffer.
 * @return uint8_t The number of bytes written.
 */
uint8_t i2c_write_reg(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *buffer, uint8_t count)
{
  Wire.beginTransmission(i2c_addr);
  Wire.write(reg_addr);
  byte bytesWritten = Wire.write(buffer, count);
  Wire.endTransmission();

  return bytesWritten;
}

/**
 * @brief Construct a new LEDMatrix::LEDMatrix object
 * 
 * @param addr1 Connection on addr1 pin.
 * @param addr2 Connection on addr2 pin.
 * @param sdbPin Adruino pin connected to SDB.
 * @param intbPin Arduino pin connected to INTB.
 */
LEDMatrix::LEDMatrix(ADDR addr1, ADDR addr2, uint8_t sdbPin, uint8_t intbPin, LEDEvent eventHandler)
{
  driver = new IS31FL3733Driver(addr1, addr2, &i2c_read_reg, &i2c_write_reg);
  _eventHandler = eventHandler;
}

void LEDMatrix::HandleInterrupt()
{
  ledState = LedState::ABMComplete;
}

/**
 * @brief Arduino initialization.
 * 
 */
void LEDMatrix::Init()
{
  // Enable the IS31FL3733 chip by setting the SDB pin high.
  pinMode(SDB_PIN, OUTPUT);
  digitalWrite(SDB_PIN, HIGH);

  // Register for interrupts when ABM completes.
  pinMode(INTB_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTB_PIN), _eventHandler, CHANGE);

  Serial.println("Initializing LED driver");
  driver->Init();

  Serial.println("Setting global current control to half");
  driver->SetGCC(127);

  Serial.println("Setting PWM state for all LEDs to full power");
  driver->SetLEDPWM(CS_LINES, SW_LINES, 255);

  Serial.println("Turning on all LEDs");
  driver->SetLEDState(CS_LINES, SW_LINES, LED_STATE::ON);

  Serial.println("Configure all LEDs for ABM1");
  driver->SetLEDMode(CS_LINES, SW_LINES, LED_MODE::ABM1);

  ABM_CONFIG ABM1;

  ABM1.T1 = ABM_T1::T1_840MS;
  ABM1.T2 = ABM_T2::T2_840MS;
  ABM1.T3 = ABM_T3::T3_840MS;
  ABM1.T4 = ABM_T4::T4_840MS;
  ABM1.Tbegin = ABM_LOOP_BEGIN::LOOP_BEGIN_T4;
  ABM1.Tend = ABM_LOOP_END::LOOP_END_T3;
  ABM1.Times = 2;

  // Write ABM structure parameters to device registers.
  driver->ConfigABM(ABM_NUM::NUM_1, &ABM1);

  // Enable interrupts when ABM completes and auto-clear them after 8ms
  driver->WriteCommonReg(COMMONREGISTER::IMR, IMR_IAB);

  // Start ABM mode operation.
  driver->StartABM();
  ledState = LedState::ABMRunning;
}

void LEDMatrix::Loop()
{
  // Simple finite state machine to switch LEDs on after ABM finishes running.
  switch (ledState)
  {
  case LedState::ABMNotStarted:
  {
    Serial.println("ABM not started");
    delay(500);
    break;
  }
  case LedState::ABMRunning:
  {
    Serial.println("ABM running");
    delay(500);
    break;
  }
  case LedState::ABMComplete:
  {
    // Read the interrupt status to force the interrupt to clear.
    uint8_t interruptStatus = driver->ReadCommonReg(COMMONREGISTER::ISR);

    // Check and see if ABM1 is the ABM that finished.
    if (interruptStatus & ISR_ABM1)
    {
      Serial.println("ABM1 completed");
      Serial.println("Configure all LEDs for full on");
      driver->SetLEDMode(CS_LINES, SW_LINES, LED_MODE::PWM);

      ledState = LedState::LEDOn;
    }
    break;
  }
  case LedState::LEDOn:
  {
    Serial.println("LEDs on");
    delay(500);
    break;
  }
  }
}