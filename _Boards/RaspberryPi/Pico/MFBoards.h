#pragma once

#include <Arduino.h>

// Pin mappings.
static constexpr uint8_t KEY_INT_PIN = D2;  // Row interrupts pin.
static constexpr uint8_t LED_SDB_PIN = D10;  // Pin connected to SDB on the LED driver.
static constexpr uint8_t LED_INTB_PIN = D11; // Pin connected to to INTB on the LED driver.

static constexpr uint8_t I2C_SDA_PIN = D0;
static constexpr uint8_t I2C_SCL_PIN = D1;

static constexpr int EEPROM_SIZE = 4096;         // EEPROMSizeRaspberryPico
static constexpr int MEMLEN_CONFIG = 1496;       // MUST be less than EEPROM_SIZE!! MEM_OFFSET_CONFIG + MEM_LEN_CONFIG <= EEPROM_SIZE, see: eeprom_write_block (MEM_OFFSET_CONFIG, configBuffer, MEM_LEN_CONFIG);
static constexpr int MEMLEN_NAMES_BUFFER = 1000; // max. size for configBuffer, contains only names from inputs
static constexpr int MF_MAX_DEVICEMEM = 2000;    // max. memory size for devices
