#pragma once

#define EEPROM_SIZE 4096         // EEPROMSizeRaspberryPico
#define MEMLEN_CONFIG 1496       // MUST be less than EEPROM_SIZE!! MEM_OFFSET_CONFIG + MEM_LEN_CONFIG <= EEPROM_SIZE, see: eeprom_write_block (MEM_OFFSET_CONFIG, configBuffer, MEM_LEN_CONFIG);
#define MEMLEN_NAMES_BUFFER 1000 // max. size for configBuffer, contains only names from inputs
#define MF_MAX_DEVICEMEM 2000    // max. memory size for devices
