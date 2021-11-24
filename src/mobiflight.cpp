#include <Arduino.h>
#include <MFBoards.h>
#include <Wire.h>

#include "CmdMessenger.h"
#include "KeyboardMatrix.h"
#include "mobiflight.h"
#include "MFEEPROM.h"
#include "PinNames.h"

// The build version comes from an environment variable
#define STRINGIZER(arg) #arg
#define STR_VALUE(arg) STRINGIZER(arg)
#define VERSION STR_VALUE(BUILD_VERSION)

const uint8_t MEM_OFFSET_SERIAL = 0;
const uint8_t MEM_LEN_SERIAL = 11;

const char type[sizeof(MOBIFLIGHT_TYPE)] = MOBIFLIGHT_TYPE;
char serial[MEM_LEN_SERIAL] = MOBIFLIGHT_SERIAL;
char name[sizeof(MOBIFLIGHT_NAME)] = MOBIFLIGHT_NAME;

const uint8_t ROW_I2C_ADDRESS = 0x20;    // I2C address of the MCP23017 IC that reads rows
const uint8_t COLUMN_I2C_ADDRESS = 0x21; // I2C address of the MCP23017 IC that reads columns
const uint8_t INTA_PIN = 2;              // Row interrupts pin

CmdMessenger cmdMessenger = CmdMessenger(Serial);
MFEEPROM MFeeprom;
KeyboardMatrix keyboardMatrix(ROW_I2C_ADDRESS, COLUMN_I2C_ADDRESS, INTA_PIN, OnKeyboardEvent, OnButtonPress);

/**
 * @brief Registers callbacks for all supported MobiFlight commands.
 * 
 */
void attachCommandCallbacks()
{
  // Attach callback methods
  cmdMessenger.attach(OnUnknownCommand);
  cmdMessenger.attach(kSetPin, OnSetPin);
  cmdMessenger.attach(kGetInfo, OnGetInfo);
  cmdMessenger.attach(kGetConfig, OnGetConfig);
  cmdMessenger.attach(kSetConfig, OnSetConfig);
  cmdMessenger.attach(kResetConfig, SendOk);
  cmdMessenger.attach(kSaveConfig, OnSaveConfig);
  cmdMessenger.attach(kActivateConfig, OnActivateConfig);
  cmdMessenger.attach(kSetName, OnSetName);
  cmdMessenger.attach(kGenNewSerial, OnGenNewSerial);
  cmdMessenger.attach(kTrigger, SendOk);
  cmdMessenger.attach(kResetBoard, OnResetBoard);
}

void OnKeyboardEvent()
{
  keyboardMatrix.HandleInterrupt();
}
/**
 * @brief General callback to simply respond OK to the desktop app for unsupported commands.
 * 
 */
void SendOk()
{
  cmdMessenger.sendCmd(kStatus, F("OK"));
}

/**
 * @brief Always reports success to MobiFlight on kSaveConfig.
 * 
 */
void OnSaveConfig()
{
  cmdMessenger.sendCmd(kConfigSaved, F("OK"));
}

/**
 * @brief Always reports success to MobiFlight on kActivateConfig.
 * 
 */
void OnActivateConfig()
{
  cmdMessenger.sendCmd(kConfigActivated, F("OK"));
}

/**
 * @brief Loads or generates a new board serial number. Sends a kConfigActivated
 * message to MobiFlight for compatibility purposes.
 * 
 */
void OnResetBoard()
{
  generateSerial(false);

  // This is required to maintain compatibility with the standard Mobiflight firmware
  // which eventually activates the config when resetting the board.
  OnActivateConfig();
}

/**
 * @brief Loads the board serial number from EEPROM and generates a new one if force is set to true
 * or no serial number was present in EEPROM.
 * 
 * @param force True if a new serial number should be created even if one already exists. 
 */
void generateSerial(bool force)
{
  MFeeprom.read_block(MEM_OFFSET_SERIAL, serial, MEM_LEN_SERIAL);
  if (!force && serial[0] == 'S' && serial[1] == 'N')
    return;
  randomSeed(analogRead(0));
  sprintf(serial, "SN-%03x-", (unsigned int)random(4095));
  sprintf(&serial[7], "%03x", (unsigned int)random(4095));
  MFeeprom.write_block(MEM_OFFSET_SERIAL, serial, MEM_LEN_SERIAL);
}

void OnButtonPress(ButtonState state, const char *name)
{
  cmdMessenger.sendCmdStart(kButtonChange);
  cmdMessenger.sendCmdArg(name);
  cmdMessenger.sendCmdArg(state);
  cmdMessenger.sendCmdEnd();
}

/**
 * @brief Stubbed event handler that always returns 512 remaining bytes for config
 * to the desktop app.
 * 
 */
void OnSetConfig()
{
  // Since this firmware has a fixed config just report back a length to make
  // the desktop app happy.
  cmdMessenger.sendCmd(kStatus, 512);
}

/**
 * @brief Event handler for unknown commands.
 * 
 */
void OnUnknownCommand()
{
  cmdMessenger.sendCmd(kStatus, F("n/a"));
}

void OnGetInfo()
{
  cmdMessenger.sendCmdStart(kInfo);
  cmdMessenger.sendCmdArg(type);
  cmdMessenger.sendCmdArg(name);
  cmdMessenger.sendCmdArg(serial);
  cmdMessenger.sendCmdArg(VERSION);
  cmdMessenger.sendCmdEnd();
}

/**
 * @brief Sends the dynamically generated board configuration to MobiFlight.
 * 
 */
void OnGetConfig()
{
  auto i = 0;
  char singleModule[20] = "";

  cmdMessenger.sendCmdStart(kInfo);
  cmdMessenger.sendFieldSeparator();

  // Send configuration for all 69 buttons.
  for (i = 0; i < 69; i++)
  {
    snprintf(singleModule, 20, "%i.%i.%s:", MFDevice::kTypeButton, i, pinNames[i]);
    cmdMessenger.sendArg(singleModule);
  }

  // Send configuration for a single output that's used to control LED brightness
  snprintf(singleModule, 20, "%i.%i.Brightness:", MFDevice::kTypeOutput, i++);
  cmdMessenger.sendArg(singleModule);

  cmdMessenger.sendCmdEnd();
}

// Callback function that sets led on or off
void OnSetPin()
{
  // Read led state argument, interpret string as boolean
  int pin = cmdMessenger.readInt16Arg();
  int state = cmdMessenger.readInt16Arg();
  // Set led
  analogWrite(pin, state);
}

void OnGenNewSerial()
{
  generateSerial(true);
  cmdMessenger.sendCmdStart(kInfo);
  cmdMessenger.sendCmdArg(serial);
  cmdMessenger.sendCmdEnd();
}

/**
 * @brief Stubbed out method to accept the name argument then discard it. The name
 * is actually hardcoded in the firmware.
 * 
 */
void OnSetName()
{
  cmdMessenger.readStringArg();

  cmdMessenger.sendCmdStart(kStatus);
  cmdMessenger.sendCmdArg(name);
  cmdMessenger.sendCmdEnd();
}

/**
 * @brief Android initialization method.
 * 
 */
void setup()
{
  MFeeprom.init();
  Wire.begin();
  Serial.begin(115200);

  while (!Serial)
    ;

  attachCommandCallbacks();
  cmdMessenger.printLfCr();

  OnResetBoard();
  keyboardMatrix.Init();
  Serial.println("Initializing complete");
}

/**
 * @brief Arduino application loop.
 * 
 */
void loop()
{
  // Process incoming serial data, and perform callbacks
  cmdMessenger.feedinSerialData();
  keyboardMatrix.Loop();
}
