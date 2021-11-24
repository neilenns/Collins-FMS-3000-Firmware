/**
 * Includes Core Arduino functionality 
 **/
char foo;
#include <Arduino.h>

#include "PinNames.h"

#include "mobiflight.h"
#include <MFBoards.h>

// The build version comes from an environment variable
#define STRINGIZER(arg) #arg
#define STR_VALUE(arg) STRINGIZER(arg)
#define VERSION STR_VALUE(BUILD_VERSION)

//#define DEBUG 1

#include "MFEEPROM.h"
#include "CmdMessenger.h"

const uint8_t MEM_OFFSET_SERIAL = 0;
const uint8_t MEM_LEN_SERIAL = 11;

const char type[sizeof(MOBIFLIGHT_TYPE)] = MOBIFLIGHT_TYPE;
char serial[MEM_LEN_SERIAL] = MOBIFLIGHT_SERIAL;
char name[sizeof(MOBIFLIGHT_NAME)] = MOBIFLIGHT_NAME;

bool powerSavingMode = false;
const unsigned long POWER_SAVING_TIME = 60 * 15; // in seconds

CmdMessenger cmdMessenger = CmdMessenger(Serial);
unsigned long lastCommand;

MFEEPROM MFeeprom;

/**
 * @brief General callback to simply respond OK to the desktop app for unsupported commands.
 * 
 */
void SendOk()
{
  cmdMessenger.sendCmd(kConfigSaved, F("OK"));
}

// Callbacks define on which received commands we take action
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

void OnSaveConfig()
{
  cmdMessenger.sendCmd(kConfigSaved, F("OK"));
}

void OnActivateConfig()
{
  cmdMessenger.sendCmd(kConfigActivated, F("OK"));
}

void OnResetBoard()
{
  generateSerial(false);
  lastCommand = millis();
}

// Setup function
void setup()
{
  Serial.begin(115200);

  attachCommandCallbacks();
  cmdMessenger.printLfCr();

  MFeeprom.init();

  OnResetBoard();
}

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

void SetPowerSavingMode(bool state)
{
  powerSavingMode = state;
}

void updatePowerSaving()
{
  if (!powerSavingMode && ((millis() - lastCommand) > (POWER_SAVING_TIME * 1000)))
  {
    // enable power saving
    SetPowerSavingMode(true);
  }
  else if (powerSavingMode && ((millis() - lastCommand) < (POWER_SAVING_TIME * 1000)))
  {
    // disable power saving
    SetPowerSavingMode(false);
  }
}

// Loop function
void loop()
{
  // Process incoming serial data, and perform callbacks
  cmdMessenger.feedinSerialData();
  updatePowerSaving();
}

void OnSetConfig()
{
  lastCommand = millis();

  // Since this firmware has a fixed config just report back a length to make
  // the desktop app happy.
  cmdMessenger.sendCmd(kStatus, 512);
}

// Called when a received command has no attached function
void OnUnknownCommand()
{
  lastCommand = millis();
  cmdMessenger.sendCmd(kStatus, F("n/a"));
}

void OnGetInfo()
{
  lastCommand = millis();
  cmdMessenger.sendCmdStart(kInfo);
  cmdMessenger.sendCmdArg(type);
  cmdMessenger.sendCmdArg(name);
  cmdMessenger.sendCmdArg(serial);
  cmdMessenger.sendCmdArg(VERSION);
  cmdMessenger.sendCmdEnd();
}

void OnGetConfig()
{
  char singleModule[20] = "";

  lastCommand = millis();
  cmdMessenger.sendCmdStart(kInfo);
  cmdMessenger.sendFieldSeparator();
  for (auto i = 0; i < 69; i++)
  {
    snprintf(singleModule, 20, "1.%i.%s:", i, pinNames[i]);
    cmdMessenger.sendArg(singleModule);
  }
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
  lastCommand = millis();
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
