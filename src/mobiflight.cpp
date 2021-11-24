#include <Arduino.h>
#include <MFBoards.h>
#include <Wire.h>

#include "ButtonNames.h"
#include "CmdMessenger.h"
#include "KeyboardMatrix.h"
#include "LEDMatrix.h"
#include "MFEEPROM.h"
#include "mobiflight.h"

// The build version comes from an environment variable.
#define STRINGIZER(arg) #arg
#define STR_VALUE(arg) STRINGIZER(arg)
#define VERSION STR_VALUE(BUILD_VERSION)

// MobiFlight expects a board name, type, and serial number to come from the board
// when requested. The serial number and name are stored in flash. The board type
// is fixed via a define.
constexpr uint8_t MEM_OFFSET_SERIAL = 0;
constexpr uint8_t MEM_LEN_SERIAL = 11;
constexpr char type[sizeof(MOBIFLIGHT_TYPE)] = MOBIFLIGHT_TYPE;
char serial[MEM_LEN_SERIAL] = MOBIFLIGHT_SERIAL;
char name[sizeof(MOBIFLIGHT_NAME)] = MOBIFLIGHT_NAME;

// I2C Addresses for the row and column IO expanders.
constexpr uint8_t ROW_I2C_ADDRESS = 0x20;    // Row MCP23017.
constexpr uint8_t COLUMN_I2C_ADDRESS = 0x21; // Column MCP23017.

// Arduino pin mappings.
constexpr uint8_t ROW_INTA_PIN = 2; // Row interrupts pin.
constexpr uint8_t LED_SDB_PIN = 4;  // Arduino pin connected to SDB on the LED driver. Blue jumper wire.
constexpr uint8_t LED_INTB_PIN = 3; // Arduino pin connected to to INTB on the LED driver.

// Virtual pins for one-off MobiFlight "modules".
constexpr uint8_t BRIGHTNESS_PIN = 69;

CmdMessenger cmdMessenger = CmdMessenger(Serial);
MFEEPROM MFeeprom;
KeyboardMatrix keyboardMatrix(ROW_I2C_ADDRESS, COLUMN_I2C_ADDRESS, ROW_INTA_PIN, OnKeyboardEvent, OnButtonPress);
LEDMatrix ledMatrix(ADDR::GND, ADDR::GND, LED_SDB_PIN, LED_INTB_PIN, OnLEDEvent);

/**
 * @brief Registers callbacks for all supported MobiFlight commands.
 * 
 */
void attachCommandCallbacks()
{
  // Attach callback methods
  cmdMessenger.attach(OnUnknownCommand);
  cmdMessenger.attach(MFMessage::kSetPin, OnSetPin);
  cmdMessenger.attach(MFMessage::kGetInfo, OnGetInfo);
  cmdMessenger.attach(MFMessage::kGetConfig, OnGetConfig);
  cmdMessenger.attach(MFMessage::kSetConfig, OnSetConfig);
  cmdMessenger.attach(MFMessage::kResetConfig, SendOk);
  cmdMessenger.attach(MFMessage::kSaveConfig, OnSaveConfig);
  cmdMessenger.attach(MFMessage::kActivateConfig, OnActivateConfig);
  cmdMessenger.attach(MFMessage::kSetName, OnSetName);
  cmdMessenger.attach(MFMessage::kGenNewSerial, OnGenNewSerial);
  cmdMessenger.attach(MFMessage::kTrigger, SendOk);
  cmdMessenger.attach(MFMessage::kResetBoard, OnResetBoard);
}

/**
 * @brief Handles an interrupt from the LEDMatrix.
 * 
 */
void OnLEDEvent()
{
  ledMatrix.HandleInterrupt();
}

/**
 * @brief Handles an interrupt from the KeyboardMatrix.
 * 
 */
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
  cmdMessenger.sendCmd(MFMessage::kStatus, F("OK"));
}

/**
 * @brief Callback for the MobiFlight event. This doesn't have to do anything so just report success.
 * 
 */
void OnSaveConfig()
{
  cmdMessenger.sendCmd(MFMessage::kConfigSaved, F("OK"));
}

/**
 * @brief Callback for the MobiFlight event. This doesn't have to do anything so just report success.
 * 
 */
void OnActivateConfig()
{
  cmdMessenger.sendCmd(MFMessage::kConfigActivated, F("OK"));
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

/**
 * @brief Callback for handling a button press from the keyboard matrix.
 * 
 * @param state State of the button (pressed or released)
 * @param row Row of the button
 * @param column Column of the button
 */
void OnButtonPress(ButtonState state, uint8_t row, uint8_t column)
{
  // While the keyboard matrix provides a row/column location that has to
  // be mapped to a button name to send the correct event to MobiFlight.
  // The button names are in a 1D array and the keyboard matrix is sparse
  // so a lookup table is used to get the correct index into the name array
  // for a given row/column in the keyboard matrix.
  char buttonName[ButtonNames::MaxNameLength] = "";
  uint8_t index = pgm_read_byte(&(ButtonNames::RowColumnLUT[row][column]));

  // If the lookup table returns 255 then it's a row/column that shouldn't
  // ever fire because it's a non-existent button.
  if (index == 255)
  {
    cmdMessenger.sendCmd(kStatus, "Row/column isn't a valid button");
    return;
  }

  // Get the button name from flash using the index.
  strcpy_P(buttonName, (char *)pgm_read_word(&(ButtonNames::Names[index])));

  // Send the button name and state to MobiFlight.
  cmdMessenger.sendCmdStart(MFMessage::kButtonChange);
  cmdMessenger.sendCmdArg(buttonName);
  cmdMessenger.sendCmdArg(state);
  cmdMessenger.sendCmdEnd();
}

/**
 * @brief Callback for setting the board configuration. Since the board configuration is fixed
 * any requests from MobiFlight to set the config are simply ignored and a remaining byte count of
 * 512 is sent back to keep the desktop app happy.
 * 
 */
void OnSetConfig()
{
  cmdMessenger.sendCmd(MFMessage::kStatus, 512);
}

/**
 * @brief Callback for unknown commands.
 * 
 */
void OnUnknownCommand()
{
  cmdMessenger.sendCmd(MFMessage::kStatus, F("n/a"));
}

/**
 * @brief Callback for sending the board information to MobiFlight.
 * 
 */
void OnGetInfo()
{
  cmdMessenger.sendCmdStart(MFMessage::kInfo);
  cmdMessenger.sendCmdArg(type);
  cmdMessenger.sendCmdArg(name);
  cmdMessenger.sendCmdArg(serial);
  cmdMessenger.sendCmdArg(VERSION);
  cmdMessenger.sendCmdEnd();
}

/**
 * @brief Callback for sending module configuration to MobiFlight.
 * The module configuration is generated on the fly rather than being stored in EEPROM.
 * 
 */
void OnGetConfig()
{
  auto i = 0;
  char singleModule[20] = "";
  char pinName[ButtonNames::MaxNameLength] = "";

  cmdMessenger.sendCmdStart(MFMessage::kInfo);
  cmdMessenger.sendFieldSeparator();

  // Send configuration for all 69 buttons.
  for (i = 0; i < 69; i++)
  {
    // Get the pin name from flash
    strcpy_P(pinName, (char *)pgm_read_word(&(ButtonNames::Names[i])));

    snprintf(singleModule, 20, "%i.%i.%s:", MFDevice::kTypeButton, i, pinName);
    cmdMessenger.sendArg(singleModule);
  }

  // Send configuration for a single output that's used to control LED brightness
  snprintf(singleModule, 20, "%i.%i.Brightness:", MFDevice::kTypeOutput, BRIGHTNESS_PIN);
  cmdMessenger.sendArg(singleModule);

  cmdMessenger.sendCmdEnd();
}

/**
 * @brief Callback for MobiFlight LED output commands.
 * 
 */
void OnSetPin()
{
  // Read led state argument, interpret string as boolean
  int pin = cmdMessenger.readInt16Arg();
  int state = cmdMessenger.readInt16Arg();

  // The brightness virtual pin is 69
  if (pin == BRIGHTNESS_PIN)
  {
    cmdMessenger.sendCmd(kStatus, "OK");
    ledMatrix.SetBrightness(state);
  }
}

/**
 * @brief Generates a new serial number for the board and stores it in EEPROM.
 * 
 */
void OnGenNewSerial()
{
  generateSerial(true);
  cmdMessenger.sendCmdStart(MFMessage::kInfo);
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

  cmdMessenger.sendCmdStart(MFMessage::kStatus);
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
  Wire.setClock(400000);
  Serial.begin(115200);

  attachCommandCallbacks();
  cmdMessenger.printLfCr();

  // Wait for a serial connection before continuing to simplify the debugging experience.
  while (!Serial)
    ;

  OnResetBoard();
  keyboardMatrix.Init();
  ledMatrix.Init();
}

/**
 * @brief Arduino application loop.
 * 
 */
void loop()
{
  cmdMessenger.feedinSerialData();
  keyboardMatrix.Loop();
  ledMatrix.Loop();
}
