#include <Arduino.h>
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
// when requested. The serial number is stored in flash. The board type
// and name are fixed and come from Board.h.
static constexpr uint8_t MEM_OFFSET_SERIAL = 0;
static constexpr uint8_t MEM_LEN_SERIAL = 11;
char serial[MEM_LEN_SERIAL];

// I2C Addresses for the row and column IO expanders.
static constexpr uint8_t ROW_I2C_ADDRESS = 0x20;    // Row MCP23017.
static constexpr uint8_t COLUMN_I2C_ADDRESS = 0x21; // Column MCP23017.

// Arduino pin mappings.
static constexpr uint8_t ROW_INTA_PIN = 0; // Row interrupts pin.
static constexpr uint8_t LED_SDB_PIN = 4;  // Arduino pin connected to SDB on the LED driver.
static constexpr uint8_t LED_INTB_PIN = 7; // Arduino pin connected to to INTB on the LED driver.

// Virtual pins for one-off MobiFlight "modules". Their pins
// start after all the keyboard matrix buttons, of which there are
// ButtonNames::ButtonCount. Since it's origin zero the next free pin
// is simply that value.
static constexpr uint8_t BRIGHTNESS_PIN = ButtonNames::ButtonCount;

// Other defines.
static constexpr unsigned long POWER_SAVING_TIME_SECS = 60 * 60; // One hour (60 minutes * 60 seconds).

unsigned long lastButtonPress = 0;
bool powerSavingMode = false;

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
  lastButtonPress = millis();

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
  cmdMessenger.sendCmdArg(F("Collins FMS 3000"));
  cmdMessenger.sendCmdArg(F("Collins FMS 3000"));
  cmdMessenger.sendCmdArg(serial);
  cmdMessenger.sendCmdArg(VERSION);
  cmdMessenger.sendCmdEnd();
}

/**
 * @brief Callback for sending module configuration to MobiFlight.
 * The module configuration is stored as a fixed string in EEPROM since it never changes.
 *
 */
void OnGetConfig()
{
  Serial.println(F("10,1.0.L1:1.1.L2:1.2.L3:1.3.L4:1.4.L5:1.5.L6:1.6.MSG:1.7.DIR:1.8.IDX:1.9.TUN:1.10.A:1.11.H:1.12.O:1.13.V:1.14.FPLN:1.15.B:1.16.I:1.17.P:1.18.W:1.19.LEGS:1.20.C:1.21.J:1.22.Q:1.23.X:1.24.DEPARR:1.25.D:1.26.K:1.27.R:1.28.Y:1.29.PERF:1.30.E:1.31.L:1.32.S:1.33.Z:1.34.DSPL_MENU:1.35.F:1.36.M:1.37.T:1.38.SP:1.39.MFD_ADV:1.40.G:1.41.N:1.42.U:1.43.DIV:1.44.MFD_DATA:1.45.1:1.46.4:1.47.7:1.48.DOT:1.49.PREVPAGE:1.50.2:1.51.5:1.52.8:1.53.0:1.54.3:1.55.6:1.56.9:1.57.PLUSMINUS:1.58.R1:1.59.R2:1.60.R3:1.61.R4:1.62.R5:1.63.R6:1.64.EXEC:1.65.NEXTPAGE:1.66.CLR:1.67.BRT:1.68.DIM:1.69.DEL:3.70.Brightness:;"));
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
  cmdMessenger.sendCmdArg(F("Collins FMS 3000"));
  cmdMessenger.sendCmdEnd();
}

/**
 * @brief Checks to see if power saving mode should be enabled or disabled
 * based on the last time a key was pressed.
 *
 */
void CheckForPowerSave()
{
  if (!powerSavingMode && ((millis() - lastButtonPress) > (POWER_SAVING_TIME_SECS * 1000)))
  {
    powerSavingMode = true;
    ledMatrix.SetPowerSaveMode(true);
  }
  else if (powerSavingMode && ((millis() - lastButtonPress) < (POWER_SAVING_TIME_SECS * 1000)))
  {
    ledMatrix.SetPowerSaveMode(false);
    powerSavingMode = false;
  }
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

  OnResetBoard();
  keyboardMatrix.Init();
  ledMatrix.Init();

  lastButtonPress = millis();
}

/**
 * @brief Arduino application loop.
 *
 */
void loop()
{
  cmdMessenger.feedinSerialData();
  keyboardMatrix.Loop();
  CheckForPowerSave();
  ledMatrix.Loop();
}
