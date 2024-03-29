#include <Arduino.h>
#include <Wire.h>

#include "ButtonNames.h"
#include "CmdMessenger.h"
#include "KeyboardMatrix.h"
#include "LEDMatrix.h"
#include "MFBoards.h"
#include "MFEEPROM.h"
#include "mobiflight.h"

// The build version comes from an environment variable.
#define STRINGIZER(arg) #arg
#define STR_VALUE(arg) STRINGIZER(arg)

// These defines are used in response to the GetInfo command.
#define VERSION STR_VALUE(BUILD_VERSION)  // This comes from get_version.py and is set by the build process.
#define NAME STR_VALUE(MOBIFLIGHT_NAME)   // This comes from get_version.py and is set by the build process.
#define MFTYPE STR_VALUE(MOBIFLIGHT_TYPE) // This comes from get_version.py and is set by the build process.

// MobiFlight expects a board name, type, and serial number to come from the board
// when requested. The serial number is stored in flash. The board type
// and name are fixed and come from Board.h.
static constexpr uint8_t MEM_OFFSET_SERIAL = 0;
static constexpr uint8_t MEM_LEN_SERIAL = 11;
char serial[MEM_LEN_SERIAL];

// I2C Addresses for the row and column IO expanders.
static constexpr uint8_t ROW_I2C_ADDRESS = 0x20;    // Row MCP23017.
static constexpr uint8_t COLUMN_I2C_ADDRESS = 0x21; // Column MCP23017.

// Virtual pins for one-off MobiFlight "modules". Their pins
// start after all the keyboard matrix buttons, of which there are
// ButtonNames::ButtonCount. Since it's origin zero the next free pin
// is simply that value.
static constexpr uint8_t BRIGHTNESS_PIN = ButtonNames::ButtonCount;

// Other defines.
static constexpr unsigned long POWER_SAVING_TIME_SECS = 60 * 60; // One hour (60 minutes * 60 seconds).

unsigned long lastButtonPress = 0;
bool powerSavingMode = false;
static char unique_serial_str[PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2 + 1 + 3] = "SN-"; // The +1 is for the null terminator and +3 is for "ID-"

CmdMessenger cmdMessenger = CmdMessenger(Serial);
MFEEPROM MFeeprom;
KeyboardMatrix keyboardMatrix(KEY_INT_PIN, OnKeyboardEvent, OnButtonEvent);
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
  // This is required to maintain compatibility with the standard Mobiflight firmware
  // which eventually activates the config when resetting the board.
  OnActivateConfig();
}

/**
 * @brief Callback for handling a button press from the keyboard matrix.
 *
 * @param keyId The ID of the key that triggered the event
 * @param state State of the button (pressed or released)
 */
void OnButtonEvent(const uint8_t keyId, const ButtonState state)
{
  lastButtonPress = millis();

  if (keyId > ButtonNames::ButtonCount)
  {
    cmdMessenger.sendCmd(MFMessage::kStatus, F("Keypress isn't valid"));
    return;
  }

  // Send the button name and state to MobiFlight.
  cmdMessenger.sendCmdStart(MFMessage::kButtonChange);
  cmdMessenger.sendCmdArg(ButtonNames::Names[keyId - 1]); // The names array is origin 0 while the key IDs are origin 1.
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
  cmdMessenger.sendCmdArg(MFTYPE);
  cmdMessenger.sendCmdArg(NAME);
  cmdMessenger.sendCmdArg(unique_serial_str);
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
  Serial.println(F("10,1.0.L1:1.1.L2:1.2.L3:1.3.L4:1.4.L5:1.5.L6:1.6.MSG:1.7.DIR:1.8.IDX:1.9.TUN:1.10.A:1.11.H:1.12.O:1.13.V:1.14.FPLN:1.15.B:1.16.I:1.17.P:1.18.W:1.19.LEGS:1.20.C:1.21.J:1.22.Q:1.23.X:1.24.DEP_ARR:1.25.D:1.26.K:1.27.R:1.28.Y:1.29.PERF:1.30.E:1.31.L:1.32.S:1.33.Z:1.34.DSPL_MENU:1.35.F:1.36.M:1.37.T:1.38.SP:1.39.MFD_ADV:1.40.G:1.41.N:1.42.U:1.43.DIV:1.44.MFD_DATA:1.45.1:1.46.4:1.47.7:1.48.DOT:1.49.PREV:1.50.2:1.51.5:1.52.8:1.53.0:1.54.3:1.55.6:1.56.9:1.57.PLUSMINUS:1.58.R1:1.59.R2:1.60.R3:1.61.R4:1.62.R5:1.63.R6:1.64.EXEC:1.65.NEXT:1.66.CLR:1.67.BRT:1.68.DIM:1.69.DEL:3.70.Brightness:;"));
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

  if (pin == BRIGHTNESS_PIN)
  {
    cmdMessenger.sendCmd(kStatus, "OK");
    ledMatrix.SetBrightness(state);
    ledMatrix.SetPowerSaveMode(false);
    powerSavingMode = false;
  }
}

/**
 * @brief Serial numbers are hardcoded on Picos in the flash memory so this just
 * returns that value.
 *
 */
void OnGenNewSerial()
{
  cmdMessenger.sendCmdStart(MFMessage::kInfo);
  cmdMessenger.sendCmdArg(unique_serial_str);
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
  cmdMessenger.sendCmdArg(NAME);
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
 * @brief Arduino initialization method.
 *
 */
void setup()
{
  MFeeprom.init();
  pico_get_unique_board_id_string(&unique_serial_str[3], sizeof(unique_serial_str) - 3);

  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
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
