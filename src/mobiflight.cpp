/**
 * Includes Core Arduino functionality 
 **/
char foo;
#if ARDUINO < 100
#include <WProgram.h>
#else
#include <Arduino.h>
#endif

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

#if MF_SEGMENT_SUPPORT == 1
#include <MFSegments.h>
#endif

#include <MFButton.h>
#include <MFEncoder.h>

#if MF_STEPPER_SUPPORT == 1
#include <AccelStepper.h>
#include <MFStepper.h>
#endif

#if MF_SERVO_SUPPORT == 1
#include <Servo.h>
#include <MFServo.h>
#endif

#include <MFOutput.h>

#if MF_LCD_SUPPORT == 1
#include <LiquidCrystal_I2C.h>
#include <MFLCDDisplay.h>
#endif

#if MF_ANALOG_SUPPORT == 1
#include <MFAnalog.h>
#endif

#if MF_SHIFTER_SUPPORT == 1
#include <MFShifter.h>
#endif

const uint8_t MEM_OFFSET_NAME = 0;
const uint8_t MEM_LEN_NAME = 48;
const uint8_t MEM_OFFSET_SERIAL = MEM_OFFSET_NAME + MEM_LEN_NAME;
const uint8_t MEM_LEN_SERIAL = 11;
const uint8_t MEM_OFFSET_CONFIG = MEM_OFFSET_NAME + MEM_LEN_NAME + MEM_LEN_SERIAL;

uint32_t lastAnalogAverage = 0;
uint32_t lastAnalogRead = 0;
uint32_t lastButtonUpdate = 0;
uint32_t lastEncoderUpdate = 0;

const char type[sizeof(MOBIFLIGHT_TYPE)] = MOBIFLIGHT_TYPE;
char serial[MEM_LEN_SERIAL] = MOBIFLIGHT_SERIAL;
char name[MEM_LEN_NAME] = MOBIFLIGHT_NAME;
const int MEM_LEN_CONFIG = MEMLEN_CONFIG;

uint16_t configLength = 0;
boolean configActivated = false;

bool powerSavingMode = false;
const unsigned long POWER_SAVING_TIME = 60 * 15; // in seconds

CmdMessenger cmdMessenger = CmdMessenger(Serial);
unsigned long lastCommand;

MFEEPROM MFeeprom;

MFOutput outputs[MAX_OUTPUTS];
uint8_t outputsRegistered = 0;

MFButton buttons[MAX_BUTTONS];
uint8_t buttonsRegistered = 0;

#if MF_SEGMENT_SUPPORT == 1
MFSegments ledSegments[MAX_LEDSEGMENTS];
uint8_t ledSegmentsRegistered = 0;
#endif

MFEncoder encoders[MAX_ENCODERS];
uint8_t encodersRegistered = 0;

#if MF_STEPPER_SUPPORT == 1
MFStepper *steppers[MAX_STEPPERS]; //
uint8_t steppersRegistered = 0;
#endif

#if MF_SERVO_SUPPORT == 1
MFServo servos[MAX_MFSERVOS];
uint8_t servosRegistered = 0;
#endif

#if MF_LCD_SUPPORT == 1
MFLCDDisplay lcd_I2C[MAX_MFLCD_I2C];
uint8_t lcd_12cRegistered = 0;
#endif

#if MF_ANALOG_SUPPORT == 1
MFAnalog analog[MAX_ANALOG_INPUTS];
uint8_t analogRegistered = 0;
#endif

#if MF_SHIFTER_SUPPORT == 1
MFShifter shiftregisters[MAX_SHIFTERS];
uint8_t shiftregisterRegistered = 0;
#endif

// Callbacks define on which received commands we take action
void attachCommandCallbacks()
{
  // Attach callback methods
  cmdMessenger.attach(OnUnknownCommand);

#if MF_SEGMENT_SUPPORT == 1
  cmdMessenger.attach(kInitModule, OnInitModule);
  cmdMessenger.attach(kSetModule, OnSetModule);
  cmdMessenger.attach(kSetModuleBrightness, OnSetModuleBrightness);
#endif

  cmdMessenger.attach(kSetPin, OnSetPin);

#if MF_STEPPER_SUPPORT == 1
  cmdMessenger.attach(kSetStepper, OnSetStepper);
#endif

#if MF_SERVO_SUPPORT == 1
  cmdMessenger.attach(kSetServo, OnSetServo);
#endif

  cmdMessenger.attach(kGetInfo, OnGetInfo);
  cmdMessenger.attach(kGetConfig, OnGetConfig);
  cmdMessenger.attach(kSetConfig, OnSetConfig);
  cmdMessenger.attach(kResetConfig, OnResetConfig);
  cmdMessenger.attach(kSaveConfig, OnSaveConfig);
  cmdMessenger.attach(kActivateConfig, OnActivateConfig);
  cmdMessenger.attach(kSetName, OnSetName);
  cmdMessenger.attach(kGenNewSerial, OnGenNewSerial);

#if MF_STEPPER_SUPPORT == 1
  cmdMessenger.attach(kResetStepper, OnResetStepper);
  cmdMessenger.attach(kSetZeroStepper, OnSetZeroStepper);
#endif

  cmdMessenger.attach(kTrigger, OnTrigger);
  cmdMessenger.attach(kResetBoard, OnResetBoard);

#if MF_LCD_SUPPORT == 1
  cmdMessenger.attach(kSetLcdDisplayI2C, OnSetLcdDisplayI2C);
#endif

#if MF_SHIFTER_SUPPORT
  cmdMessenger.attach(kSetShiftRegisterPins, OnSetShiftRegisterPins);
#endif

#ifdef DEBUG
  cmdMessenger.sendCmd(kStatus, F("Attached callbacks"));
#endif
}

void OnResetBoard()
{
  MFeeprom.init();
  generateSerial(false);
  lastCommand = millis();
  loadConfig();
  _restoreName();
}

// Setup function
void setup()
{
  Serial.begin(115200);

  attachCommandCallbacks();
  cmdMessenger.printLfCr();

  OnResetBoard();
  // Time Gap between Inputs, do not read at the same loop
  lastAnalogAverage = millis() + 4;
  lastAnalogRead = millis() + 4;
  lastButtonUpdate = millis();
  lastEncoderUpdate = millis() + 2;
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
  if (!force)
    MFeeprom.write_byte(MEM_OFFSET_CONFIG, 0x00); // First byte of config to 0x00 to ensure to start 1st time with empty config, but not if forced from the connector to generate a new one
}

void loadConfig()
{
  _activateConfig();
}

void SetPowerSavingMode(bool state)
{
  // disable the lights ;)
  powerSavingMode = state;

#if MF_SEGMENT_SUPPORT == 1
  PowerSaveLedSegment(state);
#endif

#ifdef DEBUG
  if (state)
    cmdMessenger.sendCmd(kStatus, F("On"));
  else
    cmdMessenger.sendCmd(kStatus, F("Off"));
#endif
  //PowerSaveOutputs(state);
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

  // if config has been reset
  // and still is not activated
  // do not perform updates
  // to prevent mangling input for config (shared buffers)
  if (!configActivated)
    return;

  readButtons();
  readEncoder();
#if MF_ANALOG_SUPPORT == 1
  readAnalog();
#endif

  // segments do not need update
#if MF_STEPPER_SUPPORT == 1
  updateSteppers();
#endif

#if MF_SERVO_SUPPORT == 1
  updateServos();
#endif
}

//// OUTPUT /////
void AddOutput(uint8_t pin = 1, char const *name = "Output")
{
  if (outputsRegistered == MAX_OUTPUTS)
    return;

  outputs[outputsRegistered] = MFOutput(pin);
  outputsRegistered++;
#ifdef DEBUG
  cmdMessenger.sendCmd(kStatus, F("Added output"));
#endif
}

void ClearOutputs()
{
  outputsRegistered = 0;
#ifdef DEBUG
  cmdMessenger.sendCmd(kStatus, F("Cleared outputs"));
#endif
}

//// BUTTONS /////
void AddButton(uint8_t pin = 1, char const *name = "Button")
{
  if (buttonsRegistered == MAX_BUTTONS)
    return;

  buttons[buttonsRegistered] = MFButton(pin, name);
  buttons[buttonsRegistered].attachHandler(btnOnRelease, handlerOnRelease);
  buttons[buttonsRegistered].attachHandler(btnOnPress, handlerOnRelease);

  buttonsRegistered++;
#ifdef DEBUG
  cmdMessenger.sendCmd(kStatus, F("Added button ") /* + name */);
#endif
}

//// ENCODERS /////
void AddEncoder(uint8_t pin1 = 1, uint8_t pin2 = 2, uint8_t encoder_type = 0, char const *name = "Encoder")
{
  if (encodersRegistered == MAX_ENCODERS)
    return;

  encoders[encodersRegistered] = MFEncoder();
  encoders[encodersRegistered].attach(pin1, pin2, encoder_type, name);
  encoders[encodersRegistered].attachHandler(encLeft, handlerOnEncoder);
  encoders[encodersRegistered].attachHandler(encLeftFast, handlerOnEncoder);
  encoders[encodersRegistered].attachHandler(encRight, handlerOnEncoder);
  encoders[encodersRegistered].attachHandler(encRightFast, handlerOnEncoder);

  encodersRegistered++;
#ifdef DEBUG
  cmdMessenger.sendCmd(kStatus, F("Added encoder"));
#endif
}

void ClearEncoders()
{
  encodersRegistered = 0;
#ifdef DEBUG
  cmdMessenger.sendCmd(kStatus, F("Cleared encoders"));
#endif
}

//// OUTPUTS /////

#if MF_SEGMENT_SUPPORT == 1
//// SEGMENTS /////
void AddLedSegment(int dataPin, int csPin, int clkPin, int numDevices, int brightness)
{
  if (ledSegmentsRegistered == MAX_LEDSEGMENTS)
    return;

  if (isPinRegistered(dataPin) || isPinRegistered(clkPin) || isPinRegistered(csPin))
    return;

  ledSegments[ledSegmentsRegistered].attach(dataPin, csPin, clkPin, numDevices, brightness); // lc is our object

  registerPin(dataPin, kTypeLedSegment);
  registerPin(csPin, kTypeLedSegment);
  registerPin(clkPin, kTypeLedSegment);
  ledSegmentsRegistered++;
#ifdef DEBUG
  cmdMessenger.sendCmd(kStatus, F("Added Led Segment"));
#endif
}

void ClearLedSegments()
{
  for (int i = 0; i != ledSegmentsRegistered; i++)
  {
    ledSegments[ledSegmentsRegistered].detach();
  }
  ledSegmentsRegistered = 0;
#ifdef DEBUG
  cmdMessenger.sendCmd(kStatus, F("Cleared segments"));
#endif
}

void PowerSaveLedSegment(bool state)
{
  for (int i = 0; i != ledSegmentsRegistered; ++i)
  {
    ledSegments[i].powerSavingMode(state);
  }

  for (int i = 0; i != outputsRegistered; ++i)
  {
    outputs[i].powerSavingMode(state);
  }
}
#endif

#if MF_STEPPER_SUPPORT == 1
//// STEPPER ////
void AddStepper(int pin1, int pin2, int pin3, int pin4, int btnPin1)
{
  if (steppersRegistered == MAX_STEPPERS)
    return;
  if (isPinRegistered(pin1) || isPinRegistered(pin2) || isPinRegistered(pin3) || isPinRegistered(pin4) || (btnPin1 > 0 && isPinRegistered(btnPin1)))
  {
#ifdef DEBUG
    cmdMessenger.sendCmd(kStatus, F("Conflict with stepper"));
#endif
    return;
  }

  steppers[steppersRegistered] = new MFStepper(pin1, pin2, pin3, pin4, btnPin1); // is our object
  steppers[steppersRegistered]->setMaxSpeed(STEPPER_SPEED);
  steppers[steppersRegistered]->setAcceleration(STEPPER_ACCEL);

  registerPin(pin1, kTypeStepper);
  registerPin(pin2, kTypeStepper);
  registerPin(pin3, kTypeStepper);
  registerPin(pin4, kTypeStepper);
  // autoreset is not released yet
  if (btnPin1 > 0)
  {
    registerPin(btnPin1, kTypeStepper);
    // this triggers the auto reset if we need to reset
    steppers[steppersRegistered]->reset();
  }

  // all set
  steppersRegistered++;

#ifdef DEBUG
  cmdMessenger.sendCmd(kStatus, F("Added stepper"));
#endif
}

void ClearSteppers()
{
  for (int i = 0; i != steppersRegistered; i++)
  {
    delete steppers[steppersRegistered];
  }
  clearRegisteredPins(kTypeStepper);
  steppersRegistered = 0;
#ifdef DEBUG
  cmdMessenger.sendCmd(kStatus, F("Cleared steppers"));
#endif
}
#endif

#if MF_SERVO_SUPPORT == 1
//// SERVOS /////
void AddServo(int pin)
{
  if (servosRegistered == MAX_MFSERVOS)
    return;
  if (isPinRegistered(pin))
    return;

  servos[servosRegistered].attach(pin, true);
  registerPin(pin, kTypeServo);
  servosRegistered++;
}

void ClearServos()
{
  for (int i = 0; i != servosRegistered; i++)
  {
    servos[servosRegistered].detach();
  }
  clearRegisteredPins(kTypeServo);
  servosRegistered = 0;
#ifdef DEBUG
  cmdMessenger.sendCmd(kStatus, F("Cleared servos"));
#endif
}
#endif

#if MF_LCD_SUPPORT == 1
//// LCD Display /////
void AddLcdDisplay(uint8_t address = 0x24, uint8_t cols = 16, uint8_t lines = 2, char const *name = "LCD")
{
  if (lcd_12cRegistered == MAX_MFLCD_I2C)
    return;
  registerPin(SDA, kTypeLcdDisplayI2C);
  registerPin(SCL, kTypeLcdDisplayI2C);

  lcd_I2C[lcd_12cRegistered].attach(address, cols, lines);
  lcd_12cRegistered++;
#ifdef DEBUG
  cmdMessenger.sendCmd(kStatus, F("Added lcdDisplay"));
#endif
}

void ClearLcdDisplays()
{
  for (int i = 0; i != lcd_12cRegistered; i++)
  {
    lcd_I2C[lcd_12cRegistered].detach();
  }
  clearRegisteredPins(kTypeLcdDisplayI2C);
  lcd_12cRegistered = 0;
#ifdef DEBUG
  cmdMessenger.sendCmd(kStatus, F("Cleared lcdDisplays"));
#endif
}
#endif

#if MF_ANALOG_SUPPORT == 1

void AddAnalog(uint8_t pin = 1, char const *name = "AnalogInput", uint8_t sensitivity = 3)
{
  if (analogRegistered == MAX_ANALOG_INPUTS)
    return;

  analog[analogRegistered] = MFAnalog(pin, handlerOnAnalogChange, name, sensitivity);
  analogRegistered++;
#ifdef DEBUG
  cmdMessenger.sendCmd(kStatus, F("Added analog device "));
#endif
}

void ClearAnalog()
{
  clearRegisteredPins(kTypeAnalogInput);
  analogRegistered = 0;
#ifdef DEBUG
  cmdMessenger.sendCmd(kStatus, F("Cleared analog devices"));
#endif
}

#endif

#if MF_SHIFTER_SUPPORT == 1
//// SHIFT REGISTER /////
void AddShifter(uint8_t latchPin, uint8_t clockPin, uint8_t dataPin, uint8_t modules, char const *name = "Shifter")
{
  if (shiftregisterRegistered == MAX_SHIFTERS)
    return;
  shiftregisters[shiftregisterRegistered].attach(latchPin, clockPin, dataPin, modules);
  shiftregisters[shiftregisterRegistered].clear();
  shiftregisterRegistered++;

#ifdef DEBUG
  cmdMessenger.sendCmd(kStatus, F("Added Shifter"));
#endif
}

void ClearShifters()
{
  for (int i = 0; i != shiftregisterRegistered; i++)
  {
    shiftregisters[shiftregisterRegistered].detach();
  }

  shiftregisterRegistered = 0;
#ifdef DEBUG
  cmdMessenger.sendCmd(kStatus, F("Cleared Shifter"));
#endif
}
#endif

//// EVENT HANDLER /////
void handlerOnRelease(uint8_t eventId, uint8_t pin, const char *name)
{
  cmdMessenger.sendCmdStart(kButtonChange);
  cmdMessenger.sendCmdArg(name);
  cmdMessenger.sendCmdArg(eventId);
  cmdMessenger.sendCmdEnd();
};

//// EVENT HANDLER /////
void handlerOnEncoder(uint8_t eventId, uint8_t pin, const char *name)
{
  cmdMessenger.sendCmdStart(kEncoderChange);
  cmdMessenger.sendCmdArg(name);
  cmdMessenger.sendCmdArg(eventId);
  cmdMessenger.sendCmdEnd();
};

//// EVENT HANDLER /////
void handlerOnAnalogChange(int value, uint8_t pin, const char *name)
{
  cmdMessenger.sendCmdStart(kAnalogChange);
  cmdMessenger.sendCmdArg(name);
  cmdMessenger.sendCmdArg(value);
  cmdMessenger.sendCmdEnd();
};

/**
 ** config stuff
 **/
void OnSetConfig()
{
  lastCommand = millis();

  // Since this firmware has a fixed config just report back a length to make
  // the desktop app happy.
  cmdMessenger.sendCmd(kStatus, configLength);
}

void OnResetConfig()
{
  cmdMessenger.sendCmd(kStatus, F("OK"));
}

void OnSaveConfig()
{
  cmdMessenger.sendCmd(kConfigSaved, F("OK"));
}

void OnActivateConfig()
{
  _activateConfig();
}

void _activateConfig()
{
  configActivated = true;
  cmdMessenger.sendCmd(kConfigActivated, F("OK"));
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
  // for (uint16_t i = 1; i < configLength; i++)
  // {
  //   cmdMessenger.sendArg(MFeeprom.read_char(MEM_OFFSET_CONFIG + i));
  // }
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

#if MF_SEGMENT_SUPPORT == 1
void OnInitModule()
{
  int module = cmdMessenger.readInt16Arg();
  int subModule = cmdMessenger.readInt16Arg();
  int brightness = cmdMessenger.readInt16Arg();
  ledSegments[module].setBrightness(subModule, brightness);
  lastCommand = millis();
}

void OnSetModule()
{
  int module = cmdMessenger.readInt16Arg();
  int subModule = cmdMessenger.readInt16Arg();
  char *value = cmdMessenger.readStringArg();
  uint8_t points = (uint8_t)cmdMessenger.readInt16Arg();
  uint8_t mask = (uint8_t)cmdMessenger.readInt16Arg();
  ledSegments[module].display(subModule, value, points, mask);
  lastCommand = millis();
}

void OnSetModuleBrightness()
{
  int module = cmdMessenger.readInt16Arg();
  int subModule = cmdMessenger.readInt16Arg();
  int brightness = cmdMessenger.readInt16Arg();
  ledSegments[module].setBrightness(subModule, brightness);
  lastCommand = millis();
}

#endif

#if MF_SHIFTER_SUPPORT == 1

void OnInitShiftRegister()
{
  int module = cmdMessenger.readInt16Arg();
  shiftregisters[module].clear();
  lastCommand = millis();
}

void OnSetShiftRegisterPins()
{

  int module = cmdMessenger.readInt16Arg();
  char *pins = cmdMessenger.readStringArg();
  int value = cmdMessenger.readInt16Arg();
  shiftregisters[module].setPins(pins, value);
  lastCommand = millis();
}

#endif

#if MF_STEPPER_SUPPORT == 1
void OnSetStepper()
{
  int stepper = cmdMessenger.readInt16Arg();
  long newPos = cmdMessenger.readInt32Arg();

  if (stepper >= steppersRegistered)
    return;
  steppers[stepper]->moveTo(newPos);
  lastCommand = millis();
}

void OnResetStepper()
{
  int stepper = cmdMessenger.readInt16Arg();

  if (stepper >= steppersRegistered)
    return;
  steppers[stepper]->reset();
  lastCommand = millis();
}

void OnSetZeroStepper()
{
  int stepper = cmdMessenger.readInt16Arg();

  if (stepper >= steppersRegistered)
    return;
  steppers[stepper]->setZero();
  lastCommand = millis();
}

void updateSteppers()
{
  for (int i = 0; i != steppersRegistered; i++)
  {
    steppers[i]->update();
  }
}
#endif

#if MF_SERVO_SUPPORT == 1
void OnSetServo()
{
  int servo = cmdMessenger.readInt16Arg();
  int newValue = cmdMessenger.readInt16Arg();
  if (servo >= servosRegistered)
    return;
  servos[servo].moveTo(newValue);
  lastCommand = millis();
}

void updateServos()
{
  for (int i = 0; i != servosRegistered; i++)
  {
    servos[i].update();
  }
}
#endif

#if MF_LCD_SUPPORT == 1
void OnSetLcdDisplayI2C()
{
  int address = cmdMessenger.readInt16Arg();
  char *output = cmdMessenger.readStringArg();
  lcd_I2C[address].display(output);
  lastCommand = millis();
}
#endif

void readButtons()
{
  if (millis() - lastButtonUpdate <= MF_BUTTON_DEBOUNCE_MS)
    return;
  lastButtonUpdate = millis();
  for (int i = 0; i != buttonsRegistered; i++)
  {
    buttons[i].update();
  }
}

void readEncoder()
{
  if (millis() - lastEncoderUpdate < 1)
    return;
  lastEncoderUpdate = millis();
  for (int i = 0; i != encodersRegistered; i++)
  {
    encoders[i].update();
  }
}

#if MF_ANALOG_SUPPORT == 1
void readAnalog()
{
  if (millis() - lastAnalogAverage > 10)
  {
    for (int i = 0; i != analogRegistered; i++)
    {
      analog[i].readBuffer();
    }
    lastAnalogAverage = millis();
  }
  if (millis() - lastAnalogRead < 50)
    return;
  lastAnalogRead = millis();
  for (int i = 0; i != analogRegistered; i++)
  {
    analog[i].update();
  }
}
#endif

void OnGenNewSerial()
{
  generateSerial(true);
  cmdMessenger.sendCmdStart(kInfo);
  cmdMessenger.sendCmdArg(serial);
  cmdMessenger.sendCmdEnd();
}

void OnSetName()
{
  char *cfg = cmdMessenger.readStringArg();
  memcpy(name, cfg, MEM_LEN_NAME);
  _storeName();
  cmdMessenger.sendCmdStart(kStatus);
  cmdMessenger.sendCmdArg(name);
  cmdMessenger.sendCmdEnd();
}

void _storeName()
{
  char prefix[] = "#";
  MFeeprom.write_block(MEM_OFFSET_NAME, prefix, 1);
  MFeeprom.write_block(MEM_OFFSET_NAME + 1, name, MEM_LEN_NAME - 1);
}

void _restoreName()
{
  char testHasName[1] = "";
  MFeeprom.read_block(MEM_OFFSET_NAME, testHasName, 1);
  if (testHasName[0] != '#')
    return;

  MFeeprom.read_block(MEM_OFFSET_NAME + 1, name, MEM_LEN_NAME - 1);
}

void OnTrigger()
{
  // Trigger all button release events first...
  for (int i = 0; i != buttonsRegistered; i++)
  {
    buttons[i].triggerOnRelease();
  }
  // ... then trigger all the press events
  for (int i = 0; i != buttonsRegistered; i++)
  {
    buttons[i].triggerOnPress();
  }
}
