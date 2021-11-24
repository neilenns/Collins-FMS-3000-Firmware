#pragma once

// The methods for storing all this data in PROGMEM comes from
// https://www.arduino.cc/reference/en/language/variables/utilities/progmem/

static const uint8_t rowColumnMappings[2][12] PROGMEM = {
    {0,
     255,
     255,
     255,
     255,
     255,
     255,
     255,
     255,
     255,
     255,
     58},
    {1,
     255,
     255,
     255,
     255,
     255,
     255,
     255,
     255,
     255,
     255,
     59}};

static const uint8_t MaxNameLength = 5;

const char string0[] PROGMEM = "LSK1";
const char string1[] PROGMEM = "LSK2";
const char string2[] PROGMEM = "LSK3";
const char string3[] PROGMEM = "LSK4";
const char string4[] PROGMEM = "LSK5";
const char string5[] PROGMEM = "LSK6";
const char string6[] PROGMEM = "MSG";
const char string7[] PROGMEM = "DIR";
const char string8[] PROGMEM = "IDX";
const char string9[] PROGMEM = "TUN";
const char string10[] PROGMEM = "A";
const char string11[] PROGMEM = "H";
const char string12[] PROGMEM = "O";
const char string13[] PROGMEM = "V";
const char string14[] PROGMEM = "FPLN";
const char string15[] PROGMEM = "B";
const char string16[] PROGMEM = "I";
const char string17[] PROGMEM = "P";
const char string18[] PROGMEM = "W";
const char string19[] PROGMEM = "LEGS";
const char string20[] PROGMEM = "C";
const char string21[] PROGMEM = "J";
const char string22[] PROGMEM = "Q";
const char string23[] PROGMEM = "X";
const char string24[] PROGMEM = "DEP";
const char string25[] PROGMEM = "D";
const char string26[] PROGMEM = "K";
const char string27[] PROGMEM = "R";
const char string28[] PROGMEM = "Y";
const char string29[] PROGMEM = "PERF";
const char string30[] PROGMEM = "E";
const char string31[] PROGMEM = "L";
const char string32[] PROGMEM = "S";
const char string33[] PROGMEM = "Z";
const char string34[] PROGMEM = "DSPL";
const char string35[] PROGMEM = "F";
const char string36[] PROGMEM = "M";
const char string37[] PROGMEM = "T";
const char string38[] PROGMEM = "SP";
const char string39[] PROGMEM = "ADV";
const char string40[] PROGMEM = "G";
const char string41[] PROGMEM = "N";
const char string42[] PROGMEM = "U";
const char string43[] PROGMEM = "SLSH";
const char string44[] PROGMEM = "DATA";
const char string45[] PROGMEM = "1";
const char string46[] PROGMEM = "4";
const char string47[] PROGMEM = "7";
const char string48[] PROGMEM = "DOT";
const char string49[] PROGMEM = "PREV";
const char string50[] PROGMEM = "2";
const char string51[] PROGMEM = "5";
const char string52[] PROGMEM = "8";
const char string53[] PROGMEM = "0";
const char string54[] PROGMEM = "3";
const char string55[] PROGMEM = "6";
const char string56[] PROGMEM = "9";
const char string57[] PROGMEM = "PLUS";
const char string58[] PROGMEM = "RSK1";
const char string59[] PROGMEM = "RSK2";
const char string60[] PROGMEM = "RSK3";
const char string61[] PROGMEM = "RSK4";
const char string62[] PROGMEM = "RSK5";
const char string63[] PROGMEM = "RSK6";
const char string64[] PROGMEM = "EXEC";
const char string65[] PROGMEM = "NEXT";
const char string66[] PROGMEM = "CLR";
const char string67[] PROGMEM = "BRT";
const char string68[] PROGMEM = "DIM";

const char *const pinNames[] PROGMEM = {
    string0,
    string1,
    string2,
    string3,
    string4,
    string5,
    string6,
    string7,
    string8,
    string9,
    string10,
    string11,
    string12,
    string13,
    string14,
    string15,
    string16,
    string17,
    string18,
    string19,
    string20,
    string21,
    string22,
    string23,
    string24,
    string25,
    string26,
    string27,
    string28,
    string29,
    string30,
    string31,
    string32,
    string33,
    string34,
    string35,
    string36,
    string37,
    string38,
    string39,
    string40,
    string41,
    string42,
    string43,
    string44,
    string45,
    string46,
    string47,
    string48,
    string49,
    string50,
    string51,
    string52,
    string53,
    string54,
    string55,
    string56,
    string57,
    string58,
    string59,
    string60,
    string61,
    string62,
    string63,
    string64,
    string65,
    string66,
    string67,
    string68,
};