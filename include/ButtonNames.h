#pragma once

#include <Arduino.h>

// The methods for storing all this data in PROGMEM comes from
// https://www.arduino.cc/reference/en/language/variables/utilities/progmem/

namespace ButtonNames
{
    constexpr uint8_t RowColumnLUT[12][12] PROGMEM = {
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
         59},
        {2,
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
         60},
        {3,
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
         61},
        {4,
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
         62},
        {5,
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
         63},
        {6,
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
         64},
        {7,
         255,
         14,
         19,
         24,
         29,
         34,
         39,
         44,
         49,
         255,
         65},
        {8,
         10,
         15,
         20,
         25,
         30,
         35,
         40,
         45,
         50,
         54,
         66},
        {9,
         11,
         16,
         21,
         26,
         31,
         36,
         41,
         46,
         51,
         55,
         67},
        {255,
         12,
         17,
         22,
         27,
         32,
         37,
         42,
         47,
         52,
         56,
         69},
        {255,
         13,
         18,
         23,
         28,
         33,
         38,
         43,
         48,
         53,
         57,
         255}};

    constexpr uint8_t MaxNameLength = 5;

    constexpr char string0[] PROGMEM = "LSK1";
    constexpr char string1[] PROGMEM = "LSK2";
    constexpr char string2[] PROGMEM = "LSK3";
    constexpr char string3[] PROGMEM = "LSK4";
    constexpr char string4[] PROGMEM = "LSK5";
    constexpr char string5[] PROGMEM = "LSK6";
    constexpr char string6[] PROGMEM = "MSG";
    constexpr char string7[] PROGMEM = "DIR";
    constexpr char string8[] PROGMEM = "IDX";
    constexpr char string9[] PROGMEM = "TUN";
    constexpr char string10[] PROGMEM = "A";
    constexpr char string11[] PROGMEM = "H";
    constexpr char string12[] PROGMEM = "O";
    constexpr char string13[] PROGMEM = "V";
    constexpr char string14[] PROGMEM = "FPLN";
    constexpr char string15[] PROGMEM = "B";
    constexpr char string16[] PROGMEM = "I";
    constexpr char string17[] PROGMEM = "P";
    constexpr char string18[] PROGMEM = "W";
    constexpr char string19[] PROGMEM = "LEGS";
    constexpr char string20[] PROGMEM = "C";
    constexpr char string21[] PROGMEM = "J";
    constexpr char string22[] PROGMEM = "Q";
    constexpr char string23[] PROGMEM = "X";
    constexpr char string24[] PROGMEM = "DEP";
    constexpr char string25[] PROGMEM = "D";
    constexpr char string26[] PROGMEM = "K";
    constexpr char string27[] PROGMEM = "R";
    constexpr char string28[] PROGMEM = "Y";
    constexpr char string29[] PROGMEM = "PERF";
    constexpr char string30[] PROGMEM = "E";
    constexpr char string31[] PROGMEM = "L";
    constexpr char string32[] PROGMEM = "S";
    constexpr char string33[] PROGMEM = "Z";
    constexpr char string34[] PROGMEM = "DSPL";
    constexpr char string35[] PROGMEM = "F";
    constexpr char string36[] PROGMEM = "M";
    constexpr char string37[] PROGMEM = "T";
    constexpr char string38[] PROGMEM = "SP";
    constexpr char string39[] PROGMEM = "ADV";
    constexpr char string40[] PROGMEM = "G";
    constexpr char string41[] PROGMEM = "N";
    constexpr char string42[] PROGMEM = "U";
    constexpr char string43[] PROGMEM = "SLSH";
    constexpr char string44[] PROGMEM = "DATA";
    constexpr char string45[] PROGMEM = "1";
    constexpr char string46[] PROGMEM = "4";
    constexpr char string47[] PROGMEM = "7";
    constexpr char string48[] PROGMEM = "DOT";
    constexpr char string49[] PROGMEM = "PREV";
    constexpr char string50[] PROGMEM = "2";
    constexpr char string51[] PROGMEM = "5";
    constexpr char string52[] PROGMEM = "8";
    constexpr char string53[] PROGMEM = "0";
    constexpr char string54[] PROGMEM = "3";
    constexpr char string55[] PROGMEM = "6";
    constexpr char string56[] PROGMEM = "9";
    constexpr char string57[] PROGMEM = "PLUS";
    constexpr char string58[] PROGMEM = "RSK1";
    constexpr char string59[] PROGMEM = "RSK2";
    constexpr char string60[] PROGMEM = "RSK3";
    constexpr char string61[] PROGMEM = "RSK4";
    constexpr char string62[] PROGMEM = "RSK5";
    constexpr char string63[] PROGMEM = "RSK6";
    constexpr char string64[] PROGMEM = "EXEC";
    constexpr char string65[] PROGMEM = "NEXT";
    constexpr char string66[] PROGMEM = "CLR";
    constexpr char string67[] PROGMEM = "BRT";
    constexpr char string68[] PROGMEM = "DIM";

    const char *const Names[] PROGMEM = {
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
}