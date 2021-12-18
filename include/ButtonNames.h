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
         68},
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

    constexpr uint8_t MaxNameLength = 11;

    constexpr char SW1[] PROGMEM = "L1";
    constexpr char SW2[] PROGMEM = "L2";
    constexpr char SW3[] PROGMEM = "L3";
    constexpr char SW4[] PROGMEM = "L4";
    constexpr char SW5[] PROGMEM = "L5";
    constexpr char SW6[] PROGMEM = "L6";
    constexpr char SW7[] PROGMEM = "MSG";
    constexpr char SW8[] PROGMEM = "DIR";
    constexpr char SW9[] PROGMEM = "IDX";
    constexpr char SW10[] PROGMEM = "TUN";
    constexpr char SW11[] PROGMEM = "A";
    constexpr char SW12[] PROGMEM = "H";
    constexpr char SW13[] PROGMEM = "O";
    constexpr char SW14[] PROGMEM = "V";
    constexpr char SW15[] PROGMEM = "FPLN";
    constexpr char SW16[] PROGMEM = "B";
    constexpr char SW17[] PROGMEM = "I";
    constexpr char SW18[] PROGMEM = "P";
    constexpr char SW19[] PROGMEM = "W";
    constexpr char SW20[] PROGMEM = "LEGS";
    constexpr char SW21[] PROGMEM = "C";
    constexpr char SW22[] PROGMEM = "J";
    constexpr char SW23[] PROGMEM = "Q";
    constexpr char SW24[] PROGMEM = "X";
    constexpr char SW25[] PROGMEM = "DEPARR";
    constexpr char SW26[] PROGMEM = "D";
    constexpr char SW27[] PROGMEM = "K";
    constexpr char SW28[] PROGMEM = "R";
    constexpr char SW29[] PROGMEM = "Y";
    constexpr char SW30[] PROGMEM = "PERF";
    constexpr char SW31[] PROGMEM = "E";
    constexpr char SW32[] PROGMEM = "L";
    constexpr char SW33[] PROGMEM = "S";
    constexpr char SW34[] PROGMEM = "Z";
    constexpr char SW35[] PROGMEM = "DSPL_MENU";
    constexpr char SW36[] PROGMEM = "F";
    constexpr char SW37[] PROGMEM = "M";
    constexpr char SW38[] PROGMEM = "T";
    constexpr char SW39[] PROGMEM = "SP";
    constexpr char SW40[] PROGMEM = "MFD_ADV";
    constexpr char SW41[] PROGMEM = "G";
    constexpr char SW42[] PROGMEM = "N";
    constexpr char SW43[] PROGMEM = "U";
    constexpr char SW44[] PROGMEM = "DIV";
    constexpr char SW45[] PROGMEM = "MFD_DATA";
    constexpr char SW46[] PROGMEM = "1";
    constexpr char SW47[] PROGMEM = "4";
    constexpr char SW48[] PROGMEM = "7";
    constexpr char SW49[] PROGMEM = "DOT";
    constexpr char SW50[] PROGMEM = "PREVPAGE";
    constexpr char SW51[] PROGMEM = "2";
    constexpr char SW52[] PROGMEM = "5";
    constexpr char SW53[] PROGMEM = "8";
    constexpr char SW54[] PROGMEM = "0";
    constexpr char SW55[] PROGMEM = "3";
    constexpr char SW56[] PROGMEM = "6";
    constexpr char SW57[] PROGMEM = "9";
    constexpr char SW58[] PROGMEM = "PLUSMINUS";
    constexpr char SW59[] PROGMEM = "R1";
    constexpr char SW60[] PROGMEM = "R2";
    constexpr char SW61[] PROGMEM = "R3";
    constexpr char SW62[] PROGMEM = "R4";
    constexpr char SW63[] PROGMEM = "R5";
    constexpr char SW64[] PROGMEM = "R6";
    constexpr char SW65[] PROGMEM = "EXEC";
    constexpr char SW66[] PROGMEM = "NEXTPAGE";
    constexpr char SW67[] PROGMEM = "CLR";
    constexpr char SW68[] PROGMEM = "BRT";
    constexpr char SW69[] PROGMEM = "DIM";

    const char *const Names[] PROGMEM = {
        SW1,
        SW2,
        SW3,
        SW4,
        SW5,
        SW6,
        SW7,
        SW8,
        SW9,
        SW10,
        SW11,
        SW12,
        SW13,
        SW14,
        SW15,
        SW16,
        SW17,
        SW18,
        SW19,
        SW20,
        SW21,
        SW22,
        SW23,
        SW24,
        SW25,
        SW26,
        SW27,
        SW28,
        SW29,
        SW30,
        SW31,
        SW32,
        SW33,
        SW34,
        SW35,
        SW36,
        SW37,
        SW38,
        SW39,
        SW40,
        SW41,
        SW42,
        SW43,
        SW44,
        SW45,
        SW46,
        SW47,
        SW48,
        SW49,
        SW50,
        SW51,
        SW52,
        SW53,
        SW54,
        SW55,
        SW56,
        SW57,
        SW58,
        SW59,
        SW60,
        SW61,
        SW62,
        SW63,
        SW64,
        SW65,
        SW66,
        SW67,
        SW68,
        SW69};
}