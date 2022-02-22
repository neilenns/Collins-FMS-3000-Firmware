#pragma once

#include <Arduino.h>

// The methods for storing all this data in PROGMEM comes from
// https://www.arduino.cc/reference/en/language/variables/utilities/progmem/

namespace ButtonNames
{
    static constexpr uint8_t RowColumnLUT[12][12] PROGMEM = {
        {0,
         69, // This is a special entry for the press-and-hold action on the CLR key to send DEL.
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

    static constexpr uint8_t MaxNameLength = 11;

    static constexpr char SW1[] PROGMEM = "L1";
    static constexpr char SW2[] PROGMEM = "L2";
    static constexpr char SW3[] PROGMEM = "L3";
    static constexpr char SW4[] PROGMEM = "L4";
    static constexpr char SW5[] PROGMEM = "L5";
    static constexpr char SW6[] PROGMEM = "L6";
    static constexpr char SW7[] PROGMEM = "MSG";
    static constexpr char SW8[] PROGMEM = "DIR";
    static constexpr char SW9[] PROGMEM = "IDX";
    static constexpr char SW10[] PROGMEM = "TUN";
    static constexpr char SW11[] PROGMEM = "A";
    static constexpr char SW12[] PROGMEM = "H";
    static constexpr char SW13[] PROGMEM = "O";
    static constexpr char SW14[] PROGMEM = "V";
    static constexpr char SW15[] PROGMEM = "FPLN";
    static constexpr char SW16[] PROGMEM = "B";
    static constexpr char SW17[] PROGMEM = "I";
    static constexpr char SW18[] PROGMEM = "P";
    static constexpr char SW19[] PROGMEM = "W";
    static constexpr char SW20[] PROGMEM = "LEGS";
    static constexpr char SW21[] PROGMEM = "C";
    static constexpr char SW22[] PROGMEM = "J";
    static constexpr char SW23[] PROGMEM = "Q";
    static constexpr char SW24[] PROGMEM = "X";
    static constexpr char SW25[] PROGMEM = "DEPARR";
    static constexpr char SW26[] PROGMEM = "D";
    static constexpr char SW27[] PROGMEM = "K";
    static constexpr char SW28[] PROGMEM = "R";
    static constexpr char SW29[] PROGMEM = "Y";
    static constexpr char SW30[] PROGMEM = "PERF";
    static constexpr char SW31[] PROGMEM = "E";
    static constexpr char SW32[] PROGMEM = "L";
    static constexpr char SW33[] PROGMEM = "S";
    static constexpr char SW34[] PROGMEM = "Z";
    static constexpr char SW35[] PROGMEM = "DSPL_MENU";
    static constexpr char SW36[] PROGMEM = "F";
    static constexpr char SW37[] PROGMEM = "M";
    static constexpr char SW38[] PROGMEM = "T";
    static constexpr char SW39[] PROGMEM = "SP";
    static constexpr char SW40[] PROGMEM = "MFD_ADV";
    static constexpr char SW41[] PROGMEM = "G";
    static constexpr char SW42[] PROGMEM = "N";
    static constexpr char SW43[] PROGMEM = "U";
    static constexpr char SW44[] PROGMEM = "DIV";
    static constexpr char SW45[] PROGMEM = "MFD_DATA";
    static constexpr char SW46[] PROGMEM = "1";
    static constexpr char SW47[] PROGMEM = "4";
    static constexpr char SW48[] PROGMEM = "7";
    static constexpr char SW49[] PROGMEM = "DOT";
    static constexpr char SW50[] PROGMEM = "PREVPAGE";
    static constexpr char SW51[] PROGMEM = "2";
    static constexpr char SW52[] PROGMEM = "5";
    static constexpr char SW53[] PROGMEM = "8";
    static constexpr char SW54[] PROGMEM = "0";
    static constexpr char SW55[] PROGMEM = "3";
    static constexpr char SW56[] PROGMEM = "6";
    static constexpr char SW57[] PROGMEM = "9";
    static constexpr char SW58[] PROGMEM = "PLUSMINUS";
    static constexpr char SW59[] PROGMEM = "R1";
    static constexpr char SW60[] PROGMEM = "R2";
    static constexpr char SW61[] PROGMEM = "R3";
    static constexpr char SW62[] PROGMEM = "R4";
    static constexpr char SW63[] PROGMEM = "R5";
    static constexpr char SW64[] PROGMEM = "R6";
    static constexpr char SW65[] PROGMEM = "EXEC";
    static constexpr char SW66[] PROGMEM = "NEXTPAGE";
    static constexpr char SW67[] PROGMEM = "CLR";
    static constexpr char SW68[] PROGMEM = "BRT";
    static constexpr char SW69[] PROGMEM = "DIM";
    static constexpr char SW70[] PROGMEM = "DEL";

    static constexpr uint8_t ButtonCount = 70;

    const char *const Names[ButtonCount] PROGMEM = {
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
        SW69,
        SW70};
}