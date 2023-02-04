#pragma once

#include <Arduino.h>

namespace ButtonNames
{
    static constexpr uint8_t MaxNameLength = 11;

    static constexpr char SW1[] = "NEXT";
    static constexpr char SW2[] = "PREV";
    static constexpr char SW3[] = "MFD_DATA";
    static constexpr char SW4[] = "MFD_ADV";
    static constexpr char SW5[] = "DSPL_MENU";
    static constexpr char SW6[] = "PERF";
    static constexpr char SW7[] = "DEP_ARR";
    static constexpr char SW8[] = "LEGS";
    static constexpr char SW9[] = "FPLN";
    static constexpr char SW10[] = "DEL";
    static constexpr char SW11[] = "3";
    static constexpr char SW12[] = "2";
    static constexpr char SW13[] = "1";
    static constexpr char SW14[] = "G";
    static constexpr char SW15[] = "F";
    static constexpr char SW16[] = "E";
    static constexpr char SW17[] = "D";
    static constexpr char SW18[] = "C";
    static constexpr char SW19[] = "B";
    static constexpr char SW20[] = "A";
    static constexpr char SW21[] = "6";
    static constexpr char SW22[] = "5";
    static constexpr char SW23[] = "4";
    static constexpr char SW24[] = "N";
    static constexpr char SW25[] = "M";
    static constexpr char SW26[] = "L";
    static constexpr char SW27[] = "K";
    static constexpr char SW28[] = "J";
    static constexpr char SW29[] = "I";
    static constexpr char SW30[] = "H";
    static constexpr char SW31[] = "9";
    static constexpr char SW32[] = "8";
    static constexpr char SW33[] = "7";
    static constexpr char SW34[] = "U";
    static constexpr char SW35[] = "T";
    static constexpr char SW36[] = "S";
    static constexpr char SW37[] = "R";
    static constexpr char SW38[] = "Q";
    static constexpr char SW39[] = "P";
    static constexpr char SW40[] = "O";
    static constexpr char SW41[] = "PLUSMINUS";
    static constexpr char SW42[] = "0";
    static constexpr char SW43[] = "DOT";
    static constexpr char SW44[] = "DIV";
    static constexpr char SW45[] = "SP";
    static constexpr char SW46[] = "Z";
    static constexpr char SW47[] = "Y";
    static constexpr char SW48[] = "X";
    static constexpr char SW49[] = "W";
    static constexpr char SW50[] = "V";
    static constexpr char SW51[] = "R6";
    static constexpr char SW52[] = "EXEC";
    static constexpr char SW53[] = "DIM";
    static constexpr char SW54[] = "CLR";
    static constexpr char SW55[] = "BRT";
    static constexpr char SW56[] = "TUN";
    static constexpr char SW57[] = "IDX";
    static constexpr char SW58[] = "DIR";
    static constexpr char SW59[] = "MSG";
    static constexpr char SW60[] = "L6";
    static constexpr char SW61[] = "R1";
    static constexpr char SW62[] = "R2";
    static constexpr char SW63[] = "R3";
    static constexpr char SW64[] = "R4";
    static constexpr char SW65[] = "R5";
    static constexpr char SW66[] = "L5";
    static constexpr char SW67[] = "L4";
    static constexpr char SW68[] = "L3";
    static constexpr char SW69[] = "L2";
    static constexpr char SW70[] = "L1";

    static constexpr uint8_t ButtonCount = 70;

    const char *const Names[ButtonCount] = {
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