#pragma once

#include <Arduino.h>

// The methods for storing all this data in  comes from
// https://www.arduino.cc/reference/en/language/variables/utilities//

namespace ButtonNames
{
    static constexpr uint8_t RowColumnLUT[12][12] = {
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

    static constexpr char SW1[]  = "L1";
    static constexpr char SW2[]  = "L2";
    static constexpr char SW3[]  = "L3";
    static constexpr char SW4[]  = "L4";
    static constexpr char SW5[]  = "L5";
    static constexpr char SW6[]  = "L6";
    static constexpr char SW7[]  = "MSG";
    static constexpr char SW8[]  = "DIR";
    static constexpr char SW9[]  = "IDX";
    static constexpr char SW10[]  = "TUN";
    static constexpr char SW11[]  = "A";
    static constexpr char SW12[]  = "H";
    static constexpr char SW13[]  = "O";
    static constexpr char SW14[]  = "V";
    static constexpr char SW15[]  = "FPLN";
    static constexpr char SW16[]  = "B";
    static constexpr char SW17[]  = "I";
    static constexpr char SW18[]  = "P";
    static constexpr char SW19[]  = "W";
    static constexpr char SW20[]  = "LEGS";
    static constexpr char SW21[]  = "C";
    static constexpr char SW22[]  = "J";
    static constexpr char SW23[]  = "Q";
    static constexpr char SW24[]  = "X";
    static constexpr char SW25[]  = "DEPARR";
    static constexpr char SW26[]  = "D";
    static constexpr char SW27[]  = "K";
    static constexpr char SW28[]  = "R";
    static constexpr char SW29[]  = "Y";
    static constexpr char SW30[]  = "PERF";
    static constexpr char SW31[]  = "E";
    static constexpr char SW32[]  = "L";
    static constexpr char SW33[]  = "S";
    static constexpr char SW34[]  = "Z";
    static constexpr char SW35[]  = "DSPL_MENU";
    static constexpr char SW36[]  = "F";
    static constexpr char SW37[]  = "M";
    static constexpr char SW38[]  = "T";
    static constexpr char SW39[]  = "SP";
    static constexpr char SW40[]  = "MFD_ADV";
    static constexpr char SW41[]  = "G";
    static constexpr char SW42[]  = "N";
    static constexpr char SW43[]  = "U";
    static constexpr char SW44[]  = "DIV";
    static constexpr char SW45[]  = "MFD_DATA";
    static constexpr char SW46[]  = "1";
    static constexpr char SW47[]  = "4";
    static constexpr char SW48[]  = "7";
    static constexpr char SW49[]  = "DOT";
    static constexpr char SW50[]  = "PREVPAGE";
    static constexpr char SW51[]  = "2";
    static constexpr char SW52[]  = "5";
    static constexpr char SW53[]  = "8";
    static constexpr char SW54[]  = "0";
    static constexpr char SW55[]  = "3";
    static constexpr char SW56[]  = "6";
    static constexpr char SW57[]  = "9";
    static constexpr char SW58[]  = "PLUSMINUS";
    static constexpr char SW59[]  = "R1";
    static constexpr char SW60[]  = "R2";
    static constexpr char SW61[]  = "R3";
    static constexpr char SW62[]  = "R4";
    static constexpr char SW63[]  = "R5";
    static constexpr char SW64[]  = "R6";
    static constexpr char SW65[]  = "EXEC";
    static constexpr char SW66[]  = "NEXTPAGE";
    static constexpr char SW67[]  = "CLR";
    static constexpr char SW68[]  = "BRT";
    static constexpr char SW69[]  = "DIM";
    static constexpr char SW70[]  = "DEL";

    static constexpr uint8_t ButtonCount = 70;

    const char *const Names[ButtonCount]  = {
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