#pragma once
#include <hkk/defines.h>

namespace hkk::sgp30 {

enum class Command : uint16 {
    SoftReset                = 0x0006,
    IaqInit                  = 0x2003,
    MeasureIaq               = 0x2008,
    GetIaqBaseline           = 0x2015,
    SetIaqBaseline           = 0x201E,
    SetAbsoluteHumidity      = 0x2061, 
    MeasureTest              = 0x2032,
    GetFeatureSet            = 0x202F,
    MeasureRaw               = 0x2050,
    GetTvocInceptiveBaseline = 0x20B3,
    SetTvocBaseline          = 0x2077,
    GetSerialId              = 0x3682,
};

enum class CRC : uint8 {
    Mask  = 0x31,
    Base  = 0xFF,
    Msbit = 0x80,
};

enum class Measure : uint16 {
    Test       = 0xD400,
    FeatureSet = 0x0022,
};

}