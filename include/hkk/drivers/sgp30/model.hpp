#pragma once
#include <hkk/defines.h>

namespace hkk::sgp30 {

inline constexpr size_t DATA_FRAME_LENGTH       = (size_t)(6);
inline constexpr size_t HALF_DATA_FRAME_LENGTH  = (DATA_FRAME_LENGTH / (size_t)(2));
inline constexpr size_t JUMBO_DATA_FRAME_LENGTH = (DATA_FRAME_LENGTH + HALF_DATA_FRAME_LENGTH); 
inline constexpr size_t COMMAND_FRAME_LENGTH    = (size_t)(2);

enum Command : uint16 {
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

enum CRC : uint8 {
    Mask  = 0x31,
    Base  = 0xFF,
    Msbit = 0x80,
};

enum Measure : uint16 {
    TestValue  = 0xD400,
    FeatureSet = 0x0022,
};

enum Result : int8 {
    SGP30_OK                        =  0,
    SGP30_ERROR_NULL_CONTEXT        = -1,
    SGP30_ERROR_NULL_INSTANCE       = -2,
    SGP30_ERROR_WRITE_FAILED        = -3,
    SGP30_ERROR_I2C                 = -4,
    SGP30_ERROR_I2C_TRANSACTION     = -5,
    SGP30_ERROR_SENSOR_DISABLED     = -6,
    SGP30_ERROR_READ_FAILED         = -7,
    SGP30_ERROR_DEVICE_NOT_FOUND    = -8,
    SGP30_ERROR_TIMEOUT             = -9,
    SGP30_ERROR_NULL_DATA           = -10,
    SGP30_ERROR_CRC                 = -11,
    SGP30_ERROR_ZERO_LENGTH         = -12,
    SGP30_ERROR_OOB                 = -13,
    SGP30_ERROR_NVM                 = -14,
    SGP30_ERROR_NVM_TRANSACTION     = -15,
    
    SGP30_ERROR_GENERIC             = -100,
    SGP30_FUNCTION_NOT_IMPLEMENTED  = -101,
    SGP30_ERROR_UNKNOWN             = -102
};


struct Context {
    uint32  tvoc;
    uint32  eco2;
    uint32  h2;
    uint32  c2h6o;
    float32 absolute_humidity;

    bool8  calibrated = false;

    uint8  raw_absolute_humidity[HALF_DATA_FRAME_LENGTH];
    uint8  tvoc_baseline[HALF_DATA_FRAME_LENGTH];
    uint8  measure_test[HALF_DATA_FRAME_LENGTH];
    uint8  feature_set[HALF_DATA_FRAME_LENGTH];

    uint8  raw_data[DATA_FRAME_LENGTH];
    uint8  baseline[DATA_FRAME_LENGTH];

    uint8  serial_number[JUMBO_DATA_FRAME_LENGTH];

    int8   status = SGP30_OK;
};


struct Config {
    void        *nvm = nullptr;
    void        *timer = nullptr;

    bool8       enable; 
    bool8       humid_compensation;
    uint8       address = 0x58;
    const char  *name = nullptr;
    const char  *location = nullptr;
};

}