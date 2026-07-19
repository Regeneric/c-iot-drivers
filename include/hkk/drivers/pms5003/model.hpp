#pragma once

#include <hkk/defines.h>

namespace hkk::pms5003 {

inline constexpr size_t DATA_FRAME_LENGTH    = (size_t)(32);
inline constexpr size_t COMMAND_FRAME_LENGTH = (size_t)(8);
inline constexpr size_t SECTION_FRAME_LENGTH = (size_t)(2);

inline constexpr uint8 START_BYTE0 = (uint8)(0x42);
inline constexpr uint8 START_BYTE1 = (uint8)(0x4D);

inline constexpr float32 PM_GAIN       =  0.524f;
inline constexpr float32 RH_GAIN       =  0.0862f;
inline constexpr float32 TEMP_GAIN     =  0.0753f;
inline constexpr float32 PM_OFFSET     =  5.75f;
inline constexpr float32 ROUNDING_BIAS =  0.5f;


enum Command : uint8 {
    ChangeMode = 0xE1,
    Read       = 0xE2,
    SetSleep   = 0xE4,
};

enum Mode : uint8 {
    Passive = 0x00,
    Active  = 0x01,
};

enum Power : uint8 {
    Sleep   = 0x00,
    Wakeup  = 0x01,
};

enum Result : int8 {
    PMS5003_OK                        =  0,
    PMS5003_ERROR_NULL_CONTEXT        = -1,
    PMS5003_ERROR_NULL_INSTANCE       = -2,
    PMS5003_ERROR_WRITE_FAILED        = -3,
    PMS5003_ERROR_I2C                 = -4,
    PMS5003_ERROR_I2C_TRANSACTION     = -5,
    PMS5003_ERROR_SENSOR_DISABLED     = -6,
    PMS5003_ERROR_READ_FAILED         = -7,
    PMS5003_ERROR_DEVICE_NOT_FOUND    = -8,
    PMS5003_ERROR_TIMEOUT             = -9,
    PMS5003_ERROR_NULL_DATA           = -10,
    PMS5003_ERROR_CRC                 = -11,
    PMS5003_ERROR_ZERO_LENGTH         = -12,
    PMS5003_ERROR_OOB                 = -13,
    PMS5003_ERROR_NVM                 = -14,
    PMS5003_ERROR_NVM_TRANSACTION     = -15,

    PMS5003_ERROR_GENERIC             = -100,
    PMS5003_FUNCTION_NOT_IMPLEMENTED  = -101,
    PMS5003_ERROR_UNKNOWN             = -102
};

struct Context {
    uint16 pm1;
    uint16 pm2_5;
    uint16 pm10;

    uint16 pm1_ref;
    uint16 pm2_5_ref;
    uint16 pm10_ref;

    Mode  operation_mode;
    Power sleep_mode;

    uint8 raw_data[DATA_FRAME_LENGTH];

    uint8 raw_pc_0_3[SECTION_FRAME_LENGTH];
    uint8 raw_pc_0_5[SECTION_FRAME_LENGTH];
    uint8 raw_pc_1_0[SECTION_FRAME_LENGTH];
    uint8 raw_pc_2_5[SECTION_FRAME_LENGTH];
    uint8 raw_pc_5_0[SECTION_FRAME_LENGTH];
    uint8 raw_pc_10_0[SECTION_FRAME_LENGTH];

    int8 status = PMS5003_OK;
};

struct Config {
    void        *nvm = nullptr;

    bool8       enable;
    bool8       compensate_humidity;

    Mode        operation_mode = Mode::Passive;
    Power       sleep_mode = Power::Wakeup;

    const char  *name = nullptr;
    const char  *location = nullptr;
};
    
}