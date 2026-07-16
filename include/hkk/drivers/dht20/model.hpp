#pragma once
#include <hkk/defines.h>

namespace hkk::dht20 {

inline constexpr size_t DATA_FRAME_LENGTH = (size_t)(7);

inline constexpr float64 SATURATION_VAPOR_PRESSURE = (float64)(611.2);
inline constexpr float64 MAGNUS_COEFFICIENT        = (float64)(17.62);
inline constexpr float64 TEMPERATURE_COEFFICIENT   = (float64)(243.5);
inline constexpr float64 WATER_MOLAR_MASS          = (float64)(0.01802);
inline constexpr float64 WATER_VAPOR_GAS_CONST     = (float64)(8.314);
inline constexpr float64 CELSIUS_KELVIN_OFFSET     = (float64)(273.15);

enum Command : uint8 {
    Init    = 0x71,
    Measure = 0xAC,  
};

enum class CRC : uint8 {
    Mask  = 0x31,
    Base  = 0xFF,
    Msbit = 0x80,
};

enum class Status : uint8 {
    Mask  = 0x18,
    Msbit = 0x80,
};

enum Register : uint8 {
    Calibration0 = 0x1B,
    Calibration1 = 0x1C,
    Calibration2 = 0x1D,

    Count = 3
};

enum DataMask : uint8 {
    Humidity    = 0xF0,
    Temperature = 0x0F, 
};

enum Result : int8 {
    DHT20_OK                        =  0,
    DHT20_ERROR_NULL_CONTEXT        = -1,
    DHT20_ERROR_NULL_INSTANCE       = -2,
    DHT20_ERROR_WRITE_FAILED        = -3,
    DHT20_ERROR_I2C                 = -4,
    DHT20_ERROR_I2C_TRANSACTION     = -5,
    DHT20_ERROR_SENSOR_DISABLED     = -6,
    DHT20_ERROR_READ_FAILED         = -7,
    DHT20_ERROR_DEVICE_NOT_FOUND    = -8,
    DHT20_ERROR_TIMEOUT             = -9,
    DHT20_ERROR_NULL_DATA           = -10,
    DHT20_ERROR_CRC                 = -11,
    DHT20_ERROR_ZERO_LENGTH         = -12,
    DHT20_ERROR_OOB                 = -13,
    DHT20_ERROR_NVM                 = -14,
    DHT20_ERROR_NVM_TRANSACTION     = -15,

    DHT20_ERROR_GENERIC             = -100,
    DHT20_FUNCTION_NOT_IMPLEMENTED  = -101,
    DHT20_ERROR_UNKNOWN             = -102
};

enum class Vapor : uint8 {
    Pressure,
    Saturation,
    Deficit,
};

struct Context {
    float64 temperature;
    float64 humidity;
    float64 absolute_humidity;
    float64 dew_point;

    float64 vapor_pressure;
    float64 saturation_vapor_pressure;
    float64 vapor_pressure_deficit;

    uint8 *raw_data[DATA_FRAME_LENGTH];

    int8 status = DHT20_OK;
};

struct Config {
    void        *nvm = nullptr;

    bool8       enable;
    uint8       address;
    const char  *name = nullptr;
    const char  *location = nullptr;
};

}