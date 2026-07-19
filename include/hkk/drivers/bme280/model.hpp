#pragma once
#include <hkk/defines.h>

namespace hkk::bme280 {

inline constexpr size_t CALIB00_DATA_FRAME            = (size_t)(26);
inline constexpr size_t CALIB26_DATA_FRAME            = (size_t)(7);

inline constexpr size_t JUMBO_DATA_FRAME_LENGTH       = (size_t)(20);
inline constexpr size_t DATA_FRAME_LENGTH             = (size_t)(16);

inline constexpr size_t PRESSURE_DATA_FRAME_LENGTH    = (size_t)(3);
inline constexpr size_t TEMPERATURE_DATA_FRAME_LENGTH = (size_t)(3);
inline constexpr size_t HUMIDITY_DATA_FRAME_LENGTH    = (size_t)(2);

inline constexpr uint8  OPERATON_MODE_MASK            = (uint8)(0x03);

inline constexpr float64 SATURATION_VAPOR_PRESSURE    = (float64)(611.2);
inline constexpr float64 MAGNUS_COEFFICIENT           = (float64)(17.62);
inline constexpr float64 TEMPERATURE_COEFFICIENT      = (float64)(243.5);
inline constexpr float64 WATER_MOLAR_MASS             = (float64)(0.01802);
inline constexpr float64 WATER_VAPOR_GAS_CONST        = (float64)(8.314);
inline constexpr float64 CELSIUS_KELVIN_OFFSET        = (float64)(273.15);


// 5.4 Register description
enum Command : uint8 {
    SoftReset           = 0xB6,
    OversamplingSkip    = 0x00,
    OversamplingX1      = 0x01,
    OversamplingX2      = 0x02,
    OversamplingX4      = 0x03,
    OversamplingX8      = 0x04,
    OversamplingX16     = 0x05,
    SleepMode           = 0x00,
    ForceMode           = 0x01,
    NormalMode          = 0x03,
    StandbyTime0_5      = 0x00,
    StandbyTime62_5     = 0x01,
    StandbyTime125      = 0x02,
    StandbyTime250      = 0x03,
    StandbyTime500      = 0x04,
    StandbyTime1000     = 0x05,
    StandbyTime10       = 0x06,
    StandbyTime20       = 0x07,
    FilterOff           = 0x00,
    FilterCoeff2        = 0x01,
    FilterCoeff4        = 0x02,
    FilterCoeff8        = 0x03,
    FilterCoeff16       = 0x04,
};

enum Status : uint8 {
    Measuring   = 0x08,
    ImUpdate    = 0x01,
    Ready       = 0x60,
};

// Table 18: Memory map
enum Register : uint8 {
    HumLsb    = 0xFE,
    HumMsb    = 0xFD,
    TempXlsb  = 0xFC,
    TempLsb   = 0xFB,
    TempMsb   = 0xFA,
    PressXlsb = 0xF9,
    PressLsb  = 0xF8,
    PressMsb  = 0xF7,
    ConfigReg = 0xF5,
    CtrlMeas  = 0xF4,
    Status    = 0xF3,
    CtrlHum   = 0xF2,
    Reset     = 0xE0,
    Id        = 0xD0,
    Calib00   = 0x88,
    Calib01   = 0x89,
    Calib02   = 0x8A,
    Calib03   = 0x8B,
    Calib04   = 0x8C,
    Calib05   = 0x8D,
    Calib06   = 0x8E,
    Calib07   = 0x8F,
    Calib08   = 0x90,
    Calib09   = 0x91,
    Calib10   = 0x92,
    Calib11   = 0x93,
    Calib12   = 0x94,
    Calib13   = 0x95,
    Calib14   = 0x96,
    Calib15   = 0x97,
    Calib16   = 0x98,
    Calib17   = 0x99,
    Calib18   = 0x9A,
    Calib19   = 0x9B,
    Calib20   = 0x9C,
    Calib21   = 0x9D,
    Calib22   = 0x9E,
    Calib23   = 0x9F,
    Calib24   = 0xA0,
    Calib25   = 0xA1,
    Calib26   = 0xE1,
    Calib27   = 0xE2,
    Calib28   = 0xE3,
    Calib29   = 0xE4,
    Calib30   = 0xE5,
    Calib31   = 0xE6,
    Calib32   = 0xE7,
    Calib33   = 0xE8,
    Calib34   = 0xE9,
    Calib35   = 0xEA,
    Calib36   = 0xEB,
    Calib37   = 0xEC,
    Calib38   = 0xED,
    Calib39   = 0xEE,
    Calib40   = 0xEF,
    Calib41   = 0xF0,
};

enum Result : int8 {
    BME280_OK                        =  0,
    BME280_ERROR_NULL_CONTEXT        = -1,
    BME280_ERROR_NULL_INSTANCE       = -2,
    BME280_ERROR_WRITE_FAILED        = -3,
    BME280_ERROR_I2C                 = -4,
    BME280_ERROR_I2C_TRANSACTION     = -5,
    BME280_ERROR_SENSOR_DISABLED     = -6,
    BME280_ERROR_READ_FAILED         = -7,
    BME280_ERROR_DEVICE_NOT_FOUND    = -8,
    BME280_ERROR_TIMEOUT             = -9,
    BME280_ERROR_NULL_DATA           = -10,
    BME280_ERROR_CRC                 = -11,
    BME280_ERROR_ZERO_LENGTH         = -12,
    BME280_ERROR_OOB                 = -13,
    BME280_ERROR_NVM                 = -14,
    BME280_ERROR_NVM_TRANSACTION     = -15,

    BME280_ERROR_GENERIC             = -100,
    BME280_FUNCTION_NOT_IMPLEMENTED  = -101,
    BME280_ERROR_UNKNOWN             = -102
};

enum class Vapor : uint8 {
    Pressure,
    Saturation,
    Deficit,
};

struct CalibrationParams {
    uint16 dig_T1;
    int16  dig_T2;
    int16  dig_T3;

    uint16 dig_P1;
    int16  dig_P2;
    int16  dig_P3;
    int16  dig_P4;
    int16  dig_P5;
    int16  dig_P6;
    int16  dig_P7;
    int16  dig_P8;
    int16  dig_P9;

    uint8  dig_H1;
    int16  dig_H2;
    uint8  dig_H3;
    int16  dig_H4;
    int16  dig_H5;
    int8   dig_H6;
};

struct Context {
    CalibrationParams calibration;

    float64 pressure;
    float64 temperature;
    float64 humidity;
    float64 absolute_humidity;
    float64 dew_point;

    float64 vapor_pressure;
    float64 saturation_vapor_pressure;
    float64 vapor_pressure_deficit;

    uint8 pressure_raw_data[PRESSURE_DATA_FRAME_LENGTH];
    uint8 temperature_raw_data[TEMPERATURE_DATA_FRAME_LENGTH];
    uint8 humidity_raw_data[HUMIDITY_DATA_FRAME_LENGTH];

    uint8 operation_mode;
    int8 status = BME280_OK;
};

struct Config {
    bool8 enable;
    uint8 address;

    uint8 humidity_sampling;
    uint8 iir_coefficient;
    uint8 temperature_pressure_mode;

    const char *name = nullptr;
    const char *location = nullptr;
};

}