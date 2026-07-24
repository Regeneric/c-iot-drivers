#pragma once
#include <hkk/defines.h>

#include <hkk/bus/spi/spi.hpp>
#include <hkk/bus/i2c/i2c.hpp>
#include <hkk/utils/utils.hpp>

#include <hkk/drivers/bme280/model.hpp>

namespace hkk::bme280 {

class BME280 {
public:
    BME280(hkk::bus::i2c::I2C &i2c, const Config &cfg) : i2c(&i2c), spi(nullptr), cfg(cfg) {}
    BME280(hkk::bus::spi::SPI &spi, const Config &cfg) : i2c(nullptr), spi(&spi), cfg(cfg) {}

    int8 init(void);
    int8 init(Context &res);

    int8 measure(void);
    int8 measure(Context &res);

    int8 calibrate(void);
    int8 calibrate(Context &res);

    int8 get_sensor_id(void);
    int8 get_sensor_id(Context &res);


    int8 setup(void);
    int8 setup(Context &res);

    int8 process(void);
    int8 process(Context &res);

    int8 status(void);
    int8 status(Context &res);

    int8 soft_reset(void);
    int8 soft_reset(Context &res);

    int8 write_register(Register reg, uint8 byte);
    int8 write_register(Register reg, uint8 *src, size_t len);
    int8 write_register(uint8 addr, Register reg, uint8 *src, size_t len);
    template<size_t N> int8 write_register(Register reg, uint8 (&src)[N]) {return write_register(reg, src, N);}
    template<size_t N> int8 write_register(uint8 addr, Register reg, uint8 (&src)[N]) {return write_register(addr, reg, src, N);}

    int8 read_register(Register reg, uint8 &byte);
    int8 read_register(Register reg, uint8 *dst, size_t len);
    int8 read_register(uint8 addr, Register reg, uint8 *dst, size_t len);
    template<size_t N> int8 read_register(Register reg, uint8 (&dst)[N]) {return read_register(reg, dst, N);}
    template<size_t N> int8 read_register(uint8 addr, Register reg, uint8 (&dst)[N]) {return read_register(addr, reg, dst, N);}


    float32 humidity(bool8 absolute_humidity = false);
    void humidity(Context &res);

    float32 temperature(void);
    void temperature(Context &res);

    float32 pressure(void);
    void pressure(Context &res);

    float32 dew_point(void);
    void dew_point(Context &res);

    float32 vapor(Vapor type);
    void vapor(Context &res);

private:
    hkk::bus::i2c::I2C *i2c;
    hkk::bus::spi::SPI *spi;
    
    Config cfg;
    Context ctx;

    int8 write_bus(uint8 byte, bool8 nostop = false);
    int8 write_bus(uint8 *src, size_t len, bool8 nostop = false);
    int8 write_bus(uint8 addr, uint8 *src, size_t len, bool8 nostop = false);

    int8 read_bus(uint8 *dst, size_t len, bool8 nostop = false);
    int8 read_bus(uint8 addr, uint8 *dst, size_t len, bool8 nostop = false);

    static int32  compensate_temperature(const Context &res, int32 &t_fine, int32 raw_temperature);
    static uint32 compensate_pressure(const Context &res, int32 t_fine, int32 raw_pressure);
    static uint32 compensate_humidity(const Context &res, int32 t_fine, int32 raw_humidity); 

    float32 calculate_absolute_humidity(void);
    float32 calculate_absolute_humidity(float32 humidity, float32 temperature);
    float32 calculate_absolute_humidity(Context &res);
    
    float32 calculate_dew_point(void);
    float32 calculate_dew_point(float32 humidity, float32 temperature);
    float32 calculate_dew_point(Context &res);

    hkk::utils::AlarmContext sensor_timeout_alarm{};
    bool8 sensor_timeout = false;
    static bool8 sensor_timeout_callback(void *data);

    int8 sensor_enabled(void) {
        if(!this->cfg.enable) {
            HINFO("[BME280 ] BME280 sensor disabled in the config file");
            return BME280_ERROR_SENSOR_DISABLED;
        } else return BME280_OK;
    }

    int8 validate_i2c_error(int8 error);
};

}