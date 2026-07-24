#pragma once
#include <hkk/defines.h>

#include <hkk/bus/i2c/i2c.hpp>
#include <hkk/utils/utils.hpp>

#include <hkk/drivers/dht20/model.hpp>

namespace hkk::dht20 {

class DHT20 {
public:
    DHT20(hkk::bus::i2c::I2C &i2c, const Config &cfg) : i2c(i2c), cfg(cfg) {}

    int8 init(void);
    int8 init(Context &res);

    int8 measure(void);
    int8 measure(Context &res);


    int8 setup(void);
    int8 setup(Context &res);

    int8 process(void);
    int8 process(Context &rest);

    int8 status(void);
    int8 status(Context &res);

    int8 soft_reset(void);
    int8 soft_reset(Context &res);

    int8 send_command(Command command);
    int8 send_command(uint8 address, Command command);
    
    int8 send_payload(uint8 *payload, size_t len);
    int8 send_payload(uint8 addr, uint8 *payload, size_t len);
    template<size_t N> int8 send_payload(uint8 addr, uint8 (&payload)[N]) {return send_payload(addr, payload, N);}
    template<size_t N> int8 send_payload(uint8 (&payload)[N]) {return send_payload(payload, N);}

    int8 validate_i2c_error(int8 status);
    int8 validate_nvm_error(int8 status);


    float32 humidity(bool8 absolute_humidity = false);
    void humidity(Context &res);

    float32 temperature(void);
    void temperature(Context &res);

    float32 dew_point(void);
    void dew_point(Context &res);

    float32 vapor(Vapor type);
    void vapor(Context &res);

private:
    hkk::bus::i2c::I2C &i2c;
    
    Config cfg;
    Context ctx{};


    int8 read_raw_data(uint8 *data, size_t len);

    float32 calculate_absolute_humidity(void);
    float32 calculate_absolute_humidity(float32 humidity, float32 temperature);
    float32 calculate_absolute_humidity(Context &res);
    
    float32 calculate_dew_point(void);
    float32 calculate_dew_point(float32 humidity, float32 temperature);
    float32 calculate_dew_point(Context &res);

    static int8 crc_calculate(uint8 &checksum, uint8 *data, size_t len);
    template<size_t N> static int8 crc_calculate(uint8 &checksum, uint8 (&data)[N]) {return crc_calculate(checksum, data, N);}
    template<size_t N> static int8 crc_calculate(uint8 (&data)[N]) {return crc_calculate(nullptr, data, N);}

    static int8 crc_validate(uint8 *data, size_t len);
    template<size_t N> static int8 crc_validate(uint8 (&data)[N]) {return crc_validate(data, N);}

    hkk::utils::AlarmContext sensor_timeout_alarm{};
    bool8 sensor_timeout = false;
    static bool8 sensor_timeout_callback(void *data);

    int8 sensor_enabled(void) {
        if(!this->cfg.enable) {
            HINFO("[DHT20  ] DHT20 sensor on I2C%d bus is disabled in the config file", i2c.index());
            return DHT20_ERROR_SENSOR_DISABLED;
        } else return DHT20_OK;
    }
};

}