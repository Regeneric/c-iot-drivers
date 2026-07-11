#pragma once
#include <hkk/defines.h>

#include <hkk/bus/i2c/i2c.hpp>

#include <hkk/drivers/sgp30/model.hpp>

namespace hkk::sgp30 {

class SGP30 {
public:
    SGP30(hkk::bus::I2C &i2c, const Config &cfg) : i2c(i2c), cfg(cfg) {}

    int8 setup(void);
    int8 setup(uint8 *data, size_t len);
    template <size_t N>
    int8 setup(uint8 (&data)[N]) {
        return setup(data, N); 
    }

    int8 send_command(Command command);

    int8 send_payload(uint8 *payload, size_t len);
    template <size_t N>
    int8 send_payload(uint8 (&payload)[N]) {
        return send_payload(payload, N);
    }

    int8 iaq_init();
    int8 measure_iaq(Context &result);
    int8 get_iaq_baseline(Context &result);

    int8 set_iaq_baseline(uint8 *baseline, size_t len);
    template <size_t N>
    int8 set_iaq_baseline(uint8 (&baseline)[N]) {
        return set_iaq_baseline(baseline, N);
    }
    
    int8 set_absolute_humidity(uint8 *humidity, size_t len);
    template <size_t N>
    int8 set_absolute_humidity(uint8 (&humidity)[N]) {
        return set_absolute_humidity(humidity, N);
    }

    int8 measure_test(Context &result);
    int8 feature_set(Context &result);
    int8 measure_raw(Context &result);
    int8 get_tvoc_inceptive_baseline(Context &result);

    int8 set_tvoc_baseline(uint8 *baseline, size_t len);
    template <size_t N>
    int8 set_tvoc_baseline(uint8 (&baseline)[N]) {
        return set_tvoc_baseline(baseline, N);
    }

    int8 soft_reset();
    int8 get_serial_number(Context &result);

    int8 compensate_humidity(float32 absolute_humidity);    
    int8 calibrate();

private:
    hkk::bus::I2C &i2c;
    const Config cfg;

    int8 data_frame(Command command, uint8 *data, size_t len);
    int8 read_raw_data(uint8 *data, size_t len);
    int8 sensor_enabled() {
        if(!this->cfg.enable) {
            HINFO("[SGP30  ] SGP30 sensor on I2C%d bus is disabled in the config file", i2c.index());
            return SGP30_ERROR_SENSOR_DISABLED;
        } else return SGP30_OK;
    }
};

}