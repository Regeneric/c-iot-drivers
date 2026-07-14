#pragma once
#include <hkk/defines.h>

#include <hkk/bus/i2c/i2c.hpp>

#include <hkk/drivers/sgp30/model.hpp>

namespace hkk::sgp30 {

class SGP30 {
public:
    SGP30(hkk::bus::i2c::I2C &i2c, const Config &cfg) : i2c(i2c), cfg(cfg) {}

    // === Commands --------
    int8 iaq_init(void);

    int8 measure_iaq(void);
    int8 measure_iaq(Context &result);
    
    int8 get_iaq_baseline(void);;
    int8 get_iaq_baseline(Context &result);

    int8 set_iaq_baseline(Context &result);
    int8 set_iaq_baseline(uint8 *baseline, size_t len);
    template<size_t N> int8 set_iaq_baseline(uint8 (&baseline)[N]) {return set_iaq_baseline(baseline, N);}
    
    int8 set_absolute_humidity(Context &result);
    int8 set_absolute_humidity(uint8 *humidity, size_t len);
    template<size_t N> int8 set_absolute_humidity(uint8 (&humidity)[N]) {return set_absolute_humidity(humidity, N);}

    int8 measure_test(void);
    int8 measure_test(Context &result);

    int8 feature_set(void);
    int8 feature_set(Context &result);

    int8 measure_raw(void);
    int8 measure_raw(Context &result);

    int8 get_tvoc_inceptive_baseline(void);
    int8 get_tvoc_inceptive_baseline(Context &result);

    int8 set_tvoc_baseline(Context &result);
    int8 set_tvoc_baseline(uint8 *baseline, size_t len);
    template<size_t N> int8 set_tvoc_baseline(uint8 (&baseline)[N]) {return set_tvoc_baseline(baseline, N);}

    int8 soft_reset(void);
    
    int8 get_serial_number(void);
    int8 get_serial_number(Context &result);
    // ---------------------

    
    int8 setup(void);
    int8 setup(Context &result);

    int8 process(void);
    int8 process(Context &result);

    int8 send_command(Command command);

    int8 status(void);
    int8 status(Context &result);

    int8 send_payload(uint8 *payload, size_t len);
    template<size_t N> int8 send_payload(uint8 (&payload)[N]) {return send_payload(payload, N);}

    int8 compensate_humidity(Context &result);
    int8 compensate_humidity(float32 absolute_humidity);    
    
    int8 calibrate(void);
    int8 calibrate(Context &result);
    
    int8 store_baseline(void);
    int8 store_baseline(Context &result);

    int8 load_baseline(void);
    int8 load_baseline(Context &result);
    

    uint32 eco2(void);
    void   eco2(Context &result);

    uint32 tvoc(void);
    void   tvoc(Context &result);

    uint32 h2(void);
    void   h2(Context &result);

    uint32 c2h6o(void);
    void   c2h6o(Context &result);

    uint8 *baseline(bool8 tvoc_baseline = false);
    void   baseline(Context &result, bool8 tvoc_baseline = false);

    uint8 *test(void);
    void   test(Context &result);

    uint8 *serial_number(void);
    void   serial_number(Context &result);

private:
    hkk::bus::i2c::I2C &i2c;
    
    const Config cfg;
    Context ctx;

    int8 sensor_enabled(void) {
        if(!this->cfg.enable) {
            HINFO("[SGP30  ] SGP30 sensor on I2C%d bus is disabled in the config file", i2c.index());
            return SGP30_ERROR_SENSOR_DISABLED;
        } else return SGP30_OK;
    }

    int8 data_frame(Command command, uint8 *data, size_t len);
    int8 read_raw_data(uint8 *data, size_t len);
    int8 validate_error(int8 error);
    int8 baseline_lookup(uint8 *baseline);

    
    bool8 store_baseline_request;
    bool8 sensor_calibrated;

    static bool8 baseline_store_callback(void *data);
    static bool8 sensor_calibrated_callback(void *data);
};

}