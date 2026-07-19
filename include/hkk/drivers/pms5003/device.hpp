#pragma once

#include <hkk/bus/uart/uart.hpp>

#include <hkk/drivers/pms5003/model.hpp>

namespace hkk::pms5003 {

class PMS5003 {
public:
    PMS5003(hkk::bus::uart::UART &uart, const Config &cfg) : uart(uart), cfg(cfg) {}

    int8 measure(void);
    int8 measure(Context &res);


    int8 setup(void);
    int8 setup(Context &res);

    int8 process(void);
    int8 process(Context &res);

    int8 mode(Mode mode);
    int8 mode(Context &res);

    int8 sleep(Power mode);
    int8 sleep(Context &res);

    int8 send_command(Command command, uint8 value);


    uint16 pm1(bool8 ref = false);
    void pm1(Context &res, bool8 ref = false);

    uint16 pm2_5(bool8 ref = false);
    void pm2_5(Context &res, bool8 ref = false);

    uint16 pm10(bool8 ref = false);
    void pm10(Context &res, bool8 ref = false);

private:
    hkk::bus::uart::UART &uart;
    const Config cfg;

    Context ctx{};

    int8 read_raw_data(uint8 *data, size_t len);
    int8 validate_uart_error(int8 status);
    
    int8 compensate_humidity(Context &res, float64 humidity, float64 temperature);

    static int8 crc_calculate(uint16 &checksum, uint8 *data, size_t len);
    static int8 crc_validate(uint8 *data, size_t len);

    int8 sensor_enabled(void) {
        if(!this->cfg.enable) {
            HINFO("[PMS5003] PMS5003 sensor is disabled in the config file");
            return PMS5003_ERROR_SENSOR_DISABLED;
        } else return PMS5003_OK;
    }
};
    
}