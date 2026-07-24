#include <hkk/defines.h>

#include <hkk/utils/utils.hpp>
#include <hkk/logger/logger.h>
#include <hkk/bus/i2c/i2c.hpp>

#include <hkk/drivers/dht20/dht20.hpp>

#include <cmath>
#include <cstring>

namespace hkk::dht20 {

// 7.4 Sensor Reading Process
int8 DHT20::init(void) {
    HTRACE("commands.cpp -> DHT20::init(-):int8");
    return this->init(this->ctx);
}

int8 DHT20::init(Context &res) {
    HTRACE("commands.cpp -> DHT20::init(Context&):int8");
    if(int8 status = this->sensor_enabled(); status < DHT20_OK) return res.status = status;

    int8 status = DHT20_OK;

    status = this->send_command(Command::Init);
    if(status < DHT20_OK) {
        HERROR("[DHT20  ] Could not send command to sensor %s (%s)", this->cfg.name, this->cfg.location);
        return res.status = status;
    }

    uint8 sensor_status;
    status = this->read_raw_data(&sensor_status, 1);
    if(status < DHT20_OK) return res.status = status;
    
    if((sensor_status & static_cast<uint8>(Status::Mask)) != static_cast<uint8>(Status::Mask)) {
        HWARN("[DHT20  ] Sensor state improper; %s (%s)", this->cfg.name, this->cfg.location);
        
        status = this->soft_reset(res);
        if(status < DHT20_OK) return res.status = status;
    }

    HINFO ("[DHT20  ] Sensor initialized; %s (%s)", this->cfg.name, this->cfg.location);
    HDEBUG("[DHT20  ] I2C bus: I2C%d", i2c.index());
    HDEBUG("[DHT20  ] Address: 0x%02X", this->cfg.address);

    return res.status = status;
}

int8 DHT20::measure(void) {
    HTRACE("commands.cpp -> DHT20::measure(-):int8");
    return this->measure(this->ctx);
}

int8 DHT20::measure(Context &res) {
    HTRACE("commands.cpp -> DHT20::measure(Context&):int8");
    if(int8 status = this->sensor_enabled(); status < DHT20_OK) return res.status = status;

    int8 status = DHT20_OK;

    uint8 commands[] = {Command::Measure0, Command::Measure1, Command::Measure2};

    status = this->send_payload(commands);
    if(status < DHT20_OK) return res.status = status;
    
    // 7.4.3 Sensor Reading Process
    hkk::utils::sleep_ms(80);

    uint8 sensor_status;
    status = this->read_raw_data(&sensor_status, 1);
    if(status < DHT20_OK) return res.status = status;


    this->sensor_timeout = false;
    this->sensor_timeout_alarm.callback = DHT20::sensor_timeout_callback;
    this->sensor_timeout_alarm.data = this;

    int32 alarm_id = hkk::utils::alarm_us(this->cfg.sensor_timeout_us, &this->sensor_timeout_alarm, true);
    HTRACE("[DHT20  ] Sensor timeout alarm start");
    HTRACE("[DHT20  ] ID     : %d", alarm_id);
    HTRACE("[DHT20  ] Timeout: %d us", this->cfg.sensor_timeout_us);

    // 7.4.3 Sensor Reading Process
    uint8 status_bit = static_cast<uint8>(Status::Msbit);
    while((sensor_status & status_bit) != 0) {
        HTRACE("[DHT20  ] Measurement in progress");

        if(this->sensor_timeout == true) {
            HERROR("[DHT20  ] Sensor timeout while data read; %s (%s)", this->cfg.name, this->cfg.location);

            hkk::utils::cancel_alarm(alarm_id);
            return res.status = DHT20_ERROR_TIMEOUT;
        }

        status = this->read_raw_data(&sensor_status, 1);
        if(status < DHT20_OK) {
            hkk::utils::cancel_alarm(alarm_id);    
            return res.status = status;
        }

        hkk::utils::sleep_ms(1);
    }

    hkk::utils::cancel_alarm(alarm_id);


    uint8 data[DATA_FRAME_LENGTH];
    status = this->read_raw_data(data, DATA_FRAME_LENGTH);
    if(status < DHT20_OK) return res.status = status;

    status = DHT20::crc_validate(data);
    if(status < DHT20_OK) return res.status = status;

    // 7.4.5 Sensor Reading Process
    float32 humidity    = ((static_cast<int32>(data[1]) << 12) | (static_cast<int32>(data[2]) << 4) | (static_cast<int32>(data[3]) >> 4));
    float32 temperature = (((static_cast<int32>(data[3]) & DataMask::Temperature) << 16) | (static_cast<int32>(data[4]) << 8) | static_cast<int32>(data[5]));

    // 8.1 Relative Humidity Conversion
    res.humidity = ((humidity / std::pow(2, 20)) * 100.0f);

    // 8.2 Temperature Conversion
    res.temperature = ((temperature / std::pow(2, 20)) * 200 - 50);

    std::memcpy(res.raw_data, data, DATA_FRAME_LENGTH);
    
    return res.status = status;
}


bool8 DHT20::sensor_timeout_callback(void *data) {
    if(!data) return false;

    auto *self = static_cast<DHT20*>(data);
    self->sensor_timeout = true;

    return false;
} 

}