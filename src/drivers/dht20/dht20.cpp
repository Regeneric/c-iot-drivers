#include <hkk/defines.h>

#include <hkk/utils/utils.hpp>
#include <hkk/logger/logger.h>
#include <hkk/bus/i2c/i2c.hpp>
#include <hkk/storage/nvm.hpp>

#include <hkk/drivers/dht20/dht20.hpp>

#include <cmath>

namespace hkk::dht20 {

int8 DHT20::setup(void) {
    HTRACE("dht20.cpp -> DHT20::setup(-):int8");
    return this->setup(this->ctx);
}

int8 DHT20::setup(Context &res) {
    HTRACE("dht20.cpp -> DHT20::setup(Context&):int8");
    if(int8 status = this->sensor_enabled(); status < DHT20_OK) return res.status = status;

    int8 status = DHT20_OK;

    // 7.4.1 Sensor Reading Process
    hkk::utils::sleep_ms(100);
    status = this->init(res);
    if(status < DHT20_OK) {
        this->cfg.enable = false;
        return res.status = status;
    }

    // 7.4.2 Sensor Reading Process
    hkk::utils::sleep_ms(10);

    return res.status = status;
}

int8 DHT20::process(void) {
    HTRACE("dht20.cpp -> DHT20::process(-):int8");
    return this->process(this->ctx);
}

int8 DHT20::process(Context &res) {
    HTRACE("dht20.cpp -> DHT20::process(Context&):int8");
    if(int8 status = this->sensor_enabled(); status < DHT20_OK) return res.status = status;

    int8 status = DHT20_OK;

    status = this->measure(res);
    if(status < DHT20_OK) return res.status = status;

    res.absolute_humidity = this->calculate_absolute_humidity(res);
    res.dew_point = this->calculate_dew_point(res);

    return res.status = status;
}

int8 DHT20::status(void) {
    HTRACE("dht20.cpp -> DHT20::status(-):int8");
    return this->status(this->ctx);
}

int8 DHT20::status(Context &res) {
    HTRACE("dht20.cpp -> DHT20::status(Context&):int8");
    if(int8 status = this->sensor_enabled(); status < DHT20_OK) return res.status = status;

    return res.status;
}

// 7.4.1 Sensor Reading Process - Undocumented procedure
int8 DHT20::soft_reset(void) {
    HTRACE("dht20.cpp -> DHT20::soft_reset(-):int8");
    return this->soft_reset(this->ctx);
}

int8 DHT20::soft_reset(Context &res) {
    HTRACE("dht20.cpp -> DHT20::soft_reset(Context&):int8");
    if(int8 status = this->sensor_enabled(); status < DHT20_OK) return res.status = status;

    {
        int8 status = DHT20_OK;
        
        uint8 reset_value = 0x00;
        uint8 registers[Register::Count] = {Register::Calibration0, Register::Calibration1, Register::Calibration2};

        for(size_t i = 0; i != Register::Count; ++i) {
            uint8 payload[] = {registers[i], reset_value, reset_value};            

            status = this->send_payload(payload);
            if(status < DHT20_OK) return res.status = status;
        }

        HDEBUG("[DHT20  ] Sensor recovery sequence; %s (%s)", this->cfg.name, this->cfg.location);
        return res.status = status;
    }
}

int8 DHT20::send_command(Command command) {
    HTRACE("dht20.cpp -> DHT20::send_command(Command):int8");
    return this->send_command(this->cfg.address, command);
}

int8 DHT20::send_command(uint8 address, Command command) {
    HTRACE("dht20.cpp -> DHT20::send_command(uint8, Command):int8");
    if(int8 status = this->sensor_enabled(); status < DHT20_OK) return status;

    {
        int32 status = static_cast<int8>(DHT20_OK);

        auto tx = i2c.transaction(this);
        if(tx.status < hkk::bus::i2c::I2C_OK) {
            return this->validate_i2c_error(tx.status);
        }

        if(this->cfg.sensor_timeout_us <= 0) status = i2c.write(address, command);
        else status = i2c.write_timeout(address, command, this->cfg.sensor_timeout_us);

        if(status < hkk::bus::i2c::I2C_OK) {
            HERROR("[DHT20  ] I2C bus error: %d", status);
            return this->validate_i2c_error(status);
        }

        HTRACE("[DHT20  ] Address: 0x%02X", address);
        HTRACE("[DHT20  ] Command: 0x%02X", command);

        return (status > DHT20_OK) ? DHT20_OK : static_cast<int8>(status);
    }
}

int8 DHT20::send_payload(uint8 *payload, size_t len) {
    HTRACE("[DHT20  ] dht20.cpp -> DHT20::send_payload(uint8*, size_t):int8");
    return this->send_payload(this->cfg.address, payload, len);
}

int8 DHT20::send_payload(uint8 addr, uint8 *payload, size_t len) {
    HTRACE("[DHT20  ] dht20.cpp -> DHT20::send_payload(uint8, uint8*, size_t):int8");
    if(int8 status = this->sensor_enabled(); status < DHT20_OK) return status;

    if(!payload) {
        HERROR("[DHT20  ] Null data pointer passed to function");
        return DHT20_ERROR_NULL_DATA;
    }

    if(len == 0) {
        HERROR("[DHT20  ] Data length is 0");
        return DHT20_ERROR_ZERO_LENGTH;
    }

    {
        int32 status = DHT20_OK;

        auto tx = i2c.transaction(this);
        if(tx.status < hkk::bus::i2c::I2C_OK) {
            return this->validate_i2c_error(tx.status);
        }

        if(this->cfg.sensor_timeout_us <= 0) status = i2c.write(addr, payload, len);
        else status = i2c.write_timeout(addr, payload, len, this->cfg.sensor_timeout_us);

        if(status < hkk::bus::i2c::I2C_OK) {
            return this->validate_i2c_error(static_cast<int8>(status));
        }

        HTRACE("[SGP30  ] Address: 0x%02X", addr);
        return (status > DHT20_OK) ? DHT20_OK : static_cast<int8>(status);
    }
}


int8 DHT20::read_raw_data(uint8 *data, size_t len) {
    HTRACE("dht20.cpp -> DHT20::read_raw_data(uint8*, size_t):int8");
    if(int8 status = this->sensor_enabled(); status < DHT20_OK) return status;

    int32 status;

    if(!data) {
        HERROR("[DHT20  ] Null data pointer passed to function; %s (%s)", this->cfg.name, this->cfg.location);
        return DHT20_ERROR_NULL_DATA;
    }

    if(len == 0) {
        HERROR("[DHT20  ] Data length is 0; %s (%s)", this->cfg.name, this->cfg.location);
        return DHT20_ERROR_ZERO_LENGTH;
    }

    {
        auto tx = i2c.transaction(this);
        if(tx.status < hkk::bus::i2c::I2C_OK) {
            return this->validate_i2c_error(tx.status);
        }

        if(this->cfg.sensor_timeout_us <= 0) status = i2c.read(this->cfg.address, data, len);
        else status = i2c.read_timeout(this->cfg.address, data, len, this->cfg.sensor_timeout_us);
        
        if(status < hkk::bus::i2c::I2C_OK) {
            return this->validate_i2c_error(status);
        }
    }

    HTRACE("[DHT20  ] Raw data read from address: 0x%02X", this->cfg.address);
    return (status > DHT20_OK) ? DHT20_OK : static_cast<int8>(status);
}

int8 DHT20::validate_nvm_error(int8 error) {
    HTRACE("dht20.cpp -> DHT20::validate_nvm_error(int8):int8");
    if(int8 status = this->sensor_enabled(); status < DHT20_OK) return status;

    if(error < hkk::storage::nvm::NVM_OK) {
        HERROR("[DHT20  ] Could not write data to NVM storage; %s (%s)", this->cfg.name, this->cfg.location);
        switch(error) {
            case hkk::storage::nvm::NVM_OK:
                break;

            case hkk::storage::nvm::NVM_DATA_TRUNCATED:
            case hkk::storage::nvm::NVM_NULL_ADDRESS:
            case hkk::storage::nvm::NVM_ERROR_BUSY:
                return DHT20_ERROR_NVM_TRANSACTION;

            case hkk::storage::nvm::NVM_ERROR_NULL_DATA:
            case hkk::storage::nvm::NVM_ERROR_ZERO_LENGTH:
                return DHT20_ERROR_NULL_DATA;

            case hkk::storage::nvm::NVM_ERROR_NULL_CONTEXT:
            case hkk::storage::nvm::NVM_ERROR_NULL_MUTEX:
            case hkk::storage::nvm::NVM_ERROR_OOB:
            case hkk::storage::nvm::NVM_ERROR_GENERIC:
            case hkk::storage::nvm::NVM_ERROR_UNKNOWN:
            case hkk::storage::nvm::NVM_FUNCTION_NOT_IMPLEMENTED:
                return DHT20_ERROR_NVM;
        }
    } 
    
    return DHT20_OK;
}

int8 DHT20::validate_i2c_error(int8 error) {
    HTRACE("dht20.cpp -> DHT20::validate_i2c_error(int8):int8");
    if(int8 status = this->sensor_enabled(); status < DHT20_OK) return status;

    if(error < hkk::bus::i2c::I2C_OK) {
        HERROR("[DHT20  ] Could not send data to I2C bus; %s (%s)", this->cfg.name, this->cfg.location);
        switch(error) {
            case hkk::bus::i2c::I2C_OK:
                break;

            case hkk::bus::i2c::I2C_ERROR_NULL_DATA:
            case hkk::bus::i2c::I2C_ERROR_ZERO_LENGTH:
                return DHT20_ERROR_NULL_DATA;

            case hkk::bus::i2c::I2C_ERROR_PARTIAL_WRITE:
            case hkk::bus::i2c::I2C_ERROR_WRITE_FAILED:
            case hkk::bus::i2c::I2C_ERROR_PARTIAL_READ:
            case hkk::bus::i2c::I2C_ERROR_READ_FAILED:
            case hkk::bus::i2c::I2C_ERROR_TIMEOUT:
            case hkk::bus::i2c::I2C_ERROR_BUSY:
                return DHT20_ERROR_I2C_TRANSACTION;

            case hkk::bus::i2c::I2C_ERROR_NULL_CONTEXT:
            case hkk::bus::i2c::I2C_ERROR_NULL_INSTANCE:
            case hkk::bus::i2c::I2C_ERROR_NO_ACK:
            case hkk::bus::i2c::I2C_ERROR_NULL_MUTEX:
            case hkk::bus::i2c::I2C_ERROR_NOT_SUPPORTED:
            case hkk::bus::i2c::I2C_ERROR_GENERIC:
            case hkk::bus::i2c::I2C_ERROR_UNKNOWN:
                return DHT20_ERROR_I2C;

            case hkk::bus::i2c::I2C_FUNCTION_NOT_IMPLEMENTED:
                return DHT20_FUNCTION_NOT_IMPLEMENTED;
        }
    }
    
    return DHT20_OK;
}

float32 DHT20::calculate_absolute_humidity(void) {
    HTRACE("dht20.cpp -> DHT20::absolute_humidty(-):float32");
    return this->calculate_absolute_humidity(this->ctx);
}

float32 DHT20::calculate_absolute_humidity(float32 humidity, float32 temperature) {
    HTRACE("dht20.cpp -> DHT20::absolute_humidty(float32, float32):float32");

    this->ctx.temperature = temperature;
    this->ctx.humidity = humidity;

    return this->calculate_absolute_humidity(this->ctx);
}

float32 DHT20::calculate_absolute_humidity(Context &res) {
    HTRACE("dht20.cpp -> DHT20::absolute_humidty(Context&):float32");
    
    // Magnus-Tetens approximation
    float32 saturation = (SATURATION_VAPOR_PRESSURE * std::exp((MAGNUS_COEFFICIENT * res.temperature) / (TEMPERATURE_COEFFICIENT + res.temperature)));
    float32 pressure = saturation * (res.humidity / 100.0);    
    float32 vapor_deficit = (saturation - pressure) / 1000.0;   // kPa
    
    float32 absolute_humidity = ((pressure * WATER_MOLAR_MASS) / (WATER_VAPOR_GAS_CONST * (res.temperature + CELSIUS_KELVIN_OFFSET)));  // kg/m3
    absolute_humidity = (absolute_humidity * 1000.0); // g/m3

    HTRACE("[DHT20  ] Actual vapor pressure    : %3.3f Pa", pressure);
    HTRACE("[DHT20  ] Saturation vapor pressure: %3.3f Pa", saturation);
    HTRACE("[DHT20  ] Vapor pressure defficit  : %3.3f kPa", vapor_deficit);
    HTRACE("[DHT20  ] Absolute humidity        : %3.3f g/m3", absolute_humidity);

    res.vapor_pressure = pressure;                  // Pa
    res.saturation_vapor_pressure = saturation;     // Pa
    res.vapor_pressure_deficit = vapor_deficit;     // kPa

    res.absolute_humidity = absolute_humidity;      // g/m3
    return absolute_humidity;
}

float32 DHT20::calculate_dew_point(void) {
    HTRACE("dht20.cpp -> DHT20::calculate_dew_point(-):float32");
    return this->calculate_dew_point(this->ctx);
}

float32 DHT20::calculate_dew_point(float32 humidity, float32 temperature) {
    HTRACE("dht20.cpp -> DHT20::calculate_dew_point(float32, float32):float32");

    this->ctx.temperature = temperature;
    this->ctx.humidity = humidity;

    return this->calculate_dew_point(this->ctx);
}

float32 DHT20::calculate_dew_point(Context &res) {
    HTRACE("dht20.cpp -> DHT20::calculate_dew_point(Context&):float32");

    float32 gamma = (std::log(res.humidity / 100) + (MAGNUS_COEFFICIENT * res.temperature) / (TEMPERATURE_COEFFICIENT + res.temperature));
    float32 dew_point = ((TEMPERATURE_COEFFICIENT * gamma) / (MAGNUS_COEFFICIENT - gamma));

    HTRACE("[DHT20  ] Gamma    : %3.3f", gamma);
    HTRACE("[DHT20  ] Dew point: %3.3f *C", dew_point);

    return dew_point;
}


float32 DHT20::humidity(bool8 absolute_humidity) {
    HTRACE("dht20.cpp -> DHT20::humidity(bool8 = false):float32");
    return (absolute_humidity == true) ? this->ctx.absolute_humidity : this->ctx.humidity;
} 

void DHT20::humidity(Context &res) {
    HTRACE("dht20.cpp -> DHT20::humidity(Context&):void");
    
    res.humidity = this->ctx.humidity;
    res.absolute_humidity = this->ctx.absolute_humidity;
}
    
float32 DHT20::temperature(void) {
    HTRACE("dht20.cpp -> DHT20::temperature(-):float32");
    return this->ctx.temperature;
} 

void DHT20::temperature(Context &res) {
    HTRACE("dht20.cpp -> DHT20::temperature(Context&):void");
    res.temperature = this->ctx.temperature;
}

float32 DHT20::dew_point(void) {
    HTRACE("dht20.cpp -> DHT20::dew_point(-):float32");
    return this->ctx.dew_point;
} 

void DHT20::dew_point(Context &res) {
    HTRACE("dht20.cpp -> DHT20::dew_point(Context&):void");
    res.dew_point = this->ctx.dew_point;
}

float32 DHT20::vapor(Vapor type) {
    HTRACE("dht20.cpp -> DHT20::vapor(Vapor):float32");
    
    switch(type) {
        case Vapor::Deficit:    return this->ctx.vapor_pressure_deficit;
        case Vapor::Pressure:   return this->ctx.vapor_pressure;
        case Vapor::Saturation: return this->ctx.saturation_vapor_pressure;
        default: return static_cast<float32>(DHT20_ERROR_GENERIC);   
    }
} 

void DHT20::vapor(Context &res) {
    HTRACE("dht20.cpp -> DHT20::vapor(Context&):void");
    
    res.vapor_pressure_deficit = this->ctx.vapor_pressure_deficit;
    res.vapor_pressure = this->ctx.vapor_pressure;
    res.saturation_vapor_pressure = this->ctx.saturation_vapor_pressure;
}

}