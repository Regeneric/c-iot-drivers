#include <hkk/drivers/bme280/bme280.hpp>

#include <cstring>
#include <cmath>

namespace hkk::bme280 {

int8 BME280::setup(void) {
    HTRACE("bme280.cpp -> BME280::setup(-):int8");
    return this->setup(this->ctx);
}

int8 BME280::setup(Context &res) {
    HTRACE("bme280.cpp -> BME280::setup(Context&):int8");
    if(int8 status = this->sensor_enabled(); status < BME280_OK) return res.status = status;

    int8 status = BME280_OK;

    status = this->init(res);
    if(status < BME280_OK) {
        this->cfg.enable = false;
        return res.status = status;
    }

    return res.status = status;
}

int8 BME280::process(void) {
    HTRACE("bme280.cpp -> process(-):int8");
    return this->process(this->ctx);
}

int8 BME280::process(Context &res) {
    HTRACE("bme280.cpp -> process(Context&):int8");
    if(int8 status = this->sensor_enabled(); status < BME280_OK) return res.status = status;

    int8 status = BME280_OK;

    status = this->measure(res);
    if(status < BME280_OK) return res.status = status;

    int32 raw_temperature = (res.temperature_raw_data[0] << 12) | (res.temperature_raw_data[1] << 4) | (res.temperature_raw_data[2] >> 4);
    int32 raw_pressure    = (res.pressure_raw_data[0]    << 12) | (res.pressure_raw_data[1]    << 4) | (res.pressure_raw_data[2]    >> 4);
    int32 raw_humidity    = (res.humidity_raw_data[0]    <<  8) |  res.humidity_raw_data[1];

    // IT MUST BE IN THIS ORDER
    int32 t_fine = 0;
    int32 temperature = BME280::compensate_temperature(res, t_fine, raw_temperature);
    int32 pressure    = BME280::compensate_pressure(res, t_fine, raw_pressure);
    int32 humidity    = BME280::compensate_humidity(res, t_fine, raw_humidity);

    res.pressure    = (pressure / 256.0) / 100.0;
    res.temperature = (temperature / 100.0);
    res.humidity    = (humidity / 1024.0);

    res.absolute_humidity = this->calculate_absolute_humidity(res);
    res.dew_point = this->calculate_dew_point(res);

    return res.status = status;
}

int8 BME280::status(void) {
    HTRACE("bme280.cpp -> BME280::status(-):int8");
    return this->status(this->ctx);
}

int8 BME280::status(Context &res) {
    HTRACE("bme280.cpp -> BME280::status(Context&):int8");
    if(int8 status = this->sensor_enabled(); status < BME280_OK) return res.status = status;

    return res.status;
}


int8 BME280::write_register(Register reg, uint8 *src, size_t len) {
    HTRACE("bme280.cpp -> BME280::write_register(Register, uint8*, size_t):int8");
    return this->write_register(this->cfg.address, reg, src, len);
}

int8 BME280::write_register(Register reg, uint8 byte) {
    HTRACE("bme280.cpp -> BME280::write_register(Register, uint8):int8");
    return this->write_register(this->cfg.address, reg, &byte, 1);
}

int8 BME280::write_register(uint8 addr, Register reg, uint8 *src, size_t len) {
    HTRACE("bme280.cpp -> BME280::write_register(uint8, Register, uint8*, size_t):int8");
    if(int8 status = this->sensor_enabled(); status < BME280_OK) return status;

    if(!src) {
        HERROR("[BME280 ] Null data pointer passed to function; %s (%s)", this->cfg.name, this->cfg.location);
        return BME280_ERROR_NULL_DATA;
    }

    if(len == 0) {
        HERROR("[BME280 ] Data length is 0; %s (%s)", this->cfg.name, this->cfg.location);
        return BME280_ERROR_ZERO_LENGTH;
    }

    const uint8 payload_len = (1 + len);
    uint8 payload[payload_len] = {reg};

    std::memcpy((payload + 1), src, len);

    int8 status = BME280_OK;

    status = this->write_bus(addr, payload, payload_len);
    if(status < BME280_OK) return status;

    return status;
}

int8 BME280::read_register(Register reg, uint8 &byte) {
    HTRACE("bme280.cpp -> BME280::read_register(Register, uint8&):int8");
    return this->read_register(reg, &byte, 1);
}

int8 BME280::read_register(Register reg, uint8 *dst, size_t len) {
    HTRACE("bme280.cpp -> BME280::read_register(Register, uint8*, size_t)");
    return this->read_register(this->cfg.address, reg, dst, len);
}

int8 BME280::read_register(uint8 addr, Register reg, uint8 *dst, size_t len) {
    HTRACE("bme280.cpp -> BME280::read_register(uint8, Register, uint8*, size_t)");
    if(int8 status = this->sensor_enabled(); status < BME280_OK) return status;

    if(!dst) {
        HERROR("[BME280 ] Null data pointer passed to function; %s (%s)", this->cfg.name, this->cfg.location);
        return BME280_ERROR_NULL_DATA;
    }

    if(len == 0) {
        HERROR("[BME280 ] Data length is 0; %s (%s)", this->cfg.name, this->cfg.location);
        return BME280_ERROR_ZERO_LENGTH;
    }
    
    int8 status = BME280_OK;

    uint8 data = static_cast<uint8>(reg);
    status = this->write_bus(addr, &data, 1, true);
    if(status < BME280_OK) return status;

    status = this->read_bus(addr, dst, len);
    if(status < BME280_OK) return status;

    return status;
}

float32 BME280::calculate_absolute_humidity(void) {
    HTRACE("bme280.cpp -> BME280::absolute_humidty(-):float32");
    return this->calculate_absolute_humidity(this->ctx);
}

float32 BME280::calculate_absolute_humidity(float32 humidity, float32 temperature) {
    HTRACE("bme280.cpp -> BME280::absolute_humidty(float32, float32):float32");

    this->ctx.temperature = temperature;
    this->ctx.humidity = humidity;

    return this->calculate_absolute_humidity(this->ctx);
}

float32 BME280::calculate_absolute_humidity(Context &res) {
    HTRACE("bme280.cpp -> BME280::absolute_humidty(Context&):float32");
    
    // Magnus-Tetens approximation
    float32 saturation = (SATURATION_VAPOR_PRESSURE * std::exp((MAGNUS_COEFFICIENT * res.temperature) / (TEMPERATURE_COEFFICIENT + res.temperature)));
    float32 pressure = saturation * (res.humidity / 100.0);    
    float32 vapor_deficit = (saturation - pressure) / 1000.0;   // kPa
    
    float32 absolute_humidity = ((pressure * WATER_MOLAR_MASS) / (WATER_VAPOR_GAS_CONST * (res.temperature + CELSIUS_KELVIN_OFFSET)));  // kg/m3
    absolute_humidity = (absolute_humidity * 1000.0); // g/m3

    HTRACE("[BME280  ] Actual vapor pressure    : %3.3f Pa", pressure);
    HTRACE("[BME280  ] Saturation vapor pressure: %3.3f Pa", saturation);
    HTRACE("[BME280  ] Vapor pressure defficit  : %3.3f kPa", vapor_deficit);
    HTRACE("[BME280  ] Absolute humidity        : %3.3f g/m3", absolute_humidity);

    res.vapor_pressure = pressure;                  // Pa
    res.saturation_vapor_pressure = saturation;     // Pa
    res.vapor_pressure_deficit = vapor_deficit;     // kPa

    res.absolute_humidity = absolute_humidity;      // g/m3
    return absolute_humidity;
}

float32 BME280::calculate_dew_point(void) {
    HTRACE("bme280.cpp -> BME280::calculate_dew_point(-):float32");
    return this->calculate_dew_point(this->ctx);
}

float32 BME280::calculate_dew_point(float32 humidity, float32 temperature) {
    HTRACE("bme280.cpp -> BME280::calculate_dew_point(float32, float32):float32");

    this->ctx.temperature = temperature;
    this->ctx.humidity = humidity;

    return this->calculate_dew_point(this->ctx);
}

float32 BME280::calculate_dew_point(Context &res) {
    HTRACE("bme280.cpp -> BME280::calculate_dew_point(Context&):float32");

    float32 gamma = (std::log(res.humidity / 100) + (MAGNUS_COEFFICIENT * res.temperature) / (TEMPERATURE_COEFFICIENT + res.temperature));
    float32 dew_point = ((TEMPERATURE_COEFFICIENT * gamma) / (MAGNUS_COEFFICIENT - gamma));

    HTRACE("[BME280  ] Gamma    : %3.3f", gamma);
    HTRACE("[BME280  ] Dew point: %3.3f *C", dew_point);

    return dew_point;
}


float32 BME280::humidity(bool8 absolute_humidity) {
    HTRACE("bme280.cpp -> BME280::humidity(bool8 = false):float32");
    return (absolute_humidity == true) ? this->ctx.absolute_humidity : this->ctx.humidity;
} 

void BME280::humidity(Context &res) {
    HTRACE("bme280.cpp -> BME280::humidity(Context&):void");
    
    res.humidity = this->ctx.humidity;
    res.absolute_humidity = this->ctx.absolute_humidity;
}
    
float32 BME280::temperature(void) {
    HTRACE("bme280.cpp -> BME280::temperature(-):float32");
    return this->ctx.temperature;
} 

void BME280::temperature(Context &res) {
    HTRACE("bme280.cpp -> BME280::temperature(Context&):void");
    res.temperature = this->ctx.temperature;
}

float32 BME280::dew_point(void) {
    HTRACE("bme280.cpp -> BME280::dew_point(-):float32");
    return this->ctx.dew_point;
} 

void BME280::dew_point(Context &res) {
    HTRACE("bme280.cpp -> BME280::dew_point(Context&):void");
    res.dew_point = this->ctx.dew_point;
}

float32 BME280::vapor(Vapor type) {
    HTRACE("bme280.cpp -> BME280::vapor(Vapor):float32");
    
    switch(type) {
        case Vapor::Deficit:    return this->ctx.vapor_pressure_deficit;
        case Vapor::Pressure:   return this->ctx.vapor_pressure;
        case Vapor::Saturation: return this->ctx.saturation_vapor_pressure;
        default: return static_cast<float32>(BME280_ERROR_GENERIC);   
    }
} 

void BME280::vapor(Context &res) {
    HTRACE("bme280.cpp -> BME280::vapor(Context&):void");
    
    res.vapor_pressure_deficit = this->ctx.vapor_pressure_deficit;
    res.vapor_pressure = this->ctx.vapor_pressure;
    res.saturation_vapor_pressure = this->ctx.saturation_vapor_pressure;
}

float32 BME280::pressure(void) {
    HTRACE("bme280.cpp -> BME280::pressure(-):float32");
    return this->ctx.pressure;
} 

void BME280::pressure(Context &res) {
    HTRACE("bme280.cpp -> BME280::pressure(Context&):void");
    res.pressure = this->ctx.pressure;
}



int8 BME280::write_bus(uint8 byte, bool8 nostop) {
    HTRACE("bme280.cpp -> BME280::write_bus(uint8*, size_t):int8");
    return this->write_bus(this->cfg.address, &byte, 1, nostop);
}

int8 BME280::write_bus(uint8 *src, size_t len, bool8 nostop) {
    HTRACE("bme280.cpp -> BME280::write_bus(uint8*, size_t):int8");
    return this->write_bus(this->cfg.address, src, len, nostop);
}

int8 BME280::write_bus(uint8 addr, uint8 *src, size_t len, bool8 nostop) {
    HTRACE("bme280.cpp -> BME280::write_bus(uint8, uint8*, size_t):int8");
    if(int8 status = this->sensor_enabled(); status < BME280_OK) return status;

    if(!src) {
        HERROR("[BME280 ] Null data pointer passed to function; %s (%s)", this->cfg.name, this->cfg.location);
        return BME280_ERROR_NULL_DATA;
    }

    if(len == 0) {
        HERROR("[BME280 ] Data length is 0; %s (%s)", this->cfg.name, this->cfg.location);
        return BME280_ERROR_ZERO_LENGTH;
    }


    auto transaction = [&](auto &bus) {
        int8 status = BME280_OK;
        
        auto tx = bus->transaction(this);
        if(tx.status < BME280_OK) return tx.status;

        // I know that SPI devices don't have addresses and no-stop bits, they're here for compat reasons
        if(this->cfg.sensor_timeout_us <= 0) status = bus->write(addr, src, len, nostop); 
        else status = bus->write_timeout(addr, src, len, this->cfg.sensor_timeout_us, nostop); 
 
        if(status < BME280_OK) return status;

        return status;
    };


    if(i2c != nullptr) {
        HTRACE("[BME280 ] I2C write transaction start");
        return transaction(i2c);
    }

    if(spi != nullptr) {
        HTRACE("[BME280 ] SPI write transaction start");
        return transaction(spi);
    }

    return BME280_ERROR_GENERIC;    // It should never get here
}

int8 BME280::read_bus(uint8 *dst, size_t len, bool8 nostop) {
    HTRACE("bme280.cpp -> BME280::read_bus(uint8*, size_t):int8");
    return this->read_bus(this->cfg.address, dst, len, nostop);
}

int8 BME280::read_bus(uint8 addr, uint8 *dst, size_t len, bool8 nostop) {
    HTRACE("bme280.cpp -> BME280::read_bus(uint8, uint8*, size_t):int8");
    if(int8 status = this->sensor_enabled(); status < BME280_OK) return status;

    if(!dst) {
        HERROR("[BME280 ] Null data pointer passed to function; %s (%s)", this->cfg.name, this->cfg.location);
        return BME280_ERROR_NULL_DATA;
    }

    if(len == 0) {
        HERROR("[BME280 ] Data length is 0; %s (%s)", this->cfg.name, this->cfg.location);
        return BME280_ERROR_ZERO_LENGTH;
    }

    
    auto transaction = [&](auto &bus) {
        int8 status = BME280_OK;
        
        auto tx = bus->transaction(this);
        if(tx.status < BME280_OK) return tx.status;

        // I know that SPI devices don't have addresses and no-stop bits, they're here for compat reasons
        if(this->cfg.sensor_timeout_us <= 0) status = bus->read(addr, dst, len, nostop); 
        else status = bus->read_timeout(addr, dst, len, this->cfg.sensor_timeout_us, nostop);

        if(status < BME280_OK) return status;

        return status;
    };


    if(i2c != nullptr) {
        HTRACE("[BME280 ] I2C read transaction start");
        return transaction(i2c);
    }

    if(spi != nullptr) {
        HTRACE("[BME280 ] SPI read transaction start");
        return transaction(spi);
    }

    return BME280_ERROR_GENERIC;    // It should never get here
}

// Official Bosch implementation
int32 BME280::compensate_temperature(const Context &res, int32 &t_fine, int32 raw_temperature) {
    HTRACE("bme280.cpp -> s:BME280::compensate_temperature(Context&, int32&, int32):int8");
    
    int32 var1, var2, T;
    var1 = ((((raw_temperature >> 3)  - ((int32) res.calibration.dig_T1 << 1))) * ((int32) res.calibration.dig_T2)) >> 11;
    var2 = (((((raw_temperature >> 4) - ((int32) res.calibration.dig_T1)) * ((raw_temperature >> 4) - ((int32) res.calibration.dig_T1))) >> 12) * ((int32) res.calibration.dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;   // Celsius, 0.01 resolution, `2137` equals 21.37 *C`
}

// Official Bosch implementation
uint32 BME280::compensate_pressure(const Context &res, int32 t_fine, int32 raw_pressure) {
    int64 var1, var2, p;
    var1 = ((int64) t_fine) - 128000;
    var2 = var1 * var1 * (int64)res.calibration.dig_P6;
    var2 = var2 + ((var1 * (int64)res.calibration.dig_P5) << 17);
    var2 = var2 + (((int64)res.calibration.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64)res.calibration.dig_P3) >> 8) + ((var1 * (int64)res.calibration.dig_P2) << 12);
    var1 = (((((int64)1) << 47) + var1)) * ((int64)res.calibration.dig_P1) >> 33;
    if(var1 == 0) return 0; // avoid exception caused by division by zero

    p = 1048576 - raw_pressure;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64)res.calibration.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64)res.calibration.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64)res.calibration.dig_P7) << 4);
    return (uint32)(p);  // Q24.8 format, `24674867` equals to `24674867/256 == 96386.2 Pa == 963.862 hPa`
}

// Official Bosch implementation
uint32 BME280::compensate_humidity(const Context &res, int32 t_fine, int32 raw_humidity) {
    int32 v_x1_u32r;
    v_x1_u32r = (t_fine - ((int32)76800));

    // What the fuck is going on here???
    v_x1_u32r = (((((raw_humidity << 14) - (((int32)res.calibration.dig_H4) << 20) - (((int32)res.calibration.dig_H5) * v_x1_u32r)) +
                    ((int32)16384)) >> 15) * (((((((v_x1_u32r * ((int32)res.calibration.dig_H6)) >> 10) *
                    (((v_x1_u32r * ((int32)res.calibration.dig_H3)) >> 11) +
                    ((int32)32768))) >> 10) + ((int32)2097152)) *
                    ((int32)res.calibration.dig_H2) + 8192) >> 14));

    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32)res.calibration.dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (uint32)(v_x1_u32r >> 12);  // Q22.10 format, `47445` equals to `47445/1024 == 46.333 %RH`
}

}