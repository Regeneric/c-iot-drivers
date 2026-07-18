#include <hkk/drivers/bme280/bme280.hpp>

#include <cstring>

namespace hkk::bme280 {

int8 BME280::write_register(Register reg, uint8 *data, size_t len) {
    HTRACE("bme280.cpp -> BME280::write_register(Register, uint8*, size_t):int8");
    return this->write_register(this->cfg.address, reg, data, len);
}

int8 BME280::write_register(Register reg, uint8 value) {
    HTRACE("bme280.cpp -> BME280::write_register(Register, uint8):int8");
    return this->write_register(this->cfg.address, reg, &value, 1);
}

int8 BME280::write_register(uint8 addr, Register reg, uint8 *data, size_t len) {
    HTRACE("bme280.cpp -> BME280::write_register(uint8, Register, uint8*, size_t):int8");
    if(int8 status = this->sensor_enabled(); status < BME280_OK) return status;

    if(!data) {
        HERROR("[BME280 ] Null data pointer passed to function");
        return BME280_ERROR_NULL_DATA;
    }

    if(len == 0) {
        HERROR("[BME280 ] Data length is 0");
        return BME280_ERROR_ZERO_LENGTH;
    }

    const uint8 payload_len = (1 + len);
    uint8 payload[payload_len] = {reg};

    std::memcpy((payload + 1), data, len);

    int8 status = BME280_OK;

    status = this->write_bus(payload, payload_len);
    if(status < BME280_OK) return status;

    return status;
}

int8 BME280::read_register(Register reg, uint8 &value) {
    HTRACE("bme280.cpp -> BME280::read_register(Register, uint8&):int8");
    return this->read_register(reg, &value, 1);
}

int8 BME280::read_register(Register reg, uint8 *data, size_t len) {
    HTRACE("bme280.cpp -> BME280::read_register(Register, uint8*, size_t)");
    return this->read_register(this->cfg.address, reg, data, len);
}

int8 BME280::read_register(uint8 addr, Register reg, uint8 *data, size_t len) {
    HTRACE("bme280.cpp -> BME280::read_register(uint8, Register, uint8*, size_t)");
    if(int8 status = this->sensor_enabled(); status < BME280_OK) return status;

    if(!data) {
        HERROR("[BME280 ] Null data pointer passed to function");
        return BME280_ERROR_NULL_DATA;
    }

    if(len == 0) {
        HERROR("[BME280 ] Data length is 0");
        return BME280_ERROR_ZERO_LENGTH;
    }
    
    int8 status = BME280_OK;

    status = this->write_bus(reg, true);
    if(status < BME280_OK) return status;

    status = this->read_bus(data, len);
    if(status < BME280_OK) return status;

    return status;
}


int8 BME280::write_bus(uint8 data, bool8 nostop) {
    HTRACE("bme280.cpp -> BME280::write_bus(uint8*, size_t):int8");
    return this->write_bus(this->cfg.address, &data, 1, nostop);
}

int8 BME280::write_bus(uint8 *data, size_t len, bool8 nostop) {
    HTRACE("bme280.cpp -> BME280::write_bus(uint8*, size_t):int8");
    return this->write_bus(this->cfg.address, data, len, nostop);
}

int8 BME280::write_bus(uint8 addr, uint8 *data, size_t len, bool8 nostop) {
    HTRACE("bme280.cpp -> BME280::write_bus(uint8, uint8*, size_t):int8");
    if(int8 status = this->sensor_enabled(); status < BME280_OK) return status;

    if(!data) {
        HERROR("[BME280 ] Null data pointer passed to function");
        return BME280_ERROR_NULL_DATA;
    }

    if(len == 0) {
        HERROR("[BME280 ] Data length is 0");
        return BME280_ERROR_ZERO_LENGTH;
    }


    auto transaction = [&](auto &bus) {
        int8 status = BME280_OK;
        
        auto tx = bus->transaction(this);
        if(tx.status < BME280_OK) return tx.status;

        status = bus->write(addr, data, len, nostop);    // I know that SPI devices don't have addresses and no-stop bits, they're here for compat reasons
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

int8 BME280::read_bus(uint8 *data, size_t len, bool8 nostop) {
    HTRACE("bme280.cpp -> BME280::read_bus(uint8*, size_t):int8");
    return this->read_bus(this->cfg.address, data, len, nostop);
}

int8 BME280::read_bus(uint8 addr, uint8 *data, size_t len, bool8 nostop) {
    HTRACE("bme280.cpp -> BME280::read_bus(uint8, uint8*, size_t):int8");
    if(int8 status = this->sensor_enabled(); status < BME280_OK) return status;

    if(!data) {
        HERROR("[BME280 ] Null data pointer passed to function");
        return BME280_ERROR_NULL_DATA;
    }

    if(len == 0) {
        HERROR("[BME280 ] Data length is 0");
        return BME280_ERROR_ZERO_LENGTH;
    }

    
    auto transaction = [&](auto &bus) {
        int8 status = BME280_OK;
        
        auto tx = bus->transaction(this);
        if(tx.status < BME280_OK) return tx.status;

        status = bus->read(addr, data, len, nostop);    // I know that SPI devices don't have addresses and no-stop bits, they're here for compat reasons
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

}