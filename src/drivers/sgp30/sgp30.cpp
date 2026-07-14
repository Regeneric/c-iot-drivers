#include <hkk/defines.h>

#include <hkk/utils/utils.hpp>
#include <hkk/logger/logger.h>
#include <hkk/bus/i2c/i2c.hpp>
#include <hkk/drivers/sgp30/sgp30.hpp>
#include <hkk/storage/nvm.hpp>

#include <cstring>

namespace hkk::sgp30 {


// Public
static hkk::utils::AlarmContext baseline_store_alarm;
static hkk::utils::AlarmContext calibration_alarm;

int8 SGP30::setup(void) {
    HTRACE("sgp30.cpp -> SGP30::setup(-):int8");
    return this->setup(this->ctx);
}

int8 SGP30::setup(Context &result) {
    HTRACE("sgp30.cpp -> SGP30::setup(Context&):int8");
    if(int8 status = this->sensor_enabled(); status < SGP30_OK) return result.status = status;

    int8 status = SGP30_OK;

    status = this->iaq_init();
    if(status < SGP30_OK) return result.status = status;

    status = this->get_serial_number(this->ctx);
    if(status < SGP30_OK) return result.status = status;

    int8 crc_eco2 = crc_validate((result.baseline + 0), HALF_DATA_FRAME_LENGTH);
    int8 crc_tvoc = crc_validate((result.baseline + 3), HALF_DATA_FRAME_LENGTH);
    bool8 baseline_invalid = ((crc_eco2 < SGP30_OK) || (crc_tvoc < SGP30_OK));

    if(baseline_invalid == true) HWARN("[SGP30  ] No IAQ baseline provided");
    
    if(!this->cfg.nvm) {
        HERROR("[SGP30  ] Null NVM instance in context; data will be lost on shutdown");
        return result.status = SGP30_ERROR_NVM;
    }
    auto *nvm = static_cast<hkk::storage::nvm::NVM*>(this->cfg.nvm);

    if(baseline_invalid == false) {
        status = this->set_iaq_baseline(result);
        if(status < SGP30_OK) return result.status = status;
        
        HINFO ("[SGP30  ] Storing provided IAQ baseline in NVM storage");
        HDEBUG("[SGP30  ] Baseline:  {0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X}", result.baseline[0], result.baseline[1], result.baseline[2], result.baseline[3], result.baseline[4], result.baseline[5]);
    
        status = this->store_baseline(result);
        if(status < SGP30_OK) return result.status = status;

        return status;
    }

    uint8 baseline[DATA_FRAME_LENGTH];
    status = baseline_lookup(baseline);
    if(status < SGP30_OK) return result.status = status;

    HINFO ("[SGP30  ] IAQ baseline found in NVM storage");
    HDEBUG("[SGP30  ] Using baseline {0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X}", baseline[0], baseline[1], baseline[2], baseline[3], baseline[4], baseline[5]);

    status = this->set_iaq_baseline(baseline);
    if(status < SGP30_OK) return result.status = status;

    hkk::utils::alarm_ms((1 * HOUR), &baseline_store_alarm, true);
    HDEBUG("[SGP30  ] Store baseline alarm started");

    result.status = status;
    return result.status;
}

int8 SGP30::process(void) {
    HTRACE("sgp30.cpp -> SGP30::process(-):int8");
    return this->setup(this->ctx);
}

int8 SGP30::process(Context &result) {
    HTRACE("sgp30.cpp -> SGP30::process(Context&):int8");
    if(int8 status = this->sensor_enabled(); status < SGP30_OK) return result.status = status;

    int8 status = SGP30_OK;

    // if(result.calibrated == false) {
        // HWARN("[SGP30  ] Sensor is not calibrated; starting calibration process");
        // status = this->calibrate(result);
    // }

    status = this->measure_iaq(result);
    if(status < SGP30_OK) return status;

    status = this->measure_raw(result);
    if(status < SGP30_OK) return status;

    status = this->get_iaq_baseline(result);
    if(status < SGP30_OK) return status;

    if(this->store_baseline_request == true) {
        this->store_baseline(result);
        this->store_baseline_request = false;
    }

    return status;
}

int8 SGP30::send_command(Command command) {
    HTRACE("sgp30.cpp -> SGP30::send_command(Command):int8");
    if(int8 status = this->sensor_enabled(); status < SGP30_OK) return status;

    {
        int32 status = SGP30_OK;

        auto tx = i2c.transaction(this);
        if(tx.status < hkk::bus::i2c::I2C_OK) {
            return this->validate_error(tx.status);
        }

        const uint8 commands[] = {hkk::utils::msb(command), hkk::utils::lsb(command)};
        status = i2c.write(this->cfg.address, commands);

        if(status < hkk::bus::i2c::I2C_OK) {
            return this->validate_error(static_cast<int8>(status));
        }

        HTRACE("[SGP30  ] Address: 0x%02X", this->cfg.address);
        HTRACE("[SGP30  ] Command: 0x%04X", static_cast<uint16>(command));

        return static_cast<int8>(status);
    }
}

int8 SGP30::status(void) {
    HTRACE("sgp30.cpp -> SGP30::status(-):int8");
    return this->status(this->ctx);
}

int8 SGP30::status(Context &result) {
    HTRACE("sgp30.cpp -> SGP30::status(Context&):int8");
    return result.status;
}

int8 SGP30::send_payload(uint8 *payload, size_t len) {
    HTRACE("sgp30.cpp -> SGP30::send_payload(uint8*):int8");
    if(int8 status = this->sensor_enabled(); status < SGP30_OK) return status;

    if(!payload) {
        HERROR("[SGP30  ] Null data pointer passed to function");
        return SGP30_ERROR_NULL_DATA;
    }

    if(len == 0) {
        HERROR("[SGP30  ] Data length is 0");
        return SGP30_ERROR_ZERO_LENGTH;
    }

    {
        int32 status = SGP30_OK;

        auto tx = i2c.transaction(this);
        if(tx.status < hkk::bus::i2c::I2C_OK) {
            return this->validate_error(tx.status);
        }

        status = i2c.write(this->cfg.address, payload, len);
        if(status < hkk::bus::i2c::I2C_OK) {
            return this->validate_error(static_cast<int8>(status));
        }

        HTRACE("[SGP30  ] Address: 0x%02X", this->cfg.address);
        return static_cast<int8>(status);
    }
}

int8 SGP30::compensate_humidity(Context &result) {
    HTRACE("sgp30.cpp -> SGP30::compensate_humidity(Context&):int8");
    return this->compensate_humidity(this->ctx.absolute_humidity);
}

int8 SGP30::compensate_humidity(float32 absolute_humidity) {
    HTRACE("sgp30.cpp -> SGP30::compensate_humidity(float32)");
    if(int8 status = this->sensor_enabled(); status < SGP30_OK) return status;

    int8 status = SGP30_OK;

    if(!this->cfg.humid_compensation) {
        HWARN("[SGP30  ] Humidity compensation for sensor on I2C%d bus is disabled in the config file", i2c.index());
        return SGP30_ERROR_GENERIC;
    }

    // 8.8 format
    uint16 humidity = static_cast<uint16>(absolute_humidity * 256.0f);
    uint8 data[] = {hkk::utils::msb(humidity), hkk::utils::lsb(humidity)};

    status = this->set_absolute_humidity(data);
    if(status < SGP30_OK) return status;

    HDEBUG("[SGP30  ] Humidity compensation enabled");
    HTRACE("[SGP30  ] I2C bus: I2C%d", i2c.index());
    HTRACE("[SGP30  ] Address: 0x%02X", this->cfg.address);

    return status;
}

int8 SGP30::calibrate(void) {
    HTRACE("sgp30.cpp -> SGP30::calibrate(-):int8");
    return this->ctx.status = this->calibrate(this->ctx);
}

int8 SGP30::calibrate(Context &result) {
    HTRACE("sgp30.cpp -> SGP30::calibrate(Context&):int8");
    if(int8 status = this->sensor_enabled(); status < SGP30_OK) return result.status = status;

    int8 status = SGP30_OK;

    HWARN("[SGP30  ] Calibration process takes up to 12 hours before it produces any usable baseline value");
    HINFO("[SGP30  ] Calibration started for SGP30 sensor on bus I2C%d", i2c.index());

    if(!this->cfg.nvm) {
        HERROR("[SGP30  ] Null NVM instance in context; nowhere to save baseline");
        return result.status = SGP30_ERROR_NVM;
    }
    auto *nvm = static_cast<hkk::storage::nvm::NVM*>(this->cfg.nvm);

    {
        auto tx = nvm->transaction(this);
        if(tx.status < hkk::storage::nvm::NVM_OK) {
            return result.status = this->validate_error(tx.status);
        }

        status = nvm->clear(nvm->offset(), nvm->sectors());
        if(status < hkk::storage::nvm::NVM_OK) {
            return result.status = this->validate_error(status);
        }

        HTRACE("[SGP30  ] Address: 0x%02X", this->cfg.address);
    }

    calibration_alarm.callback = SGP30::sensor_calibrated_callback;
    calibration_alarm.data = this;

    hkk::utils::alarm_ms((12 * HOUR), &calibration_alarm, true);
    HDEBUG("[SGP30  ] Calibration alarm started");

    HDEBUG("[SGP30  ] NVM store cleared");
    HTRACE("[SGP30  ] NVM storage offset: %d bytes", nvm->offset());
    HTRACE("[SGP30  ] NVM sector number : %d", nvm->sectors());

    while(this->sensor_calibrated == false) {
        status = this->measure_iaq(result);
        if(status < SGP30_OK) {
            HWARN("[SGP30  ] Error during SGP30 sensor calibration: %s (%d)", hkk::sgp30::rts(status), status);
        }

        status = this->get_iaq_baseline(result);
        if(status < SGP30_OK) {
            HWARN("[SGP30  ] Error during SGP30 sensor calibration: %s (%d)", hkk::sgp30::rts(status), status);
        }

        HTRACE("[SGP30  ] eCO2    : %d", result.eco2);
        HTRACE("[SGP30  ] TVOC    : %d", result.tvoc);
        HTRACE("[SGP30  ] Baseline: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X", result.baseline[0], result.baseline[1], result.baseline[2], result.baseline[3] ,result.baseline[4] ,result.baseline[5]);

        hkk::utils::sleep_ms(1 * SECOND);
    }

    status = this->store_baseline(result);
    if(status < SGP30_OK) {
        HERROR("[SGP30  ] Could not save baseline to NMM storage: %s (%d)", hkk::sgp30::rts(status), status);
    }

    HINFO ("[SGP30  ] Calibration process complete");
    HDEBUG("[SGP30  ] Baseline: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X", result.baseline[0], result.baseline[1], result.baseline[2], result.baseline[3] ,result.baseline[4] ,result.baseline[5]);

    return result.status = status;
}

int8 SGP30::store_baseline(void) {
    HTRACE("sgp30.cpp -> SGP30::store_baseline(-):int8");
    return this->ctx.status = this->store_baseline(this->ctx);
}

int8 SGP30::store_baseline(Context &result) {
    HTRACE("sgp30.cpp -> SGP30::store_baseline(Context&):int8");
    if(int8 status = this->sensor_enabled(); status < SGP30_OK) return result.status = status;

    int8 status = SGP30_OK;

    if(!this->cfg.nvm) {
        HERROR("[SGP30  ] Null NVM instance in context; nowhere to save baseline");
        return result.status = SGP30_ERROR_NVM;
    }
    auto *nvm = static_cast<hkk::storage::nvm::NVM*>(this->cfg.nvm);

    status = this->get_iaq_baseline(result);
    if(status < SGP30_OK) return result.status = status;

    uint8 baseline[DATA_FRAME_LENGTH];
    status = baseline_lookup(baseline);
    if(status < SGP30_OK) return status;

    return result.status = status;
}

int8 SGP30::load_baseline(void) {
    HTRACE("sgp30.cpp -> SGP30::load_baseline(-):int8");
    return this->ctx.status = this->load_baseline(this->ctx);
}

int8 SGP30::load_baseline(Context &result) {
    HTRACE("sgp30.cpp -> SGP30::load_baseline(Context&):int8");
    if(int8 status = this->sensor_enabled(); status < SGP30_OK) return result.status = status;

    int8 status = SGP30_OK;

    if(!this->cfg.nvm) {
        HERROR("[SGP30  ] Null NVM instance in context; nowhere to save baseline");
        return result.status = SGP30_ERROR_NVM;
    }
    auto *nvm = static_cast<hkk::storage::nvm::NVM*>(this->cfg.nvm);

    {
        auto tx = nvm->transaction(this);
        if(tx.status < hkk::storage::nvm::NVM_OK) {
            return result.status = this->validate_error(tx.status);
        }

        status = nvm->read(result.baseline);
        if(status < hkk::storage::nvm::NVM_OK) {
            return result.status = this->validate_error(status);
        }
    }

    status = this->set_iaq_baseline(result.baseline);
    if(status < SGP30_OK) return result.status = status;

    HTRACE("[SGP30  ] Address: 0x%02X", this->cfg.address);
    return result.status = status;
}


uint32 SGP30::eco2(void) {
    HTRACE("sgp30.cpp -> SGP30::eco2(-):int8");
    return this->ctx.eco2;
}

void SGP30::eco2(Context &result) {
    HTRACE("sgp30.cpp -> SGP30::eco2(Context&):void");
    result.eco2 = this->ctx.eco2;
}

uint32 SGP30::tvoc(void) {
    HTRACE("sgp30.cpp -> SGP30::tvoc(-):int8");
    return this->ctx.tvoc;
}

void SGP30::tvoc(Context &result) {
    HTRACE("sgp30.cpp -> SGP30::tvoc(Context&):void");
    result.tvoc = this->ctx.tvoc;
}

uint32 SGP30::h2(void) {
    HTRACE("sgp30.cpp -> SGP30::h2(-):int8");
    return this->ctx.h2;
}

void SGP30::h2(Context &result) {
    HTRACE("sgp30.cpp -> SGP30::h2(Context&):void");
    result.h2 = this->ctx.h2;
}

uint32 SGP30::c2h6o(void) {
    HTRACE("sgp30.cpp -> SGP30::c2h6o(-):int8");
    return this->ctx.c2h6o;
}

void SGP30::c2h6o(Context &result) {
    HTRACE("sgp30.cpp -> SGP30::c2h6o(Context&):void");
    result.c2h6o = this->ctx.c2h6o;
}

uint8 *SGP30::baseline(bool8 tvoc_baseline) {
    HTRACE("sgp30.cpp -> SGP30::baseline(bool8):uint8*");
    return tvoc_baseline ? this->ctx.tvoc_baseline : this->ctx.baseline;
}

void SGP30::baseline(Context &result, bool8 tvoc_baseline) {
    HTRACE("sgp30.cpp -> SGP30::baseline(Context&, bool8):void");

    if(tvoc_baseline == true) {
        std::memcpy(result.tvoc_baseline, this->ctx.tvoc_baseline, DATA_FRAME_LENGTH);
    } else {
        std::memcpy(result.baseline, this->ctx.baseline, DATA_FRAME_LENGTH);
    }
}

uint8 *SGP30::test(void) {
    HTRACE("sgp30.cpp -> SGP30::test(-):uint8*");
    return this->ctx.measure_test;
}

void SGP30::test(Context &result) {
    HTRACE("sgp30.cpp -> SGP30::test(Context&):void");
    std::memcpy(result.measure_test, this->ctx.measure_test, DATA_FRAME_LENGTH);
}

uint8 *SGP30::serial_number(void) {
    HTRACE("sgp30.cpp -> SGP30::serial_number(-):uint8*");
    return this->ctx.serial_number;
}

void SGP30::serial_number(Context &result) {
    HTRACE("sgp30.cpp -> SGP30::serial_number(Context&):void");
    std::memcpy(result.serial_number, this->ctx.serial_number, JUMBO_DATA_FRAME_LENGTH);
}


// Private
bool8 SGP30::baseline_store_callback(void *data) {
    if(!data) return false;

    auto *self = static_cast<SGP30*>(data);
    return self->store_baseline_request = true;
}

bool8 SGP30::sensor_calibrated_callback(void *data) {
    if(!data) return false;

    auto *self = static_cast<SGP30*>(data);
    self->sensor_calibrated = true;

    return false;
}


int8 SGP30::read_raw_data(uint8 *data, size_t len) {
    HTRACE("sgp30.cpp -> SGP30::read_raw_data(uint8*):int8");
    if(int8 status = this->sensor_enabled(); status < SGP30_OK) return status;

    int32 status = SGP30_OK;

    if(!data) {
        HERROR("[SGP30  ] Null data pointer passed to function");
        return SGP30_ERROR_NULL_DATA;
    }

    if(len == 0) {
        HERROR("[SGP30  ] Data length is 0");
        return SGP30_ERROR_ZERO_LENGTH;
    }

    {
        auto tx = i2c.transaction(this);
        if(tx.status < hkk::bus::i2c::I2C_OK) {
            return this->validate_error(tx.status);
        }

        status = i2c.read(this->cfg.address, data, len);
        if(status < hkk::bus::i2c::I2C_OK) {
            return this->validate_error(static_cast<int32>(status));
        }
    }

    HTRACE("[SGP30  ] Raw data read from address: 0x%02X", this->cfg.address);
    return static_cast<int32>(status);
}

int8 SGP30::validate_error(int8 error) {
    HTRACE("sgp30.cpp -> SGP30::validate_error(int8):int8");
    if(int8 status = this->sensor_enabled(); status < SGP30_OK) return status;

    if(error < hkk::storage::nvm::NVM_OK) {
        HERROR("[SGP30  ] Could not write data to NVM storage");
        switch(error) {
            case hkk::storage::nvm::NVM_OK:
                break;

            case hkk::storage::nvm::NVM_DATA_TRUNCATED:
            case hkk::storage::nvm::NVM_NULL_ADDRESS:
            case hkk::storage::nvm::NVM_ERROR_BUSY:
                return SGP30_ERROR_NVM_TRANSACTION;

            case hkk::storage::nvm::NVM_ERROR_NULL_DATA:
            case hkk::storage::nvm::NVM_ERROR_ZERO_LENGTH:
                return SGP30_ERROR_NULL_DATA;

            case hkk::storage::nvm::NVM_ERROR_NULL_CONTEXT:
            case hkk::storage::nvm::NVM_ERROR_NULL_MUTEX:
            case hkk::storage::nvm::NVM_ERROR_OOB:
            case hkk::storage::nvm::NVM_ERROR_GENERIC:
            case hkk::storage::nvm::NVM_ERROR_UNKNOWN:
            case hkk::storage::nvm::NVM_FUNCTION_NOT_IMPLEMENTED:
                return SGP30_ERROR_NVM;
        }
    } 
    
    if(error < hkk::bus::i2c::I2C_OK) {
        HERROR("[SGP30  ] Could not send data to I2C bus");
        switch(error) {
            case hkk::bus::i2c::I2C_OK:
                break;

            case hkk::bus::i2c::I2C_ERROR_NULL_DATA:
            case hkk::bus::i2c::I2C_ERROR_ZERO_LENGTH:
                return SGP30_ERROR_NULL_DATA;

            case hkk::bus::i2c::I2C_ERROR_PARTIAL_WRITE:
            case hkk::bus::i2c::I2C_ERROR_WRITE_FAILED:
            case hkk::bus::i2c::I2C_ERROR_PARTIAL_READ:
            case hkk::bus::i2c::I2C_ERROR_READ_FAILED:
            case hkk::bus::i2c::I2C_ERROR_TIMEOUT:
            case hkk::bus::i2c::I2C_ERROR_BUSY:
                return SGP30_ERROR_I2C_TRANSACTION;

            case hkk::bus::i2c::I2C_ERROR_NULL_CONTEXT:
            case hkk::bus::i2c::I2C_ERROR_NULL_INSTANCE:
            case hkk::bus::i2c::I2C_ERROR_NO_ACK:
            case hkk::bus::i2c::I2C_ERROR_NULL_MUTEX:
            case hkk::bus::i2c::I2C_ERROR_NOT_SUPPORTED:
            case hkk::bus::i2c::I2C_ERROR_GENERIC:
            case hkk::bus::i2c::I2C_FUNCTION_NOT_IMPLEMENTED:
            case hkk::bus::i2c::I2C_ERROR_UNKNOWN:
                return SGP30_ERROR_I2C;
        }
    }
    
    return SGP30_OK;
}

int8 SGP30::baseline_lookup(uint8 *baseline) {
    HTRACE("sgp30.cpp -> SGP30::baseline_lookup(uint8*):int8");
    if(int8 status = this->sensor_enabled(); status < SGP30_OK) return status;

    int8 status = SGP30_OK;

    if(!this->cfg.nvm) {
        HERROR("[SGP30  ] Null NVM instance in context; nowhere to save baseline");
        return SGP30_ERROR_NVM;
    }
    auto *nvm = static_cast<hkk::storage::nvm::NVM*>(this->cfg.nvm);

    {
        auto tx = nvm->transaction(this);
        if(tx.status < hkk::storage::nvm::NVM_OK) {
            return this->validate_error(tx.status);
        }

        uint8 data[DATA_FRAME_LENGTH];
        int8  crc_status = SGP30_ERROR_GENERIC;

        // Clear whole sector if the page number is 0
        // It means that either it's a cold start or we got to the end of a sector
        // We check for previous stored baseline, if there is any, and carry it on 
        if(nvm->pages(true) == 0) {
            for(int8 page = 0; page != nvm->pages(); ++page) {
                status = nvm->read(hkk::storage::nvm::NVM_NULL_ADDRESS, page, data, DATA_FRAME_LENGTH);
                if(status < hkk::storage::nvm::NVM_OK) {
                    return this->validate_error(status);
                }

                int8 crc_eco2 = crc_validate((data + 0), HALF_DATA_FRAME_LENGTH);
                int8 crc_tvoc = crc_validate((data + 3), HALF_DATA_FRAME_LENGTH);

                if(crc_eco2 == SGP30_OK && crc_tvoc == SGP30_OK) {
                    crc_status = SGP30_OK;
                    break;
                }
            }

            if(crc_status < SGP30_OK) {
                HWARN("[SGP30  ] No IAQ baseline stored in NVM storage");
                return SGP30_ERROR_NVM;
            }
            if(crc_status == SGP30_OK) {
                std::memcpy(baseline, data, DATA_FRAME_LENGTH);
            }

            status = nvm->clear(nvm->offset(), nvm->sectors());
            if(status < hkk::storage::nvm::NVM_OK) {
                return this->validate_error(status);
            }
        }

        status = nvm->write(baseline, DATA_FRAME_LENGTH);
        if(status < hkk::storage::nvm::NVM_OK) {
            return this->validate_error(status);
        }
    }

    HTRACE("[SGP30  ] Address: 0x%02X", this->cfg.address);
    return status;
}

}