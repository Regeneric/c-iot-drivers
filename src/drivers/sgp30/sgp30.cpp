#include <hkk/defines.h>

#include <hkk/utils/utils.hpp>
#include <hkk/logger/logger.h>
#include <hkk/bus/i2c/i2c.hpp>
#include <hkk/drivers/sgp30/sgp30.hpp>
#include <hkk/storage/nvm.hpp>

#include <cstring>

namespace hkk::sgp30 {

static bool8 save_baseline = false;
static bool8 baseline_callback(void *data) {return save_baseline = true;}
static hkk::utils::AlarmContext baseline_alarm {
    .callback = baseline_callback,
    .data = nullptr,
};

int8 SGP30::setup(uint8 *baseline, size_t len) {
    HTRACE("sgp30.cpp -> SGP30::init(uint8*, size_t):int8");
    if(int8 status = this->sensor_enabled(); status < SGP30_OK) return status;

    if(!this->cfg.enable) {
        HINFO("[SGP30  ] SGP30 sensor on I2C%d bus is disabled in the config file", i2c.index());
        return SGP30_ERROR_SENSOR_DISABLED;
    }

    if(!baseline) {
        HERROR("[SGP30  ] Null data pointer passed to function");
        return SGP30_ERROR_NULL_DATA;
    }

    if(len == 0) {
        HERROR("[SGP30  ] Data length is 0");
        return SGP30_ERROR_ZERO_LENGTH;
    } 

    if(len > HALF_DATA_FRAME_LENGTH) {
        HERROR("[SGP30  ] Data length out of bands");
        return SGP30_ERROR_OOB;
    }

    int8 status = this->iaq_init();
    if(status < SGP30_OK) return status;

    status = this->set_iaq_baseline(baseline, len);
    if(status < SGP30_OK) return status;

    return SGP30_OK;
}

// TODO: now setup assumes that baseline exists on page 0 and only there
int8 SGP30::setup(void) {
    HTRACE("sgp30.cpp -> SGP30::init(-):int8");
    if(int8 status = this->sensor_enabled(); status < SGP30_OK) return status;

    if(!this->cfg.enable) {
        HINFO("[SGP30  ] SGP30 sensor on I2C%d bus is disabled in the config file", i2c.index());
        return SGP30_ERROR_SENSOR_DISABLED;
    }

    HWARN("[SGP30  ] No IAQ baseline provided");
    
    if(!this->cfg.nvm) {
        HERROR("[SGP30  ] Null NVM instance in context; nowhere to save baseline");
        return SGP30_ERROR_NVM;
    }
    auto *nvm = static_cast<hkk::storage::nvm::NVM*>(this->cfg.nvm);

    uint8 baseline[6];
    int8 status = nvm->read(baseline);

    int8 crc_eco2 = crc_validate((baseline + 0), HALF_DATA_FRAME_LENGTH);
    int8 crc_tvoc = crc_validate((baseline + 3), HALF_DATA_FRAME_LENGTH);

    if(status < SGP30_OK || crc_eco2 < SGP30_OK || crc_tvoc < SGP30_OK) {
        HWARN("[SGP30  ] No IAQ baseline stored in NVM storage");
        HWARN("[SGP30  ] Performing a cold start");

        status = this->iaq_init();
        if(status < SGP30_OK) return status;

        return SGP30_OK;
    }

    HINFO ("[SGP30  ] IAQ baseline found in NVM storage");
    HDEBUG("[SGP30  ] Using baseline {0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X}", baseline[0], baseline[1], baseline[2], baseline[3], baseline[4], baseline[5]);

    status = this->iaq_init();
    if(status < SGP30_OK) return status;

    status = this->set_iaq_baseline(baseline);
    if(status < SGP30_OK) return status;

    // hkk::utils::alarm_ms((1 * HOUR), &baseline_alarm, true);

    return SGP30_OK;
}

int8 SGP30::read_raw_data(uint8 *data, size_t len) {
    HTRACE("sgp30.cpp -> SGP30::read_raw_data(uint8*):int8");
    if(int8 status = this->sensor_enabled(); status < SGP30_OK) return status;

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

        int32 status = i2c.read(this->cfg.address, data, len);
        if(status < hkk::bus::i2c::I2C_OK) {
            return this->validate_error(status);
        }
    }

    HTRACE("[SGP30  ] Raw data read from address: 0x%02X", this->cfg.address);
    return SGP30_OK;
}

int8 SGP30::send_command(Command command) {
    HTRACE("sgp30.cpp -> SGP30::send_command(Command):int8");
    if(int8 status = this->sensor_enabled(); status < SGP30_OK) return status;

    {
        auto tx = i2c.transaction(this);
        if(tx.status < hkk::bus::i2c::I2C_OK) {
            return this->validate_error(tx.status);
        }

        const uint8 commands[] = {hkk::utils::msb(command), hkk::utils::lsb(command)};
        int32 status = i2c.write(this->cfg.address, commands);

        if(status < hkk::bus::i2c::I2C_OK) {
            return this->validate_error(status);
        }

        HTRACE("[SGP30  ] Address: 0x%02X", this->cfg.address);
        HTRACE("[SGP30  ] Command: 0x%04X", static_cast<uint16>(command));

        return SGP30_OK;
    }
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
        auto tx = i2c.transaction(this);
        if(tx.status < hkk::bus::i2c::I2C_OK) {
            return this->validate_error(tx.status);
        }

        int32 status = i2c.write(this->cfg.address, payload, len);
        if(status < hkk::bus::i2c::I2C_OK) {
            return this->validate_error(status);
        }

        HTRACE("[SGP30  ] Address: 0x%02X", this->cfg.address);
        return SGP30_OK;
    }
}

int8 SGP30::compensate_humidity(float32 absolute_humidity) {
    HTRACE("sgp30.cpp -> SGP30::compensate_humidity(float32)");
    if(int8 status = this->sensor_enabled(); status < SGP30_OK) return status;

    if(!this->cfg.humid_compensation) {
        HWARN("[SGP30  ] Humidity compensation for sensor on I2C%d bus is disabled in the config file", i2c.index());
        return SGP30_ERROR_GENERIC;
    }

    // 8.8 format
    uint16 humidity = static_cast<uint16>(absolute_humidity * 256.0f);
    uint8 data[] = {hkk::utils::msb(humidity), hkk::utils::lsb(humidity)};

    int8 status = this->set_absolute_humidity(data);
    if(status < SGP30_OK) return status;

    HDEBUG("[SGP30  ] Humidity compensation enabled");
    HTRACE("[SGP30  ] I2C bus: I2C%d", i2c.index());
    HTRACE("[SGP30  ] Address: 0x%02X", this->cfg.address);

    return SGP30_OK;
}


static bool8 calibrated = false;
static bool8 calibration_callback(void *data) {
    calibrated = true;
    return false;
}
static hkk::utils::AlarmContext calibration_alarm {
    .callback = calibration_callback,
    .data = nullptr,
};

int8 SGP30::calibrate(Context &result) {
    HTRACE("sgp30.cpp -> SGP30::calibrate(Context&):int8");
    if(int8 status = this->sensor_enabled(); status < SGP30_OK) return result.status = status;

    HWARN ("[SGP30  ] Calibration process takes up to 12 hours before it produces any usable baseline value");
    HDEBUG("[SGP30  ] Calibration started for SGP30 sensor on bus I2C%d", i2c.index());

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

        int8 status = nvm->clear(nvm->offset(), nvm->sectors());
        if(status < hkk::storage::nvm::NVM_OK) {
            return result.status = this->validate_error(status);
        }

        HTRACE("[SGP30  ] Address: 0x%02X", this->cfg.address);
    }

    hkk::utils::alarm_ms((12 * HOUR), &calibration_alarm, true);

    HDEBUG("[SGP30  ] NVM store cleared");
    HTRACE("[SGP30  ] NVM storage offset: %d bytes", nvm->offset());
    HTRACE("[SGP30  ] NVM sector number : %d", nvm->sectors());

    while(calibrated == false) {
        int8 status = this->measure_iaq(result);
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

    int8 status = this->store_baseline(result);
    if(status < SGP30_OK) {
        HERROR("[SGP30  ] Could not save baseline to NMM storage: %s (%d)", hkk::sgp30::rts(status), status);
    }

    HINFO ("[SGP30  ] Calibration process complete");
    HDEBUG("[SGP30  ] Baseline: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X", result.baseline[0], result.baseline[1], result.baseline[2], result.baseline[3] ,result.baseline[4] ,result.baseline[5]);

    return result.status = SGP30_OK;
}

int8 SGP30::store_baseline(Context &result) {
    HTRACE("sgp30.cpp -> SGP30::store_baseline(Context&):int8");
    if(int8 status = this->sensor_enabled(); status < SGP30_OK) return result.status = status;

    if(!this->cfg.nvm) {
        HERROR("[SGP30  ] Null NVM instance in context; nowhere to save baseline");
        return result.status = SGP30_ERROR_NVM;
    }
    auto *nvm = static_cast<hkk::storage::nvm::NVM*>(this->cfg.nvm);

    int8 status = this->get_iaq_baseline(result);
    if(status < SGP30_OK) return result.status = status;

    {
        auto tx = nvm->transaction(this);
        if(tx.status < hkk::storage::nvm::NVM_OK) {
            return result.status = this->validate_error(tx.status);
        }

        uint8 baseline[DATA_FRAME_LENGTH];
        int8  crc_status = SGP30_ERROR_GENERIC;

        // Clear whole sector if the page number is 0
        // It means that either it's a cold start or we got to the end of a sector
        // We check for previous stored baseline, if there is any, and carry it on 
        if(nvm->pages(true) == 0) {
            for(int8 page = 0; page != nvm->pages(); ++page) {
                int8 status = nvm->read(hkk::storage::nvm::NVM_NULL_ADDRESS, page, baseline, DATA_FRAME_LENGTH);
                if(status < hkk::storage::nvm::NVM_OK) {
                    return result.status = this->validate_error(status);
                }

                int8 crc_eco2 = crc_validate((baseline + 0), HALF_DATA_FRAME_LENGTH);
                int8 crc_tvoc = crc_validate((baseline + 3), HALF_DATA_FRAME_LENGTH);

                if(crc_eco2 == SGP30_OK && crc_tvoc == SGP30_OK) {
                    crc_status = SGP30_OK;
                    break;
                }
            }

            if(crc_status < SGP30_OK) HWARN("[SGP30  ] No IAQ baseline stored in NVM storage");
            if(crc_status == SGP30_OK) {
                std::memcpy(result.baseline, baseline, DATA_FRAME_LENGTH);
            }

            int8 status = nvm->clear(nvm->offset(), nvm->sectors());
            if(status < hkk::storage::nvm::NVM_OK) {
                return result.status = this->validate_error(status);
            }
        }

        int8 status = nvm->write(result.baseline);
        if(status < hkk::storage::nvm::NVM_OK) {
            return result.status = this->validate_error(status);
        }

        HTRACE("[SGP30  ] Address: 0x%02X", this->cfg.address);
        return result.status = SGP30_OK;
    }
}

int8 SGP30::load_baseline(Context &result) {
    HTRACE("sgp30.cpp -> SGP30::load_baseline(Context&):int8");
    if(int8 status = this->sensor_enabled(); status < SGP30_OK) return result.status = status;

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

        int8 status = nvm->read(result.baseline);
        if(status < hkk::storage::nvm::NVM_OK) {
            return result.status = this->validate_error(status);
        }
    }

    int8 status = this->set_iaq_baseline(result.baseline);
    if(status < SGP30_OK) return result.status = status;

    HTRACE("[SGP30  ] Address: 0x%02X", this->cfg.address);
    return result.status = SGP30_OK;
}

int8 SGP30::validate_error(int8 error) {
    HTRACE("sgp30.cpp -> SGP30::validate_error(int8):int8");

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

// int8 SGP30::data_frame(Command command, uint8 *data, size_t len) {
//     HTRACE("sgp30.cpp -> SGP30::data_frame(uint8*, size_t):int8");
//
//     uint8 crc = 0;
//     int8 status = crc_calculate(crc, data, len);
//     if(status < SGP30_OK) return status;
//
//     uint8 payload[COMMAND_FRAME_LENGTH + HALF_DATA_FRAME_LENGTH]; 
//     uint8 cmd[COMMAND_FRAME_LENGTH] = {hkk::utils::msb(command), hkk::utils::lsb(command)};
//
//     std::memcpy(payload, cmd, COMMAND_FRAME_LENGTH);
//     std::memcpy((payload + COMMAND_FRAME_LENGTH), data, len);
//     payload[COMMAND_FRAME_LENGTH + len] = crc;
//
//     return SGP30_OK;
// }


// static Context calibration_result;

// bool8 calibration_callback(void *data) {
//     auto *self = static_cast<SGP30*>(data);
//     if(!self) return false;

//     if(calibration_result.calibrated == true) {
//         self->set_iaq_baseline(calibration_result.baseline);

//         HINFO("[SGP30  ] Sensor calibrated");
//         HINFO("[SGP30  ] Baseline: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X", 
//             calibration_result.baseline[0], calibration_result.baseline[1], 
//             calibration_result.baseline[2], calibration_result.baseline[3], 
//             calibration_result.baseline[4], calibration_result.baseline[5]);

//         return false;
//     }

//     int8 status = self->measure_iaq(calibration_result);
//     if(status < SGP30_OK) {
//         HWARN("[SGP30  ] Error during SGP30 sensor calibration: %s (%d)", hkk::sgp30::rts(status), status);
//     } 

//     return true;
// }

// int8 calibrated_callback(void *data) {
//     calibration_result.calibrated = true;
//     return 0;
// }

// int8 SGP30::calibrate() {
//     HTRACE("sgp30.cpp -> SGP30::calibrate():int8");
//     if(int8 status = this->sensor_enabled(); status < SGP30_OK) return status;

//     HWARN ("[SGP30  ] Calibration process takes up to 12 hours before it produces any usable baseline value");

//     int8 status = hkk::utils::repeating_timer_ms(-(1 * SECOND), (void*)calibration_callback, this);
//     if(status) HTRACE("[SGP30  ] Repeating timer started");
//     else {
//         HERROR("[SGP30  ] Could not start repeating timer");
//         return SGP30_ERROR_GENERIC;
//     }

//     status = hkk::utils::alarm_ms((12 * HOUR), (void*)calibrated_callback, NULL);
//     if(status) HTRACE("[SGP30  ] Alarm timer started");
//     else {
//         HERROR("[SGP30  ] Could not start alarm timer");
//         return SGP30_ERROR_GENERIC;
//     }

//     HINFO("[SGP30  ] Calibration started for SGP30 sensor on bus I2C%d", i2c.index());
//     return SGP30_OK;
// }

}