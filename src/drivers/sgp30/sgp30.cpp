#include <hkk/defines.h>

#include <hkk/utils/utils.hpp>
#include <hkk/logger/logger.h>
#include <hkk/bus/i2c/i2c.hpp>

#include <hkk/drivers/sgp30/sgp30.hpp>

#include <cstring>

namespace hkk::sgp30 {

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

int8 SGP30::setup(void) {
    HTRACE("sgp30.cpp -> SGP30::init(-):int8");
    if(int8 status = this->sensor_enabled(); status < SGP30_OK) return status;

    if(!this->cfg.enable) {
        HINFO("[SGP30  ] SGP30 sensor on I2C%d bus is disabled in the config file", i2c.index());
        return SGP30_ERROR_SENSOR_DISABLED;
    }

    HWARN("[SGP30  ] No IAQ baseline provided");
    HWARN("[SGP30  ] Performing a cold start");

    int8 status = this->iaq_init();
    if(status < SGP30_OK) return status;

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
        switch(tx.status) {
            case hkk::bus::I2C_OK: break;

            case hkk::bus::I2C_ERROR_BUSY:               
                return SGP30_ERROR_I2C_TRANSACTION;
            
            case hkk::bus::I2C_ERROR_NULL_CONTEXT:       
            case hkk::bus::I2C_ERROR_NULL_INSTANCE:      
            case hkk::bus::I2C_ERROR_NULL_MUTEX:            
            case hkk::bus::I2C_FUNCTION_NOT_IMPLEMENTED: 
            default: 
                return SGP30_ERROR_I2C;
        }

        int32 status = i2c.read(this->cfg.address, data, len);

        if(status < hkk::bus::I2C_OK) {
            HERROR("[SGP30  ] Could not read data from device at address: 0x%02X", this->cfg.address);

            switch(status) {
                case hkk::bus::I2C_OK: break;

                case hkk::bus::I2C_ERROR_NO_ACK:        
                    return SGP30_ERROR_DEVICE_NOT_FOUND;

                case hkk::bus::I2C_ERROR_NULL_DATA:
                case hkk::bus::I2C_ERROR_ZERO_LENGTH:
                    return SGP30_ERROR_NULL_DATA;
                
                case hkk::bus::I2C_ERROR_WRITE_FAILED:
                case hkk::bus::I2C_ERROR_PARTIAL_WRITE: 
                    return SGP30_ERROR_WRITE_FAILED;
                
                case hkk::bus::I2C_ERROR_NULL_CONTEXT:
                case hkk::bus::I2C_ERROR_NULL_INSTANCE:
                case hkk::bus::I2C_ERROR_NULL_MUTEX:
                case hkk::bus::I2C_FUNCTION_NOT_IMPLEMENTED:
                default:
                    return SGP30_ERROR_I2C;
            }
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
        switch(tx.status) {
            case hkk::bus::I2C_OK: break;

            case hkk::bus::I2C_ERROR_BUSY:               
                return SGP30_ERROR_I2C_TRANSACTION;
            
            case hkk::bus::I2C_ERROR_NULL_CONTEXT:       
            case hkk::bus::I2C_ERROR_NULL_INSTANCE:      
            case hkk::bus::I2C_ERROR_NULL_MUTEX:            
            case hkk::bus::I2C_FUNCTION_NOT_IMPLEMENTED: 
            default: 
                return SGP30_ERROR_I2C;
        }

        const uint8 commands[] = {hkk::utils::msb(command), hkk::utils::lsb(command)};
        int32 status = i2c.write(this->cfg.address, commands);

        if(status < hkk::bus::I2C_OK) {
            HERROR("[SGP30  ] Could not write data to device at address: 0x%02X", this->cfg.address);
        
            switch(status) {
                case hkk::bus::I2C_OK: break;

                case hkk::bus::I2C_ERROR_NO_ACK:        
                    return SGP30_ERROR_DEVICE_NOT_FOUND;

                case hkk::bus::I2C_ERROR_NULL_DATA:
                case hkk::bus::I2C_ERROR_ZERO_LENGTH:
                    return SGP30_ERROR_NULL_DATA;
                
                case hkk::bus::I2C_ERROR_WRITE_FAILED:
                case hkk::bus::I2C_ERROR_PARTIAL_WRITE: 
                    return SGP30_ERROR_WRITE_FAILED;
                
                case hkk::bus::I2C_ERROR_NULL_CONTEXT:
                case hkk::bus::I2C_ERROR_NULL_INSTANCE:
                case hkk::bus::I2C_ERROR_NULL_MUTEX:
                case hkk::bus::I2C_FUNCTION_NOT_IMPLEMENTED:
                default:
                    return SGP30_ERROR_I2C;
            }
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
        switch(tx.status) {
            case hkk::bus::I2C_OK: break;

            case hkk::bus::I2C_ERROR_BUSY:               
                return SGP30_ERROR_I2C_TRANSACTION;
            
            case hkk::bus::I2C_ERROR_NULL_CONTEXT:       
            case hkk::bus::I2C_ERROR_NULL_INSTANCE:      
            case hkk::bus::I2C_ERROR_NULL_MUTEX:            
            case hkk::bus::I2C_FUNCTION_NOT_IMPLEMENTED: 
            default: 
                return SGP30_ERROR_I2C;
        }

        int32 status = i2c.write(this->cfg.address, payload, len);
        
        if(status < hkk::bus::I2C_OK) {
            HERROR("[SGP30  ] Could not write data to device at address: 0x%02X", this->cfg.address);

            switch(status) {
                case hkk::bus::I2C_OK: break;

                case hkk::bus::I2C_ERROR_NO_ACK:        
                    return SGP30_ERROR_DEVICE_NOT_FOUND;

                case hkk::bus::I2C_ERROR_NULL_DATA:
                case hkk::bus::I2C_ERROR_ZERO_LENGTH:
                    return SGP30_ERROR_NULL_DATA;
                
                case hkk::bus::I2C_ERROR_WRITE_FAILED:
                case hkk::bus::I2C_ERROR_PARTIAL_WRITE: 
                    return SGP30_ERROR_WRITE_FAILED;
                
                case hkk::bus::I2C_ERROR_NULL_CONTEXT:
                case hkk::bus::I2C_ERROR_NULL_INSTANCE:
                case hkk::bus::I2C_ERROR_NULL_MUTEX:
                case hkk::bus::I2C_FUNCTION_NOT_IMPLEMENTED:
                default:
                    return SGP30_ERROR_I2C;
            }
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


static Context calibration_result;

bool8 calibration_callback(void *data) {
    auto *self = static_cast<SGP30*>(data);
    if(!self) return false;

    if(calibration_result.calibrated == true) {
        self->set_iaq_baseline(calibration_result.baseline);

        HINFO("[SGP30  ] Sensor calibrated");
        HINFO("[SGP30  ] Baseline: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X", 
            calibration_result.baseline[0], calibration_result.baseline[1], 
            calibration_result.baseline[2], calibration_result.baseline[3], 
            calibration_result.baseline[4], calibration_result.baseline[5]);

        return false;
    }

    int8 status = self->measure_iaq(calibration_result);
    if(status < SGP30_OK) {
        HWARN("[SGP30  ] Error during SGP30 sensor calibration: %s (%d)", hkk::sgp30::rts(status), status);
    } 

    return true;
}

int8 calibrated_callback(void *data) {
    calibration_result.calibrated = true;
    return 0;
}

int8 SGP30::calibrate() {
    HTRACE("sgp30.cpp -> SGP30::calibrate():int8");
    if(int8 status = this->sensor_enabled(); status < SGP30_OK) return status;

    HWARN ("[SGP30  ] Calibration process takes up to 12 hours before it produces any usable baseline value");

    int8 status = hkk::utils::repeating_timer_ms(-(1 * SECOND), calibration_callback, this);
    if(status) HTRACE("[SGP30  ] Repeating timer started");
    else {
        HERROR("[SGP30  ] Could not start repeating timer");
        return SGP30_ERROR_GENERIC;
    }

    status = hkk::utils::alarm_ms((12 * HOUR), calibrated_callback, NULL);
    if(status) HTRACE("[SGP30  ] Alarm timer started");
    else {
        HERROR("[SGP30  ] Could not start alarm timer");
        return SGP30_ERROR_GENERIC;
    }

    HINFO("[SGP30  ] Calibration started for SGP30 sensor on bus I2C%d", i2c.index());
    return SGP30_OK;
}

}