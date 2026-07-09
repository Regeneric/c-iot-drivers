#include <hkk/defines.h>

#include <hkk/utils/utils.hpp>
#include <hkk/logger/logger.h>
#include <hkk/bus/i2c/i2c.hpp>

#include <hkk/drivers/sgp30/sgp30.hpp>


namespace hkk::sgp30 {

int8 SGP30::setup(uint8 *baseline, size_t len) {
    HTRACE("sgp30.cpp -> SGP30::init(uint8*, size_t):int8");

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

int8 crc_calculate(uint8 &checksum, uint8 *data, size_t len) {
    HTRACE("sgp30.cpp -> crc_calculate(uint8&, uint8*, size_t):int8");

    size_t data_frame_length = (HALF_DATA_FRAME_LENGTH - 1);
    if(len == 0 || len > data_frame_length) {
        HERROR("[SGP30  ] Data frame length invalid");
        HERROR("[SGP30  ] Expected: %d", data_frame_length);
        HERROR("[SGP30  ] Got     : %d", len);

        return SGP30_ERROR_CRC;
    }

    uint8 crc   = Base; // 0b11111111
    uint8 mask  = Mask; // 0b00110001
    uint8 msbit = 0;

    for(size_t i = 0; i != data_frame_length; ++i) {
        crc = (crc ^ data[i]);
        for(size_t j = 0; j != 8; ++j) {
            msbit = (crc & Msbit);  // Most significant BIT
            crc = (crc << 1);
            if(msbit != 0) {
                crc = (crc ^ mask); // XOR
            }
        }
    }

    checksum = crc;
    HTRACE("[SGP30  ] Checksum calculated: %d", checksum);

    return SGP30_OK;
}

int8 crc_validate(uint8 *data, size_t len) {
    HTRACE("sgp30.cpp -> crc_validate(uint8*, size_t):int8");

    size_t data_frame_length = HALF_DATA_FRAME_LENGTH;
    if(len == 0 || len > data_frame_length) {
        HERROR("[SGP30  ] Data frame length invalid");
        HERROR("[SGP30  ] Expected: %d", data_frame_length);
        HERROR("[SGP30  ] Got     : %d", len);

        return SGP30_ERROR_CRC;
    }

    uint8 crc = 0;
    uint8 sgp_crc = data[2];

    int8 status = crc_calculate(crc, data, (len-1));
    if(status < SGP30_OK) return status;

    if(crc != sgp_crc) {
        HWARN ("[SGP30  ] Checksum invalid");
        HTRACE("[SGP30  ] Expected: 0x%X", sgp_crc);
        HTRACE("[SGP30  ] Got: 0x%X", crc);

        return SGP30_ERROR_CRC;
    }

    HTRACE("[SGP30  ] Checksum valid");
    return SGP30_OK;
}


int8 SGP30::read_raw_data(uint8 *data, size_t len) {
    HTRACE("sgp30.cpp -> SGP30::read_raw_data(uint8*):int8");

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

}