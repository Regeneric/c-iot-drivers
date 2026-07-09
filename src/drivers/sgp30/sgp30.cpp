#include <hkk/defines.h>

#include <hkk/utils/utils.hpp>
#include <hkk/logger/logger.h>
#include <hkk/bus/i2c/i2c.hpp>

#include <hkk/drivers/sgp30/sgp30.hpp>


namespace hkk::sgp30 {

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


int8 SGP30::init() {
    HTRACE("sgp30.cpp -> init(-):int8");

}

}