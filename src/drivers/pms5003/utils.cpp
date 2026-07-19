#include <hkk/defines.h>

#include <hkk/bus/uart/uart.hpp>

#include <hkk/drivers/pms5003/pms5003.hpp>

namespace hkk::pms5003 {

int8 PMS5003::checksum_calculate(uint16 &checksum, uint8 *data, size_t len) {
    HTRACE("pms5003.cpp -> crc_calculate(uint8&, uint8*, size_t):int8");

    if(!data) {
        HERROR("[PMS5003] Null data pointer passed to function");
        return PMS5003_ERROR_NULL_DATA;
    }

    if(len < 2) {
        HERROR("[PMS5003] Data frame length invalid");
        return PMS5003_ERROR_CRC;
    }

    uint16 crc = 0;

    for(size_t i = 0; i != (len - 2); ++i) crc += data[i];                           // We need to calculate the sum of the first 30 bytes of the data frame

    checksum = crc;
    HTRACE("[PMS5003] Checksum calculated: 0x%04X", checksum);

    return PMS5003_OK;
}

int8 PMS5003::checksum_validate(uint8 *data, size_t len) {
    HTRACE("pms5003.cpp -> crc_validate(uint8*, size_t):int8");

    if(!data) {
        HERROR("[PMS5003] Null data pointer passed to function");
        return PMS5003_ERROR_NULL_DATA;
    }

    if(len < 2) {
        HERROR("[PMS5003] Data frame length invalid");
        return PMS5003_ERROR_CRC;
    }

    int8 status = PMS5003_OK;

    uint16 crc = 0;
    uint16 pms_crc = 0;

    pms_crc = ((data[len - 2] << 8) | data[len - 1]);

    status = checksum_calculate(crc, data, len);
    if(status < PMS5003_OK) return status;

    if(crc != pms_crc) {
        HWARN ("[PMS5003] Checksum invalid");
        HTRACE("[PMS5003] Expected: 0x%04X", pms_crc);
        HTRACE("[PMS5003] Got: 0x%04X", crc);

        return PMS5003_ERROR_CRC;
    }

    HTRACE("[PMS5003] Checksum valid");
    return status;
}

const char *rts(int8 status) {
    switch(status) {
        case PMS5003_OK:                          return "PMS5003_OK";                      
        case PMS5003_ERROR_NULL_CONTEXT:          return "PMS5003_ERROR_NULL_CONTEXT";      
        case PMS5003_ERROR_NULL_INSTANCE:         return "PMS5003_ERROR_NULL_INSTANCE";     
        case PMS5003_ERROR_WRITE_FAILED:          return "PMS5003_ERROR_WRITE_FAILED";      
        case PMS5003_ERROR_I2C:                   return "PMS5003_ERROR_I2C";               
        case PMS5003_ERROR_I2C_TRANSACTION:       return "PMS5003_ERROR_I2C_TRANSACTION";   
        case PMS5003_ERROR_SENSOR_DISABLED:       return "PMS5003_ERROR_SENSOR_DISABLED";
        case PMS5003_ERROR_READ_FAILED:           return "PMS5003_ERROR_READ_FAILED";       
        case PMS5003_ERROR_DEVICE_NOT_FOUND:      return "PMS5003_ERROR_DEVICE_NOT_FOUND";  
        case PMS5003_ERROR_TIMEOUT:               return "PMS5003_ERROR_TIMEOUT";           
        case PMS5003_ERROR_NULL_DATA:             return "PMS5003_ERROR_NULL_DATA";          
        case PMS5003_ERROR_CRC:                   return "PMS5003_ERROR_CRC";                
        case PMS5003_ERROR_ZERO_LENGTH:           return "PMS5003_ERROR_ZERO_LENGTH";        
        case PMS5003_ERROR_OOB:                   return "PMS5003_ERROR_OOB";
        case PMS5003_ERROR_GENERIC:               return "PMS5003_ERROR_GENERIC";
        case PMS5003_ERROR_NVM:                   return "PMS5003_ERROR_NVM";
        case PMS5003_ERROR_NVM_TRANSACTION:       return "PMS5003_ERROR_NVM_TRANSACTION";
        case PMS5003_FUNCTION_NOT_IMPLEMENTED:    return "PMS5003_FUNCTION_NOT_IMPLEMENTED"; 
        default:                                  return "PMS5003_ERROR_UNKNOWN";
    }
}
    
}