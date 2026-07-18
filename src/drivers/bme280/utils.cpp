#include <hkk/defines.h>

#include <hkk/utils/utils.hpp>
#include <hkk/logger/logger.h>

#include <hkk/drivers/bme280/bme280.hpp>

namespace hkk::bme280 {

const char *rts(int8 status) {
    switch(status) {
        case BME280_OK:                          return "BME280_OK";                      
        case BME280_ERROR_NULL_CONTEXT:          return "BME280_ERROR_NULL_CONTEXT";      
        case BME280_ERROR_NULL_INSTANCE:         return "BME280_ERROR_NULL_INSTANCE";     
        case BME280_ERROR_WRITE_FAILED:          return "BME280_ERROR_WRITE_FAILED";      
        case BME280_ERROR_I2C:                   return "BME280_ERROR_I2C";               
        case BME280_ERROR_I2C_TRANSACTION:       return "BME280_ERROR_I2C_TRANSACTION";   
        case BME280_ERROR_SENSOR_DISABLED:       return "BME280_ERROR_SENSOR_DISABLED";
        case BME280_ERROR_READ_FAILED:           return "BME280_ERROR_READ_FAILED";       
        case BME280_ERROR_DEVICE_NOT_FOUND:      return "BME280_ERROR_DEVICE_NOT_FOUND";  
        case BME280_ERROR_TIMEOUT:               return "BME280_ERROR_TIMEOUT";           
        case BME280_ERROR_NULL_DATA:             return "BME280_ERROR_NULL_DATA";          
        case BME280_ERROR_CRC:                   return "BME280_ERROR_CRC";                
        case BME280_ERROR_ZERO_LENGTH:           return "BME280_ERROR_ZERO_LENGTH";        
        case BME280_ERROR_OOB:                   return "BME280_ERROR_OOB";
        case BME280_ERROR_GENERIC:               return "BME280_ERROR_GENERIC";
        case BME280_ERROR_NVM:                   return "BME280_ERROR_NVM";
        case BME280_ERROR_NVM_TRANSACTION:       return "BME280_ERROR_NVM_TRANSACTION";
        case BME280_FUNCTION_NOT_IMPLEMENTED:    return "BME280_FUNCTION_NOT_IMPLEMENTED"; 
        default:                                 return "BME280_ERROR_UNKNOWN";
    }
}

}