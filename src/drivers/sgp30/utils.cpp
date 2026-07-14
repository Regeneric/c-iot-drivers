#include <hkk/defines.h>

#include <hkk/utils/utils.hpp>
#include <hkk/logger/logger.h>

#include <hkk/drivers/sgp30/sgp30.hpp>

namespace hkk::sgp30 {

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

    int8 status = SGP30_OK;

    size_t data_frame_length = HALF_DATA_FRAME_LENGTH;
    if(len == 0 || len > data_frame_length) {
        HERROR("[SGP30  ] Data frame length invalid");
        HERROR("[SGP30  ] Expected: %d", data_frame_length);
        HERROR("[SGP30  ] Got     : %d", len);

        return SGP30_ERROR_CRC;
    }

    uint8 crc = 0;
    uint8 sgp_crc = data[2];

    status = crc_calculate(crc, data, (len-1));
    if(status < SGP30_OK) return status;

    if(crc != sgp_crc) {
        HWARN ("[SGP30  ] Checksum invalid");
        HTRACE("[SGP30  ] Expected: 0x%02X", sgp_crc);
        HTRACE("[SGP30  ] Got: 0x%02X", crc);

        return SGP30_ERROR_CRC;
    }

    HTRACE("[SGP30  ] Checksum valid");
    return status;
}

const char *rts(int8 status) {
    switch(status) {
        case SGP30_OK:                          return "SGP30_OK";                      
        case SGP30_ERROR_NULL_CONTEXT:          return "SGP30_ERROR_NULL_CONTEXT";      
        case SGP30_ERROR_NULL_INSTANCE:         return "SGP30_ERROR_NULL_INSTANCE";     
        case SGP30_ERROR_WRITE_FAILED:          return "SGP30_ERROR_WRITE_FAILED";      
        case SGP30_ERROR_I2C:                   return "SGP30_ERROR_I2C";               
        case SGP30_ERROR_I2C_TRANSACTION:       return "SGP30_ERROR_I2C_TRANSACTION";   
        case SGP30_ERROR_SENSOR_DISABLED:       return "SGP30_ERROR_SENSOR_DISABLED";
        case SGP30_ERROR_READ_FAILED:           return "SGP30_ERROR_READ_FAILED";       
        case SGP30_ERROR_DEVICE_NOT_FOUND:      return "SGP30_ERROR_DEVICE_NOT_FOUND";  
        case SGP30_ERROR_TIMEOUT:               return "SGP30_ERROR_TIMEOUT";           
        case SGP30_ERROR_NULL_DATA:             return "SGP30_ERROR_NULL_DATA";          
        case SGP30_ERROR_CRC:                   return "SGP30_ERROR_CRC";                
        case SGP30_ERROR_ZERO_LENGTH:           return "SGP30_ERROR_ZERO_LENGTH";        
        case SGP30_ERROR_OOB:                   return "SGP30_ERROR_OOB";
        case SGP30_ERROR_GENERIC:               return "SGP30_ERROR_GENERIC";
        case SGP30_FUNCTION_NOT_IMPLEMENTED:    return "SGP30_FUNCTION_NOT_IMPLEMENTED"; 
        default:                                return "SGP30_ERROR_UNKNOWN";
    }
}

}