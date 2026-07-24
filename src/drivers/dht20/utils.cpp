#include <hkk/defines.h>

#include <hkk/utils/utils.hpp>
#include <hkk/logger/logger.h>

#include <hkk/drivers/dht20/dht20.hpp>

namespace hkk::dht20 {

int8 DHT20::crc_calculate(uint8 &checksum, uint8 *data, size_t len) {
    HTRACE("dht20.cpp -> s:DHT20::crc_calculate(uint8&, uint8*, size_t):int8");

    if(len == 0 || len > (DATA_FRAME_LENGTH - 1)) {
        HERROR("[DHT20  ] Data frame length invalid");
        HERROR("[DHT20  ] Expected: %d", (DATA_FRAME_LENGTH - 1));
        HERROR("[DHT20  ] Got     : %d", len);

        return DHT20_ERROR_CRC;
    }

    uint8 crc   = static_cast<uint8>(CRC::Base); // 0b11111111
    uint8 mask  = static_cast<uint8>(CRC::Mask); // 0b00110001
    uint8 msbit = 0;

    for(size_t i = 0; i != len; ++i) {
        crc = (crc ^ data[i]);
        for(size_t j = 0; j != 8; ++j) {
            msbit = (crc & static_cast<uint8>(CRC::Msbit));  // Most significant BIT
            crc = (crc << 1);
            if(msbit != 0) {
                crc = (crc ^ mask); // XOR
            }
        }
    }

    checksum = crc;
    HTRACE("[DHT20  ] Checksum calculated: 0x%02X", checksum);

    return DHT20_OK;
}

int8 DHT20::crc_validate(uint8 *data, size_t len) {
    HTRACE("dht20.cpp -> s:DHT20::crc_validate(uint8*, size_t):int8");

    int8 status = DHT20_OK;

    if(len == 0 || len > DATA_FRAME_LENGTH) {
        HERROR("[DHT20  ] Data frame length invalid");
        HERROR("[DHT20  ] Expected: %d", DATA_FRAME_LENGTH);
        HERROR("[DHT20  ] Got     : %d", len);

        return DHT20_ERROR_CRC;
    }

    uint8 crc = 0;
    uint8 dht_crc = data[(DATA_FRAME_LENGTH - 1)];    // Last element of array

    status = DHT20::crc_calculate(crc, data, (len - 1));
    if(status < DHT20_OK) return status;

    if(crc != dht_crc) {
        HWARN ("[DHT20  ] Checksum invalid");
        HTRACE("[DHT20  ] Expected: 0x%02X", dht_crc);
        HTRACE("[DHT20  ] Got: 0x%02X", crc);

        return DHT20_ERROR_CRC;
    }

    HTRACE("[DHT20  ] Checksum valid");
    return status;
}

const char *rts(int8 status) {
    switch(status) {
        case DHT20_OK:                          return "DHT20_OK";                      
        case DHT20_ERROR_NULL_CONTEXT:          return "DHT20_ERROR_NULL_CONTEXT";      
        case DHT20_ERROR_NULL_INSTANCE:         return "DHT20_ERROR_NULL_INSTANCE";     
        case DHT20_ERROR_WRITE_FAILED:          return "DHT20_ERROR_WRITE_FAILED";      
        case DHT20_ERROR_I2C:                   return "DHT20_ERROR_I2C";               
        case DHT20_ERROR_I2C_TRANSACTION:       return "DHT20_ERROR_I2C_TRANSACTION";   
        case DHT20_ERROR_SENSOR_DISABLED:       return "DHT20_ERROR_SENSOR_DISABLED";
        case DHT20_ERROR_READ_FAILED:           return "DHT20_ERROR_READ_FAILED";       
        case DHT20_ERROR_DEVICE_NOT_FOUND:      return "DHT20_ERROR_DEVICE_NOT_FOUND";  
        case DHT20_ERROR_TIMEOUT:               return "DHT20_ERROR_TIMEOUT";           
        case DHT20_ERROR_NULL_DATA:             return "DHT20_ERROR_NULL_DATA";          
        case DHT20_ERROR_CRC:                   return "DHT20_ERROR_CRC";                
        case DHT20_ERROR_ZERO_LENGTH:           return "DHT20_ERROR_ZERO_LENGTH";        
        case DHT20_ERROR_OOB:                   return "DHT20_ERROR_OOB";
        case DHT20_ERROR_GENERIC:               return "DHT20_ERROR_GENERIC";
        case DHT20_ERROR_NVM:                   return "DHT20_ERROR_NVM";
        case DHT20_ERROR_NVM_TRANSACTION:       return "DHT20_ERROR_NVM_TRANSACTION";
        case DHT20_FUNCTION_NOT_IMPLEMENTED:    return "DHT20_FUNCTION_NOT_IMPLEMENTED"; 
        default:                                return "DHT20_ERROR_UNKNOWN";
    }
}

}