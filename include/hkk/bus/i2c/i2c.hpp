#pragma once
#include <defines.h>

enum I2CResult : int32 {
    I2C_OK                  =  0,
    I2C_ERROR_NULL_CONTEXT  = -1,
    I2C_ERROR_NULL_INSTANCE = -2,
    I2C_ERROR_NULL_DATA     = -3,
    I2C_ERROR_ZERO_LENGTH   = -4,
    I2C_ERROR_NO_ACK        = -5,
    I2C_ERROR_PARTIAL_WRITE = -6,
    I2C_ERROR_WRITE_FAILED  = -7,
    I2C_ERROR_PARTIAL_READ  = -8,
    I2C_ERROR_READ_FAILED   = -9,  
    I2C_ERROR_NOT_SUPPORTED = -10, 
    I2C_ERROR_GENERIC       = -100,
    I2C_FUNCTION_NOT_IMPLEMENTED = -128,
};

namespace hkk::bus {   
    // write_read
    // probe
    // recover
    // status - OK, TIMEOUT, NACK, BUS_ERROR, INVALID_ARGUMENT 

struct I2C {
    void *ctx = nullptr;


    int8 (*i2c_init)(void *ctx) = nullptr;
    int8 (*i2c_deinit)(void *ctx) = nullptr;
    
    int8  (*i2c_set_baudrate)(void *ctx, uint32 value) = nullptr;
    int32 (*i2c_get_baudrate)(void *ctx) = nullptr;

    int8  (*i2c_set_index)(void *ctx, int8 value) = nullptr;
    int32 (*i2c_get_index)(void *ctx) = nullptr;

    int32 (*i2c_write_blocking)(void *ctx, uint8 address, const uint8 *src, size_t len, bool8 nostop) = nullptr;
    int32 (*i2c_read_blocking)(void *ctx, uint8 address, uint8 *dst, size_t len, bool8 nostop) = nullptr;


    int8 init() {return i2c_init ? i2c_init(ctx) : I2C_FUNCTION_NOT_IMPLEMENTED;}
    int8 deinit() {return i2c_deinit ? i2c_deinit(ctx) : I2C_FUNCTION_NOT_IMPLEMENTED;}
    
    int8  baudrate(uint32 value) {return i2c_set_baudrate ? i2c_set_baudrate(ctx, value) : I2C_FUNCTION_NOT_IMPLEMENTED;}
    int32 baudrate() {return i2c_get_baudrate ? i2c_get_baudrate(ctx) : I2C_FUNCTION_NOT_IMPLEMENTED;}
    
    int8  index(int8 value) {return i2c_set_index ? i2c_set_index(ctx, value) : I2C_FUNCTION_NOT_IMPLEMENTED;}
    int32 index() {return i2c_get_index ? i2c_get_index(ctx) : I2C_FUNCTION_NOT_IMPLEMENTED;}


    int32 write(uint8 address, const uint8 *src, size_t len, bool8 nostop) {
        return i2c_write_blocking ? i2c_write_blocking(ctx, address, src, len, nostop) : I2C_FUNCTION_NOT_IMPLEMENTED;
    }

    template <size_t N>
    int32 write(uint8 address, const uint8 (&src)[N], bool8 nostop) {
        return write(address, src, N, nostop);
    }


    int32 read(uint8 address, uint8 *dst, size_t len, bool8 nostop) {
        return i2c_read_blocking ? i2c_read_blocking(ctx, address, dst, len, nostop) : I2C_FUNCTION_NOT_IMPLEMENTED;
    }

    template <size_t N>
    int32 read(uint8 address, uint8 (&dst)[N], bool8 nostop) {
        return read(address, dst, N, nostop);
    }
};
}  