#include <hkk/defines.h>

#include <hkk/logger/logger.h>
#include <hkk/bus/i2c/i2c.hpp>

#include <hardware/i2c.h>
#include <hardware/gpio.h>


namespace hkk::rp2350 {
// https://www.raspberrypi.com/documentation/pico-sdk/hardware.html#group_hardware_i2c

struct I2CContext {
    i2c_inst_t *instance;
    uint32 baudrate = 100000;   // 100 kHz
    uint8 sda = 4;              // Default I2C0 SDA
    uint8 scl = 5;              // Default I2C0 SCL
    int8 index = -1;
    int8 status = I2C_OK;
};

static int8 i2c_init(void *ctx_raw) {
    HTRACE("i2c.cpp -> i2c_init(void*):int8");

    if(!ctx_raw) {
        HERROR("[  I2C  ] Null context passed to function");
        return I2C_ERROR_NULL_CONTEXT;
    }

    I2CContext *ctx = static_cast<I2CContext*>(ctx_raw);
    if(!ctx->instance) {
        HERROR("[  I2C  ] Null I2C instance in context");
        return I2C_ERROR_NULL_INSTANCE;
    }
    
    ::i2c_init(ctx->instance, ctx->baudrate);

    ::gpio_set_function(ctx->sda, GPIO_FUNC_I2C);
    ::gpio_set_function(ctx->scl, GPIO_FUNC_I2C);

    ::gpio_pull_up(ctx->sda);
    ::gpio_pull_up(ctx->scl);

    uint8 index = ::i2c_get_index(ctx->instance);
    ctx->index = static_cast<int8>(index);

    if(ctx->index != 0 && ctx->index != 1) {
        HERROR("[  I2C  ] Invalid I2C instance index: %d", ctx->index);
        return I2C_ERROR_GENERIC;
    }

    HDEBUG("[  I2C  ] SDA pin: %d", ctx->sda);
    HDEBUG("[  I2C  ] SCL pin: %d", ctx->scl);
    HDEBUG("[  I2C  ] Baud rate set to %d kHz", (ctx->baudrate / 1000));

    HINFO("[  I2C  ] I2C%d initialization successful", ctx->index);
    return I2C_OK;
}

static int8 i2c_deinit(void *ctx_raw) {
    HTRACE("i2c.cpp -> i2c_deinit(void*):int8");

    if(!ctx_raw) {
        HERROR("[  I2C  ] Null context passed to function");
        return I2C_ERROR_NULL_CONTEXT;
    }

    I2CContext *ctx = static_cast<I2CContext*>(ctx_raw);
    if(!ctx->instance) {
        HERROR("[  I2C  ] Null I2C instance in context");
        return I2C_ERROR_NULL_INSTANCE;
    }

    ::i2c_deinit(ctx->instance);

    ::gpio_deinit(ctx->sda);
    ::gpio_deinit(ctx->scl);

    ::gpio_set_pulls(ctx->sda, false, false);
    ::gpio_set_pulls(ctx->scl, false, false);

    HINFO("[  I2C  ] I2C%d deinitialization successful", ctx->index);

    ctx->index = -1;
    HDEBUG("[  I2C  ] Instance index set to %d", ctx->index);

    return I2C_OK;
}

static int8 i2c_set_baudrate(void *ctx_raw, uint32 value) {
    HTRACE("i2c.cpp -> i2c_set_baudrate(void*, uint32):int8");

    if(!ctx_raw) {
        HERROR("[  I2C  ] Null context passed to function");
        return I2C_ERROR_NULL_CONTEXT;
    }

    I2CContext *ctx = static_cast<I2CContext*>(ctx_raw);
    
    if(!ctx->instance) {
        HERROR("[  I2C  ] Null I2C instance in context");
        return I2C_ERROR_NULL_INSTANCE;
    }

    if(value == 0) {
        HWARN("[  I2C  ] Baud rate cannot be set to 0");
        HWARN("[  I2C  ] Baud rate set to %d kHz", (ctx->baudrate / 1000));

        value = ctx->baudrate;
    }

    uint32 baudrate = ::i2c_set_baudrate(ctx->instance, value);
    
    if(baudrate == 0) {
        HERROR("[  I2C  ] Could not set baud rate to %d", value);
        return I2C_ERROR_GENERIC;
    }

    if(baudrate != value) {
        HWARN("[  I2C  ] Actual baud rate differs from desired baud rate");
        HWARN("[  I2C  ] Desired: %d kHz", (ctx->baudrate / 1000));
        HWARN("[  I2C  ] Actual : %d kHz", (baudrate/1000));
    }

    ctx->baudrate = baudrate;
    HINFO("[  I2C  ] Baud rate set to %d kHz", (ctx->baudrate / 1000));

    return I2C_OK;
}

static int32 i2c_get_baudrate(void *ctx_raw) {
    HTRACE("i2c.cpp -> i2c_get_baudrate(void*):int32");

    if(!ctx_raw) {
        HERROR("[  I2C  ] Null context passed to function");
        return I2C_ERROR_NULL_CONTEXT;
    }

    I2CContext *ctx = static_cast<I2CContext*>(ctx_raw);
    if(!ctx->instance) {
        HERROR("[  I2C  ] Null I2C instance in context");
        return I2C_ERROR_NULL_INSTANCE;
    }

    HDEBUG("[  I2C  ] Current baud rate: %d kHz", (ctx->baudrate / 1000));
    return ctx->baudrate;
}

static int8 i2c_set_index(void *ctx_raw, int8 value) {
    HTRACE("i2c.cpp -> i2c_set_index(void*, int8):int8");
    
    HWARN("[  I2C  ] Function not implemented on RP2350");
    return I2C_ERROR_NOT_SUPPORTED;
}

static int32 i2c_get_index(void *ctx_raw) {
    HTRACE("i2c.cpp -> i2c_get_index(void*):int32");

    if(!ctx_raw) {
        HERROR("[  I2C  ] Null context passed to function");
        return I2C_ERROR_NULL_CONTEXT;
    }

    I2CContext *ctx = static_cast<I2CContext*>(ctx_raw);
    if(!ctx->instance) {
        HERROR("[  I2C  ] Null I2C instance in context");
        return I2C_ERROR_NULL_INSTANCE;
    }

    uint8 index = ::i2c_get_index(ctx->instance);
    if(index != 0 && index != 1) {
        HERROR("[  I2C  ] Invalid I2C instance index: %d", index);
        return I2C_ERROR_GENERIC;
    }

    HDEBUG("[  I2C  ] Current index: %d", index);
    ctx->index = static_cast<int8>(index);
    
    return static_cast<int32>(ctx->index);
}

static int32 i2c_write_blocking(void *ctx_raw, uint8 address, const uint8 *src, size_t len, bool8 nostop) {
    HTRACE("i2c.cpp -> i2c_write_blocking(void*, uint8, const uint8*, size_t, bool8):int32");

    if(!ctx_raw) {
        HERROR("[  I2C  ] Null context passed to function");
        return I2C_ERROR_NULL_CONTEXT;
    }

    I2CContext *ctx = static_cast<I2CContext*>(ctx_raw);

    if(!ctx->instance) {
        HERROR("[  I2C  ] Null I2C instance in context");
        return I2C_ERROR_NULL_INSTANCE;
    }

    if(!src) {
        HERROR("[  I2C  ] Null data pointer passed to function");
        return I2C_ERROR_NULL_DATA;
    }

    if(len == 0) {
        HERROR("[  I2C  ] Data length is 0");
        return I2C_ERROR_ZERO_LENGTH;
    }

    int32 written = ::i2c_write_blocking(ctx->instance, address, src, len, nostop);

    if(written == PICO_ERROR_GENERIC) {
        HERROR("[  I2C  ] Address not acknowledged or device not present");
        return I2C_ERROR_NO_ACK;
    }

    if(written < 0) {
        HERROR("[  I2C  ] Write failed");
        return I2C_ERROR_WRITE_FAILED;
    }

    if(static_cast<size_t>(written) != len) {
        HERROR("[  I2C  ] Partial write: %d/%zu bytes written", written, len);
        return I2C_ERROR_PARTIAL_WRITE;
    }

    HDEBUG("[  I2C  ] First byte   : 0x%02X", src[0]);
    HDEBUG("[  I2C  ] Data length  : %zu", len);
    HDEBUG("[  I2C  ] Bytes written: %d", written);
    HDEBUG("[  I2C  ] No STOP      : %s", nostop ? "true" : "false");

    HINFO("[  I2C  ] Write completed successfully");
    return written;
}

static int32 i2c_read_blocking(void *ctx_raw, uint8 address, uint8 *dst, size_t len, bool8 nostop) {
    HTRACE("i2c.cpp -> i2c_read_blocking(void*, uint8, uint8*, size_t, bool8):int32");

    if(!ctx_raw) {
        HERROR("[  I2C  ] Null context passed to function");
        return I2C_ERROR_NULL_CONTEXT;
    }

    I2CContext *ctx = static_cast<I2CContext*>(ctx_raw);

    if(!ctx->instance) {
        HERROR("[  I2C  ] Null I2C instance in context");
        return I2C_ERROR_NULL_INSTANCE;
    }

    if(!dst) {
        HERROR("[  I2C  ] Null data pointer passed to function");
        return I2C_ERROR_NULL_DATA;
    }

    if(len == 0) {
        HERROR("[  I2C  ] Data length is 0");
        return I2C_ERROR_ZERO_LENGTH;
    }

    int32 read = ::i2c_read_blocking(ctx->instance, address, dst, len, nostop);

    if(read == PICO_ERROR_GENERIC) {
        HERROR("[  I2C  ] Address not acknowledged or device not present");
        return I2C_ERROR_NO_ACK;
    }

    if(read < 0) {
        HERROR("[  I2C  ] Read failed");
        return I2C_ERROR_READ_FAILED;
    }

    if(static_cast<size_t>(read) != len) {
        HERROR("[  I2C  ] Partial read: %d/%zu bytes read", read, len);
        return I2C_ERROR_PARTIAL_READ;
    }

    HDEBUG("[  I2C  ] First byte   : 0x%02X", dst[0]);
    HDEBUG("[  I2C  ] Data length  : %zu", len);
    HDEBUG("[  I2C  ] Bytes read   : %d", read);
    HDEBUG("[  I2C  ] No STOP      : %s", nostop ? "true" : "false");

    HINFO("[  I2C  ] Read completed successfully");
    return read;
}

}