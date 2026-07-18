#include <hkk/defines.h>

#include <hkk/logger/logger.h>
#include <hkk/bus/i2c/i2c.hpp>

#include <pico/mutex.h>
#include <hardware/i2c.h>
#include <hardware/gpio.h>


namespace hkk::rp2350::i2c {
// https://www.raspberrypi.com/documentation/pico-sdk/hardware.html#group_hardware_i2c


static int8 deinit_fn(void *ctx_raw) {
    HTRACE("i2c.cpp -> s:deinit_fn(void*):int8");

    if(!ctx_raw) {
        HERROR("[I2C    ] Null context passed to function");
        return hkk::bus::i2c::I2C_ERROR_NULL_CONTEXT;
    }

    auto *ctx = static_cast<hkk::bus::i2c::ConfigContext*>(ctx_raw);

    if(!ctx->instance) {
        HERROR("[I2C    ] Null I2C instance in context");

        ctx->status = hkk::bus::i2c::I2C_ERROR_NULL_INSTANCE;
        return ctx->status;
    } 
    ::i2c_inst *instance = static_cast<::i2c_inst*>(ctx->instance);

    ::i2c_deinit(instance);

    ::gpio_deinit(ctx->sda);
    ::gpio_deinit(ctx->scl);

    ::gpio_set_pulls(ctx->sda, false, false);
    ::gpio_set_pulls(ctx->scl, false, false);

    if(!ctx->transaction || !ctx->transaction->mutex) {
        HWARN("[I2C    ] Null I2C mutex in context");
        ctx->status = hkk::bus::i2c::I2C_ERROR_NULL_MUTEX;
    } else {
        ::mutex_exit(static_cast<::mutex_t*>(ctx->transaction->mutex));
    }

    HINFO("[I2C    ] I2C%d deinitialization successful", ctx->index);

    ctx->index  = -1;
    HDEBUG("[I2C    ] Instance index set to %d", ctx->index);

    ctx->status = hkk::bus::i2c::I2C_OK;
    return ctx->status;
}

static int8 init_fn(void *ctx_raw) {
    HTRACE("i2c.cpp -> s:init_fn(void*):int8");

    if(!ctx_raw) {
        HERROR("[I2C    ] Null context passed to function");
        return hkk::bus::i2c::I2C_ERROR_NULL_CONTEXT;
    }

    auto *ctx = static_cast<hkk::bus::i2c::ConfigContext*>(ctx_raw);

    if(!ctx->instance) {
        HERROR("[I2C    ] Null I2C instance in context");

        ctx->status = hkk::bus::i2c::I2C_ERROR_NULL_INSTANCE;
        return ctx->status;
    } 
    ::i2c_inst *instance = static_cast<::i2c_inst*>(ctx->instance);

    ::i2c_init(instance, ctx->baudrate);

    ::gpio_set_function(ctx->sda, GPIO_FUNC_I2C);
    ::gpio_set_function(ctx->scl, GPIO_FUNC_I2C);

    ::gpio_pull_up(ctx->sda);
    ::gpio_pull_up(ctx->scl);

    uint8 index = ::i2c_get_index(instance);
    ctx->index = static_cast<int8>(index);

    if(ctx->index != 0 && ctx->index != 1) {
        HERROR("[I2C    ] Invalid I2C instance index: %d", ctx->index);
        
        ctx->status = hkk::bus::i2c::I2C_ERROR_GENERIC;
        return ctx->status;
    }

    if(!ctx->transaction || !ctx->transaction->mutex) {
        HERROR("[I2C    ] Null I2C mutex in context");
        
        deinit_fn(ctx_raw);

        ctx->status = hkk::bus::i2c::I2C_ERROR_NULL_MUTEX;
        return ctx->status;
    }
    ::mutex_init(static_cast<::mutex_t*>(ctx->transaction->mutex));

    HDEBUG("[I2C    ] SDA pin: %d", ctx->sda);
    HDEBUG("[I2C    ] SCL pin: %d", ctx->scl);
    HDEBUG("[I2C    ] Baud rate set to %d kHz", (ctx->baudrate / 1000));

    HINFO("[I2C    ] I2C%d initialization successful", ctx->index);
    
    ctx->status = hkk::bus::i2c::I2C_OK;
    return ctx->status;
}

static int8 set_baudrate_fn(void *ctx_raw, uint32 value) {
    HTRACE("i2c.cpp -> s:set_baudrate_fn(void*, uint32):int8");

    if(!ctx_raw) {
        HERROR("[I2C    ] Null context passed to function");
        return hkk::bus::i2c::I2C_ERROR_NULL_CONTEXT;
    }

    auto *ctx = static_cast<hkk::bus::i2c::ConfigContext*>(ctx_raw);
    
    if(!ctx->instance) {
        HERROR("[I2C    ] Null I2C instance in context");
        
        ctx->status = hkk::bus::i2c::I2C_ERROR_NULL_INSTANCE;
        return ctx->status;
    } 
    ::i2c_inst *instance = static_cast<::i2c_inst*>(ctx->instance);

    if(value == 0) {
        HWARN("[I2C    ] Baud rate cannot be set to 0");
        HWARN("[I2C    ] Baud rate set to %d kHz", (ctx->baudrate / 1000));

        value = ctx->baudrate;
    }

    uint32 baudrate = ::i2c_set_baudrate(instance, value);
    
    if(baudrate == 0) {
        HERROR("[I2C    ] Could not set baud rate to %d", value);
        
        ctx->status = hkk::bus::i2c::I2C_ERROR_GENERIC;
        return ctx->status;
    }

    if(baudrate != value) {
        HWARN("[I2C    ] Actual baud rate differs from desired baud rate");
        HWARN("[I2C    ] Desired: %d kHz", (ctx->baudrate / 1000));
        HWARN("[I2C    ] Actual : %d kHz", (baudrate/1000));
    }

    ctx->baudrate = baudrate;
    HINFO("[I2C    ] Baud rate set to %d kHz", (ctx->baudrate / 1000));

    ctx->status = hkk::bus::i2c::I2C_OK;
    return ctx->status;
}

static int32 get_baudrate_fn(void *ctx_raw) {
    HTRACE("i2c.cpp -> s:get_baudrate_fn(void*):int32");

    if(!ctx_raw) {
        HERROR("[I2C    ] Null context passed to function");
        return static_cast<int32>(hkk::bus::i2c::I2C_ERROR_NULL_CONTEXT);
    }

    auto *ctx = static_cast<hkk::bus::i2c::ConfigContext*>(ctx_raw);

    if(!ctx->instance) {
        HERROR("[I2C    ] Null I2C instance in context");

        ctx->status = hkk::bus::i2c::I2C_ERROR_NULL_INSTANCE;
        return static_cast<int32>(ctx->status);
    } 
    ::i2c_inst *instance = static_cast<::i2c_inst*>(ctx->instance);

    HDEBUG("[I2C    ] Current baud rate: %d kHz", (ctx->baudrate / 1000));

    ctx->status = hkk::bus::i2c::I2C_OK;
    return static_cast<int32>(ctx->baudrate);
}

static int8 set_index_fn(void *ctx_raw, int8 value) {
    HTRACE("i2c.cpp -> s:set_index_fn(void*, int8):int8");
    
    HFATAL("[I2C    ] Function not implemented on RP2350");
    return hkk::bus::i2c::I2C_ERROR_NOT_SUPPORTED;
}

static int32 get_index_fn(void *ctx_raw) {
    HTRACE("i2c.cpp -> s:get_index_fn(void*):int32");

    if(!ctx_raw) {
        HERROR("[I2C    ] Null context passed to function");
        return static_cast<int32>(hkk::bus::i2c::I2C_ERROR_NULL_CONTEXT);
    }

    auto *ctx = static_cast<hkk::bus::i2c::ConfigContext*>(ctx_raw);

    if(!ctx->instance) {
        HERROR("[I2C    ] Null I2C instance in context");

        ctx->status = hkk::bus::i2c::I2C_ERROR_NULL_INSTANCE;
        return static_cast<int32>(ctx->status);
    } 
    ::i2c_inst *instance = static_cast<::i2c_inst*>(ctx->instance);

    uint8 index = ::i2c_get_index(instance);
    if(index != 0 && index != 1) {
        HERROR("[I2C    ] Invalid I2C instance index: %d", index);
        
        ctx->status = hkk::bus::i2c::I2C_ERROR_GENERIC;
        return static_cast<int32>(ctx->status);
    }

    HTRACE("[I2C    ] Current I2C index: I2C%d", index);
    ctx->index = static_cast<int8>(index);
    
    ctx->status = hkk::bus::i2c::I2C_OK;
    return static_cast<int32>(ctx->index);
}

static int32 write_blocking_fn(void *ctx_raw, uint8 addr, const uint8 *src, size_t len, bool8 nostop) {
    HTRACE("i2c.cpp -> s:write_blocking_fn(void*, uint8, const uint8*, size_t, bool8 = false):int32");

    if(!ctx_raw) {
        HERROR("[I2C    ] Null context passed to function");
        return static_cast<int32>(hkk::bus::i2c::I2C_ERROR_NULL_CONTEXT);
    }

    auto *ctx = static_cast<hkk::bus::i2c::ConfigContext*>(ctx_raw);

    if(!ctx->instance) {
        HERROR("[I2C    ] Null I2C instance in context");

        ctx->status = hkk::bus::i2c::I2C_ERROR_NULL_INSTANCE;
        return static_cast<int32>(ctx->status);
    } 
    ::i2c_inst *instance = static_cast<::i2c_inst*>(ctx->instance);

    if(!src) {
        HERROR("[I2C    ] Null data pointer passed to function");
        
        ctx->status = hkk::bus::i2c::I2C_ERROR_NULL_DATA;
        return static_cast<int32>(ctx->status);
    }

    if(len == 0) {
        HERROR("[I2C    ] Data length is 0");

        ctx->status = hkk::bus::i2c::I2C_ERROR_ZERO_LENGTH;
        return static_cast<int32>(ctx->status);
    }

    if(!ctx->transaction || !ctx->transaction->mutex) {
        HERROR("[I2C    ] Null I2C mutex in context");

        ctx->status = hkk::bus::i2c::I2C_ERROR_NULL_MUTEX;
        return static_cast<int32>(ctx->status);
    }

    auto *transaction = static_cast<hkk::bus::i2c::LockState*>(ctx->transaction);
    transaction->nostop = nostop;

    int32 written = PICO_ERROR_GENERIC;
    if(transaction->active == false) {
        ::mutex_t *mutex = static_cast<::mutex_t*>(transaction->mutex);
        ::mutex_enter_blocking(mutex);
        transaction->active = true;
        
        written = ::i2c_write_blocking(instance, addr, src, len, nostop);

        if(nostop == false) {
            transaction->active = false;
            ::mutex_exit(mutex);
        }
    } else {
        written = ::i2c_write_blocking(instance, addr, src, len, nostop);
    }

    if(written == PICO_ERROR_GENERIC) {
        HERROR("[I2C    ] Address not acknowledged or device not present");
        
        ctx->status = hkk::bus::i2c::I2C_ERROR_NO_ACK;
        return static_cast<int32>(ctx->status);
    }

    if(static_cast<size_t>(written) != len) {
        HERROR("[I2C    ] Partial write: %d/%zu bytes written", written, len);
        
        ctx->status = hkk::bus::i2c::I2C_ERROR_PARTIAL_WRITE;
        return static_cast<int32>(ctx->status);
    }

    HTRACE("[I2C    ] First byte   : 0x%02X", src[0]);
    HTRACE("[I2C    ] Data length  : %zu", len);
    HTRACE("[I2C    ] Bytes written: %d", written);
    HTRACE("[I2C    ] No STOP      : %s", nostop ? "true" : "false");

    HTRACE("[I2C    ] Write complete");

    ctx->status = hkk::bus::i2c::I2C_OK;
    return written;
}

static int32 read_blocking_fn(void *ctx_raw, uint8 addr, uint8 *dst, size_t len, bool8 nostop) {
    HTRACE("i2c.cpp -> s:read_blocking_fn(void*, uint8, uint8*, size_t, bool8 = false):int32");

    if(!ctx_raw) {
        HERROR("[I2C    ] Null context passed to function");
        return static_cast<int32>(hkk::bus::i2c::I2C_ERROR_NULL_CONTEXT);
    }

    auto *ctx = static_cast<hkk::bus::i2c::ConfigContext*>(ctx_raw);

    if(!ctx->instance) {
        HERROR("[I2C    ] Null I2C instance in context");

        ctx->status = hkk::bus::i2c::I2C_ERROR_NULL_INSTANCE;
        return static_cast<int32>(ctx->status);
    } 
    ::i2c_inst *instance = static_cast<::i2c_inst*>(ctx->instance);

    if(!dst) {
        HERROR("[I2C    ] Null data pointer passed to function");

        ctx->status = hkk::bus::i2c::I2C_ERROR_NULL_DATA;
        return static_cast<int32>(ctx->status);
    }

    if(len == 0) {
        HERROR("[I2C    ] Data length is 0");

        ctx->status = hkk::bus::i2c::I2C_ERROR_ZERO_LENGTH;
        return static_cast<int32>(ctx->status);
    }

    if(!ctx->transaction || !ctx->transaction->mutex) {
        HERROR("[I2C    ] Null I2C mutex in context");

        ctx->status = hkk::bus::i2c::I2C_ERROR_NULL_MUTEX;
        return static_cast<int32>(ctx->status);
    }

    auto *transaction = static_cast<hkk::bus::i2c::LockState*>(ctx->transaction);
    transaction->nostop = nostop;
    
    int32 read = PICO_ERROR_GENERIC;
    if(transaction->active == false) {
        ::mutex_t *mutex = static_cast<::mutex_t*>(ctx->transaction->mutex);
        ::mutex_enter_blocking(mutex);
        transaction->active = true;
        
        read = ::i2c_read_blocking(instance, addr, dst, len, nostop);

        if(nostop == false) {
            transaction->active = false;
            ::mutex_exit(mutex);
        }
    } else {
        read = ::i2c_read_blocking(instance, addr, dst, len, nostop);
    }

    if(read == PICO_ERROR_GENERIC) {
        HERROR("[I2C    ] Address not acknowledged or device not present");

        ctx->status = hkk::bus::i2c::I2C_ERROR_NO_ACK;
        return static_cast<int32>(ctx->status);
    }

    if(static_cast<size_t>(read) != len) {
        HERROR("[I2C    ] Partial read: %d/%zu bytes read", read, len);

        ctx->status = hkk::bus::i2c::I2C_ERROR_PARTIAL_READ;
        return static_cast<int32>(ctx->status);
    }

    HTRACE("[I2C    ] First byte   : 0x%02X", dst[0]);
    HTRACE("[I2C    ] Data length  : %zu", len);
    HTRACE("[I2C    ] Bytes read   : %d", read);
    HTRACE("[I2C    ] No STOP      : %s", nostop ? "true" : "false");

    HTRACE("[I2C    ] Read complete");

    ctx->status = hkk::bus::i2c::I2C_OK;
    return read;
}

static int32 write_timeout_fn(void *ctx_raw, uint8 addr, const uint8 *src, size_t len, uint32 timeout_us, bool8 nostop) {
    HTRACE("i2c.cpp -> s:write_timeout_fn(void*, uint8, const uint8*, size_t, uint32, bool8 = false):int32");

    if(!ctx_raw) {
        HERROR("[I2C    ] Null context passed to function");
        return static_cast<int32>(hkk::bus::i2c::I2C_ERROR_NULL_CONTEXT);
    }

    auto *ctx = static_cast<hkk::bus::i2c::ConfigContext*>(ctx_raw);

    if(!ctx->instance) {
        HERROR("[I2C    ] Null I2C instance in context");

        ctx->status = hkk::bus::i2c::I2C_ERROR_NULL_INSTANCE;
        return static_cast<int32>(ctx->status);
    }
    ::i2c_inst *instance = static_cast<::i2c_inst*>(ctx->instance);

    if(!src) {
        HERROR("[I2C    ] Null data pointer passed to function");

        ctx->status = hkk::bus::i2c::I2C_ERROR_NULL_DATA;
        return static_cast<int32>(ctx->status);   
    }
    
    if(!len) {
        HERROR("[I2C    ] Data length is 0");

        ctx->status = hkk::bus::i2c::I2C_ERROR_ZERO_LENGTH;
        return static_cast<int32>(ctx->status);
    }

    if(!ctx->transaction || !ctx->transaction->mutex) {
        HERROR("[I2C    ] Null I2C mutex in context");

        ctx->status = hkk::bus::i2c::I2C_ERROR_NULL_MUTEX;
        return static_cast<int32>(ctx->status);
    }

    auto *transaction = static_cast<hkk::bus::i2c::LockState*>(ctx->transaction);
    transaction->nostop = nostop;

    int32 written = PICO_ERROR_GENERIC;
    if(transaction->active == false) {
        ::mutex_t *mutex = static_cast<::mutex_t*>(ctx->transaction->mutex);
        ::mutex_enter_timeout_us(mutex, timeout_us);
        transaction->active = true;
        
        written = ::i2c_write_timeout_us(instance, addr, src, len, nostop, timeout_us);

        if(nostop == false) {
            transaction->active = false;
            ::mutex_exit(mutex);
        }
    } else {
        written = ::i2c_write_timeout_us(instance, addr, src, len, nostop, timeout_us);
    }

    switch(written) {
        case PICO_ERROR_GENERIC:
            HERROR("[I2C    ] Address not acknowledged or device not present");

            ctx->status = hkk::bus::i2c::I2C_ERROR_NO_ACK;
            return static_cast<int32>(ctx->status);
        case PICO_ERROR_TIMEOUT:
            HERROR("[I2C    ] Timeout reached on write");

            ctx->status = hkk::bus::i2c::I2C_ERROR_TIMEOUT;
            return static_cast<int32>(ctx->status);
    }

    if(static_cast<size_t>(written) != len) {
        HERROR("[I2C    ] Partial write: %d/%zu bytes written", written, len);

        ctx->status = hkk::bus::i2c::I2C_ERROR_PARTIAL_WRITE;
        return static_cast<int32>(ctx->status);
    }

    HTRACE("[I2C    ] First byte   : 0x%02X", src[0]);
    HTRACE("[I2C    ] Data length  : %zu", len);
    HDEBUG("[I2C    ] Bytes written: %d", written);
    HTRACE("[I2C    ] Timeout      : %dus", timeout_us);
    HTRACE("[I2C    ] No STOP      : %s", nostop ? "true" : "false");

    HDEBUG("[I2C    ] Write complete");

    ctx->status = hkk::bus::i2c::I2C_OK;
    return written;
}

static int32 read_timeout_fn(void *ctx_raw, uint8 addr, uint8 *dst, size_t len, uint32 timeout_us, bool8 nostop) {
    HTRACE("i2c.cpp -> s:read_timeout_fn(void*, uint8, uint8*, size_t, uint32, bool8 = false):int32");

    if(!ctx_raw) {
        HERROR("[I2C    ] Null context passed to function");
        return static_cast<int32>(hkk::bus::i2c::I2C_ERROR_NULL_CONTEXT);
    }

    auto *ctx = static_cast<hkk::bus::i2c::ConfigContext*>(ctx_raw);

    if(!ctx->instance) {
        HERROR("[I2C    ] Null I2C instance in context");

        ctx->status = hkk::bus::i2c::I2C_ERROR_NULL_INSTANCE;
        return static_cast<int32>(ctx->status);
    }
    ::i2c_inst *instance = static_cast<::i2c_inst*>(ctx->instance);

    if(!dst) {
        HERROR("[I2C    ] Null data pointer passed to function");

        ctx->status = hkk::bus::i2c::I2C_ERROR_NULL_DATA;
        return static_cast<int32>(ctx->status);
    }

    if(len == 0) {
        HERROR("[I2C    ] Data length is 0");

        ctx->status = hkk::bus::i2c::I2C_ERROR_ZERO_LENGTH;
        return static_cast<int32>(ctx->status);
    }

    if(!ctx->transaction || !ctx->transaction->mutex) {
        HERROR("[I2C    ] Null I2C mutex in context");

        ctx->status = hkk::bus::i2c::I2C_ERROR_NULL_MUTEX;
        return static_cast<int32>(ctx->status);
    }

    auto *transaction = static_cast<hkk::bus::i2c::LockState*>(ctx->transaction);
    transaction->nostop = nostop;

    int32 read = PICO_ERROR_GENERIC;
    if(transaction->active == false) {
        ::mutex_t *mutex = static_cast<::mutex_t*>(ctx->transaction->mutex);
        ::mutex_enter_timeout_us(mutex, timeout_us);
        transaction->active = true;
        
        read = ::i2c_read_timeout_us(instance, addr, dst, len, nostop, timeout_us);

        if(nostop == false) {
            transaction->active = false;
            ::mutex_exit(mutex);
        }
    } else {
        read = ::i2c_read_timeout_us(instance, addr, dst, len, nostop, timeout_us);
    }

    switch(read) {
        case PICO_ERROR_GENERIC:
            HERROR("[I2C    ] Address not acknowledged or device not present");

            ctx->status = hkk::bus::i2c::I2C_ERROR_NO_ACK;
            return static_cast<int32>(ctx->status);
        case PICO_ERROR_TIMEOUT:
            HERROR("[I2C    ] Timeout reached on write");

            ctx->status = hkk::bus::i2c::I2C_ERROR_TIMEOUT;
            return static_cast<int32>(ctx->status);
    }

    if(static_cast<size_t>(read) != len) {
        HERROR("[I2C    ] Partial read: %d/%zu bytes read", read, len);

        ctx->status = hkk::bus::i2c::I2C_ERROR_PARTIAL_READ;
        return static_cast<int32>(ctx->status);
    }

    HTRACE("[I2C    ] First byte   : 0x%02X", dst[0]);
    HTRACE("[I2C    ] Data length  : %zu", len);
    HDEBUG("[I2C    ] Bytes read   : %d", read);
    HTRACE("[I2C    ] No STOP      : %s", nostop ? "true" : "false");

    HDEBUG("[I2C    ] Read complete");

    ctx->status = hkk::bus::i2c::I2C_OK;
    return read;
}


static hkk::bus::i2c::BackendTable backend {
    .init_fn = init_fn,
    .deinit_fn = deinit_fn,

    .set_baudrate_fn = set_baudrate_fn,
    .get_baudrate_fn = get_baudrate_fn,

    .set_index_fn = set_index_fn,
    .get_index_fn = get_index_fn,

    .write_blocking_fn = write_blocking_fn,
    .read_blocking_fn = read_blocking_fn,

    .write_timeout_fn = write_timeout_fn,
    .read_timeout_fn = read_timeout_fn
};


}



namespace hkk::bus::i2c {

inline constexpr uint8 PIN_SDA0 = 4;
inline constexpr uint8 PIN_SCL0 = 5;

inline constexpr uint8 PIN_SDA1 = 6;
inline constexpr uint8 PIN_SCL1 = 7;


const char *rts(int8 status) {
    switch(status) {
        case I2C_OK:                        return "I2C_OK";    
        case I2C_ERROR_NULL_CONTEXT:        return "I2C_ERROR_NULL_CONTEXT";     
        case I2C_ERROR_NULL_INSTANCE:       return "I2C_ERROR_NULL_INSTANCE";    
        case I2C_ERROR_NULL_DATA:           return "I2C_ERROR_NULL_DATA";        
        case I2C_ERROR_ZERO_LENGTH:         return "I2C_ERROR_ZERO_LENGTH";      
        case I2C_ERROR_NO_ACK:              return "I2C_ERROR_NO_ACK";           
        case I2C_ERROR_PARTIAL_WRITE:       return "I2C_ERROR_PARTIAL_WRITE";    
        case I2C_ERROR_WRITE_FAILED:        return "I2C_ERROR_WRITE_FAILED";     
        case I2C_ERROR_PARTIAL_READ:        return "I2C_ERROR_PARTIAL_READ";     
        case I2C_ERROR_READ_FAILED:         return "I2C_ERROR_READ_FAILED";        
        case I2C_ERROR_NOT_SUPPORTED:       return "I2C_ERROR_NOT_SUPPORTED";      
        case I2C_ERROR_TIMEOUT:             return "I2C_ERROR_TIMEOUT";           
        case I2C_ERROR_NULL_MUTEX:          return "I2C_ERROR_NULL_MUTEX";        
        case I2C_ERROR_BUSY:                return "I2C_ERROR_BUSY";              
        case I2C_ERROR_GENERIC:             return "I2C_ERROR_GENERIC";            
        case I2C_FUNCTION_NOT_IMPLEMENTED:  return "I2C_FUNCTION_NOT_IMPLEMENTED"; 
        default:                            return "I2C_ERROR_UNKNOWN";
    }
}


static int8 transaction_fn(void *ctx_raw, void *owner) {
    HTRACE("i2c.cpp -> s:transaction_fn(void*, void* = nullptr):int8");

    if(!ctx_raw) {
        HERROR("[I2C    ] Null context passed to function");
        return I2C_ERROR_NULL_CONTEXT;
    }

    auto *ctx = static_cast<ConfigContext*>(ctx_raw);

    // There is no need for the I2C instance to be present
    // But why would you want to begin transaction without it?
    if(!ctx->instance) {
        HERROR("[I2C    ] Null I2C instance in context");
        return ctx->status = I2C_ERROR_NULL_INSTANCE;
    }

    if(!ctx->transaction || !ctx->transaction->mutex) {
        HERROR("[I2C    ] Null I2C mutex in context");
        return ctx->status = I2C_ERROR_NULL_MUTEX;
    }

    auto *transaction = static_cast<LockState*>(ctx->transaction);

    if(transaction->active && transaction->owner == owner) {
        if(transaction->nostop == true) {
            HTRACE("[I2C    ] Transaction keep alive for owner: %p", transaction->owner);
            return ctx->status = I2C_KEEP_ALIVE;
        }

        HWARN("[I2C    ] Transaction already active for this owner: %p", transaction->owner);
        return ctx->status = I2C_ERROR_BUSY;
    } 

    if(transaction->active && transaction->owner != owner) {
        HWARN("[I2C    ] Transaction already active for another owner: %p (%p)", transaction->owner, owner);
        ctx->status = I2C_ERROR_BUSY;
    }

    ::mutex_t *mutex = static_cast<::mutex_t*>(transaction->mutex);
    ::mutex_enter_blocking(mutex);

    transaction->owner  = owner;
    transaction->active = true;

    HTRACE("[I2C    ] Transaction begin for owner %p", transaction->owner);

    ctx->status = I2C_OK;
    return ctx->status;
}

static int8 commit_fn(void *ctx_raw, void *owner) {
    HTRACE("i2c.cpp -> s:commit_fn(void*, void* = nullptr):int8");

    if(!ctx_raw) {
        HERROR("[I2C    ] Null context passed to function");
        return I2C_ERROR_NULL_CONTEXT;
    }

    auto *ctx = static_cast<ConfigContext*>(ctx_raw);

    // There is no need for the I2C instance to be present
    // But why would you want to commit transaction without it?
    if(!ctx->instance) {
        HERROR("[I2C    ] Null I2C instance in context");
        return ctx->status = I2C_ERROR_NULL_INSTANCE;
    }

    if(!ctx->transaction || !ctx->transaction->mutex) {
        HERROR("[I2C    ] Null I2C mutex in context");
        return ctx->status = I2C_ERROR_NULL_MUTEX;
    }

    auto *transaction = static_cast<LockState*>(ctx->transaction);

    if(transaction->active == false) {
        HWARN("[I2C    ] No transaction in progress");
        return ctx->status = I2C_OK;
    } 

    if(transaction->active && transaction->owner != owner) {
        HWARN("[I2C    ] Transaction already active for another owner: %p (%p)", transaction->owner, owner);
        return ctx->status = I2C_ERROR_BUSY;
    }

    if(transaction->active && transaction->nostop == true) {
        HTRACE("[I2C    ] Transaction keep alive for owner: %p", transaction->owner);
        return ctx->status = I2C_KEEP_ALIVE;
    }

    void *old_owner = transaction->owner;

    transaction->owner  = nullptr;
    transaction->active = false;

    ::mutex_t *mutex = static_cast<::mutex_t*>(ctx->transaction->mutex);
    ::mutex_exit(mutex);

    HTRACE("[I2C    ] Transaction end for owner: %p", old_owner);

    ctx->status = I2C_OK;
    return ctx->status;
}


void bind(I2C &i2c, ConfigContext &cfg, BackendTable &backend) {
    HTRACE("i2c.cpp -> bind(I2C&, ConfigContext&, BackendTable&):void");

    i2c.ctx = static_cast<ConfigContext*>(&cfg);

    i2c.init_fn = backend.init_fn;
    i2c.deinit_fn = backend.deinit_fn;

    i2c.set_baudrate_fn = backend.set_baudrate_fn;
    i2c.get_baudrate_fn = backend.get_baudrate_fn;

    i2c.set_index_fn = backend.set_index_fn;
    i2c.get_index_fn = backend.get_index_fn;

    i2c.write_blocking_fn = backend.write_blocking_fn;
    i2c.read_blocking_fn  = backend.read_blocking_fn;

    i2c.write_timeout_fn = backend.write_timeout_fn;
    i2c.read_timeout_fn  = backend.read_timeout_fn;

    i2c.transaction_fn = transaction_fn;
    i2c.commit_fn = commit_fn;

    HINFO("[I2C    ] I2C instance bound to config context");
}

void bind(I2C &i2c, ConfigContext &cfg) {
    HTRACE("i2c.cpp -> bind(I2C&, ConfigContext&):void");
    bind(i2c, cfg, hkk::rp2350::i2c::backend);
}

static ::mutex_t i2c0_mutex;
static LockState i2c0_transaction {
    .mutex  = &i2c0_mutex,
    .owner  = nullptr,
    .active = false,
};

static ::mutex_t i2c1_mutex;
static LockState i2c1_transaction {
    .mutex = &i2c1_mutex,
    .owner  = nullptr,
    .active = false,
};

static ConfigContext i2c0_default_config {
    .transaction = &i2c0_transaction,
    .instance = i2c0,
    .baudrate = 100000,
    .sda = PIN_SDA0,
    .scl = PIN_SCL0,
    .index = 0
};

static ConfigContext i2c1_default_config {
    .transaction = &i2c1_transaction,
    .instance = i2c1,
    .baudrate = 100000,
    .sda = PIN_SDA1,
    .scl = PIN_SCL1,
    .index = 1
};

I2C I2C0 {
    &i2c0_default_config,
    hkk::rp2350::i2c::init_fn,
    hkk::rp2350::i2c::deinit_fn,
    hkk::rp2350::i2c::set_baudrate_fn,
    hkk::rp2350::i2c::get_baudrate_fn,
    hkk::rp2350::i2c::set_index_fn,
    hkk::rp2350::i2c::get_index_fn,
    hkk::rp2350::i2c::write_blocking_fn,
    hkk::rp2350::i2c::read_blocking_fn,
    hkk::bus::i2c::transaction_fn,
    hkk::bus::i2c::commit_fn
};

I2C I2C1 {
    &i2c1_default_config,
    hkk::rp2350::i2c::init_fn,
    hkk::rp2350::i2c::deinit_fn,
    hkk::rp2350::i2c::set_baudrate_fn,
    hkk::rp2350::i2c::get_baudrate_fn,
    hkk::rp2350::i2c::set_index_fn,
    hkk::rp2350::i2c::get_index_fn,
    hkk::rp2350::i2c::write_blocking_fn,
    hkk::rp2350::i2c::read_blocking_fn,
    hkk::bus::i2c::transaction_fn,
    hkk::bus::i2c::commit_fn
};

I2C I2C2 {};    // Not present on RP2350
I2C I2C3 {};    // Not present on RP2350
I2C I2C4 {};    // Not present on RP2350
I2C I2C5 {};    // Not present on RP2350
I2C I2C6 {};    // Not present on RP2350
I2C I2C7 {};    // Not present on RP2350
}