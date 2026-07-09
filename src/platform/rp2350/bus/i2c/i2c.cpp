#include <hkk/defines.h>

#include <hkk/logger/logger.h>
#include <hkk/bus/i2c/i2c.hpp>

#include <pico/mutex.h>
#include <hardware/i2c.h>
#include <hardware/gpio.h>


namespace hkk::rp2350 {
// https://www.raspberrypi.com/documentation/pico-sdk/hardware.html#group_hardware_i2c

static int8 deinit_fn(void *ctx_raw) {
    HTRACE("i2c.cpp -> s:deinit_fn(void*):int8");

    if(!ctx_raw) {
        HERROR("[I2C    ] Null context passed to function");
        return hkk::bus::I2C_ERROR_NULL_CONTEXT;
    }

    auto *ctx = static_cast<hkk::bus::I2C_Config_Context*>(ctx_raw);

    if(!ctx->instance) {
        HERROR("[I2C    ] Null I2C instance in context");
        return hkk::bus::I2C_ERROR_NULL_INSTANCE;
    } 
    ::i2c_inst *instance = static_cast<::i2c_inst*>(ctx->instance);

    ::i2c_deinit(instance);

    ::gpio_deinit(ctx->sda);
    ::gpio_deinit(ctx->scl);

    ::gpio_set_pulls(ctx->sda, false, false);
    ::gpio_set_pulls(ctx->scl, false, false);

    if(!ctx->transaction || !ctx->transaction->mutex) {
        HWARN("[I2C    ] Null I2C mutex in context");
    } else {
        ::mutex_exit(static_cast<::mutex_t*>(ctx->transaction->mutex));
    }

    HINFO("[I2C    ] I2C%d deinitialization successful", ctx->index);

    ctx->index = -1;
    HDEBUG("[I2C    ] Instance index set to %d", ctx->index);

    return hkk::bus::I2C_OK;
}

static int8 init_fn(void *ctx_raw) {
    HTRACE("i2c.cpp -> s:init_fn(void*):int8");

    if(!ctx_raw) {
        HERROR("[I2C    ] Null context passed to function");
        return hkk::bus::I2C_ERROR_NULL_CONTEXT;
    }

    auto *ctx = static_cast<hkk::bus::I2C_Config_Context*>(ctx_raw);

    if(!ctx->instance) {
        HERROR("[I2C    ] Null I2C instance in context");
        return hkk::bus::I2C_ERROR_NULL_INSTANCE;
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
        return hkk::bus::I2C_ERROR_GENERIC;
    }

    if(!ctx->transaction || !ctx->transaction->mutex) {
        HERROR("[I2C    ] Null I2C mutex in context");
        
        deinit_fn(ctx_raw);
        return hkk::bus::I2C_ERROR_NULL_MUTEX;
    }
    ::mutex_init(static_cast<::mutex_t*>(ctx->transaction->mutex));

    HDEBUG("[I2C    ] SDA pin: %d", ctx->sda);
    HDEBUG("[I2C    ] SCL pin: %d", ctx->scl);
    HDEBUG("[I2C    ] Baud rate set to %d kHz", (ctx->baudrate / 1000));

    HINFO("[I2C    ] I2C%d initialization successful", ctx->index);
    return hkk::bus::I2C_OK;
}

static int8 set_baudrate_fn(void *ctx_raw, uint32 value) {
    HTRACE("i2c.cpp -> s:set_baudrate_fn(void*, uint32):int8");

    if(!ctx_raw) {
        HERROR("[I2C    ] Null context passed to function");
        return hkk::bus::I2C_ERROR_NULL_CONTEXT;
    }

    auto *ctx = static_cast<hkk::bus::I2C_Config_Context*>(ctx_raw);
    
    if(!ctx->instance) {
        HERROR("[I2C    ] Null I2C instance in context");
        return hkk::bus::I2C_ERROR_NULL_INSTANCE;
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
        return hkk::bus::I2C_ERROR_GENERIC;
    }

    if(baudrate != value) {
        HWARN("[I2C    ] Actual baud rate differs from desired baud rate");
        HWARN("[I2C    ] Desired: %d kHz", (ctx->baudrate / 1000));
        HWARN("[I2C    ] Actual : %d kHz", (baudrate/1000));
    }

    ctx->baudrate = baudrate;
    HINFO("[I2C    ] Baud rate set to %d kHz", (ctx->baudrate / 1000));

    return hkk::bus::I2C_OK;
}

static int32 get_baudrate_fn(void *ctx_raw) {
    HTRACE("i2c.cpp -> s:get_baudrate_fn(void*):int32");

    if(!ctx_raw) {
        HERROR("[I2C    ] Null context passed to function");
        return static_cast<int32>(hkk::bus::I2C_ERROR_NULL_CONTEXT);
    }

    auto *ctx = static_cast<hkk::bus::I2C_Config_Context*>(ctx_raw);

    if(!ctx->instance) {
        HERROR("[I2C    ] Null I2C instance in context");
        return static_cast<int32>(hkk::bus::I2C_ERROR_NULL_INSTANCE);
    } 
    ::i2c_inst *instance = static_cast<::i2c_inst*>(ctx->instance);

    HDEBUG("[I2C    ] Current baud rate: %d kHz", (ctx->baudrate / 1000));
    return static_cast<int32>(ctx->baudrate);
}

static int8 set_index_fn(void *ctx_raw, int8 value) {
    HTRACE("i2c.cpp -> s:set_index_fn(void*, int8):int8");
    
    HWARN("[I2C    ] Function not implemented on RP2350");
    return hkk::bus::I2C_ERROR_NOT_SUPPORTED;
}

static int32 get_index_fn(void *ctx_raw) {
    HTRACE("i2c.cpp -> s:get_index_fn(void*):int32");

    if(!ctx_raw) {
        HERROR("[I2C    ] Null context passed to function");
        return static_cast<int32>(hkk::bus::I2C_ERROR_NULL_CONTEXT);
    }

    auto *ctx = static_cast<hkk::bus::I2C_Config_Context*>(ctx_raw);

    if(!ctx->instance) {
        HERROR("[I2C    ] Null I2C instance in context");
        return static_cast<int32>(hkk::bus::I2C_ERROR_NULL_INSTANCE);
    } 
    ::i2c_inst *instance = static_cast<::i2c_inst*>(ctx->instance);

    uint8 index = ::i2c_get_index(instance);
    if(index != 0 && index != 1) {
        HERROR("[I2C    ] Invalid I2C instance index: %d", index);
        return static_cast<int32>(hkk::bus::I2C_ERROR_GENERIC);
    }

    HDEBUG("[I2C    ] Current index: %d", index);
    ctx->index = static_cast<int8>(index);
    
    return static_cast<int32>(ctx->index);
}

static int32 write_blocking_fn(void *ctx_raw, uint8 addr, const uint8 *src, size_t len, bool8 nostop) {
    HTRACE("i2c.cpp -> s:write_blocking_fn(void*, uint8, const uint8*, size_t, bool8 = false):int32");

    if(!ctx_raw) {
        HERROR("[I2C    ] Null context passed to function");
        return static_cast<int32>(hkk::bus::I2C_ERROR_NULL_CONTEXT);
    }

    auto *ctx = static_cast<hkk::bus::I2C_Config_Context*>(ctx_raw);

    if(!ctx->instance) {
        HERROR("[I2C    ] Null I2C instance in context");
        return static_cast<int32>(hkk::bus::I2C_ERROR_NULL_INSTANCE);
    } 
    ::i2c_inst *instance = static_cast<::i2c_inst*>(ctx->instance);

    if(!src) {
        HERROR("[I2C    ] Null data pointer passed to function");
        return static_cast<int32>(hkk::bus::I2C_ERROR_NULL_DATA);
    }

    if(len == 0) {
        HERROR("[I2C    ] Data length is 0");
        return static_cast<int32>(hkk::bus::I2C_ERROR_ZERO_LENGTH);
    }

    if(!ctx->transaction || !ctx->transaction->mutex) {
        HERROR("[I2C    ] Null I2C mutex in context");
        return hkk::bus::I2C_ERROR_NULL_MUTEX;
    }

    auto *transaction = static_cast<hkk::bus::I2C_Bus_Lock_State*>(ctx->transaction);

    int32 written = PICO_ERROR_GENERIC;
    if(transaction->active == false) {
        ::mutex_t *mutex = static_cast<::mutex_t*>(ctx->transaction->mutex);
        ::mutex_enter_blocking(mutex);
        
        written = ::i2c_write_blocking(instance, addr, src, len, nostop);

        ::mutex_exit(mutex);
    } else {
        written = ::i2c_write_blocking(instance, addr, src, len, nostop);
    }

    switch(written) {
        case PICO_ERROR_GENERIC:
            HERROR("[I2C    ] Address not acknowledged or device not present");
            return static_cast<int32>(hkk::bus::I2C_ERROR_NO_ACK);
        default:
            HERROR("[I2C    ] Write failed");
            return static_cast<int32>(hkk::bus::I2C_ERROR_WRITE_FAILED);
    }

    if(static_cast<size_t>(written) != len) {
        HERROR("[I2C    ] Partial write: %d/%zu bytes written", written, len);
        return static_cast<int32>(hkk::bus::I2C_ERROR_PARTIAL_WRITE);
    }

    HTRACE("[I2C    ] First byte   : 0x%02X", src[0]);
    HTRACE("[I2C    ] Data length  : %zu", len);
    HDEBUG("[I2C    ] Bytes written: %d", written);
    HTRACE("[I2C    ] No STOP      : %s", nostop ? "true" : "false");

    HINFO("[I2C    ] Write completed successfully");
    return written;
}

static int32 read_blocking_fn(void *ctx_raw, uint8 addr, uint8 *dst, size_t len, bool8 nostop) {
    HTRACE("i2c.cpp -> s:read_blocking_fn(void*, uint8, uint8*, size_t, bool8 = false):int32");

    if(!ctx_raw) {
        HERROR("[I2C    ] Null context passed to function");
        return static_cast<int32>(hkk::bus::I2C_ERROR_NULL_CONTEXT);
    }

    auto *ctx = static_cast<hkk::bus::I2C_Config_Context*>(ctx_raw);

    if(!ctx->instance) {
        HERROR("[I2C    ] Null I2C instance in context");
        return static_cast<int32>(hkk::bus::I2C_ERROR_NULL_INSTANCE);
    } 
    ::i2c_inst *instance = static_cast<::i2c_inst*>(ctx->instance);

    if(!dst) {
        HERROR("[I2C    ] Null data pointer passed to function");
        return static_cast<int32>(hkk::bus::I2C_ERROR_NULL_DATA);
    }

    if(len == 0) {
        HERROR("[I2C    ] Data length is 0");
        return static_cast<int32>(hkk::bus::I2C_ERROR_ZERO_LENGTH);
    }

    if(!ctx->transaction || !ctx->transaction->mutex) {
        HERROR("[I2C    ] Null I2C mutex in context");
        return hkk::bus::I2C_ERROR_NULL_MUTEX;
    }

    auto *transaction = static_cast<hkk::bus::I2C_Bus_Lock_State*>(ctx->transaction);

    int32 read = PICO_ERROR_GENERIC;
    if(transaction->active == false) {
        ::mutex_t *mutex = static_cast<::mutex_t*>(ctx->transaction->mutex);
        ::mutex_enter_blocking(mutex);
        
        read = ::i2c_read_blocking(instance, addr, dst, len, nostop);

        ::mutex_exit(mutex);
    } else {
        read = ::i2c_read_blocking(instance, addr, dst, len, nostop);
    }

    switch(read) {
        case PICO_ERROR_GENERIC:
            HERROR("[I2C    ] Address not acknowledged or device not present");
            return static_cast<int32>(hkk::bus::I2C_ERROR_NO_ACK);
        default:
            HERROR("[I2C    ] Read failed");
            return static_cast<int32>(hkk::bus::I2C_ERROR_READ_FAILED);
    }

    if(static_cast<size_t>(read) != len) {
        HERROR("[I2C    ] Partial read: %d/%zu bytes read", read, len);
        return static_cast<int32>(hkk::bus::I2C_ERROR_PARTIAL_READ);
    }

    HTRACE("[I2C    ] First byte   : 0x%02X", dst[0]);
    HTRACE("[I2C    ] Data length  : %zu", len);
    HDEBUG("[I2C    ] Bytes read   : %d", read);
    HTRACE("[I2C    ] No STOP      : %s", nostop ? "true" : "false");

    HINFO("[I2C    ] Read completed successfully");
    return read;
}

static int32 write_timeout_fn(void *ctx_raw, uint8 addr, const uint8 *src, size_t len, uint32 timeout_us, bool8 nostop) {
    HTRACE("i2c.cpp -> s:write_timeout_fn(void*, uint8, const uint8*, size_t, uint32, bool8 = false):int32");

    if(!ctx_raw) {
        HERROR("[I2C    ] Null context passed to function");
        return static_cast<int32>(hkk::bus::I2C_ERROR_NULL_CONTEXT);
    }

    auto *ctx = static_cast<hkk::bus::I2C_Config_Context*>(ctx_raw);

    if(!ctx->instance) {
        HERROR("[I2C    ] Null I2C instance in context");
        return static_cast<int32>(hkk::bus::I2C_ERROR_NULL_INSTANCE);
    }
    ::i2c_inst *instance = static_cast<::i2c_inst*>(ctx->instance);

    if(!src) {
        HERROR("[I2C    ] Null data pointer passed to function");
        return static_cast<int32>(hkk::bus::I2C_ERROR_NULL_DATA);   
    }
    
    if(!len) {
        HERROR("[I2C    ] Data length is 0");
        return static_cast<int32>(hkk::bus::I2C_ERROR_ZERO_LENGTH);
    }

    if(!ctx->transaction || !ctx->transaction->mutex) {
        HERROR("[I2C    ] Null I2C mutex in context");
        return hkk::bus::I2C_ERROR_NULL_MUTEX;
    }

    auto *transaction = static_cast<hkk::bus::I2C_Bus_Lock_State*>(ctx->transaction);

    int32 written = PICO_ERROR_GENERIC;
    if(transaction->active == false) {
        ::mutex_t *mutex = static_cast<::mutex_t*>(ctx->transaction->mutex);
        ::mutex_enter_timeout_us(mutex, timeout_us);
        
        written = ::i2c_write_timeout_us(instance, addr, src, len, nostop, timeout_us);

        ::mutex_exit(mutex);
    } else {
        written = ::i2c_write_timeout_us(instance, addr, src, len, nostop, timeout_us);
    }

    switch(written) {
        case PICO_ERROR_GENERIC:
            HERROR("[I2C    ] Address not acknowledged or device not present");
            return static_cast<int32>(hkk::bus::I2C_ERROR_NO_ACK);
        case PICO_ERROR_TIMEOUT:
            HERROR("[I2C    ] Timeout reached on write");
            return static_cast<int32>(hkk::bus::I2C_ERROR_TIMEOUT);
        default:
            HERROR("[I2C    ] Write failed");
            return static_cast<int32>(hkk::bus::I2C_ERROR_WRITE_FAILED);
    }

    if(static_cast<size_t>(written) != len) {
        HERROR("[I2C    ] Partial write: %d/%zu bytes written", written, len);
        return static_cast<int32>(hkk::bus::I2C_ERROR_PARTIAL_WRITE);
    }

    HTRACE("[I2C    ] First byte   : 0x%02X", src[0]);
    HTRACE("[I2C    ] Data length  : %zu", len);
    HDEBUG("[I2C    ] Bytes written: %d", written);
    HTRACE("[I2C    ] Timeout      : %dus", timeout_us);
    HTRACE("[I2C    ] No STOP      : %s", nostop ? "true" : "false");

    HINFO("[I2C    ] Write completed successfully");
    return written;
}

static int32 read_timeout_fn(void *ctx_raw, uint8 addr, uint8 *dst, size_t len, uint32 timeout_us, bool8 nostop) {
    HTRACE("i2c.cpp -> s:read_timeout_fn(void*, uint8, uint8*, size_t, uint32, bool8 = false):int32");

    if(!ctx_raw) {
        HERROR("[I2C    ] Null context passed to function");
        return static_cast<int32>(hkk::bus::I2C_ERROR_NULL_CONTEXT);
    }

    auto *ctx = static_cast<hkk::bus::I2C_Config_Context*>(ctx_raw);

    if(!ctx->instance) {
        HERROR("[I2C    ] Null I2C instance in context");
        return static_cast<int32>(hkk::bus::I2C_ERROR_NULL_INSTANCE);
    }
    ::i2c_inst *instance = static_cast<::i2c_inst*>(ctx->instance);

    if(!dst) {
        HERROR("[I2C    ] Null data pointer passed to function");
        return static_cast<int32>(hkk::bus::I2C_ERROR_NULL_DATA);
    }

    if(len == 0) {
        HERROR("[I2C    ] Data length is 0");
        return static_cast<int32>(hkk::bus::I2C_ERROR_ZERO_LENGTH);
    }

    if(!ctx->transaction || !ctx->transaction->mutex) {
        HERROR("[I2C    ] Null I2C mutex in context");
        return hkk::bus::I2C_ERROR_NULL_MUTEX;
    }

    auto *transaction = static_cast<hkk::bus::I2C_Bus_Lock_State*>(ctx->transaction);

    int32 read = PICO_ERROR_GENERIC;
    if(transaction->active == false) {
        ::mutex_t *mutex = static_cast<::mutex_t*>(ctx->transaction->mutex);
        ::mutex_enter_timeout_us(mutex, timeout_us);
        
        read = ::i2c_read_timeout_us(instance, addr, dst, len, nostop, timeout_us);

        ::mutex_exit(mutex);
    } else {
        read = ::i2c_read_timeout_us(instance, addr, dst, len, nostop, timeout_us);
    }

    switch(read) {
        case PICO_ERROR_GENERIC:
            HERROR("[I2C    ] Address not acknowledged or device not present");
            return static_cast<int32>(hkk::bus::I2C_ERROR_NO_ACK);
        case PICO_ERROR_TIMEOUT:
            HERROR("[I2C    ] Timeout reached on write");
            return static_cast<int32>(hkk::bus::I2C_ERROR_TIMEOUT);
        default:
            HERROR("[I2C    ] Read failed");
            return static_cast<int32>(hkk::bus::I2C_ERROR_READ_FAILED);
    }

    if(static_cast<size_t>(read) != len) {
        HERROR("[I2C    ] Partial read: %d/%zu bytes read", read, len);
        return static_cast<int32>(hkk::bus::I2C_ERROR_PARTIAL_READ);
    }

    HTRACE("[I2C    ] First byte   : 0x%02X", dst[0]);
    HTRACE("[I2C    ] Data length  : %zu", len);
    HDEBUG("[I2C    ] Bytes read   : %d", read);
    HTRACE("[I2C    ] No STOP      : %s", nostop ? "true" : "false");

    HINFO("[I2C    ] Read completed successfully");
    return read;
}

static int8 transaction_fn(void *ctx_raw, void *owner) {
    HTRACE("i2c.cpp -> s:transaction_fn(void*, void* = nullptr):int8");

    if(!ctx_raw) {
        HERROR("[I2C    ] Null context passed to function");
        return hkk::bus::I2C_ERROR_NULL_CONTEXT;
    }

    auto *ctx = static_cast<hkk::bus::I2C_Config_Context*>(ctx_raw);

    // There is no need for the I2C instance to be present
    // But why would you want to begin transaction without it?
    if(!ctx->instance) {
        HERROR("[I2C    ] Null I2C instance in context");
        return hkk::bus::I2C_ERROR_NULL_INSTANCE;
    }

    if(!ctx->transaction || !ctx->transaction->mutex) {
        HERROR("[I2C    ] Null I2C mutex in context");
        return hkk::bus::I2C_ERROR_NULL_MUTEX;
    }

    auto *transaction = static_cast<hkk::bus::I2C_Bus_Lock_State*>(ctx->transaction);

    if(transaction->active && transaction->owner == owner) {
        HWARN("[I2C    ] Transaction already active for this owner: %p", transaction->owner);
        return hkk::bus::I2C_ERROR_BUSY;
    } 

    if (transaction->active && transaction->owner != owner) {
        HWARN("[I2C    ] Transaction already active for another owner: %p", transaction->owner);
    }

    ::mutex_t *mutex = static_cast<::mutex_t*>(transaction->mutex);
    ::mutex_enter_blocking(mutex);

    transaction->owner  = owner;
    transaction->active = true;

    HTRACE("[I2C    ] Transaction begin for owner %p", transaction->owner);
    return hkk::bus::I2C_OK;
}

static int8 commit_fn(void *ctx_raw, void *owner) {
    HTRACE("i2c.cpp -> s:commit_fn(void*, void* = nullptr):int8");

    if(!ctx_raw) {
        HERROR("[I2C    ] Null context passed to function");
        return hkk::bus::I2C_ERROR_NULL_CONTEXT;
    }

    auto *ctx = static_cast<hkk::bus::I2C_Config_Context*>(ctx_raw);

    // There is no need for the I2C instance to be present
    // But why would you want to commit transaction without it?
    if(!ctx->instance) {
        HERROR("[I2C    ] Null I2C instance in context");
        return hkk::bus::I2C_ERROR_NULL_INSTANCE;
    }

    if(!ctx->transaction || !ctx->transaction->mutex) {
        HERROR("[I2C    ] Null I2C mutex in context");
        return hkk::bus::I2C_ERROR_NULL_MUTEX;
    }

    auto *transaction = static_cast<hkk::bus::I2C_Bus_Lock_State*>(ctx->transaction);

    if(transaction->active == false) {
        HWARN("[I2C    ] No transaction in progress");
        return hkk::bus::I2C_OK;
    } 

    if (transaction->active && transaction->owner != owner) {
        HWARN("[I2C    ] Transaction already active for another owner: %p", transaction->owner);
        return hkk::bus::I2C_ERROR_BUSY;
    }

    void *old_owner = transaction->owner;

    transaction->owner  = nullptr;
    transaction->active = false;

    ::mutex_t *mutex = static_cast<::mutex_t*>(ctx->transaction->mutex);
    ::mutex_exit(mutex);

    HTRACE("[I2C    ] Transaction end for owner: %p", old_owner);
    return hkk::bus::I2C_OK;
}

}

namespace hkk::bus {

int8 bind(I2C &i2c, I2C_Config_Context &cfg) {
    HTRACE("i2c.cpp -> bind(I2C&, I2C_Config_Context&):int8");

    i2c.ctx = static_cast<hkk::bus::I2C_Config_Context*>(&cfg);

    i2c.init_fn = hkk::rp2350::init_fn;
    i2c.deinit_fn = hkk::rp2350::deinit_fn;

    i2c.set_baudrate_fn = hkk::rp2350::set_baudrate_fn;
    i2c.get_baudrate_fn = hkk::rp2350::get_baudrate_fn;

    i2c.set_index_fn = hkk::rp2350::set_index_fn;
    i2c.get_index_fn = hkk::rp2350::get_index_fn;

    i2c.write_blocking_fn = hkk::rp2350::write_blocking_fn;
    i2c.read_blocking_fn  = hkk::rp2350::read_blocking_fn;

    HDEBUG("[I2C    ] I2C instance bound to config context");
    return I2C_OK;
}

static ::mutex_t i2c0_mutex;
static I2C_Bus_Lock_State i2c0_transaction {
    .mutex  = &i2c0_mutex,
    .owner  = nullptr,
    .active = false,
};

static ::mutex_t i2c1_mutex;
static I2C_Bus_Lock_State i2c1_transaction {
    .mutex = &i2c1_mutex,
    .owner  = nullptr,
    .active = false,
};

static I2C_Config_Context i2c0_default_config {
    .transaction = &i2c0_transaction,
    .instance = i2c0,
    .baudrate = 100000,
    .sda = 4,
    .scl = 5,
    .index = 0
};

static I2C_Config_Context i2c1_default_config {
    .transaction = &i2c1_transaction,
    .instance = i2c1,
    .baudrate = 100000,
    .sda = 6,
    .scl = 7,
    .index = 1
};

I2C I2C0 {
    &i2c0_default_config,
    hkk::rp2350::init_fn,
    hkk::rp2350::deinit_fn,
    hkk::rp2350::set_baudrate_fn,
    hkk::rp2350::get_baudrate_fn,
    hkk::rp2350::set_index_fn,
    hkk::rp2350::get_index_fn,
    hkk::rp2350::write_blocking_fn,
    hkk::rp2350::read_blocking_fn,
    hkk::rp2350::transaction_fn,
    hkk::rp2350::commit_fn
};

I2C I2C1 {
    &i2c1_default_config,
    hkk::rp2350::init_fn,
    hkk::rp2350::deinit_fn,
    hkk::rp2350::set_baudrate_fn,
    hkk::rp2350::get_baudrate_fn,
    hkk::rp2350::set_index_fn,
    hkk::rp2350::get_index_fn,
    hkk::rp2350::write_blocking_fn,
    hkk::rp2350::read_blocking_fn,
    hkk::rp2350::transaction_fn,
    hkk::rp2350::commit_fn
};

I2C I2C2 {};    // Not present on RP2350
I2C I2C3 {};    // Not present on RP2350
I2C I2C4 {};    // Not present on RP2350
I2C I2C5 {};    // Not present on RP2350
I2C I2C6 {};    // Not present on RP2350
I2C I2C7 {};    // Not present on RP2350
}