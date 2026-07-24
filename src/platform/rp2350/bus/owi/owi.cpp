#include <hkk/bus/owi/owi.hpp>

#include <hardware/pio.h>
#include <pico/mutex.h>

#include "owi.pio.h"


namespace hkk::rp2350::owi {

static int8 init_fn(void *ctx_raw) {
    HTRACE("owi.cpp -> s:init_fn(void*):int8");

    if(!ctx_raw) {
        HERROR("[OWI    ] Null context passed to function");
        return hkk::bus::owi::OWI_ERROR_NULL_CONTEXT;
    }
    auto *ctx = static_cast<hkk::bus::owi::ConfigContext*>(ctx_raw);

    if(!ctx->instance) {
        HERROR("[OWI    ] Null OWI instance in context");
        return ctx->status = hkk::bus::owi::OWI_ERROR_NULL_INSTANCE;
    }
    auto instance = *(static_cast<PIO*>(ctx->instance));

    if(ctx->pin < 0) {
        HERROR("[OWI    ] No OWI pin configured");
        return ctx->status = hkk::bus::owi::OWI_ERROR_GENERIC;
    }

    if(ctx->state_machine < 0) {
        HERROR("[OWI    ] No PIO state machine configured");
        return ctx->status = hkk::bus::owi::OWI_ERROR_GENERIC;
    }

    ctx->owi_state_machine = ::pio_claim_unused_sm(instance, false);
    if(ctx->owi_state_machine <= PICO_ERROR_GENERIC) {
        HERROR("[OWI    ] Could not claim free PIO state machine");
        return ctx->status = hkk::bus::owi::OWI_ERROR_GENERIC;
    }

    ::gpio_init(ctx->pin);
    ::pio_gpio_init(instance, ctx->pin);

    ctx->owi_state_machine_offset = ::pio_add_program(instance, &owi_program);
    owi_sm_init(instance, ctx->state_machine, ctx->owi_state_machine_offset, ctx->pin, ctx->bit_mode);

    ctx->owi_jmp_reset_instruction = owi_reset_instr(ctx->owi_state_machine_offset);

    HDEBUG("[OWI    ] Pin                 : %d", ctx->pin);
    HDEBUG("[OWI    ] State machine       : %d", ctx->state_machine);
    HDEBUG("[OWI    ] State machine offset: %d", ctx->owi_state_machine_offset);

    HINFO("[OWI    ] OW%d initialized", ctx->state_machine);
    return ctx->status = hkk::bus::owi::OWI_OK;
}

static int8 deinit_fn(void *ctx_raw) {
    HTRACE("owi.cpp -> s:deinit_fn(void*):int8");

    if(!ctx_raw) {
        HERROR("[OWI    ] Null context passed to function");
        return hkk::bus::owi::OWI_ERROR_NULL_CONTEXT;
    }
    auto *ctx = static_cast<hkk::bus::owi::ConfigContext*>(ctx_raw);

    if(!ctx->instance) {
        HERROR("[OWI    ] Null OWI instance in context");
        return ctx->status = hkk::bus::owi::OWI_ERROR_NULL_INSTANCE;
    }
    auto instance = *(static_cast<PIO*>(ctx->instance));

    if(ctx->pin < 0) {
        HERROR("[OWI    ] No OWI pin configured");
        return ctx->status = hkk::bus::owi::OWI_ERROR_GENERIC;
    }    

    if(ctx->state_machine < 0) {
        HERROR("[OWI    ] No PIO state machine configured");
        return ctx->status = hkk::bus::owi::OWI_ERROR_GENERIC;
    }

    ::pio_remove_program(instance, &owi_program, ctx->owi_state_machine_offset);
    ::pio_sm_unclaim(instance, ctx->state_machine);
    ::gpio_deinit(ctx->pin);

    HINFO("[OWI    ] OW%d deinitialized", ctx->state_machine);
    return ctx->status = hkk::bus::owi::OWI_OK;
}

static int8 set_baudrate_fn(void *ctx_raw, uint32 value) {
    HTRACE("owi.cpp -> s:set_baudrate_fn(void*, uint32):int8");

    HFATAL("[OWI    ] Function not implemented on RP2350");
    return hkk::bus::owi::OWI_ERROR_NOT_SUPPORTED;
}

static int8 get_baudrate_fn(void *ctx_raw) {
    HTRACE("owi.cpp -> s:get_baudrate_fn(void*):int8");

    HFATAL("[OWI    ] Function not implemented on RP2350");
    return hkk::bus::owi::OWI_ERROR_NOT_SUPPORTED;
}

static int8 set_index_fn(void *ctx_raw, int8 value) {
    HTRACE("owi.cpp -> s:set_index_fn(void*, int8):int8");

    HFATAL("[OWI    ] Function not implemented on RP2350");
    return hkk::bus::owi::OWI_ERROR_NOT_SUPPORTED;
}

static int8 get_index_fn(void *ctx_raw) {
    HTRACE("owi.cpp -> s:get_index_fn(void*):int8");

    HFATAL("[OWI    ] Function not implemented on RP2350");
    return hkk::bus::owi::OWI_ERROR_NOT_SUPPORTED;
}

static int8 write_blocking_fn(void *ctx_raw, const uint8 *src, size_t len) {
    HTRACE("owi.cpp -> s:write_blocking_fn(void*, const uint8*, size_t):int8");

    if(!ctx_raw) {
        HERROR("[OWI    ] Null context passed to function");
        return hkk::bus::owi::OWI_ERROR_NULL_CONTEXT;
    }
    auto *ctx = static_cast<hkk::bus::owi::ConfigContext*>(ctx_raw);

    if(!ctx->instance) {
        HERROR("[OWI    ] Null OWI instance in context");
        return ctx->status = hkk::bus::owi::OWI_ERROR_NULL_INSTANCE;
    }
    auto instance = *(static_cast<PIO*>(ctx->instance));

    if(!src) {
        HERROR("[OWI    ] Null data pointer passed to function");
        return ctx->status = hkk::bus::owi::OWI_ERROR_NULL_DATA;
    }

    if(len == 0) {
        HERROR("[OWI    ] Data length is 0");
        return ctx->status = hkk::bus::owi::OWI_ERROR_ZERO_LENGTH;
    }

    if(!ctx->transaction || !ctx->transaction->mutex) {
        HERROR("[OWI    ] Null OWI mutex in context");
        return ctx->status = hkk::bus::owi::OWI_ERROR_NULL_MUTEX;
    }
    auto *transaction = static_cast<hkk::bus::owi::LockState*>(ctx->transaction);


    if(transaction->active == false) {
        auto *mutex = static_cast<::mutex_t*>(transaction->mutex);
        ::mutex_enter_blocking(mutex);
        transaction->active = true;

        for(size_t i = 0; i != len; ++i) {
            uint32 data = src[i];
            ::pio_sm_put_blocking(instance, ctx->state_machine, data);
            
            uint32 written = ::pio_sm_get_blocking(instance, ctx->state_machine);
            if(written != data) {
                HWARN ("[OWI    ] OWI state improper");
                HDEBUG("[OWI    ] Expected: %d", data);
                HDEBUG("[OWI    ] Got     : %d", written);

                return hkk::bus::owi::OWI_ERROR_WRITE_FAILED;
            }
        }

        transaction->active = false;
        ::mutex_exit(mutex);
    } else {
        for(size_t i = 0; i != len; ++i) {
            uint32 data = src[i];
            ::pio_sm_put_blocking(instance, ctx->state_machine, data);
            
            uint32 written = ::pio_sm_get_blocking(instance, ctx->state_machine);
            if(written != data) {
                HWARN ("[OWI    ] OWI state improper");
                HDEBUG("[OWI    ] Expected: %d", data);
                HDEBUG("[OWI    ] Got     : %d", written);

                return hkk::bus::owi::OWI_ERROR_WRITE_FAILED;
            }
        }   
    }

    HTRACE("[OWI    ] First byte   : 0x%02X", src[0]);
    HTRACE("[OWI    ] Data length  : %zu", len);

    HTRACE("[OWI    ] Write complete");

    return ctx->status = hkk::bus::owi::OWI_OK;
}

static int8 read_blocking_fn(void *ctx_raw, uint8 *dst, size_t len) {
    HTRACE("owi.cpp -> s:read_blocking_fn(void*, uint8*, size_t):int8");

    if(!ctx_raw) {
        HERROR("[OWI    ] Null context passed to function");
        return hkk::bus::owi::OWI_ERROR_NULL_CONTEXT;
    }
    auto *ctx = static_cast<hkk::bus::owi::ConfigContext*>(ctx_raw);

    if(!ctx->instance) {
        HERROR("[OWI    ] Null OWI instance in context");
        return ctx->status = hkk::bus::owi::OWI_ERROR_NULL_INSTANCE;
    }
    auto instance = *(static_cast<PIO*>(ctx->instance));

    if(!dst) {
        HERROR("[OWI    ] Null data pointer passed to function");
        return ctx->status = hkk::bus::owi::OWI_ERROR_NULL_DATA;
    }

    if(len == 0) {
        HERROR("[OWI    ] Data length is 0");
        return ctx->status = hkk::bus::owi::OWI_ERROR_ZERO_LENGTH;
    }

    if(!ctx->transaction || !ctx->transaction->mutex) {
        HERROR("[OWI    ] Null OWI mutex in context");
        return ctx->status = hkk::bus::owi::OWI_ERROR_NULL_MUTEX;
    }
    auto *transaction = static_cast<hkk::bus::owi::LockState*>(ctx->transaction);


    const uint8 shift = (32 - ctx->bit_mode);
    const uint32 read_command = ((uint32{1} << ctx->bit_mode) - 1u) << shift;

    if(transaction->active == false) {
        auto *mutex = static_cast<::mutex_t*>(transaction->mutex);
        ::mutex_enter_blocking(mutex);
        transaction->active = true;

        for(size_t i = 0; i != len; ++i) {
            ::pio_sm_put_blocking(instance, ctx->state_machine, read_command);
            dst[i] = static_cast<uint8>(::pio_sm_get_blocking(instance, ctx->state_machine) >> shift);
        }   

        transaction->active = false;
        ::mutex_exit(mutex);
    } else {
        for(size_t i = 0; i != len; ++i) {
            ::pio_sm_put_blocking(instance, ctx->state_machine, read_command);
            dst[i] = static_cast<uint8>(::pio_sm_get_blocking(instance, ctx->state_machine) >> shift);
        }   
    }

    HTRACE("[OWI    ] First byte   : 0x%02X", src[0]);
    HTRACE("[OWI    ] Data length  : %zu", len);

    HTRACE("[OWI    ] Write complete");

    return ctx->status = hkk::bus::owi::OWI_OK;
}

static int8 write_timeout_fn(void *ctx_raw, const uint8 *src, size_t len, uint32 timeout_us) {
    HTRACE("owi.cpp -> s:write_timeout_fn(void*, const uint8*, size_t, uint32):int8");

    if(!ctx_raw) {
        HERROR("[OWI    ] Null context passed to function");
        return hkk::bus::owi::OWI_ERROR_NULL_CONTEXT;
    }
    auto *ctx = static_cast<hkk::bus::owi::ConfigContext*>(ctx_raw);

    if(!ctx->instance) {
        HERROR("[OWI    ] Null OWI instance in context");
        return ctx->status = hkk::bus::owi::OWI_ERROR_NULL_INSTANCE;
    }
    auto instance = *(static_cast<PIO*>(ctx->instance));

    if(!src) {
        HERROR("[OWI    ] Null data pointer passed to function");
        return ctx->status = hkk::bus::owi::OWI_ERROR_NULL_DATA;
    }

    if(len == 0) {
        HERROR("[OWI    ] Data length is 0");
        return ctx->status = hkk::bus::owi::OWI_ERROR_ZERO_LENGTH;
    }

    if(!ctx->transaction || !ctx->transaction->mutex) {
        HERROR("[OWI    ] Null OWI mutex in context");
        return ctx->status = hkk::bus::owi::OWI_ERROR_NULL_MUTEX;
    }
    auto *transaction = static_cast<hkk::bus::owi::LockState*>(ctx->transaction);


    hkk::utils::AlarmContext owi_timeout_alarm {
        .callback = [](void *data) -> bool8 {
            if(!data) return false;

            auto *transaction = static_cast<hkk::bus::owi::LockState*>(data);
            transaction->timeout = true;

            return false;
        },
        .data = transaction
    };

    ctx->status = hkk::bus::owi::OWI_OK;
    if(transaction->active == false) {
        auto *mutex = static_cast<::mutex_t*>(transaction->mutex);
        ::mutex_enter_blocking(mutex);
        transaction->active = true;

        int32 alarm_id = -1;
        for(size_t i = 0; i != len; ++i) {
            transaction->timeout = false;

            alarm_id = hkk::utils::alarm_us(timeout_us, &owi_timeout_alarm, true);
            if(alarm_id < 0) {
                HERROR("[OWI    ] Failed to start write timeout alarm");
                ctx->status = hkk::bus::owi::OWI_ERROR_GENERIC;
                break;
            }            
            HTRACE("[OWI   ] Write alarm started: %d us", timeout_us);


            while(::pio_sm_is_tx_fifo_full(instance, ctx->state_machine)) {
                if(transaction->timeout == true) {
                    HERROR("[OWI    ] Timeout reached on write");
                    hkk::utils::cancel_alarm(alarm_id);

                    hkk::bus::owi::OWI::soft_reset(ctx);

                    ctx->status = hkk::bus::owi::OWI_ERROR_TIMEOUT;
                    break;
                } else continue;
            }
            if(ctx->status == hkk::bus::owi::OWI_ERROR_TIMEOUT) break;


            uint32 data = src[i];
            ::pio_sm_put(instance, ctx->state_machine, data);


            while(::pio_sm_is_rx_fifo_empty(instance, ctx->state_machine)) {
                if(transaction->timeout == true) {
                    HERROR("[OWI    ] Timeout reached on write");
                    hkk::utils::cancel_alarm(alarm_id);

                    hkk::bus::owi::OWI::soft_reset(ctx);

                    ctx->status = hkk::bus::owi::OWI_ERROR_TIMEOUT;
                    break;
                } else continue;
            }
            if(ctx->status == hkk::bus::owi::OWI_ERROR_TIMEOUT) break;


            uint32 written = ::pio_sm_get(instance, ctx->state_machine);
            hkk::utils::cancel_alarm(alarm_id);

            if(written != data) {
                HWARN ("[OWI    ] OWI state improper");
                HDEBUG("[OWI    ] Expected: %d", data);
                HDEBUG("[OWI    ] Got     : %d", written);

                ctx->status = hkk::bus::owi::OWI_ERROR_WRITE_FAILED;
                break;
            }
        }

        transaction->active = false;
        ::mutex_exit(mutex);
    } else {
        int32 alarm_id = -1;
        for(size_t i = 0; i != len; ++i) {
            transaction->timeout = false;

            alarm_id = hkk::utils::alarm_us(timeout_us, &owi_timeout_alarm, true);
            if(alarm_id < 0) {
                HERROR("[OWI    ] Failed to start write timeout alarm");
                ctx->status = hkk::bus::owi::OWI_ERROR_GENERIC;
                break;
            }            
            HTRACE("[OWI   ] Write alarm started: %d us", timeout_us);


            while(::pio_sm_is_tx_fifo_full(instance, ctx->state_machine)) {
                if(transaction->timeout == true) {
                    HERROR("[OWI    ] Timeout reached on write");
                    hkk::utils::cancel_alarm(alarm_id);

                    hkk::bus::owi::OWI::soft_reset(ctx);

                    ctx->status = hkk::bus::owi::OWI_ERROR_TIMEOUT;
                    break;
                } else continue;
            }
            if(ctx->status == hkk::bus::owi::OWI_ERROR_TIMEOUT) break;


            uint32 data = src[i];
            ::pio_sm_put(instance, ctx->state_machine, data);


            while(::pio_sm_is_rx_fifo_empty(instance, ctx->state_machine)) {
                if(transaction->timeout == true) {
                    HERROR("[OWI    ] Timeout reached on write");
                    hkk::utils::cancel_alarm(alarm_id);

                    hkk::bus::owi::OWI::soft_reset(ctx);

                    ctx->status = hkk::bus::owi::OWI_ERROR_TIMEOUT;
                    break;
                } else continue;
            }
            if(ctx->status == hkk::bus::owi::OWI_ERROR_TIMEOUT) break;


            uint32 written = ::pio_sm_get(instance, ctx->state_machine);
            hkk::utils::cancel_alarm(alarm_id);

            if(written != data) {
                HWARN ("[OWI    ] OWI state improper");
                HDEBUG("[OWI    ] Expected: %d", data);
                HDEBUG("[OWI    ] Got     : %d", written);

                ctx->status = hkk::bus::owi::OWI_ERROR_WRITE_FAILED;
                break;
            }
        }
    }

    HTRACE("[OWI    ] First byte   : 0x%02X", src[0]);
    HTRACE("[OWI    ] Data length  : %zu", len);

    HTRACE("[OWI    ] Write complete");

    return ctx->status;
}

static int8 read_timeout_fn(void *ctx_raw, uint8 *dst, size_t len, uint32 timeout_us) {
    HTRACE("owi.cpp -> s:read_blocking_fn(void*, uint8*, size_t, uint32):int8");

    if(!ctx_raw) {
        HERROR("[OWI    ] Null context passed to function");
        return hkk::bus::owi::OWI_ERROR_NULL_CONTEXT;
    }
    auto *ctx = static_cast<hkk::bus::owi::ConfigContext*>(ctx_raw);

    if(!ctx->instance) {
        HERROR("[OWI    ] Null OWI instance in context");
        return ctx->status = hkk::bus::owi::OWI_ERROR_NULL_INSTANCE;
    }
    auto instance = *(static_cast<PIO*>(ctx->instance));

    if(!dst) {
        HERROR("[OWI    ] Null data pointer passed to function");
        return ctx->status = hkk::bus::owi::OWI_ERROR_NULL_DATA;
    }

    if(len == 0) {
        HERROR("[OWI    ] Data length is 0");
        return ctx->status = hkk::bus::owi::OWI_ERROR_ZERO_LENGTH;
    }

    if(!ctx->transaction || !ctx->transaction->mutex) {
        HERROR("[OWI    ] Null OWI mutex in context");
        return ctx->status = hkk::bus::owi::OWI_ERROR_NULL_MUTEX;
    }
    auto *transaction = static_cast<hkk::bus::owi::LockState*>(ctx->transaction);


    hkk::utils::AlarmContext owi_timeout_alarm {
        .callback = [](void *data) -> bool8 {
            if(!data) return false;

            auto *transaction = static_cast<hkk::bus::owi::LockState*>(data);
            transaction->timeout = true;

            return false;
        },
        .data = transaction
    };

    const uint8 shift = (32 - ctx->bit_mode);
    const uint32 read_command = ((uint32{1} << ctx->bit_mode) - 1u) << shift;

    ctx->status = hkk::bus::owi::OWI_OK;
    if(transaction->active == false) {
        auto *mutex = static_cast<::mutex_t*>(transaction->mutex);
        ::mutex_enter_blocking(mutex);
        transaction->active = true;

        int32 alarm_id = -1;
        for(size_t i = 0; i != len; ++i) {
            transaction->timeout = false;

            alarm_id = hkk::utils::alarm_us(timeout_us, &owi_timeout_alarm, true);
            if(alarm_id < 0) {
                HERROR("[OWI    ] Failed to start read timeout alarm");
                ctx->status = hkk::bus::owi::OWI_ERROR_GENERIC;
                break;
            }            
            HTRACE("[OWI   ] Read alarm started: %d us", timeout_us);


            while(::pio_sm_is_tx_fifo_full(instance, ctx->state_machine)) {
                if(transaction->timeout == true) {
                    HERROR("[OWI    ] Timeout reached on read");
                    hkk::utils::cancel_alarm(alarm_id);

                    hkk::bus::owi::OWI::soft_reset(ctx);
                    
                    ctx->status = hkk::bus::owi::OWI_ERROR_TIMEOUT;
                    break;
                } else continue;
            }
            if(ctx->status == hkk::bus::owi::OWI_ERROR_TIMEOUT) break;


            ::pio_sm_put(instance, ctx->state_machine, read_command);


            while(::pio_sm_is_rx_fifo_empty(instance, ctx->state_machine)) {
                if(transaction->timeout == true) {
                    HERROR("[OWI    ] Timeout reached on read");
                    hkk::utils::cancel_alarm(alarm_id);

                    hkk::bus::owi::OWI::soft_reset(ctx);

                    ctx->status = hkk::bus::owi::OWI_ERROR_TIMEOUT;
                    break;
                } else continue;
            }
            if(ctx->status == hkk::bus::owi::OWI_ERROR_TIMEOUT) break;

            dst[i] = static_cast<uint8>(::pio_sm_get(instance, ctx->state_machine) >> shift);
            hkk::utils::cancel_alarm(alarm_id);
        }   

        transaction->active = false;
        ::mutex_exit(mutex);
    } else {
        int32 alarm_id = -1;
        for(size_t i = 0; i != len; ++i) {
            transaction->timeout = false;

            alarm_id = hkk::utils::alarm_us(timeout_us, &owi_timeout_alarm, true);
            if(alarm_id < 0) {
                HERROR("[OWI    ] Failed to start read timeout alarm");
                ctx->status = hkk::bus::owi::OWI_ERROR_GENERIC;
                break;
            }            
            HTRACE("[OWI   ] Read alarm started: %d us", timeout_us);


            while(::pio_sm_is_tx_fifo_full(instance, ctx->state_machine)) {
                if(transaction->timeout == true) {
                    HERROR("[OWI    ] Timeout reached on read");
                    hkk::utils::cancel_alarm(alarm_id);

                    hkk::bus::owi::OWI::soft_reset(ctx);

                    ctx->status = hkk::bus::owi::OWI_ERROR_TIMEOUT;
                    break;
                } else continue;
            }
            if(ctx->status == hkk::bus::owi::OWI_ERROR_TIMEOUT) break;


            ::pio_sm_put(instance, ctx->state_machine, read_command);


            while(::pio_sm_is_rx_fifo_empty(instance, ctx->state_machine)) {
                if(transaction->timeout == true) {
                    HERROR("[OWI    ] Timeout reached on read");
                    hkk::utils::cancel_alarm(alarm_id);

                    hkk::bus::owi::OWI::soft_reset(ctx);

                    ctx->status = hkk::bus::owi::OWI_ERROR_TIMEOUT;
                    break;
                } else continue;
            }
            if(ctx->status == hkk::bus::owi::OWI_ERROR_TIMEOUT) break;

            dst[i] = static_cast<uint8>(::pio_sm_get(instance, ctx->state_machine) >> shift);
            hkk::utils::cancel_alarm(alarm_id);
        }  
    }

    HTRACE("[OWI    ] First byte   : 0x%02X", src[0]);
    HTRACE("[OWI    ] Data length  : %zu", len);

    HTRACE("[OWI    ] Read complete");

    return ctx->status;
}


static hkk::bus::owi::BackendTable backend {
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


namespace hkk::bus::owi {

inline constexpr uint8 DATA_PIN0 = 20;
inline constexpr uint8 DATA_PIN1 = 22;


const char *rts(int8 status) {
    switch(status) {
        case OWI_OK:                        return "OWI_OK";    
        case OWI_ERROR_NULL_CONTEXT:        return "OWI_ERROR_NULL_CONTEXT";     
        case OWI_ERROR_NULL_INSTANCE:       return "OWI_ERROR_NULL_INSTANCE";    
        case OWI_ERROR_NULL_DATA:           return "OWI_ERROR_NULL_DATA";        
        case OWI_ERROR_ZERO_LENGTH:         return "OWI_ERROR_ZERO_LENGTH";      
        case OWI_ERROR_NO_ACK:              return "OWI_ERROR_NO_ACK";           
        case OWI_ERROR_PARTIAL_WRITE:       return "OWI_ERROR_PARTIAL_WRITE";    
        case OWI_ERROR_WRITE_FAILED:        return "OWI_ERROR_WRITE_FAILED";     
        case OWI_ERROR_PARTIAL_READ:        return "OWI_ERROR_PARTIAL_READ";     
        case OWI_ERROR_READ_FAILED:         return "OWI_ERROR_READ_FAILED";        
        case OWI_ERROR_NOT_SUPPORTED:       return "OWI_ERROR_NOT_SUPPORTED";      
        case OWI_ERROR_TIMEOUT:             return "OWI_ERROR_TIMEOUT";           
        case OWI_ERROR_NULL_MUTEX:          return "OWI_ERROR_NULL_MUTEX";        
        case OWI_ERROR_BUSY:                return "OWI_ERROR_BUSY";              
        case OWI_ERROR_GENERIC:             return "OWI_ERROR_GENERIC";            
        case OWI_FUNCTION_NOT_IMPLEMENTED:  return "OWI_FUNCTION_NOT_IMPLEMENTED"; 
        default:                            return "OWI_ERROR_UNKNOWN";
    }
}

static int8 transaction_fn(void *ctx_raw, void *owner) {
    HTRACE("owi.cpp -> s:transaction_fn(void*, void* = nullptr):int8");

    if(!ctx_raw) {
        HERROR("[OWI    ] Null context passed to function");
        return OWI_ERROR_NULL_CONTEXT;
    }
    auto *ctx = static_cast<ConfigContext*>(ctx_raw);

    // There is no need for the OWI instance to be present
    // But why would you want to begin transaction without it?
    if(!ctx->instance) {
        HERROR("[OWI    ] Null OWI instance in context");
        return ctx->status = OWI_ERROR_NULL_INSTANCE;
    }

    if(!ctx->transaction || !ctx->transaction->mutex) {
        HERROR("[OWI    ] Null OWI mutex in context");
        return ctx->status = OWI_ERROR_NULL_MUTEX;
    }
    auto *transaction = static_cast<LockState*>(ctx->transaction);

    if(transaction->active && transaction->owner == owner) {
        if(transaction->nostop == true) {
            HTRACE("[OWI    ] Transaction keep alive for owner: %p", transaction->owner);
            return ctx->status = OWI_KEEP_ALIVE;
        }

        HWARN("[OWI    ] Transaction already active for this owner: %p", transaction->owner);
        return ctx->status = OWI_ERROR_BUSY;
    } 

    if(transaction->active && transaction->owner != owner) {
        HWARN("[OWI    ] Transaction already active for another owner: %p (%p)", transaction->owner, owner);
        ctx->status = OWI_ERROR_BUSY;
    }

    ::mutex_t *mutex = static_cast<::mutex_t*>(transaction->mutex);
    ::mutex_enter_blocking(mutex);

    transaction->owner  = owner;
    transaction->active = true;

    HTRACE("[OWI    ] Transaction begin for owner %p", transaction->owner);

    ctx->status = OWI_OK;
    return ctx->status;
}

static int8 commit_fn(void *ctx_raw, void *owner) {
    HTRACE("owi.cpp -> s:commit_fn(void*, void* = nullptr):int8");

    if(!ctx_raw) {
        HERROR("[OWI    ] Null context passed to function");
        return OWI_ERROR_NULL_CONTEXT;
    }
    auto *ctx = static_cast<ConfigContext*>(ctx_raw);

    // There is no need for the OWI instance to be present
    // But why would you want to commit transaction without it?
    if(!ctx->instance) {
        HERROR("[OWI    ] Null OWI instance in context");
        return ctx->status = OWI_ERROR_NULL_INSTANCE;
    }

    if(!ctx->transaction || !ctx->transaction->mutex) {
        HERROR("[OWI    ] Null OWI mutex in context");
        return ctx->status = OWI_ERROR_NULL_MUTEX;
    }
    auto *transaction = static_cast<LockState*>(ctx->transaction);

    if(transaction->active == false) {
        HWARN("[OWI    ] No transaction in progress");
        return ctx->status = OWI_OK;
    } 

    if(transaction->active && transaction->owner != owner) {
        HWARN("[OWI    ] Transaction already active for another owner: %p (%p)", transaction->owner, owner);
        return ctx->status = OWI_ERROR_BUSY;
    }

    if(transaction->active && transaction->nostop == true) {
        HTRACE("[OWI    ] Transaction keep alive for owner: %p", transaction->owner);
        return ctx->status = OWI_KEEP_ALIVE;
    }

    void *old_owner = transaction->owner;

    transaction->owner  = nullptr;
    transaction->active = false;

    ::mutex_t *mutex = static_cast<::mutex_t*>(ctx->transaction->mutex);
    ::mutex_exit(mutex);

    HTRACE("[OWI    ] Transaction end for owner: %p", old_owner);

    ctx->status = OWI_OK;
    return ctx->status;
}

int8 OWI::soft_reset(ConfigContext *ctx) {
    HTRACE("owi.cpp -> s:OWI::soft_reset(void*, ConfigContext*):int8");

    if(!ctx) {
        HERROR("[OWI    ] Null context passed to function");
        return OWI_ERROR_NULL_CONTEXT;
    }

    ctx->status = OWI_OK;

    if(!ctx->instance) {
        HERROR("[OWI    ] Null OWI instance in context");
        return ctx->status = OWI_ERROR_NULL_INSTANCE;
    }
    auto instance = *(static_cast<PIO*>(ctx->instance));


    ::pio_sm_set_enabled(instance, ctx->state_machine, false);

    ::pio_sm_clear_fifos(instance, ctx->state_machine);
    ::pio_sm_restart(instance, ctx->state_machine);

    ::pio_sm_exec(instance, ctx->state_machine, ctx->owi_jmp_reset_instruction);

    ::pio_sm_set_enabled(instance, ctx->state_machine, true);

    return ctx->status;
}

void bind(OWI &owi, ConfigContext &cfg, BackendTable &backend) {
    HTRACE("owi.cpp -> bind(OWI&, ConfigContext&, BackendTable&):void");

    owi.ctx = static_cast<ConfigContext*>(&cfg);

    owi.init_fn = backend.init_fn;
    owi.deinit_fn = backend.deinit_fn;

    owi.set_baudrate_fn = backend.set_baudrate_fn;
    owi.get_baudrate_fn = backend.get_baudrate_fn;

    owi.set_index_fn = backend.set_index_fn;
    owi.get_index_fn = backend.get_index_fn;

    owi.write_blocking_fn = backend.write_blocking_fn;
    owi.read_blocking_fn  = backend.read_blocking_fn;

    owi.write_timeout_fn = backend.write_timeout_fn;
    owi.read_timeout_fn  = backend.read_timeout_fn;

    owi.transaction_fn = transaction_fn;
    owi.commit_fn = commit_fn;

    HINFO("[OWI    ] OWI instance bound to config context");
}

void bind(OWI &owi, ConfigContext &cfg) {
    HTRACE("owi.cpp -> bind(OWI&, ConfigContext&):void");
    bind(owi, cfg, hkk::rp2350::owi::backend);
}


static ::mutex_t owi0_mutex;
static LockState owi0_transaction {
    .mutex  = &owi0_mutex,
    .owner  = nullptr,
    .active = false,
};

static ConfigContext owi0_default_config {
    .transaction   = &owi0_transaction,
    .instance      = pio0,
    .pin           = DATA_PIN0,
    .state_machine = 0,
    .bit_mode      = 8,
    .index         = 0
};

static ::mutex_t owi1_mutex;
static LockState owi1_transaction {
    .mutex  = &owi1_mutex,
    .owner  = nullptr,
    .active = false,
};

static ConfigContext owi1_default_config {
    .transaction   = &owi1_transaction,
    .instance      = pio1,
    .pin           = DATA_PIN1,
    .state_machine = 1,
    .bit_mode      = 8,
    .index         = 1
};


OWI OWI0 {
    &owi0_default_config,
    hkk::rp2350::owi::init_fn,
    hkk::rp2350::owi::deinit_fn,
    hkk::rp2350::owi::set_baudrate_fn,
    hkk::rp2350::owi::get_baudrate_fn,
    hkk::rp2350::owi::set_index_fn,
    hkk::rp2350::owi::get_index_fn,
    hkk::rp2350::owi::write_blocking_fn,
    hkk::rp2350::owi::read_blocking_fn,
    hkk::rp2350::owi::write_timeout_fn,
    hkk::rp2350::owi::read_timeout_fn,
    hkk::bus::owi::transaction_fn,
    hkk::bus::owi::commit_fn
};

OWI OWI1 {
    &owi1_default_config,
    hkk::rp2350::owi::init_fn,
    hkk::rp2350::owi::deinit_fn,
    hkk::rp2350::owi::set_baudrate_fn,
    hkk::rp2350::owi::get_baudrate_fn,
    hkk::rp2350::owi::set_index_fn,
    hkk::rp2350::owi::get_index_fn,
    hkk::rp2350::owi::write_blocking_fn,
    hkk::rp2350::owi::read_blocking_fn,
    hkk::rp2350::owi::write_timeout_fn,
    hkk::rp2350::owi::read_timeout_fn,
    hkk::bus::owi::transaction_fn,
    hkk::bus::owi::commit_fn
};

OWI OWI2 {};    // Not implemented by default
OWI OWI3 {};    // Not implemented by default
OWI OWI4 {};    // Not implemented by default
OWI OWI5 {};    // Not implemented by default
OWI OWI6 {};    // Not implemented by default
OWI OWI7 {};    // Not implemented by default

}