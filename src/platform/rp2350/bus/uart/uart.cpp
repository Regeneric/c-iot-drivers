#include <hkk/defines.h>

#include <hkk/utils/utils.hpp>
#include <hkk/logger/logger.h>
#include <hkk/bus/uart/uart.hpp>

#include <pico/mutex.h>
#include <hardware/uart.h>
#include <hardware/dma.h>
#include <hardware/irq.h>
#include <hardware/gpio.h>

namespace hkk::rp2350::uart {

extern int8  init_dma_fn(void *ctx_raw);
extern int32 read_dma_fn(void *ctx_raw, uint8 *dst, size_t len);


static int8 init_fn(void *ctx_raw, bool8 use_dma) {
    HTRACE("uart.cpp -> s:init_fn(void*):int8");

    if(!ctx_raw) {
        HERROR("[UART   ] Null context passed to function");
        return hkk::bus::uart::UART_ERROR_NULL_CONTEXT;
    }
    auto *ctx = static_cast<hkk::bus::uart::ConfigContext*>(ctx_raw);

    if(!ctx->instance) {
        HERROR("[UART   ] Null UART instance in context");
        return ctx->status = hkk::bus::uart::UART_ERROR_NULL_INSTANCE;
    }
    ::uart_inst *instance = static_cast<::uart_inst*>(ctx->instance);

    uint32 baudrate = ::uart_init(instance, ctx->baudrate);
    if(baudrate != ctx->baudrate) {
        HWARN("[UART   ] Actual baud rate differs from desired baud rate");
        HWARN("[UART   ] Desired: %d bps", ctx->baudrate);
        HWARN("[UART   ] Actual : %d bps", baudrate);

        ctx->baudrate = baudrate;
    }

    ::uart_set_format(instance, ctx->data_bits, ctx->stop_bits, static_cast<::uart_parity_t>(ctx->parity_bits));
    ::uart_set_fifo_enabled(instance, true);

    ::gpio_init(ctx->tx);
    ::gpio_init(ctx->rx);

    ::gpio_set_function(ctx->tx, GPIO_FUNC_UART);
    ::gpio_set_function(ctx->rx, GPIO_FUNC_UART);

    ctx->index = static_cast<int8>(::uart_get_index(instance));

    if(ctx->index != 0 && ctx->index != 1) {
        HERROR("[UART   ] Invalid UART instance index: %d", ctx->index);
        return ctx->status = hkk::bus::uart::UART_ERROR_GENERIC;
    }

    HDEBUG("[UART   ] TX pin: %d", ctx->tx);
    HDEBUG("[UART   ] RX pin: %d", ctx->rx);
    HDEBUG("[UART   ] Baud rate set to %d bps", ctx->baudrate);
    HDEBUG("[UART   ] Data bits: %d ; Parity bits: %d ; Stop bits: %d", ctx->data_bits, ctx->parity_bits, ctx->stop_bits);

    if(use_dma == true) {
        if(init_dma_fn(ctx) < hkk::bus::uart::UART_OK) {
            HERROR("[UART   ] DMA initialization fail");
        }
    }

    HINFO("[UART   ] UART%d initialized", ctx->index);
    return ctx->status = hkk::bus::uart::UART_OK;
}

static int8 deinit_fn(void *ctx_raw) {
    HTRACE("uart.cpp -> s:deinit_fn(void*):int8");

    if(!ctx_raw) {
        HERROR("[UART   ] Null context passed to function");
        return hkk::bus::uart::UART_ERROR_NULL_CONTEXT;
    }
    auto *ctx = static_cast<hkk::bus::uart::ConfigContext*>(ctx_raw);

    if(!ctx->instance) {
        HERROR("[UART   ] Null UART instance in context");
        return ctx->status = hkk::bus::uart::UART_ERROR_NULL_INSTANCE;
    }
    ::uart_inst *instance = static_cast<::uart_inst*>(ctx->instance);

    ::uart_deinit(instance);

    ::gpio_deinit(ctx->tx);
    ::gpio_deinit(ctx->rx);

    ::gpio_set_pulls(ctx->tx, false, false);
    ::gpio_set_pulls(ctx->rx, false, false);

    HINFO("[UART   ] UART%d deinitialized", ctx->index);

    ctx->index  = -1;
    HDEBUG("[UART   ] Instance index set to %d", ctx->index);

    return ctx->status = hkk::bus::uart::UART_OK;
}

static int8 set_baudrate_fn(void *ctx_raw, uint32 value) {
    HTRACE("uart.cpp -> s:set_baudrate_fn(void*, uint32):int8");

    if(!ctx_raw) {
        HERROR("[UART   ] Null context passed to function");
        return hkk::bus::uart::UART_ERROR_NULL_CONTEXT;
    }
    auto *ctx = static_cast<hkk::bus::uart::ConfigContext*>(ctx_raw);

    if(!ctx->instance) {
        HERROR("[UART   ] Null UART instance in context");
        return ctx->status = hkk::bus::uart::UART_ERROR_NULL_INSTANCE;
    }
    ::uart_inst *instance = static_cast<::uart_inst*>(ctx->instance);

    if(value == 0) {
        HWARN("[UART   ] Baud rate cannot be set to 0");
        HWARN("[UART   ] Baud rate set to %d bps", ctx->baudrate);

        value = ctx->baudrate;
    }

    uint32 baudrate = ::uart_set_baudrate(instance, value);

    if(baudrate == 0) {
        HERROR("[UART   ] Could not set baud rate to %d", value);
        return ctx->status = hkk::bus::uart::UART_ERROR_GENERIC;
    }

    if(baudrate != value) {
        HWARN("[UART   ] Actual baud rate differs from desired baud rate");
        HWARN("[UART   ] Desired: %d bps", ctx->baudrate);
        HWARN("[UART   ] Actual : %d bps", baudrate);
    }

    ctx->baudrate = baudrate;
    HINFO("[UART   ] Baud rate set to %d bps", ctx->baudrate);

    return ctx->status = hkk::bus::uart::UART_OK;
}

static int32 get_baudrate_fn(void *ctx_raw) {
    HTRACE("uart.cpp -> s:set_baudrate_fn(void*):int8");

    if(!ctx_raw) {
        HERROR("[UART   ] Null context passed to function");
        return hkk::bus::uart::UART_ERROR_NULL_CONTEXT;
    }
    auto *ctx = static_cast<hkk::bus::uart::ConfigContext*>(ctx_raw);

    if(!ctx->instance) {
        HERROR("[UART   ] Null UART instance in context");
        return ctx->status = hkk::bus::uart::UART_ERROR_NULL_INSTANCE;
    }
    ::uart_inst *instance = static_cast<::uart_inst*>(ctx->instance);

    HDEBUG("[UART   ] Current baud rate: %d bps", ctx->baudrate);

    ctx->status = hkk::bus::uart::UART_OK;
    return static_cast<int32>(ctx->baudrate);
} 

static int8 set_index_fn(void *ctx_raw, int8 value) {
    HTRACE("uart.cpp -> s:set_index_fn(void*, int8):int8");
    
    HFATAL("[UART   ] Function not implemented on RP2350");
    return hkk::bus::uart::UART_ERROR_NOT_SUPPORTED;
}

static int32 get_index_fn(void *ctx_raw) {
    HTRACE("uart.cpp -> s:get_index_fn(void*):int32");

    if(!ctx_raw) {
        HERROR("[UART   ] Null context passed to function");
        return static_cast<int32>(hkk::bus::uart::UART_ERROR_NULL_CONTEXT);
    }
    auto *ctx = static_cast<hkk::bus::uart::ConfigContext*>(ctx_raw);

    if(!ctx->instance) {
        HERROR("[UART   ] Null UART instance in context");

        ctx->status = hkk::bus::uart::UART_ERROR_NULL_INSTANCE;
        return static_cast<int32>(ctx->status);
    } 
    ::uart_inst *instance = static_cast<::uart_inst*>(ctx->instance);

    uint8 index = ::uart_get_index(instance);
    if(index != 0 && index != 1) {
        HERROR("[UART   ] Invalid UART instance index: %d", index);
        
        ctx->status = hkk::bus::uart::UART_ERROR_GENERIC;
        return static_cast<int32>(ctx->status);
    }

    HTRACE("[UART   ] Current UART index: UART%d", index);
    ctx->index = static_cast<int8>(index);
    
    ctx->status = hkk::bus::uart::UART_OK;
    return static_cast<int32>(ctx->index);
}

static int32 write_blocking_fn(void *ctx_raw, const uint8 *src, size_t len) {
    HTRACE("uart.cpp -> s:write_blocking_fn(void*, const uint8*, size_t):int32");

    if(!ctx_raw) {
        HERROR("[UART   ] Null context passed to function");
        return static_cast<int32>(hkk::bus::uart::UART_ERROR_NULL_CONTEXT);
    }
    auto *ctx = static_cast<hkk::bus::uart::ConfigContext*>(ctx_raw);

    if(!ctx->instance) {
        HERROR("[UART   ] Null UARTinstance in context");

        ctx->status = hkk::bus::uart::UART_ERROR_NULL_INSTANCE;
        return static_cast<int32>(ctx->status);
    } 
    ::uart_inst *instance = static_cast<::uart_inst*>(ctx->instance);

    if(!src) {
        HERROR("[UART   ] Null data pointer passed to function");
        
        ctx->status = hkk::bus::uart::UART_ERROR_NULL_DATA;
        return static_cast<int32>(ctx->status);
    }

    if(len == 0) {
        HERROR("[UART   ] Data length is 0");

        ctx->status = hkk::bus::uart::UART_ERROR_ZERO_LENGTH;
        return static_cast<int32>(ctx->status);
    }

    ::uart_write_blocking(instance, src, len);

    HTRACE("[UART   ] First byte   : 0x%02X", src[0]);
    HTRACE("[UART   ] Data length  : %zu", len);

    HTRACE("[UART   ] Write complete");

    ctx->status = hkk::bus::uart::UART_OK;
    return static_cast<int32>(ctx->status);
}

static int32 read_fn(void *ctx_raw, uint8 *dst, size_t len) {
    HTRACE("uart.cpp -> s:read_fn(void*, uint8*, size_t):int32");

    if(!ctx_raw) {
        HERROR("[UART   ] Null context passed to function");
        return static_cast<int32>(hkk::bus::uart::UART_ERROR_NULL_CONTEXT);
    }
    auto *ctx = static_cast<hkk::bus::uart::ConfigContext*>(ctx_raw);

    if(!ctx->instance) {
        HERROR("[UART   ] Null UARTinstance in context");

        ctx->status = hkk::bus::uart::UART_ERROR_NULL_INSTANCE;
        return static_cast<int32>(ctx->status);
    } 
    ::uart_inst *instance = static_cast<::uart_inst*>(ctx->instance);

    if(!ctx->dma) {
        HERROR("[UART   ] Null DMA context passed to function");
        
        ctx->status = hkk::bus::uart::UART_ERROR_NULL_DMA_CONTEXT;
        return static_cast<int32>(ctx->status);
    }
    auto *dma = static_cast<hkk::bus::uart::DMAContext*>(ctx->dma);

    if(!dst) {
        HERROR("[UART   ] Null data pointer passed to function");

        ctx->status = hkk::bus::uart::UART_ERROR_NULL_DATA;
        return static_cast<int32>(ctx->status);
    }

    if(len == 0) {
        HERROR("[UART   ] Data length is 0");

        ctx->status = hkk::bus::uart::UART_ERROR_ZERO_LENGTH;
        return static_cast<int32>(ctx->status);
    }


    if(ctx->dma_enable == true) {
        dma->rx_active = true;
        ::dma_channel_set_write_addr(dma->rx_channel, dst, false);
        ::dma_channel_set_transfer_count(dma->rx_channel, ::dma_encode_transfer_count(len), true);
    } else {
        ::uart_read_blocking(instance, dst, len);
    }

    
    HTRACE("[UART   ] First byte   : 0x%02X", dst[0]);
    HTRACE("[UART   ] Data length  : %zu", len);
    HTRACE("[UART   ] DMA channnel : %d", dma->rx_channel);

    HTRACE("[UART   ] Read complete");

    ctx->status = hkk::bus::uart::UART_OK;
    return static_cast<int32>(ctx->status);
}

static int32 write_timeout_fn(void *ctx_raw, const uint8 *src, size_t len, uint32 timeout_us) {
    HTRACE("uart.cpp -> s:write_blocking_fn(void*, const uint8*, size_t, uint32):int32");

    if(!ctx_raw) {
        HERROR("[UART   ] Null context passed to function");
        return static_cast<int32>(hkk::bus::uart::UART_ERROR_NULL_CONTEXT);
    }
    auto *ctx = static_cast<hkk::bus::uart::ConfigContext*>(ctx_raw);

    if(!ctx->instance) {
        HERROR("[UART   ] Null UARTinstance in context");

        ctx->status = hkk::bus::uart::UART_ERROR_NULL_INSTANCE;
        return static_cast<int32>(ctx->status);
    } 
    ::uart_inst *instance = static_cast<::uart_inst*>(ctx->instance);

    if(!src) {
        HERROR("[UART   ] Null data pointer passed to function");
        
        ctx->status = hkk::bus::uart::UART_ERROR_NULL_DATA;
        return static_cast<int32>(ctx->status);
    }

    if(len == 0) {
        HERROR("[UART   ] Data length is 0");

        ctx->status = hkk::bus::uart::UART_ERROR_ZERO_LENGTH;
        return static_cast<int32>(ctx->status);
    }

    // TODO: timeout
    ::uart_write_blocking(instance, src, len);

    HTRACE("[UART   ] First byte   : 0x%02X", src[0]);
    HTRACE("[UART   ] Data length  : %zu", len);

    HTRACE("[UART   ] Write complete");

    ctx->status = hkk::bus::uart::UART_OK;
    return static_cast<int32>(ctx->status);
}

static int32 read_timeout_fn(void *ctx_raw, uint8 *dst, size_t len, uint32 timeout_us) {
    HTRACE("uart.cpp -> s:read_timeout_fn(void*, uint8*, size_t, uint32):int32");

    if(!ctx_raw) {
        HERROR("[UART   ] Null context passed to function");
        return static_cast<int32>(hkk::bus::uart::UART_ERROR_NULL_CONTEXT);
    }
    auto *ctx = static_cast<hkk::bus::uart::ConfigContext*>(ctx_raw);

    if(!ctx->instance) {
        HERROR("[UART   ] Null UARTinstance in context");

        ctx->status = hkk::bus::uart::UART_ERROR_NULL_INSTANCE;
        return static_cast<int32>(ctx->status);
    } 
    ::uart_inst *instance = static_cast<::uart_inst*>(ctx->instance);

    if(!dst) {
        HERROR("[UART   ] Null data pointer passed to function");

        ctx->status = hkk::bus::uart::UART_ERROR_NULL_DATA;
        return static_cast<int32>(ctx->status);
    }

    if(len == 0) {
        HERROR("[UART   ] Data length is 0");

        ctx->status = hkk::bus::uart::UART_ERROR_ZERO_LENGTH;
        return static_cast<int32>(ctx->status);
    }


    if(::uart_is_readable_within_us(instance, timeout_us)) return read_fn(ctx_raw, dst, len);
    else return static_cast<int32>(hkk::bus::uart::UART_ERROR_TIMEOUT);
}

static hkk::bus::uart::BackendTable backend {
    .init_dma_fn = init_dma_fn,

    .init_fn = init_fn,
    .deinit_fn = deinit_fn,

    .set_baudrate_fn = set_baudrate_fn,
    .get_baudrate_fn = get_baudrate_fn,

    .set_index_fn = set_index_fn,
    .get_index_fn = get_index_fn,

    .write_blocking_fn = write_blocking_fn,
    .read_fn = read_fn,

    .write_timeout_fn = write_timeout_fn,
    .read_timeout_fn = read_timeout_fn
};

}


namespace hkk::bus::uart {

inline constexpr uint8 PIN_TX0 = 0;
inline constexpr uint8 PIN_RX0 = 1;

inline constexpr uint8 PIN_TX1 = 8;
inline constexpr uint8 PIN_RX1 = 9;

inline constexpr uint8 data_bits   = 8;
inline constexpr uint8 parity_bits = 0;
inline constexpr uint8 stop_bits   = 1;


const char *rts(int8 status) {
    switch(status) {
        case UART_OK:                        return "UART_OK";    
        case UART_ERROR_NULL_CONTEXT:        return "UART_ERROR_NULL_CONTEXT";     
        case UART_ERROR_NULL_INSTANCE:       return "UART_ERROR_NULL_INSTANCE";    
        case UART_ERROR_NULL_DATA:           return "UART_ERROR_NULL_DATA";        
        case UART_ERROR_ZERO_LENGTH:         return "UART_ERROR_ZERO_LENGTH";      
        case UART_ERROR_NO_ACK:              return "UART_ERROR_NO_ACK";           
        case UART_ERROR_PARTIAL_WRITE:       return "UART_ERROR_PARTIAL_WRITE";    
        case UART_ERROR_WRITE_FAILED:        return "UART_ERROR_WRITE_FAILED";     
        case UART_ERROR_PARTIAL_READ:        return "UART_ERROR_PARTIAL_READ";     
        case UART_ERROR_READ_FAILED:         return "UART_ERROR_READ_FAILED";        
        case UART_ERROR_NOT_SUPPORTED:       return "UART_ERROR_NOT_SUPPORTED";      
        case UART_ERROR_TIMEOUT:             return "UART_ERROR_TIMEOUT";           
        case UART_ERROR_NULL_MUTEX:          return "UART_ERROR_NULL_MUTEX";        
        case UART_ERROR_BUSY:                return "UART_ERROR_BUSY";              
        case UART_ERROR_GENERIC:             return "UART_ERROR_GENERIC";            
        case UART_FUNCTION_NOT_IMPLEMENTED:  return "UART_FUNCTION_NOT_IMPLEMENTED"; 
        default:                             return "UART_ERROR_UNKNOWN";
    }
}

void bind(UART &uart, ConfigContext &cfg, BackendTable &backend) {
    HTRACE("uart.cpp -> bind(UART&, ConfigContext&, BackendTable&):void");

    uart.ctx = static_cast<ConfigContext*>(&cfg);

    uart.init_dma_fn = backend.init_dma_fn,

    uart.init_fn = backend.init_fn;
    uart.deinit_fn = backend.deinit_fn;

    uart.set_baudrate_fn = backend.set_baudrate_fn;
    uart.get_baudrate_fn = backend.get_baudrate_fn;

    uart.set_index_fn = backend.set_index_fn;
    uart.get_index_fn = backend.get_index_fn;

    uart.write_blocking_fn = backend.write_blocking_fn;
    uart.read_fn  = backend.read_fn;

    uart.write_timeout_fn = backend.write_timeout_fn;
    uart.read_timeout_fn  = backend.read_timeout_fn;

    HINFO("[UART    ] UART instance bound to config context");
}

void bind(UART &uart, ConfigContext &cfg) {
    HTRACE("uart.cpp -> bind(UART&, ConfigContext&):void");
    bind(uart, cfg, hkk::rp2350::uart::backend);
}

static DMAContext uart0_dma_context {
    .irq = IRQ_0
};
static ConfigContext uart0_default_config {
    .instance    = uart0,
    .dma         = &uart0_dma_context,
    .baudrate    = 9600,
    .data_bits   = data_bits,   // 8
    .parity_bits = parity_bits, // N
    .stop_bits   = stop_bits,   // 1
    .tx          = PIN_TX0,
    .rx          = PIN_RX0,
    .index       = 0
};

static DMAContext uart1_dma_context {
    .irq = IRQ_1
};
static ConfigContext uart1_default_config {
    .instance    = uart1,
    .dma         = &uart1_dma_context,
    .baudrate    = 9600,
    .data_bits   = data_bits,   // 8
    .parity_bits = parity_bits, // N
    .stop_bits   = stop_bits,   // 1
    .tx          = PIN_TX1,
    .rx          = PIN_RX1,
    .index       = 1
};

UART UART0 {
    &uart0_default_config,
    hkk::rp2350::uart::init_dma_fn,
    hkk::rp2350::uart::init_fn,
    hkk::rp2350::uart::deinit_fn,
    hkk::rp2350::uart::set_baudrate_fn,
    hkk::rp2350::uart::get_baudrate_fn,
    hkk::rp2350::uart::set_index_fn,
    hkk::rp2350::uart::get_index_fn,
    hkk::rp2350::uart::write_blocking_fn,
    hkk::rp2350::uart::read_fn
};

UART UART1 {
    &uart1_default_config,
    hkk::rp2350::uart::init_dma_fn,
    hkk::rp2350::uart::init_fn,
    hkk::rp2350::uart::deinit_fn,
    hkk::rp2350::uart::set_baudrate_fn,
    hkk::rp2350::uart::get_baudrate_fn,
    hkk::rp2350::uart::set_index_fn,
    hkk::rp2350::uart::get_index_fn,
    hkk::rp2350::uart::write_blocking_fn,
    hkk::rp2350::uart::read_fn
};

UART UART2 {};    // Not present on RP2350
UART UART3 {};    // Not present on RP2350
UART UART4 {};    // Not present on RP2350
UART UART5 {};    // Not present on RP2350
UART UART6 {};    // Not present on RP2350
UART UART7 {};    // Not present on RP2350

}