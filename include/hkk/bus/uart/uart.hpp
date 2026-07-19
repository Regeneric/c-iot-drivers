#pragma once
#include <hkk/defines.h>

#include <hkk/logger/logger.h>

namespace hkk::bus::uart {   

class UART;
struct ConfigContext;
struct BackendTable;

// Implement only those present on your uC
// Keep the rest empty
extern UART UART0;
extern UART UART1;
extern UART UART2;
extern UART UART3;
extern UART UART4;
extern UART UART5;
extern UART UART6;
extern UART UART7;

void bind(UART &uart, ConfigContext &cfg);
void bind(UART &uart, ConfigContext &cfg, BackendTable &backend);
const char *rts(int8 status);


enum Result : int8 {
    UART_OK                       =  0,
    UART_ERROR_NULL_CONTEXT       = -1,
    UART_ERROR_NULL_INSTANCE      = -2,
    UART_ERROR_NULL_DATA          = -3,
    UART_ERROR_ZERO_LENGTH        = -4,
    UART_ERROR_NO_ACK             = -5,
    UART_ERROR_PARTIAL_WRITE      = -6,
    UART_ERROR_WRITE_FAILED       = -7,
    UART_ERROR_PARTIAL_READ       = -8,
    UART_ERROR_READ_FAILED        = -9,  
    UART_ERROR_NOT_SUPPORTED      = -10, 
    UART_ERROR_TIMEOUT            = -11,
    UART_ERROR_NULL_MUTEX         = -12,
    UART_ERROR_BUSY               = -13,
    UART_DMA_ERROR_GENERIC        = -14,
    UART_ERROR_NULL_DMA_CONTEXT   = -15,
    UART_ERROR_GENERIC            = -100,
    UART_FUNCTION_NOT_IMPLEMENTED = -101,
    UART_ERROR_UNKNOWN            = -102,
};


struct ConfigContext {    
    void *instance;
    void *dma;

    // 9600 bps, 8N1
    uint32 baudrate = 9600;
    uint8  data_bits   = 8;
    uint8  parity_bits = 0;
    uint8  stop_bits   = 1;

    uint8  tx;
    uint8  rx;

    bool8 rx_dma_enabled = false;
    bool8 tx_dma_enabled = false;

    int8 index = -1;

    int8 status = UART_OK;
};

struct BackendTable {
    int8  (*init_fn)(void *ctx, bool8 use_dma) = nullptr;
    int8  (*deinit_fn)(void *ctx) = nullptr;
    
    int8  (*set_baudrate_fn)(void *ctx, uint32 value) = nullptr;
    int32 (*get_baudrate_fn)(void *ctx) = nullptr;

    int8  (*set_index_fn)(void *ctx, int8 value) = nullptr;
    int32 (*get_index_fn)(void *ctx) = nullptr;

    int32 (*write_blocking_fn)(void *ctx, const uint8 *src, size_t len) = nullptr;
    int32 (*read_blocking_fn)(void *ctx, uint8 *dst, size_t len) = nullptr;

    int32 (*write_timeout_fn)(void *ctx, const uint8 *src, size_t len, uint32 timeout_us) = nullptr;
    int32 (*read_timeout_fn)(void *ctx, uint8 *dst, size_t len, uint32 timeout_us) = nullptr;

    int32 (*read_dma_start_fn)(void *ctx, uint8 *dst, size_t len) = nullptr;
};


class UART {
public:
    UART() = default;
    UART(
        ConfigContext *cfg,
        int8  (*init_fn)(void *ctx, bool8 use_dma),
        int8  (*deinit_fn)(void *ctx),
        int8  (*set_baudrate_fn)(void *ctx, uint32 value),
        int32 (*get_baudrate_fn)(void *ctx),
        int8  (*set_index_fn)(void *ctx, int8 value),
        int32 (*get_index_fn)(void *ctx),
        int32 (*write_blocking_fn)(void *ctx, const uint8 *src, size_t len),
        int32 (*read_blocking_fn)(void *ctx, uint8 *dst, size_t len),
        int32 (*read_dma_start_fn)(void *ctx, uint8 *dst, size_t len)
    ) : ctx(cfg),
        init_fn(init_fn),
        deinit_fn(deinit_fn),
        set_baudrate_fn(set_baudrate_fn),
        get_baudrate_fn(get_baudrate_fn),
        set_index_fn(set_index_fn),
        get_index_fn(get_index_fn),
        write_blocking_fn(write_blocking_fn),
        read_blocking_fn(read_blocking_fn),
        read_dma_start_fn(read_dma_start_fn)
    {}


    int8 init(bool8 use_dma = false) {
        return init_fn ? init_fn(ctx, use_dma) : UART_FUNCTION_NOT_IMPLEMENTED;
    }
    
    int8 deinit() {
        return deinit_fn ? deinit_fn(ctx) : UART_FUNCTION_NOT_IMPLEMENTED;
    }
    
    int8 baudrate(uint32 value) {
        return set_baudrate_fn ? set_baudrate_fn(ctx, value) : UART_FUNCTION_NOT_IMPLEMENTED;
    }
    int32 baudrate() {
        return get_baudrate_fn ? get_baudrate_fn(ctx) : UART_FUNCTION_NOT_IMPLEMENTED;
    }
    
    
    int8 index(int8 value) {
        return set_index_fn ? set_index_fn(ctx, value) : UART_FUNCTION_NOT_IMPLEMENTED;
    }
    int32 index() {
        return get_index_fn ? get_index_fn(ctx) : UART_FUNCTION_NOT_IMPLEMENTED;
    }


    int32 write(const uint8 *src, size_t len) {
        return write_blocking_fn ? write_blocking_fn(ctx, src, len) : UART_FUNCTION_NOT_IMPLEMENTED;
    }
    template <size_t N>
    int32 write(const uint8 (&src)[N]) {
        return write(src, N);
    }
    int32 write(uint8 byte) {
        return write(&byte, 1);
    }


    int32 read(uint8 *dst, size_t len) {
        return read_blocking_fn ? read_blocking_fn(ctx, dst, len) : UART_FUNCTION_NOT_IMPLEMENTED;
    }
    template <size_t N>
    int32 read(uint8 (&dst)[N]) {
        return read(dst, N);
    }
    int32 read(uint8 &byte) {
        return read(&byte, 1);
    }


    int32 write_timeout(const uint8 *src, size_t len, uint32 timeout_us) {
        return write_timeout_fn ? write_timeout_fn(ctx, src, len, timeout_us) : UART_FUNCTION_NOT_IMPLEMENTED;
    }
    template <size_t N>
    int32 write_timeout(uint8 (&src)[N], uint32 timeout_us) {
        return write_timeout(src, N, timeout_us);
    }
    int32 write_timeout(uint8 byte, uint32 timeout_us) {
        return write_timeout(&byte, 1, timeout_us);
    }


    int32 read_dma(uint8 *dst, size_t len) {
        return read_dma_start_fn ? read_dma_start_fn(ctx, dst, len) : UART_FUNCTION_NOT_IMPLEMENTED;
    }
    template <size_t N>
    int32 read_dma(uint8 (&dst)[N]) {
        return read_dma(dst, N);
    }
    int32 read_dma(uint8 &byte) {
        return read_dma(&byte, 1);
    }

private:
    void *ctx = nullptr;

    friend void bind(UART &uart, ConfigContext &cfg, BackendTable &backend);

    int8 (*init_fn)(void *ctx, bool8 use_dma) = nullptr;
    int8 (*deinit_fn)(void *ctx) = nullptr;
    
    int8  (*set_baudrate_fn)(void *ctx, uint32 value) = nullptr;
    int32 (*get_baudrate_fn)(void *ctx) = nullptr;

    int8  (*set_index_fn)(void *ctx, int8 value) = nullptr;
    int32 (*get_index_fn)(void *ctx) = nullptr;

    int32 (*write_blocking_fn)(void *ctx, const uint8 *src, size_t len) = nullptr;
    int32 (*read_blocking_fn)(void *ctx, uint8 *dst, size_t len) = nullptr;

    int32 (*write_timeout_fn)(void *ctx, const uint8 *src, size_t len, uint32 timeout_us) = nullptr;
    int32 (*read_timeout_fn)(void *ctx, uint8 *dst, size_t len, uint32 timeout_us) = nullptr;

    int32 (*read_dma_start_fn)(void *ctx, uint8 *dst, size_t len) = nullptr;
};

}  