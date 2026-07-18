#pragma once
#include <hkk/defines.h>
#include <hkk/logger/logger.h>

namespace hkk::bus::spi {

class SPI;
struct ConfigContext;
struct LockState;
struct BackendTable;

// Implement only those present on your uC
// Keep the rest empty
extern SPI SPI0;
extern SPI SPI1;
extern SPI SPI2;
extern SPI SPI3;
extern SPI SPI4;
extern SPI SPI5;
extern SPI SPI6;
extern SPI SPI7;

void bind(SPI &spi, ConfigContext &cfg);
void bind(SPI &spi, ConfigContext &cfg, BackendTable &backend);
const char *rts(int8 status);


enum Result : int8 {
    SPI_OK                       =  0,
    SPI_ERROR_NULL_CONTEXT       = -1,
    SPI_ERROR_NULL_INSTANCE      = -2,
    SPI_ERROR_NULL_DATA          = -3,
    SPI_ERROR_ZERO_LENGTH        = -4,
    SPI_ERROR_NO_ACK             = -5,
    SPI_ERROR_PARTIAL_WRITE      = -6,
    SPI_ERROR_WRITE_FAILED       = -7,
    SPI_ERROR_PARTIAL_READ       = -8,
    SPI_ERROR_READ_FAILED        = -9,  
    SPI_ERROR_NOT_SUPPORTED      = -10, 
    SPI_ERROR_TIMEOUT            = -11,
    SPI_ERROR_NULL_MUTEX         = -12,
    SPI_ERROR_BUSY               = -13,
    SPI_ERROR_GENERIC            = -100,
    SPI_FUNCTION_NOT_IMPLEMENTED = -101,
    SPI_ERROR_UNKNOWN            = -102,
};


class TransactionGuard {
private:
    void *ctx;
    void *owner;

    int8 (*commit_fn)(void *ctx, void *owner) = nullptr;

public:
    int8 status;

    // No copy allowed
    TransactionGuard(const TransactionGuard&) = delete;
    TransactionGuard& operator=(const TransactionGuard&) = delete;

    TransactionGuard(
        void *ctx,
        void *owner,
        int8 (*commit_fn)(void *ctx, void *owner),
        int8 status
    ) : ctx(ctx),
        owner(owner),
        commit_fn(commit_fn),
        status(status)
    {}

    ~TransactionGuard() {
        if(status == SPI_OK && commit_fn != nullptr && ctx != nullptr) {
            commit_fn(ctx, owner);
        } else {
            HFATAL("[I2C    ] Transaction function not implemented");
        }
    }
};

struct LockState {
    void *mutex = nullptr;
    void *owner = nullptr;
    bool8 active = false;
};

struct ConfigContext {
    LockState *transaction;
    void      *instance;

    uint8 miso;
    uint8 mosi;
    uint8 sck;
    uint8 cs;
    uint8 reset;

    int8 index = -1;

    int8 status = SPI_OK;
};

struct BackendTable {
    int8 (*init_fn)(void *ctx) = nullptr;
    int8 (*deinit_fn)(void *ctx) = nullptr;
    
    int8  (*set_baudrate_fn)(void *ctx, uint32 value) = nullptr;
    int32 (*get_baudrate_fn)(void *ctx) = nullptr;

    int8  (*set_index_fn)(void *ctx, int8 value) = nullptr;
    int32 (*get_index_fn)(void *ctx) = nullptr;

    int32 (*write_blocking_fn)(void *ctx, const uint8 *src, size_t len) = nullptr;
    int32 (*read_blocking_fn)(void *ctx, uint8 *dst, size_t len) = nullptr;

    int32 (*write_timeout_fn)(void *ctx, const uint8 *src, size_t len, uint32 timeout_us) = nullptr;
    int32 (*read_timeout_fn)(void *ctx, uint8 *dst, size_t len, uint32 timeout_us) = nullptr;
};


class SPI {
public:
    SPI() = default;
    SPI(
        ConfigContext *cfg,
        int8  (*init_fn)(void *ctx),
        int8  (*deinit_fn)(void *ctx),
        int8  (*set_baudrate_fn)(void *ctx, uint32 value),
        int32 (*get_baudrate_fn)(void *ctx),
        int8  (*set_index_fn)(void *ctx, int8 value),
        int32 (*get_index_fn)(void *ctx),
        int32 (*write_blocking_fn)(void *ctx, const uint8 *src, size_t len),
        int32 (*read_blocking_fn)(void *ctx, uint8 *dst, size_t len),
        int8  (*transaction_fn)(void *ctx, void *owner),
        int8  (*commit_fn)(void *ctx, void *owner)
    ) : ctx(cfg),
        init_fn(init_fn),
        deinit_fn(deinit_fn),
        set_baudrate_fn(set_baudrate_fn),
        get_baudrate_fn(get_baudrate_fn),
        set_index_fn(set_index_fn),
        get_index_fn(get_index_fn),
        write_blocking_fn(write_blocking_fn),
        read_blocking_fn(read_blocking_fn),
        transaction_fn(transaction_fn),
        commit_fn(commit_fn)
    {}


    int8 init() {
        return init_fn ? init_fn(ctx) : SPI_FUNCTION_NOT_IMPLEMENTED;
    }
    
    int8 deinit() {
        return deinit_fn ? deinit_fn(ctx) : SPI_FUNCTION_NOT_IMPLEMENTED;
    }
    
    int8 baudrate(uint32 value) {
        return set_baudrate_fn ? set_baudrate_fn(ctx, value) : SPI_FUNCTION_NOT_IMPLEMENTED;
    }
    int32 baudrate() {
        return get_baudrate_fn ? get_baudrate_fn(ctx) : SPI_FUNCTION_NOT_IMPLEMENTED;
    }
    
    
    int8 index(int8 value) {
        return set_index_fn ? set_index_fn(ctx, value) : SPI_FUNCTION_NOT_IMPLEMENTED;
    }
    int32 index() {
        return get_index_fn ? get_index_fn(ctx) : SPI_FUNCTION_NOT_IMPLEMENTED;
    }


    int32 write(uint8 addr, const uint8 *src, size_t len, bool8 nostop = false) {
        HTRACE("[SPI    ] SPI does not use bus addressing; ignoring address 0x%02X", addr);
        return write_blocking_fn ? write_blocking_fn(ctx, src, len) : SPI_FUNCTION_NOT_IMPLEMENTED;
    }
    int32 write(const uint8 *src, size_t len, bool8 nostop = false) {
        return write(src, len, nostop);
    }
    int32 write(uint8 addr, uint8 byte, bool8 nostop = false) {
        return write(addr, &byte, 1, nostop);
    }
    int32 write(uint8 byte, bool8 nostop = false) {
        return write(&byte, 1, nostop);
    }
    template<size_t N>
    int32 write(uint8 addr, const uint8 (&src)[N], bool8 nostop = false){
        return write(addr, src, N, nostop);
    }
    template<size_t N>
    int32 write(const uint8 (&src)[N], bool8 nostop = false) {
        return write(src, N, nostop);
    }


    int32 read(uint8 addr, uint8 *dst, size_t len, bool8 nostop = false) {
        HTRACE("[SPI    ] SPI does not use bus addressing; ignoring address 0x%02X", addr);
        return read_blocking_fn ? read_blocking_fn(ctx, dst, len) : SPI_FUNCTION_NOT_IMPLEMENTED;
    }
    int32 read(uint8 *dst, size_t len, bool8 nostop = false) {
        return read(dst, len, nostop);
    }
    int32 read(uint8 addr, uint8 &byte, bool8 nostop = false) {
        return read(addr, &byte, 1, nostop);
    }
    int32 read(uint8 &byte, bool8 nostop = false) {
        return read(&byte, 1, nostop);
    }
    template<size_t N>
    int32 read(uint8 addr, uint8 (&dst)[N], bool8 nostop = false){
        return read(addr, dst, N, nostop);
    }
    template<size_t N>
    int32 read(uint8 (&dst)[N], bool8 nostop = false) {
        return read(dst, N, nostop);
    }


    int32 write_timeout(uint8 addr, const uint8 *src, size_t len, uint32 timeout_us, bool8 nostop = false) {
        HTRACE("[SPI    ] SPI does not use bus addressing; ignoring address 0x%02X", addr);
        return write_timeout_fn ? write_timeout_fn(ctx, src, len, timeout_us) : SPI_FUNCTION_NOT_IMPLEMENTED;
    }
    int32 write_timeout(const uint8 *src, size_t len, uint32 timeout_us, bool8 nostop = false) {
        return write_timeout(src, len, timeout_us, nostop);
    }
    int32 write_timeout(uint8 addr, uint8 byte, uint32 timeout_us, bool8 nostop = false) {
        return write_timeout(addr, &byte, 1, timeout_us, nostop);
    }
    int32 write_timeout(uint8 byte, uint32 timeout_us, bool8 nostop = false) {
        return write_timeout(&byte, 1, timeout_us, nostop);
    }
    template<size_t N>
    int32 write_timeout(uint8 addr, const uint8 (&src)[N], uint32 timeout_us, bool8 nostop = false){
        return write_timeout(addr, src, N, timeout_us, nostop);
    }
    template<size_t N>
    int32 write_timeout(const uint8 (&src)[N], uint32 timeout_us, bool8 nostop = false) {
        return write_timeout(src, N, timeout_us, nostop);
    }


    TransactionGuard transaction(void *owner = nullptr) {
        if(transaction_fn) {
            int8 status = transaction_fn(ctx, owner);
            return TransactionGuard(ctx, owner, commit_fn, status);
        } else {
            return TransactionGuard(ctx, owner, commit_fn, SPI_FUNCTION_NOT_IMPLEMENTED);
        }
    }
    
    int8 commit(void *owner = nullptr) {return commit_fn ? commit_fn(ctx, owner) : SPI_FUNCTION_NOT_IMPLEMENTED;};

private:
    void *ctx = nullptr;

    friend void bind(SPI &spi, ConfigContext &cfg, BackendTable &backend);

    int8  (*init_fn)(void *ctx) = nullptr;
    int8  (*deinit_fn)(void *ctx) = nullptr;
    
    int8  (*set_baudrate_fn)(void *ctx, uint32 value) = nullptr;
    int32 (*get_baudrate_fn)(void *ctx) = nullptr;

    int8  (*set_index_fn)(void *ctx, int8 value) = nullptr;
    int32 (*get_index_fn)(void *ctx) = nullptr;

    int32 (*write_blocking_fn)(void *ctx, const uint8 *src, size_t len) = nullptr;
    int32 (*read_blocking_fn)(void *ctx, uint8 *dst, size_t len) = nullptr;

    int32 (*write_timeout_fn)(void *ctx, const uint8 *src, size_t len, uint32 timeout_us) = nullptr;
    int32 (*read_timeout_fn)(void *ctx, uint8 *dst, size_t len, uint32 timeout_us) = nullptr;

    int8  (*transaction_fn)(void *ctx, void *owner) = nullptr;
    int8  (*commit_fn)(void *ctx, void *owner) = nullptr;
};

}