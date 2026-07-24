#pragma once
#include <hkk/defines.h>

#include <hkk/utils/utils.hpp>
#include <hkk/logger/logger.h>

namespace hkk::bus::owi {   

class OWI;
struct ConfigContext;
struct LockState;
struct BackendTable;

// Implement only those present on your uC
// Keep the rest empty
extern OWI OWI0;
extern OWI OWI1;
extern OWI OWI2;
extern OWI OWI3;
extern OWI OWI4;
extern OWI OWI5;
extern OWI OWI6;
extern OWI OWI7;

void bind(OWI &owi, ConfigContext &cfg);
void bind(OWI &owi, ConfigContext &cfg, BackendTable &backend);
const char *rts(int8 status);


enum Result : int8 {
    OWI_OK                       =  0,
    OWI_ERROR_NULL_CONTEXT       = -1,
    OWI_ERROR_NULL_INSTANCE      = -2,
    OWI_ERROR_NULL_DATA          = -3,
    OWI_ERROR_ZERO_LENGTH        = -4,
    OWI_ERROR_NO_ACK             = -5,
    OWI_ERROR_PARTIAL_WRITE      = -6,
    OWI_ERROR_WRITE_FAILED       = -7,
    OWI_ERROR_PARTIAL_READ       = -8,
    OWI_ERROR_READ_FAILED        = -9,  
    OWI_ERROR_NOT_SUPPORTED      = -10, 
    OWI_ERROR_TIMEOUT            = -11,
    OWI_ERROR_NULL_MUTEX         = -12,
    OWI_ERROR_BUSY               = -13,
    OWI_KEEP_ALIVE               = OWI_OK,
    OWI_ERROR_GENERIC            = -100,
    OWI_FUNCTION_NOT_IMPLEMENTED = -101,
    OWI_ERROR_UNKNOWN            = -102,
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
        void  *ctx,
        void  *owner,
        int8  (*commit_fn)(void *ctx, void *owner),
        int8  status
    ) : ctx(ctx),
        owner(owner),
        commit_fn(commit_fn),
        status(status)
    {}

    ~TransactionGuard() {
        if(status == OWI_OK && commit_fn != nullptr && ctx != nullptr) {
            commit_fn(ctx, owner);
        } else {
            HFATAL("[OWI    ] Transaction function not implemented");
        }
    }
};

struct LockState {
    void *mutex = nullptr;
    void *owner = nullptr;
    bool8 active = false;
    bool8 nostop = false;
    volatile bool8 timeout;
};

struct ConfigContext {
    LockState *transaction;
    
    void *instance;
    
    uint8 pin = -1; 
    uint8 state_machine = -1;
    uint8 bit_mode = 8;

    uint8  owi_state_machine;
    uint32 owi_state_machine_offset;
    uint32 owi_jmp_reset_instruction;

    int8  index = -1;
    int8 status = OWI_OK;
};

struct BackendTable {
    int8  (*init_fn)(void *ctx) = nullptr;
    int8  (*deinit_fn)(void *ctx) = nullptr;
    
    int8  (*set_baudrate_fn)(void *ctx, uint32 value) = nullptr;
    int8  (*get_baudrate_fn)(void *ctx) = nullptr;

    int8  (*set_index_fn)(void *ctx, int8 value) = nullptr;
    int8  (*get_index_fn)(void *ctx) = nullptr;

    int8  (*write_blocking_fn)(void *ctx, const uint8 *src, size_t len) = nullptr;
    int8  (*read_blocking_fn)(void *ctx, uint8 *dst, size_t len) = nullptr;

    int8  (*write_timeout_fn)(void *ctx, const uint8 *src, size_t len, uint32 timeout_us) = nullptr;
    int8  (*read_timeout_fn)(void *ctx, uint8 *dst, size_t len, uint32 timeout_us) = nullptr;
};


class OWI {
public:
    OWI() = default;
    OWI(
        ConfigContext *cfg,
        int8  (*init_fn)(void *ctx),
        int8  (*deinit_fn)(void *ctx),
        int8  (*set_baudrate_fn)(void *ctx, uint32 value),
        int8  (*get_baudrate_fn)(void *ctx),
        int8  (*set_index_fn)(void *ctx, int8 value),
        int8  (*get_index_fn)(void *ctx),
        int8  (*write_blocking_fn)(void *ctx, const uint8 *src, size_t len),
        int8  (*read_blocking_fn)(void *ctx, uint8 *dst, size_t len),
        int8  (*write_timeout_fn)(void *ctx, const uint8 *src, size_t len, uint32 timeout_us),
        int8  (*read_timeout_fn)(void *ctx, uint8 *dst, size_t len, uint32 timeout_us),
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
        write_timeout_fn(write_timeout_fn),
        read_timeout_fn(read_timeout_fn),
        transaction_fn(transaction_fn),
        commit_fn(commit_fn)
    {}

    static int8 soft_reset(ConfigContext *ctx);

    int8 init() {
        return init_fn ? init_fn(ctx) : OWI_FUNCTION_NOT_IMPLEMENTED;
    }
    
    int8 deinit() {
        return deinit_fn ? deinit_fn(ctx) : OWI_FUNCTION_NOT_IMPLEMENTED;
    }
    
    int8 baudrate(uint32 value) {
        return set_baudrate_fn ? set_baudrate_fn(ctx, value) : OWI_FUNCTION_NOT_IMPLEMENTED;
    }
    int8 baudrate() {
        return get_baudrate_fn ? get_baudrate_fn(ctx) : OWI_FUNCTION_NOT_IMPLEMENTED;
    }
    
    
    int8 index(int8 value) {
        return set_index_fn ? set_index_fn(ctx, value) : OWI_FUNCTION_NOT_IMPLEMENTED;
    }
    int8 index() {
        return get_index_fn ? get_index_fn(ctx) : OWI_FUNCTION_NOT_IMPLEMENTED;
    }


    int8 write(const uint8 *src, size_t len) {
        return write_blocking_fn ? write_blocking_fn(ctx, src, len) : OWI_FUNCTION_NOT_IMPLEMENTED;
    }
    template <size_t N>
    int8 write(const uint8 (&src)[N]) {
        return write(src, N);
    }
    int8 write(uint8 byte) {
        return write(&byte, 1);
    }


    int8 read(uint8 *dst, size_t len) {
        return read_blocking_fn ? read_blocking_fn(ctx, dst, len) : OWI_FUNCTION_NOT_IMPLEMENTED;
    }
    template <size_t N>
    int8 read(uint8 (&dst)[N]) {
        return read(dst, N);
    }
    int8 read(uint8 &byte) {
        return read(&byte, 1);
    }


    int8 write_timeout(const uint8 *src, size_t len, uint32 timeout_us) {
        return write_timeout_fn ? write_timeout_fn(ctx, src, len, timeout_us) : OWI_FUNCTION_NOT_IMPLEMENTED;
    }
    template <size_t N>
    int8 write_timeout(uint8 (&src)[N], uint32 timeout_us) {
        return write_timeout(src, N, timeout_us);
    }
    int8 write_timeout(uint8 byte, uint32 timeout_us) {
        return write_timeout(&byte, 1, timeout_us);
    }


    int8 read_timeout(uint8 *dst, size_t len, uint32 timeout_us) {
        return read_timeout_fn ? read_timeout_fn(ctx, dst, len, timeout_us) : OWI_FUNCTION_NOT_IMPLEMENTED;
    }
    template <size_t N>
    int8 read_timeout(uint8 (&dst)[N], uint32 timeout_us) {
        return read_timeout(dst, N, timeout_us);
    }
    int8 read_timeout(uint8 &byte, uint32 timeout_us) {
        return read_timeout(&byte, 1, timeout_us);
    }


    TransactionGuard transaction(void *owner = nullptr) {
        if(transaction_fn) {
            int8 status = transaction_fn(ctx, owner);
            return TransactionGuard(ctx, owner, commit_fn, status);
        } else {
            return TransactionGuard(ctx, owner, commit_fn, OWI_FUNCTION_NOT_IMPLEMENTED);
        }
    }
    
    int8 commit(void *owner = nullptr) {return commit_fn ? commit_fn(ctx, owner) : OWI_FUNCTION_NOT_IMPLEMENTED;};

private:
    void *ctx = nullptr;

    friend void bind(OWI &owi, ConfigContext &cfg, BackendTable &backend);

    int8  (*init_fn)(void *ctx) = nullptr;
    int8  (*deinit_fn)(void *ctx) = nullptr;
    
    int8  (*set_baudrate_fn)(void *ctx, uint32 value) = nullptr;
    int8  (*get_baudrate_fn)(void *ctx) = nullptr;

    int8  (*set_index_fn)(void *ctx, int8 value) = nullptr;
    int8  (*get_index_fn)(void *ctx) = nullptr;

    int8  (*write_blocking_fn)(void *ctx, const uint8 *src, size_t len) = nullptr;
    int8  (*read_blocking_fn)(void *ctx, uint8 *dst, size_t len) = nullptr;

    int8  (*write_timeout_fn)(void *ctx, const uint8 *src, size_t len, uint32 timeout_us) = nullptr;
    int8  (*read_timeout_fn)(void *ctx, uint8 *dst, size_t len, uint32 timeout_us) = nullptr;

    int8  (*transaction_fn)(void *ctx, void *owner) = nullptr;
    int8  (*commit_fn)(void *ctx, void *owner) = nullptr;
};

}  
