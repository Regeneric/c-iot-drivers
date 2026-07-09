#pragma once
#include <hkk/defines.h>


namespace hkk::bus {   

class I2C;
struct I2C_Config_Context;
struct I2C_Bus_Lock_State;

// Implement only those present on your uC
// Keep the rest empty
extern I2C I2C0;
extern I2C I2C1;
extern I2C I2C2;
extern I2C I2C3;
extern I2C I2C4;
extern I2C I2C5;
extern I2C I2C6;
extern I2C I2C7;

int8 bind(I2C &i2c, I2C_Config_Context &cfg);


enum I2C_Result : int8 {
    I2C_OK                       =  0,
    I2C_ERROR_NULL_CONTEXT       = -1,
    I2C_ERROR_NULL_INSTANCE      = -2,
    I2C_ERROR_NULL_DATA          = -3,
    I2C_ERROR_ZERO_LENGTH        = -4,
    I2C_ERROR_NO_ACK             = -5,
    I2C_ERROR_PARTIAL_WRITE      = -6,
    I2C_ERROR_WRITE_FAILED       = -7,
    I2C_ERROR_PARTIAL_READ       = -8,
    I2C_ERROR_READ_FAILED        = -9,  
    I2C_ERROR_NOT_SUPPORTED      = -10, 
    I2C_ERROR_TIMEOUT            = -11,
    I2C_ERROR_NULL_MUTEX         = -12,
    I2C_ERROR_BUSY               = -13,
    I2C_ERROR_GENERIC            = -100,
    I2C_FUNCTION_NOT_IMPLEMENTED = -101,
};


class I2C_Transaction_Guard {
private:
    void *ctx;
    void *owner;

    int8 (*commit_fn)(void *ctx, void *owner) = nullptr;

public:
    int8 status;

    // No copy allowed
    I2C_Transaction_Guard(const I2C_Transaction_Guard&) = delete;
    I2C_Transaction_Guard& operator=(const I2C_Transaction_Guard&) = delete;

    I2C_Transaction_Guard(
        void *ctx,
        void *owner,
        int8 (*commit_fn)(void *ctx, void *owner),
        int8 status
    ) : ctx(ctx),
        owner(owner),
        commit_fn(commit_fn),
        status(status)
    {}

    ~I2C_Transaction_Guard() {
        if(status == I2C_OK && commit_fn != nullptr && ctx != nullptr) {
            commit_fn(ctx, owner);
        }
    }
};

struct I2C_Bus_Lock_State {
    void *mutex = nullptr;
    void *owner = nullptr;
    bool8 active = false;
};

struct I2C_Config_Context {
    I2C_Bus_Lock_State *transaction;
    
    void *instance;
    uint32 baudrate = 100000;   // 100 kHz
    uint8 sda;
    uint8 scl;
    int8 index = -1;
    int8 status = I2C_OK;
};

class I2C {
public:
    I2C() = default;
    I2C(
        I2C_Config_Context *cfg,
        int8  (*init_fn)(void *ctx),
        int8  (*deinit_fn)(void *ctx),
        int8  (*set_baudrate_fn)(void *ctx, uint32 value),
        int32 (*get_baudrate_fn)(void *ctx),
        int8  (*set_index_fn)(void *ctx, int8 value),
        int32 (*get_index_fn)(void *ctx),
        int32 (*write_blocking_fn)(void *ctx, uint8 addr, const uint8 *src, size_t len, bool8 nostop),
        int32 (*read_blocking_fn)(void *ctx, uint8 addr, uint8 *dst, size_t len, bool8 nostop),
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
        return init_fn ? init_fn(ctx) : I2C_FUNCTION_NOT_IMPLEMENTED;
    }
    
    int8 deinit() {
        return deinit_fn ? deinit_fn(ctx) : I2C_FUNCTION_NOT_IMPLEMENTED;
    }
    
    int8 baudrate(uint32 value) {
        return set_baudrate_fn ? set_baudrate_fn(ctx, value) : I2C_FUNCTION_NOT_IMPLEMENTED;
    }
    int32 baudrate() {
        return get_baudrate_fn ? get_baudrate_fn(ctx) : I2C_FUNCTION_NOT_IMPLEMENTED;
    }
    
    
    int8 index(int8 value) {
        return set_index_fn ? set_index_fn(ctx, value) : I2C_FUNCTION_NOT_IMPLEMENTED;
    }
    int32 index() {
        return get_index_fn ? get_index_fn(ctx) : I2C_FUNCTION_NOT_IMPLEMENTED;
    }


    int32 write(uint8 addr, const uint8 *src, size_t len, bool8 nostop = false) {
        return write_blocking_fn ? write_blocking_fn(ctx, addr, src, len, nostop) : I2C_FUNCTION_NOT_IMPLEMENTED;
    }
    template <size_t N>
    int32 write(uint8 addr, const uint8 (&src)[N], bool8 nostop = false) {
        return write(addr, src, N, nostop);
    }


    int32 read(uint8 addr, uint8 *dst, size_t len, bool8 nostop = false) {
        return read_blocking_fn ? read_blocking_fn(ctx, addr, dst, len, nostop) : I2C_FUNCTION_NOT_IMPLEMENTED;
    }
    template <size_t N>
    int32 read(uint8 addr, uint8 (&dst)[N], bool8 nostop = false) {
        return read(addr, dst, N, nostop);
    }


    int32 write_timeout(uint8 addr, const uint8 *src, size_t len, uint32 timeout_us, bool8 nostop = false) {
        return write_timeout_fn ? write_timeout_fn(ctx, addr, src, len, timeout_us, nostop) : I2C_FUNCTION_NOT_IMPLEMENTED;
    }
    template <size_t N>
    int32 write_timeout(uint8 addr, uint8 (&src)[N], uint32 timeout_us, bool8 nostop = true) {
        return write_timeout(addr, src, N, timeout_us, nostop);
    }


    I2C_Transaction_Guard transaction(void *owner = nullptr) {
        if(transaction_fn) {
            int8 status = transaction_fn(ctx, owner);
            return I2C_Transaction_Guard(ctx, owner, commit_fn, status);
        } else {
            return I2C_Transaction_Guard(ctx, owner, commit_fn, I2C_FUNCTION_NOT_IMPLEMENTED);
        }
    }
    
    int8 commit(void *owner = nullptr) {return commit_fn ? commit_fn(ctx, owner) : I2C_FUNCTION_NOT_IMPLEMENTED;};

private:
    void *ctx = nullptr;

    friend int8 bind(I2C &i2c, I2C_Config_Context &cfg);

    int8 (*init_fn)(void *ctx) = nullptr;
    int8 (*deinit_fn)(void *ctx) = nullptr;
    
    int8  (*set_baudrate_fn)(void *ctx, uint32 value) = nullptr;
    int32 (*get_baudrate_fn)(void *ctx) = nullptr;

    int8  (*set_index_fn)(void *ctx, int8 value) = nullptr;
    int32 (*get_index_fn)(void *ctx) = nullptr;

    int32 (*write_blocking_fn)(void *ctx, uint8 addr, const uint8 *src, size_t len, bool8 nostop) = nullptr;
    int32 (*read_blocking_fn)(void *ctx, uint8 addr, uint8 *dst, size_t len, bool8 nostop) = nullptr;

    int32 (*write_timeout_fn)(void *ctx, uint8 addr, const uint8 *src, size_t len, uint32 timeout_us, bool8 nostop) = nullptr;
    int32 (*read_timeout_fn)(void *ctx, uint8 addr, uint8 *dst, size_t len, uint32 timeout_us, bool8 nostop) = nullptr;

    int8 (*transaction_fn)(void *ctx, void *owner) = nullptr;
    int8 (*commit_fn)(void *ctx, void *owner) = nullptr;
};

}  