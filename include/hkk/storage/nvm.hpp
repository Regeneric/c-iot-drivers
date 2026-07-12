#pragma once
#include <hkk/defines.h>

#include <hkk/logger/logger.h>


namespace hkk::storage::nvm {

class  NVM;
struct ConfigContext;
struct LockState;
struct BackendTable;

void bind(NVM &nvm, ConfigContext &cfg);
void bind(NVM &nvm, ConfigContext &cfg, const BackendTable &backend);
const char *rts(int8 status);


enum Result : int8 {
    NVM_OK                        = 0,
    NVM_ERROR_NULL_CONTEXT        = -1,
    NVM_ERROR_NULL_MUTEX          = -2,
    NVM_ERROR_BUSY                = -3,
    
    NVM_ERROR_GENERIC             = -100,
    NVM_FUNCTION_NOT_IMPLEMENTED  = -101,
    NVM_ERROR_UNKNOWN             = -102,
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
        if(status == NVM_OK && commit_fn != nullptr && ctx != nullptr) {
            commit_fn(ctx, owner);
        } else {
            HERROR("[NVM    ] Transaction function not implemented");
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

    void *instance;

    uint32 sector_size;
    uint32 page_size;
    uint32 pages_per_sector;
    uint32 sectors_number;
    uint32 storage_offset;

    int8 status = NVM_OK;
};


class NVM {
public:
    NVM() = default;
    NVM(
        ConfigContext *cfg,
        int8  (*init_fn)(void *ctx),
        int8  (*deinit_fn)(void *ctx),
        int8  (*config_fn)(void *ctx),
        int8  (*clear_sector_fn)(void *ctx),
        int32 (*write_blocking_fn)(void *ctx, uint8 addr, const uint8 *src, size_t len),
        int32 (*read_blocking_fn)(void *ctx, uint8 addr, uint8 *dst, size_t len),
        int8  (*transaction_fn)(void *ctx, void *owner),
        int8  (*commit_fn)(void *ctx, void *owner)
    ) : ctx(cfg),
        init_fn(init_fn),
        deinit_fn(deinit_fn),
        // config_fn(config_fn),
        // clear_sector_fn(clear_sector_fn),
        write_blocking_fn(write_blocking_fn),
        read_blocking_fn(read_blocking_fn),
        transaction_fn(transaction_fn),
        commit_fn(commit_fn)
    {}


    int8 init() {
        return init_fn ? init_fn(ctx) : NVM_FUNCTION_NOT_IMPLEMENTED;
    }
    
    int8 deinit() {
        return deinit_fn ? deinit_fn(ctx) : NVM_FUNCTION_NOT_IMPLEMENTED;
    }


    int32 write(uint8 addr, const uint8 *src, size_t len) {
        return write_blocking_fn ? write_blocking_fn(ctx, addr, src, len) : NVM_FUNCTION_NOT_IMPLEMENTED;
    }
    template <size_t N>
    int32 write(uint8 addr, const uint8 (&src)[N]) {
        return write(addr, src, N);
    }


    int32 read(uint8 addr, uint8 *dst, size_t len) {
        return read_blocking_fn ? read_blocking_fn(ctx, addr, dst, len) : NVM_FUNCTION_NOT_IMPLEMENTED;
    }
    template <size_t N>
    int32 read(uint8 addr, uint8 (&dst)[N]) {
        return read(addr, dst, N);
    }


    TransactionGuard transaction(void *owner = nullptr) {
        if(transaction_fn) {
            int8 status = transaction_fn(ctx, owner);
            return TransactionGuard(ctx, owner, commit_fn, status);
        } else {
            return TransactionGuard(ctx, owner, commit_fn, NVM_FUNCTION_NOT_IMPLEMENTED);
        }
    }
    
    int8 commit(void *owner = nullptr) {return commit_fn ? commit_fn(ctx, owner) : NVM_FUNCTION_NOT_IMPLEMENTED;};

private:
    void *ctx = nullptr;

    friend void bind(NVM &nvm, ConfigContext &cfg, const BackendTable &backend);

    int8  (*init_fn)(void *ctx) = nullptr;
    int8  (*deinit_fn)(void *ctx) = nullptr;

    int8  (*config_fn)(void *ctx) = nullptr;
    int8  (*clear_sector_fn)(void *ctx) = nullptr;

    int32 (*write_blocking_fn)(void *ctx, uint8 addr, const uint8 *src, size_t len) = nullptr;
    int32 (*read_blocking_fn)(void *ctx, uint8 addr, uint8 *dst, size_t len) = nullptr;

    int8 (*transaction_fn)(void *ctx, void *owner) = nullptr;
    int8 (*commit_fn)(void *ctx, void *owner) = nullptr;
};

struct BackendTable {
    int8  (*init_fn)(void *ctx) = nullptr;
    int8  (*deinit_fn)(void *ctx) = nullptr;

    int8  (*config_fn)(void *ctx) = nullptr;
    int8  (*clear_sector_fn)(void *ctx) = nullptr;

    int32 (*write_blocking_fn)(void *ctx, uint8 addr, const uint8 *src, size_t len) = nullptr;
    int32 (*read_blocking_fn)(void *ctx, uint8 addr, uint8 *dst, size_t len) = nullptr;
};

}

