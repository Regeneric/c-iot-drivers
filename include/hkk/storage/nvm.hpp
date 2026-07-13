#pragma once
#include <hkk/defines.h>

#include <hkk/logger/logger.h>

#include <span>


namespace hkk::storage::nvm {

class  NVM;
struct ConfigContext;
struct LockState;
struct BackendTable;

void bind(NVM &nvm, ConfigContext &cfg);
void bind(NVM &nvm, ConfigContext &cfg, const BackendTable &backend);
const char *rts(int8 status);


enum Result : int8 {
    NVM_OK                        =  0, 
    NVM_ERROR_NULL_CONTEXT        = -1,
    NVM_ERROR_NULL_MUTEX          = -2,
    NVM_ERROR_BUSY                = -3,
    NVM_ERROR_NULL_DATA           = -4,
    NVM_ERROR_ZERO_LENGTH         = -5,
    NVM_ERROR_OOB                 = -6,
    
    NVM_ERROR_GENERIC             = -100,
    NVM_FUNCTION_NOT_IMPLEMENTED  = -101,
    NVM_ERROR_UNKNOWN             = -102,
    NVM_DATA_TRUNCATED            = -103,
    NVM_NULL_ADDRESS              = -104,
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

    const uint32 page_size  = 0;
    uint32 sector_size      = 0;
    uint32 pages_per_sector = 0;
    uint32 sectors_number   = 0;
    uint32 storage_offset   = 0;
    uint32 current_page     = 0;

    int8 status = NVM_OK;
};


class NVM {
public:
    NVM() = default;
    NVM(
        ConfigContext *cfg,
        int8   (*init_fn)(void *ctx, bool8 clear_data),
        int8   (*deinit_fn)(void *ctx),
        int8   (*clear_sector_fn)(void *ctx, int32 offset, size_t sectors_number),
        int8   (*set_storage_offset_fn)(void *ctx, int32 offset),
        uint32 (*get_storage_offset_fn)(void *ctx),
        int8   (*set_sectors_number_fn)(void *ctx, int32 sectors),
        uint32 (*get_sectors_number_fn)(void *ctx),
        uint32 (*get_current_page_fn)(void *ctx),
        int8   (*write_blocking_fn)(void *ctx, int32 addr, const uint8 *src, size_t len),
        int8   (*read_blocking_fn)(void *ctx, int32 addr, int32 page, uint8 *dst, size_t len),
        int8   (*transaction_fn)(void *ctx, void *owner),
        int8   (*commit_fn)(void *ctx, void *owner)
    ) : ctx(cfg),
        init_fn(init_fn),
        deinit_fn(deinit_fn),
        clear_sector_fn(clear_sector_fn),
        set_storage_offset_fn(set_storage_offset_fn),
        get_storage_offset_fn(get_storage_offset_fn),
        set_sectors_number_fn(set_sectors_number_fn),
        get_sectors_number_fn(get_sectors_number_fn),
        get_current_page_fn(get_current_page_fn),
        write_blocking_fn(write_blocking_fn),
        read_blocking_fn(read_blocking_fn),
        transaction_fn(transaction_fn),
        commit_fn(commit_fn)
    {}


    int8 init(bool8 clear_data = false) {
        return init_fn ? init_fn(ctx, clear_data) : NVM_FUNCTION_NOT_IMPLEMENTED;
    }
    
    int8 deinit() {
        return deinit_fn ? deinit_fn(ctx) : NVM_FUNCTION_NOT_IMPLEMENTED;
    }

    int8 clear(int32 offset, size_t sectors_number) {
        return clear_sector_fn ? clear_sector_fn(ctx, offset, sectors_number) : NVM_FUNCTION_NOT_IMPLEMENTED;
    }
    int8 clear() {
        return clear(offset(), sectors());
    }

    int8 offset(int32 offset) {
        return set_storage_offset_fn ? set_storage_offset_fn(ctx, offset) : NVM_FUNCTION_NOT_IMPLEMENTED;
    }
    uint32 offset() {
        return get_storage_offset_fn ? get_storage_offset_fn(ctx) : NVM_FUNCTION_NOT_IMPLEMENTED; 
    }

    int8 sectors(int32 sectors) {
        return set_sectors_number_fn ? set_sectors_number_fn(ctx, sectors) : NVM_FUNCTION_NOT_IMPLEMENTED;
    }
    uint32 sectors() {
        return get_sectors_number_fn ? get_sectors_number_fn(ctx) : NVM_FUNCTION_NOT_IMPLEMENTED; 
    }

    uint32 page() {
        return get_current_page_fn ? get_current_page_fn(ctx) : NVM_FUNCTION_NOT_IMPLEMENTED;
    }


    int8 write(int32 addr, const uint8 *src, size_t len) {
        return write_blocking_fn ? write_blocking_fn(ctx, addr, src, len) : NVM_FUNCTION_NOT_IMPLEMENTED;
    }
    int8 write(const uint8 *src, size_t len) {
        return write(NVM_NULL_ADDRESS, src, len);
    }
    template <size_t N>
    int8 write(int32 addr, const uint8 (&src)[N]) {
        return write(addr, src, N);
    }
    template <size_t N>
    int8 write(const uint8 (&src)[N]) {
        return write(NVM_NULL_ADDRESS, src, N);
    }


    int8 read(int32 addr, int32 page, uint8 *dst, size_t len) {
        return read_blocking_fn ? read_blocking_fn(ctx, addr, page, dst, len) : NVM_FUNCTION_NOT_IMPLEMENTED;
    }
    int8 read(int32 addr, uint8 *dst, size_t len) {
        return read(addr, 0, dst, len);
    }
    int8 read(uint8 *dst, size_t len) {
        return read(NVM_NULL_ADDRESS, dst, len);
    }
    template <size_t N>
    int8 read(int32 addr, int32 page, uint8 (&dst)[N]) {
        return read(addr, page, dst, N);
    }
    template <size_t N>
    int8 read(int32 addr, uint8 (&dst)[N]) {
        return read(addr, 0, dst, N);
    }
    template <size_t N>
    int8 read(uint8 (&dst)[N]) {
        return read(NVM_NULL_ADDRESS, 0, dst, N);
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

    int8   (*init_fn)(void *ctx, bool8 clear_data) = nullptr;
    int8   (*deinit_fn)(void *ctx) = nullptr;

    int8   (*clear_sector_fn)(void *ctx, int32 offset, size_t sectors_number) = nullptr;

    int8   (*set_storage_offset_fn)(void *ctx, int32 offset) = nullptr;
    uint32 (*get_storage_offset_fn)(void *ctx) = nullptr;

    int8   (*set_sectors_number_fn)(void *ctx, int32 sectors) = nullptr;
    uint32 (*get_sectors_number_fn)(void *ctx) = nullptr;

    uint32 (*get_current_page_fn)(void *ctx) = nullptr;

    int8   (*write_blocking_fn)(void *ctx, int32 addr, const uint8 *src, size_t len) = nullptr;
    int8   (*read_blocking_fn)(void *ctx, int32 addr, int32 page, uint8 *dst, size_t len) = nullptr;

    int8   (*transaction_fn)(void *ctx, void *owner) = nullptr;
    int8   (*commit_fn)(void *ctx, void *owner) = nullptr;
};

struct BackendTable {
    int8   (*init_fn)(void *ctx, bool8 clear_data) = nullptr;
    int8   (*deinit_fn)(void *ctx) = nullptr;

    int8   (*clear_sector_fn)(void *ctx, int32 offset, size_t sectors_number) = nullptr;

    int8   (*write_blocking_fn)(void *ctx, int32 addr, const uint8 *src, size_t len) = nullptr;
    int8   (*read_blocking_fn)(void *ctx, int32 addr, int32 page, uint8 *dst, size_t len) = nullptr;
};

}

namespace hkk::storage::flash {
    void bind(nvm::NVM &nvm, nvm::ConfigContext &cfg);
}

namespace hkk::storage::eeprom {
    void bind(nvm::NVM &nvm, nvm::ConfigContext &cfg);
}

namespace hkk::storage::fram {
    void bind(nvm::NVM &nvm, nvm::ConfigContext &cfg);
}