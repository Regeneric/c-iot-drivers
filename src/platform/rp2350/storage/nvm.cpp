#include <hkk/defines.h>

#include <hkk/logger/logger.h>
#include <hkk/storage/nvm.hpp>

#include <pico/mutex.h>

namespace hkk::storage::nvm {

static int8 transaction_fn(void *ctx_raw, void *owner) {
    HTRACE("nvm.cpp -> s:transaction_fn(void*, void* = nullptr):int8");

    if(!ctx_raw) {
        HERROR("[NVM    ] Null context passed to function");
        return NVM_ERROR_NULL_CONTEXT;
    }
    auto *ctx = static_cast<ConfigContext*>(ctx_raw);

    if(!ctx->transaction || !ctx->transaction->mutex) {
        HERROR("[NVM    ] Null NVM mutex in context");

        ctx->status = NVM_ERROR_NULL_MUTEX;
        return ctx->status;
    }
    auto *transaction = static_cast<LockState*>(ctx->transaction);

    if(transaction->active && transaction->owner == owner) {
        HWARN("[NVM    ] Transaction already active for this owner: %p", transaction->owner);

        ctx->status = NVM_ERROR_BUSY;
        return ctx->status;
    } 

    if(transaction->active && transaction->owner != owner) {
        HWARN("[NVM    ] Transaction already active for another owner: %p", transaction->owner);
        ctx->status = NVM_ERROR_BUSY;
    }

    ::mutex_t *mutex = static_cast<::mutex_t*>(transaction->mutex);
    ::mutex_enter_blocking(mutex);

    transaction->owner  = owner;
    transaction->active = true;

    HTRACE("[NVM    ] Transaction begin for owner %p", transaction->owner);

    ctx->status = NVM_OK;
    return ctx->status;
}

static int8 commit_fn(void *ctx_raw, void *owner) {
    HTRACE("nvm.cpp -> s:commit_fn(void*, void* = nullptr):int8");

    if(!ctx_raw) {
        HERROR("[NVM    ] Null context passed to function");
        return NVM_ERROR_NULL_CONTEXT;
    }
    auto *ctx = static_cast<ConfigContext*>(ctx_raw);

    if(!ctx->transaction || !ctx->transaction->mutex) {
        HERROR("[NVM    ] Null NVM mutex in context");

        ctx->status = NVM_ERROR_NULL_MUTEX;
        return ctx->status;
    }
    auto *transaction = static_cast<LockState*>(ctx->transaction);

    if(transaction->active == false) {
        HWARN("[NVM    ] No transaction in progress");

        ctx->status = NVM_OK;
        return ctx->status;
    } 

    if(transaction->active && transaction->owner != owner) {
        HWARN("[NVM    ] Transaction already active for another owner: %p", transaction->owner);

        ctx->status = NVM_ERROR_BUSY;
        return ctx->status;
    }

    void *old_owner = transaction->owner;

    transaction->owner  = nullptr;
    transaction->active = false;

    ::mutex_t *mutex = static_cast<::mutex_t*>(ctx->transaction->mutex);
    ::mutex_exit(mutex);

    HTRACE("[NVM    ] Transaction end for owner: %p", old_owner);

    ctx->status = NVM_OK;
    return ctx->status;
}

void bind(NVM &nvm, ConfigContext &cfg, const BackendTable &backend) {
    HTRACE("nvm.cpp -> bind(NVM&, ConfigContext&, const BackendTable&):void");

    nvm.ctx = static_cast<ConfigContext*>(&cfg);

    nvm.init_fn = backend.init_fn;
    nvm.deinit_fn = backend.deinit_fn;

    nvm.config_fn = backend.config_fn;
    nvm.clear_sector_fn = backend.clear_sector_fn;

    nvm.write_blocking_fn = backend.write_blocking_fn;
    nvm.read_blocking_fn = backend.read_blocking_fn;

    nvm.transaction_fn = transaction_fn;
    nvm.commit_fn = commit_fn;
}



// static ::mutex_t nvm0_mutex;
// static LockState nvm0_transaction {
//     .mutex = &nvm0_mutex,
//     .owner = nullptr,
//     .active = false
// };

// static ::mutex_t nvm1_mutex;
// static LockState nvm1_transaction {
//     .mutex = &nvm1_mutex,
//     .owner = nullptr,
//     .active = false
// };

// static ConfigContext nvm0_default_config {
//     .transaction = &nvm0_transaction
// };

// static ConfigContext nvm1_default_config {
//     .transaction = &nvm1_transaction
// };

// NVM nvm0 {};
// NVM nvm1 {};


}