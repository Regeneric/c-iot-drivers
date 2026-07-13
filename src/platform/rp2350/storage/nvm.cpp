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


static int8 set_storage_offset_fn(void *ctx_raw, int32 offset) {
    HTRACE("nvm.cpp -> s:set_storage_offset_fn(void*, int32):int8");

    if(!ctx_raw) {
        HERROR("[NVM    ] Null context passed to function");
        return NVM_ERROR_NULL_CONTEXT;
    }
    auto *ctx = static_cast<ConfigContext*>(ctx_raw);

    ctx->storage_offset = static_cast<uint32>(offset);
    HDEBUG("[NVM    ] Storage offset set to %d bytes", ctx->storage_offset);

    return NVM_OK;
}

static uint32 get_storage_offset_fn(void *ctx_raw) {
   HTRACE("nvm.cpp -> s:get_storage_offset_fn(void*):uint32"); 

    if(!ctx_raw) {
        HERROR("[NVM    ] Null context passed to function");
        return static_cast<uint32>(NVM_ERROR_NULL_CONTEXT);
    }
    auto *ctx = static_cast<ConfigContext*>(ctx_raw);

    HTRACE("[NVM    ] Storage offset: %d bytes", ctx->storage_offset);
    return ctx->storage_offset;
}

static int8 set_sectors_number_fn(void *ctx_raw, int32 sectors) {
    HTRACE("nvm.cpp -> s:set_sectors_number_fn(void*, int32):int8");

    if(!ctx_raw) {
        HERROR("[NVM    ] Null context passed to function");
        return NVM_ERROR_NULL_CONTEXT;
    }
    auto *ctx = static_cast<ConfigContext*>(ctx_raw);

    ctx->sectors_number = static_cast<uint32>(sectors);
    HDEBUG("[NVM    ] Sectors number set to %d bytes", ctx->sectors_number);

    return NVM_OK;
}

static uint32 get_sectors_number_fn(void *ctx_raw) {
    HTRACE("nvm.cpp -> s:get_sectors_number_fn(void*):uint32");

    if(!ctx_raw) {
        HERROR("[NVM    ] Null context passed to function");
        return NVM_ERROR_NULL_CONTEXT;
    }
    auto *ctx = static_cast<ConfigContext*>(ctx_raw);

    HTRACE("[NVM    ] Sectors number: %d", ctx->sectors_number);
    return ctx->sectors_number;
}

static uint32 get_pages_per_sector_fn(void *ctx_raw, bool8 current_page) {
    HTRACE("nvm.cpp -> s:get_pages_in_sector_fn(void*):uint32");

    if(!ctx_raw) {
        HERROR("[NVM    ] Null context passed to function");
        return NVM_ERROR_NULL_CONTEXT;
    }
    auto *ctx = static_cast<ConfigContext*>(ctx_raw);

    HTRACE("[NVM    ] Pages per sector: %d", ctx->pages_per_sector);
    HTRACE("[NVM    ] Current page    : %d", ctx->current_page);
    
    if(current_page == true) return ctx->current_page;
    else return ctx->pages_per_sector;
}


void bind(NVM &nvm, ConfigContext &cfg, const BackendTable &backend) {
    HTRACE("nvm.cpp -> bind(NVM&, ConfigContext&, const BackendTable&):void");

    nvm.ctx = static_cast<ConfigContext*>(&cfg);

    nvm.init_fn = backend.init_fn;
    nvm.deinit_fn = backend.deinit_fn;

    nvm.clear_sector_fn = backend.clear_sector_fn;

    nvm.get_storage_offset_fn = get_storage_offset_fn;
    nvm.set_storage_offset_fn = set_storage_offset_fn;

    nvm.get_sectors_number_fn = get_sectors_number_fn;
    nvm.set_sectors_number_fn = set_sectors_number_fn;

    nvm.get_pages_per_sector_fn = get_pages_per_sector_fn;

    nvm.write_blocking_fn = backend.write_blocking_fn;
    nvm.read_blocking_fn = backend.read_blocking_fn;

    nvm.transaction_fn = transaction_fn;
    nvm.commit_fn = commit_fn;
}


const char *rts(int8 status) {
    switch(status) {
        case NVM_OK:                        return "NVM_OK";                      
        case NVM_ERROR_NULL_CONTEXT:        return "NVM_ERROR_NULL_CONTEXT";             
        case NVM_ERROR_NULL_DATA:           return "NVM_ERROR_NULL_DATA";                    
        case NVM_ERROR_ZERO_LENGTH:         return "NVM_ERROR_ZERO_LENGTH";        
        case NVM_ERROR_GENERIC:             return "NVM_ERROR_GENERIC";
        case NVM_FUNCTION_NOT_IMPLEMENTED:  return "NVM_FUNCTION_NOT_IMPLEMENTED"; 
        case NVM_ERROR_NULL_MUTEX:          return "NVM_ERROR_NULL_MUTEX"; 
        case NVM_ERROR_BUSY:                return "NVM_ERROR_BUSY"; 
        case NVM_ERROR_UNKNOWN:             return "NVM_ERROR_UNKNOWN"; 
        case NVM_DATA_TRUNCATED:            return "NVM_DATA_TRUNCATED";
        case NVM_ERROR_OOB:                 return "NVM_ERROR_OOB"; 
        default:                            return "NVM_ERROR_UNKNOWN";
    }
}

}