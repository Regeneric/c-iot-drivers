#include <hkk/defines.h>

#include <hkk/logger/logger.h>
#include <hkk/storage/nvm.hpp>

#include <hardware/flash.h> 

#include <pico/mutex.h>

#include <cstring>
#include <cmath>

namespace hkk::rp2350::flash {

// TODO: move it to config file
inline constexpr uint32 FLASH_SECTOR_SIZE_B     = 4096;
// inline constexpr uint32 FLASH_PAGE_SIZE_B       = 256;
// inline constexpr uint32 FLASH_PAGES_PER_SECTOR  = (FLASH_SECTOR_SIZE_B / FLASH_PAGE_SIZE_B);
inline constexpr uint32 FLASH_SECTORS_NUMBER    = 1;
// inline constexpr uint32 FLASH_STORAGE_OFFSET_B  = (PICO_FLASH_SIZE_BYTES - (FLASH_SECTORS_NUMBER * FLASH_SECTOR_SIZE_B));


static uint8 current_page_g = 0;


static int8 deinit_fn(void *ctx_raw) {
    HTRACE("flash.cpp -> s:deinit_fn(void*):int8"); 

    if(!ctx_raw) {
        HERROR("[FLASH  ] Null context passed to function");
        return hkk::storage::nvm::NVM_ERROR_NULL_CONTEXT;
    }
    auto *ctx = static_cast<hkk::storage::nvm::ConfigContext*>(ctx_raw);

    if(!ctx->transaction || !ctx->transaction->mutex) {
        HWARN("[FLASH  ] Null NVM mutex in context");
        ctx->status = hkk::storage::nvm::NVM_ERROR_NULL_MUTEX;
    } else {
        ::mutex_exit(static_cast<::mutex_t*>(ctx->transaction->mutex));
    }

    ctx->sector_size        = 0;
    ctx->pages_per_sector   = 0;
    ctx->sectors_number     = 0;
    ctx->storage_offset     = 0;

    HINFO("[FLASH  ] NVM storage deinitialized");

    ctx->status = hkk::storage::nvm::NVM_OK;
    return ctx->status;
}

static int8 init_fn(void *ctx_raw, bool8 clear_data) {
    HTRACE("flash.cpp -> s:init_fn(void*, bool8 = false):int8");

    if(!ctx_raw) {
        HERROR("[FLASH  ] Null context passed to function");
        return hkk::storage::nvm::NVM_ERROR_NULL_CONTEXT;
    }
    auto *ctx = static_cast<hkk::storage::nvm::ConfigContext*>(ctx_raw);

    if(!ctx->transaction || !ctx->transaction->mutex) {
        HERROR("[FLASH  ] Null NVM mutex in context");
        
        deinit_fn(ctx_raw);

        ctx->status = hkk::storage::nvm::NVM_ERROR_NULL_MUTEX;
        return ctx->status;
    }
    ::mutex_init(static_cast<::mutex_t*>(ctx->transaction->mutex));

    if(ctx->page_size == 0) {
        HERROR("[FLASH  ] Page size initialized with value of 0");
        return hkk::storage::nvm::NVM_ERROR_ZERO_LENGTH;
    }

    ctx->sector_size      = (ctx->sector_size      ? ctx->sector_size      : FLASH_SECTOR_SIZE_B);
    ctx->pages_per_sector = (ctx->pages_per_sector ? ctx->pages_per_sector : (ctx->sector_size / ctx->page_size));
    ctx->sectors_number   = (ctx->sectors_number   ? ctx->sectors_number   : FLASH_SECTORS_NUMBER);
    ctx->storage_offset   = (ctx->storage_offset   ? ctx->storage_offset   : (PICO_FLASH_SIZE_BYTES - (ctx->sectors_number * ctx->sector_size)));
    ctx->current_page     = (ctx->current_page     ? ctx->current_page     : 0);

    HDEBUG("[FLASH  ] Sector size     : %d bytes", ctx->sector_size);
    HDEBUG("[FLASH  ] Page size       : %d bytes", ctx->page_size);
    HDEBUG("[FLASH  ] Storage offset  : %d bytes", ctx->storage_offset);
    HDEBUG("[FLASH  ] Pages per sector: %d", ctx->pages_per_sector);
    HDEBUG("[FLASH  ] Sectors number  : %d", ctx->sectors_number);
    HDEBUG("[FLASH  ] Current page    : %d", ctx->current_page);

    if(clear_data == true) {
        HWARN("[FLASH  ] Specified sector will be erased");
        ::flash_range_erase(ctx->storage_offset, ctx->sector_size);
    }

    HINFO("[FLASH  ] NVM storage initialized");
    
    ctx->status = hkk::storage::nvm::NVM_OK;
    return ctx->status;
}

static int8 write_blocking_fn(void *ctx_raw, int32 addr, const uint8 *src, size_t len) {
    HTRACE("flash.cpp -> s:write_blocking_fn(void*, int32, const uint8*, size_t):int8");

    if(!ctx_raw) {
        HERROR("[FLASH  ] Null context passed to function");
        return hkk::storage::nvm::NVM_ERROR_NULL_CONTEXT;
    }
    auto *ctx = static_cast<hkk::storage::nvm::ConfigContext*>(ctx_raw);

    if(!src) {
        HERROR("[FLASH  ] Null data pointer passed to function");
        
        ctx->status = hkk::storage::nvm::NVM_ERROR_NULL_DATA;
        return ctx->status;
    }

    if(len == 0) {
        HERROR("[FLASH  ] Data length is 0");

        ctx->status = hkk::storage::nvm::NVM_ERROR_ZERO_LENGTH;
        return ctx->status;
    }

    if(len > (ctx->sector_size * ctx->sectors_number)) {
        HERROR("[FLASH  ] Data length out of bands");
        
        ctx->status = hkk::storage::nvm::NVM_ERROR_OOB;
        return ctx->status;
    }


    uint8 payload[ctx->page_size];
    std::memset(payload, 0xFF, ctx->page_size);

    // TODO: pagination
    if(len > ctx->page_size) {
        HWARN("[FLASH  ] Data length exceeds page size; truncating to %u bytes", ctx->page_size);
        
        std::memcpy(payload, src, ctx->page_size);
        ctx->status = hkk::storage::nvm::NVM_DATA_TRUNCATED;
    } else {
        std::memcpy(payload, src, len);
        ctx->status = hkk::storage::nvm::NVM_OK;
    }
    


    if(!ctx->transaction || !ctx->transaction->mutex) {
        HERROR("[FLASH  ] Null NVM mutex in context");

        ctx->status = hkk::storage::nvm::NVM_ERROR_NULL_MUTEX;
        return ctx->status;
    }
    auto *transaction = static_cast<hkk::storage::nvm::LockState*>(ctx->transaction);

    if(current_page_g >= ctx->pages_per_sector) current_page_g = 0;
    
    uint32 offset = ctx->storage_offset;
    
    if(addr == hkk::storage::nvm::NVM_NULL_ADDRESS) {
        HTRACE("[FLASH  ] NVM storage address is NULL; using config value");
        offset = static_cast<uint32>(ctx->storage_offset + (current_page_g * ctx->page_size));
    } else {
        offset = static_cast<uint32>(addr + (current_page_g * ctx->page_size));
    }
    
    uint32 interrupts = ::save_and_disable_interrupts();
    if(transaction->active == false) {
        ::mutex_t *mutex = static_cast<::mutex_t*>(transaction->mutex);
        ::mutex_enter_blocking(mutex);
        transaction->active = true;

        ::flash_range_program(offset, payload, ctx->page_size);

        transaction->active = false;
        ::mutex_exit(mutex);
    } else {
        ::flash_range_program(offset, payload, ctx->page_size);
    }
    ::restore_interrupts(interrupts);

    ctx->current_page = ++current_page_g;

    HTRACE("[FLASH  ] First byte    : 0x%02X", payload[0]);
    HTRACE("[FLASH  ] Last byte     : 0x%02X", payload[len - 1]);
    HTRACE("[FLASH  ] Storage offset: %d bytes", offset);
    HTRACE("[FLASH  ] Current page  : %d", ctx->current_page);


    HDEBUG("[FLASH  ] Write complete");

    ctx->status = hkk::storage::nvm::NVM_OK;
    return ctx->status;
}

static int8 read_blocking_fn(void *ctx_raw, int32 addr, int32 page, uint8 *dst, size_t len) {
    HTRACE("flash.cpp -> s:read_blocking_fn(void*, int32, int32, const uint8*, size_t):int8");

    if(!ctx_raw) {
        HERROR("[FLASH  ] Null context passed to function");
        return hkk::storage::nvm::NVM_ERROR_NULL_CONTEXT;
    }
    auto *ctx = static_cast<hkk::storage::nvm::ConfigContext*>(ctx_raw);

    if(!dst) {
        HERROR("[FLASH  ] Null data pointer passed to function");
        
        ctx->status = hkk::storage::nvm::NVM_ERROR_NULL_DATA;
        return ctx->status;
    }

    if(len == 0) {
        HERROR("[FLASH  ] Data length is 0");

        ctx->status = hkk::storage::nvm::NVM_ERROR_ZERO_LENGTH;
        return ctx->status;
    }

    if(len > (ctx->sector_size * ctx->sectors_number)) {
        HERROR("[FLASH  ] Data length out of bands");
        
        ctx->status = hkk::storage::nvm::NVM_ERROR_OOB;
        return ctx->status;
    }

    if(!ctx->transaction || !ctx->transaction->mutex) {
        HERROR("[FLASH  ] Null NVM mutex in context");

        ctx->status = hkk::storage::nvm::NVM_ERROR_NULL_MUTEX;
        return ctx->status;
    }
    auto *transaction = static_cast<hkk::storage::nvm::LockState*>(ctx->transaction);

    uint32 offset = ctx->storage_offset;
    if(addr == hkk::storage::nvm::NVM_NULL_ADDRESS) {
        HTRACE("[FLASH  ] NVM storage address is NULL; using config value");
        offset = static_cast<uint32>(ctx->storage_offset + (page * ctx->page_size));
    } else {
        offset = static_cast<uint32>(addr + (page * ctx->page_size));
    }
    
    uint32 interrupts = ::save_and_disable_interrupts();
    if(transaction->active == false) {
        ::mutex_t *mutex = static_cast<::mutex_t*>(transaction->mutex);
        ::mutex_enter_blocking(mutex);
        transaction->active = true;

        const uint8 *nvm_data = reinterpret_cast<const uint8*>(XIP_BASE + offset);
        std::memcpy(dst, nvm_data, len);

        transaction->active = false;
        ::mutex_exit(mutex);
    } else {
        const uint8 *nvm_data = reinterpret_cast<const uint8*>(XIP_BASE + offset);
        std::memcpy(dst, nvm_data, len);
    }
    ::restore_interrupts(interrupts);

    HTRACE("[FLASH  ] First byte    : 0x%02X", dst[0]);
    HTRACE("[FLASH  ] Last byte     : 0x%02X", dst[len - 1]);
    HTRACE("[FLASH  ] Storage offset: %d bytes", offset);
    HTRACE("[FLASH  ] Page number   : %d", page);

    HDEBUG("[FLASH  ] Read complete");

    ctx->status = hkk::storage::nvm::NVM_OK;
    return ctx->status;
}

static int8 clear_sector_fn(void *ctx_raw, int32 offset, size_t sectors_number) {
    HTRACE("flash.cpp -> s:clear_sector_fn(void*, int32, size_t):int8");

    if(!ctx_raw) {
        HERROR("[FLASH  ] Null context passed to function");
        return hkk::storage::nvm::NVM_ERROR_NULL_CONTEXT;
    }
    auto *ctx = static_cast<hkk::storage::nvm::ConfigContext*>(ctx_raw);

    if(!ctx->transaction || !ctx->transaction->mutex) {
        HERROR("[FLASH  ] Null NVM mutex in context");

        ctx->status = hkk::storage::nvm::NVM_ERROR_NULL_MUTEX;
        return ctx->status;
    }
    auto *transaction = static_cast<hkk::storage::nvm::LockState*>(ctx->transaction);

    uint32 interrupts = ::save_and_disable_interrupts();
    size_t count = (static_cast<size_t>(ctx->sector_size) * sectors_number);

    HTRACE("[FLASH  ] Storage offset: %d bytes", offset);
    HTRACE("[FLASH  ] Sector size   : %d bytes", ctx->sector_size);
    HTRACE("[FLASH  ] Sectors number: %d", sectors_number);
    
    if(transaction->active == false) {
        ::mutex_t *mutex = static_cast<::mutex_t*>(transaction->mutex);
        ::mutex_enter_blocking(mutex);
        transaction->active = true;

        ::flash_range_erase(offset, count);

        transaction->active = false;
        ::mutex_exit(mutex);
    } else {
        ::flash_range_erase(offset, count);
    }
    ::restore_interrupts(interrupts);

    HDEBUG("[FLASH  ] Clear sector complete");

    ctx->status = hkk::storage::nvm::NVM_OK;
    return ctx->status;
}


static hkk::storage::nvm::BackendTable backend {
    .init_fn = init_fn,
    .deinit_fn = deinit_fn,

    .clear_sector_fn = clear_sector_fn,

    .write_blocking_fn = write_blocking_fn,
    .read_blocking_fn  = read_blocking_fn,
};

}


namespace hkk::storage::flash {

void bind(nvm::NVM &nvm, nvm::ConfigContext &cfg) {
    HTRACE("flash.cpp -> bind(NVM&, ConfigContext&):void");
    bind(nvm, cfg, hkk::rp2350::flash::backend);
}

}
