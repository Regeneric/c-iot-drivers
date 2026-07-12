#include <hkk/defines.h>

#include <hkk/logger/logger.h>
#include <hkk/storage/nvm.hpp>

#include <hardware/flash.h> 

#include <pico/mutex.h>

namespace hkk::rp2350::flash {

// TODO: move it to config file
inline constexpr uint32 FLASH_SECTOR_SIZE_B     = 4096;
inline constexpr uint32 FLASH_PAGE_SIZE_B       = 256;
inline constexpr uint32 FLASH_PAGES_PER_SECTOR  = (FLASH_SECTOR_SIZE_B / FLASH_PAGE_SIZE_B);
inline constexpr uint32 FLASH_SECTORS_NUMBER    = 1;
inline constexpr uint32 FLASH_STORAGE_OFFSET_B  = (PICO_FLASH_SIZE_BYTES - (FLASH_SECTORS_NUMBER * FLASH_SECTOR_SIZE_B));


static int8 deinit_fn(void *ctx_raw) {
    HTRACE("flash.cpp -> s:deinit_fn(void*):int8"); 

    if(!ctx_raw) {
        HERROR("[FLASH  ] Null context passed to function");
        return hkk::storage::nvm::NVM_ERROR_NULL_CONTEXT;
    }
    auto *ctx = static_cast<hkk::storage::nvm::ConfigContext*>(ctx_raw);

    if(!ctx->transaction || !ctx->transaction->mutex) {
        HWARN("[FLASH  ] Null I2C mutex in context");
        ctx->status = hkk::storage::nvm::NVM_ERROR_NULL_MUTEX;
    } else {
        ::mutex_exit(static_cast<::mutex_t*>(ctx->transaction->mutex));
    }

    ctx->sector_size        = 0;
    ctx->page_size          = 0;
    ctx->pages_per_sector   = 0;
    ctx->sectors_number     = 0;
    ctx->storage_offset     = 0;

    HINFO("[FLASH  ] NVM storage deinitialization successful");

    ctx->status = hkk::storage::nvm::NVM_OK;
    return ctx->status;
}

static int8 init_fn(void *ctx_raw) {
    HTRACE("flash.cpp -> s:init_fn(void*):int8");

    if(!ctx_raw) {
        HERROR("[FLASH  ] Null context passed to function");
        return hkk::storage::nvm::NVM_ERROR_NULL_CONTEXT;
    }
    auto *ctx = static_cast<hkk::storage::nvm::ConfigContext*>(ctx_raw);

    if(!ctx->transaction || !ctx->transaction->mutex) {
        HERROR("[FLASH  ] Null I2C mutex in context");
        
        deinit_fn(ctx_raw);

        ctx->status = hkk::storage::nvm::NVM_ERROR_NULL_MUTEX;
        return ctx->status;
    }
    ::mutex_init(static_cast<::mutex_t*>(ctx->transaction->mutex));

    ctx->sector_size        = FLASH_SECTOR_SIZE_B;
    ctx->page_size          = FLASH_PAGE_SIZE_B;
    ctx->pages_per_sector   = FLASH_PAGES_PER_SECTOR;
    ctx->sectors_number     = FLASH_SECTORS_NUMBER;
    ctx->storage_offset     = FLASH_STORAGE_OFFSET_B;

    HDEBUG("[FLASH  ] Sector size     : %d bytes", ctx->sector_size);
    HDEBUG("[FLASH  ] Page size       : %d bytes", ctx->page_size);
    HDEBUG("[FLASH  ] Storage offset  : %d bytes", ctx->storage_offset);
    HDEBUG("[FLASH  ] Pages per sector: %d", ctx->pages_per_sector);
    HDEBUG("[FLASH  ] Sectors number  : %d", ctx->sectors_number);

    HINFO("[FLASH  ] NVM storage initialization successful");
    
    ctx->status = hkk::storage::nvm::NVM_OK;
    return ctx->status;
}

static int8 config_fn(void *ctx_raw) {
    HTRACE("flash.cpp -> s:config_fn(void*, void*):int8");
    return hkk::storage::nvm::NVM_OK;
}



static hkk::storage::nvm::BackendTable backend {
    .init_fn = init_fn,
    .deinit_fn = deinit_fn,

    // .clear_sector_fn = clear_sector_fn,

    // .write_blocking_fn = write_blocking_fn,
    // .read_blocking_fn = read_blocking_fn,
};

}


namespace hkk::storage::nvm {

void bind(NVM &nvm, ConfigContext &cfg) {
    HTRACE("i2c.cpp -> bind(NVM&, ConfigContext&):void");
    bind(nvm, cfg, hkk::rp2350::flash::backend);
}

}
