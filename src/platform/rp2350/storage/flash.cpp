#include <hkk/defines.h>

#include <hkk/logger/logger.h>
#include <hkk/storage/nvm.hpp>

#include <hardware/flash.h> 

#include <pico/mutex.h>

#include <cstring>

namespace hkk::rp2350::flash {

// TODO: move it to config file
inline constexpr uint32 FLASH_SECTOR_SIZE_B     = 4096;
inline constexpr uint32 FLASH_PAGE_SIZE_B       = 256;
inline constexpr uint32 FLASH_PAGES_PER_SECTOR  = (FLASH_SECTOR_SIZE_B / FLASH_PAGE_SIZE_B);
inline constexpr uint32 FLASH_SECTORS_NUMBER    = 1;
inline constexpr uint32 FLASH_STORAGE_OFFSET_B  = (PICO_FLASH_SIZE_BYTES - (FLASH_SECTORS_NUMBER * FLASH_SECTOR_SIZE_B));


static uint8 current_page = 0;


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
    // ctx->page_size          = 0;
    ctx->pages_per_sector   = 0;
    ctx->sectors_number     = 0;
    ctx->storage_offset     = 0;

    HINFO("[FLASH  ] NVM storage deinitialization successful");

    ctx->status = hkk::storage::nvm::NVM_OK;
    return ctx->status;
}

static int8 init_fn(void *ctx_raw, bool8 clear_data) {
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
    // ctx->page_size          = FLASH_PAGE_SIZE_B;
    ctx->pages_per_sector   = FLASH_PAGES_PER_SECTOR;
    ctx->sectors_number     = FLASH_SECTORS_NUMBER;
    ctx->storage_offset     = FLASH_STORAGE_OFFSET_B;

    HDEBUG("[FLASH  ] Sector size     : %d bytes", ctx->sector_size);
    // HDEBUG("[FLASH  ] Page size       : %d bytes", ctx->page_size);
    HDEBUG("[FLASH  ] Storage offset  : %d bytes", ctx->storage_offset);
    HDEBUG("[FLASH  ] Pages per sector: %d", ctx->pages_per_sector);
    HDEBUG("[FLASH  ] Sectors number  : %d", ctx->sectors_number);

    if(clear_data == true) {
        HWARN("[FLASH  ] Specified sector will be erased");
        ::flash_range_erase(ctx->storage_offset, ctx->sector_size);
    }

    HINFO("[FLASH  ] NVM storage initialization successful");
    
    ctx->status = hkk::storage::nvm::NVM_OK;
    return ctx->status;
}

static int8 write_blocking_fn(void *ctx_raw, uint8 addr, const uint8 *src, size_t len) {
    HTRACE("flash.cpp -> s:write_blocking_fn(void*, uint8, const uint8*, size_t):int32");

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


    uint8 payload[FLASH_PAGE_SIZE_B];
    std::memset(payload, 0xFF, FLASH_PAGE_SIZE_B);

    if(len > 255) {
        HWARN("[FLASH  ] Data length exceeds page size; truncating to %u bytes", FLASH_PAGE_SIZE_B);
        
        std::memcpy(payload, src, FLASH_PAGE_SIZE_B);
        ctx->status = hkk::storage::nvm::NVM_DATA_TRUNCATED;
    } else {
        std::memcpy(payload, src, len);
        ctx->status = hkk::storage::nvm::NVM_OK;
    }


    if(!ctx->transaction || !ctx->transaction->mutex) {
        HERROR("[FLASH  ] Null I2C mutex in context");

        ctx->status = hkk::storage::nvm::NVM_ERROR_NULL_MUTEX;
        return static_cast<int32>(ctx->status);
    }
    auto *transaction = static_cast<hkk::storage::nvm::LockState*>(ctx->transaction);

    if(current_page >= FLASH_PAGES_PER_SECTOR) current_page = 0;
    
    uint32 offset = (ctx->storage_offset + (current_page * FLASH_PAGE_SIZE_B));
    if(transaction->active == false) {
        ::mutex_t *mutex = static_cast<::mutex_t*>(transaction->mutex);
        ::mutex_enter_blocking(mutex);
        transaction->active = true;

        ::flash_range_program(offset, src, FLASH_PAGE_SIZE_B);
        current_page++;

        transaction->active = false;
        ::mutex_exit(mutex);
    } else {
        ::flash_range_program(offset, src, FLASH_PAGE_SIZE_B);
        current_page++;
    }

    HTRACE("[FLASH  ] First byte   : 0x%02X", payload[0]);
    HTRACE("[FLASH  ] Last byte    : 0x%02X", payload[FLASH_PAGE_SIZE_B - 1]);

    HINFO("[FLASH  ] Write completed successfully");

    ctx->status = hkk::storage::nvm::NVM_OK;
    return ctx->status;
}




static hkk::storage::nvm::BackendTable backend {
    .init_fn = init_fn,
    .deinit_fn = deinit_fn,

    // .clear_sector_fn = clear_sector_fn,

    .write_blocking_fn = write_blocking_fn,
    // .read_blocking_fn = read_blocking_fn,
};

}


namespace hkk::storage::nvm {

void bind(NVM &nvm, ConfigContext &cfg) {
    HTRACE("i2c.cpp -> bind(NVM&, ConfigContext&):void");
    bind(nvm, cfg, hkk::rp2350::flash::backend);
}

}
