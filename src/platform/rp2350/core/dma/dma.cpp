#include <hkk/defines.h>

#include <hkk/logger/logger.h>

#include <hkk/core/dma/dma.hpp>

#include <hardware/dma.h>
#include <hardware/gpio.h>

namespace hkk::core::dma {

int8 init_channel(void *instance, int32 channel) {
    HTRACE("dma.cpp -> init_channel(int32):int8");

    if(!instance) {
        HERROR("[DMA    ] Null DMA instance in context");
        return DMA_ERROR_NULL_INSTANCE;
    }
}

}