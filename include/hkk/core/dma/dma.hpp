#pragma once

#include <hkk/defines.h>

namespace hkk::core::dma {

enum Result : int8 {
    DMA_OK                       =  0,
    DMA_ERROR_NULL_CONTEXT       = -1,
    DMA_ERROR_NULL_INSTANCE      = -2,
    DMA_ERROR_NULL_DATA          = -3,
    DMA_ERROR_ZERO_LENGTH        = -4,
    DMA_ERROR_NO_ACK             = -5,
    DMA_ERROR_PARTIAL_WRITE      = -6,
    DMA_ERROR_WRITE_FAILED       = -7,
    DMA_ERROR_PARTIAL_READ       = -8,
    DMA_ERROR_READ_FAILED        = -9,  
    DMA_ERROR_NOT_SUPPORTED      = -10, 
    DMA_ERROR_TIMEOUT            = -11,
    DMA_ERROR_NULL_MUTEX         = -12,
    DMA_ERROR_BUSY               = -13,
    DMA_DMA_ERROR_GENERIC        = -14,
    DMA_ERROR_NULL_DMA_CONTEXT   = -15,
    DMA_ERROR_GENERIC            = -100,
    DMA_FUNCTION_NOT_IMPLEMENTED = -101,
    DMA_ERROR_UNKNOWN            = -102,
};


int8 init_channel(void *instance, int32 channel);

}