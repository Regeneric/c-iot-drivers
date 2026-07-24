#include <hkk/bus/uart/uart.hpp>

#include <hardware/uart.h>
#include <hardware/dma.h>
#include <hardware/irq.h>

#include <cstring>

namespace hkk::rp2350::uart {

static hkk::bus::uart::DMAContext *rx_dma_owner[NUM_DMA_CHANNELS] {};

static void dma_irq_handler(uint8 irq_index) {
    HWARN("DMA UART ISR %d", irq_index);

    for(size_t channel = 0; channel < NUM_DMA_CHANNELS; ++channel) {
        if(!::dma_irqn_get_channel_status(irq_index, channel)) continue;

        ::dma_irqn_acknowledge_channel(irq_index, channel);

        auto *dma = static_cast<hkk::bus::uart::DMAContext*>(rx_dma_owner[channel]);
        if(!dma || dma->irq_index != irq_index) continue;

        dma->rx_active = false;
    }
}

static void dma_irq0_handler(void) {dma_irq_handler(0);}
static void dma_irq1_handler(void) {dma_irq_handler(1);}
static void dma_irq2_handler(void) {dma_irq_handler(2);}
static void dma_irq3_handler(void) {dma_irq_handler(3);}


int8 init_dma_fn(void *ctx_raw) {
    HTRACE("dma.cpp -> s:init_dma_fn(void*):int8");

    if(!ctx_raw) {
        HERROR("[UART   ] Null context passed to function");
        return hkk::bus::uart::UART_ERROR_NULL_CONTEXT;
    }
    auto *ctx = static_cast<hkk::bus::uart::ConfigContext*>(ctx_raw);

    if(!ctx->instance) {
        HERROR("[UART   ] Null UART instance in context");
        return ctx->status = hkk::bus::uart::UART_ERROR_NULL_INSTANCE;
    }
    ::uart_inst *instance = static_cast<::uart_inst*>(ctx->instance);

    if(!ctx->dma) {
        HERROR("[UART   ] Null DMA context passed to function");
        return ctx->status = hkk::bus::uart::UART_ERROR_NULL_DMA_CONTEXT;
    }
    auto *dma = static_cast<hkk::bus::uart::DMAContext*>(ctx->dma);


    if(dma->rx_channel != -1 || dma->tx_channel != -1) {
        HERROR("[UART   ] DMA already initialized");
        return ctx->status = hkk::bus::uart::UART_ERROR_DMA;
    }

    dma->rx_active = false;
    dma->tx_active = false;
    
    dma->rx_channel = ::dma_claim_unused_channel(false);
    dma->tx_channel = ::dma_claim_unused_channel(false);

    if(dma->rx_channel < 0 || dma->tx_channel < 0) {
        HWARN("[UART   ] No free DMA channels to claim");

        ctx->dma_enable = false;
        dma->rx_channel = -1;
        dma->tx_channel = -1;

        return ctx->status = hkk::bus::uart::UART_ERROR_DMA;
    }


    ctx->dma_enable = false;

    // RX
    ::dma_channel_config rx_dma_config = ::dma_channel_get_default_config(dma->rx_channel);

    ::channel_config_set_transfer_data_size(&rx_dma_config, DMA_SIZE_8);
    ::channel_config_set_read_increment(&rx_dma_config, false);                             // Read always from the same address aka UART address
    ::channel_config_set_write_increment(&rx_dma_config, true);                             // Write to buffer and increment address on every write
    ::channel_config_set_dreq(&rx_dma_config, ::uart_get_dreq_num(instance, false));        // Data request comes from UART itself
    ::channel_config_set_enable(&rx_dma_config, true);
    ::dma_channel_configure(dma->rx_channel, &rx_dma_config, nullptr, &::uart_get_hw(instance)->dr, 0, false);  // No internal write buffer for DMA
    
    switch(dma->irq) {
        case hkk::bus::uart::IRQ_0: 
            dma->irq_index = 0; 
            ::irq_set_exclusive_handler(DMA_IRQ_0, dma_irq0_handler);
            ::irq_set_enabled(DMA_IRQ_0, true);
        break;
        case hkk::bus::uart::IRQ_1:
            dma->irq_index = 1; 
            ::irq_set_exclusive_handler(DMA_IRQ_1, dma_irq1_handler);
            ::irq_set_enabled(DMA_IRQ_1, true);
        break;
        case hkk::bus::uart::IRQ_2:
            dma->irq_index = 2; 
            ::irq_set_exclusive_handler(DMA_IRQ_2, dma_irq2_handler);
            ::irq_set_enabled(DMA_IRQ_2, true);
        break;
        case hkk::bus::uart::IRQ_3:
            dma->irq_index = 3; 
            ::irq_set_exclusive_handler(DMA_IRQ_3, dma_irq3_handler);
            ::irq_set_enabled(DMA_IRQ_3, true);
        break;
        default: 
            HERROR("[UART   ] DMA index invalid: %d", dma->irq_index);
            HDEBUG("[UART   ] DMA_IRQ_NUM: %d", dma->irq);
            dma->irq_index = -1; 
        break;
    }
    ::dma_irqn_set_channel_enabled(dma->irq_index, dma->rx_channel, true);

    rx_dma_owner[dma->rx_channel] = dma;

    HDEBUG("[UART   ] RX channel: %d", dma->rx_channel);

    // TX
    // ::dma_channel_config tx_dma_config = ::dma_channel_get_default_config(dma->tx_channel);

    // ::channel_config_set_transfer_data_size(&tx_dma_config, DMA_SIZE_8);
    // ::channel_config_set_read_increment(&tx_dma_config, true);
    // ::channel_config_set_write_increment(&tx_dma_config, false);
    // ::channel_config_set_ring(&tx_dma_config, true, hkk::bus::uart::TX_BUFFER_SIZE_POW);
    // ::channel_config_set_dreq(&tx_dma_config, ::uart_get_dreq_num(instance, true));
    // ::dma_channel_set_config(dma->tx_channel, &tx_dma_config, false);
    // ::dma_channel_set_write_addr(dma->tx_channel, &uart_get_hw(instance)->dr, false);

    // ::dma_channel_start(dma->tx_channel);
    HDEBUG("[UART   ] TX channel: %d", dma->tx_channel);

    ctx->dma_enable = true;
    HDEBUG("[UART   ] IRQ index : %d", dma->irq_index);
    HINFO ("[UART   ] DMA initialized");

    return ctx->status = hkk::bus::uart::UART_OK;
}
    
}
