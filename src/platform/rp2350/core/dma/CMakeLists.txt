add_library(hkk_core_dma_rp2350 
    STATIC EXCLUDE_FROM_ALL
        dma.cpp
)

add_library(hkk::rp2350::core::dma ALIAS hkk_core_dma_rp2350)

target_include_directories(hkk_core_dma_rp2350 
    PUBLIC
        ${HKK_IOT_DRIVERS_ROOT}/include
)

target_link_libraries(hkk_core_dma_rp2350 
    PUBLIC
        hardware_uart
        hardware_dma
        hardware_gpio
)