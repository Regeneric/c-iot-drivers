add_library(hkk_bus_uart_rp2350 
    STATIC EXCLUDE_FROM_ALL
        uart.cpp
)

add_library(hkk::rp2350::bus::uart ALIAS hkk_bus_uart_rp2350)

target_include_directories(hkk_bus_uart_rp2350 
    PUBLIC
        ${HKK_IOT_DRIVERS_ROOT}/include
)

target_link_libraries(hkk_bus_uart_rp2350 
    PUBLIC
        hardware_uart
        hardware_dma
        hardware_gpio
)