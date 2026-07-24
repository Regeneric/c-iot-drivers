add_library(hkk_bus_owi_rp2350 
    STATIC EXCLUDE_FROM_ALL
        owi.cpp
        owi.pio
)

add_library(hkk::rp2350::bus::owi ALIAS hkk_bus_owi_rp2350)

pico_generate_pio_header(hkk_bus_owi_rp2350
    ${CMAKE_CURRENT_LIST_DIR}/owi.pio
    OUTPUT_DIR ${HKK_IOT_DRIVERS_ROOT}/src/platform/rp2350/bus/owi
)

target_include_directories(hkk_bus_owi_rp2350 
    PUBLIC
        ${HKK_IOT_DRIVERS_ROOT}/include
)

target_link_libraries(hkk_bus_owi_rp2350 
    PUBLIC
        hardware_pio
        hardware_gpio
        hardware_clocks
        pico_sync
)