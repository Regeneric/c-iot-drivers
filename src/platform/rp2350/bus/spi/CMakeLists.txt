add_library(hkk_bus_spi_rp2350 STATIC EXCLUDE_FROM_ALL
    spi.cpp
)

add_library(hkk::rp2350::bus_spi ALIAS hkk_bus_spi_rp2350)

target_include_directories(hkk_bus_spi_rp2350 PUBLIC
    ${HKK_IOT_DRIVERS_ROOT}/include
)

target_link_libraries(hkk_bus_spi_rp2350 PUBLIC
    hardware_spi
    hardware_gpio
)