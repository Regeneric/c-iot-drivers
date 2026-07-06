add_library(hkk_bus_i2c_rp2350 STATIC EXCLUDE_FROM_ALL
    i2c.cpp
)

add_library(hkk::bus_i2c_rp2350 ALIAS hkk_bus_i2c_rp2350)

target_include_directories(hkk_bus_i2c_rp2350 PUBLIC
    ${HKK_IOT_DRIVERS_ROOT}/include
)

target_link_libraries(hkk_bus_i2c_rp2350 PUBLIC
    hardware_i2c
    hardware_gpio
)