# src/drivers/bme280/CMakeLists.txt

add_library(hkk_drivers_bme280 
    STATIC EXCLUDE_FROM_ALL
        bme280.cpp
        commands.cpp
        utils.cpp
)

add_library(hkk::drivers::bme280 ALIAS hkk_drivers_bme280)

target_include_directories(hkk_drivers_bme280
    PUBLIC
        ${HKK_IOT_DRIVERS_ROOT}/include
)