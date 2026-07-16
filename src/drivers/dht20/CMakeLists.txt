# src/drivers/dht20/CMakeLists.txt

add_library(hkk_drivers_dht20 
    STATIC EXCLUDE_FROM_ALL
        dht20.cpp
        commands.cpp
        utils.cpp
)

add_library(hkk::drivers::dht20 ALIAS hkk_drivers_dht20)

target_include_directories(hkk_drivers_dht20
    PUBLIC
        ${HKK_IOT_DRIVERS_ROOT}/include
)