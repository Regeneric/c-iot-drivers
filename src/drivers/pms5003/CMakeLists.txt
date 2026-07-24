# src/drivers/pms5003/CMakeLists.txt

add_library(hkk_drivers_pms5003 
    STATIC EXCLUDE_FROM_ALL
        pms5003.cpp
        commands.cpp
        utils.cpp
)

add_library(hkk::drivers::pms5003 ALIAS hkk_drivers_pms5003)

target_include_directories(hkk_drivers_pms5003
    PUBLIC
        ${HKK_IOT_DRIVERS_ROOT}/include
)

target_link_libraries(hkk_drivers_pms5003 
    PUBLIC
        hardware_gpio
)