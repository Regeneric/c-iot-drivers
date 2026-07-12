add_library(hkk_nvm_rp2350 
    STATIC EXCLUDE_FROM_ALL
        nvm.cpp
        flash.cpp
)

add_library(hkk::rp2350::nvm ALIAS hkk_nvm_rp2350)

target_include_directories(hkk_nvm_rp2350 
    PUBLIC
        ${HKK_IOT_DRIVERS_ROOT}/include
)

target_link_libraries(hkk_nvm_rp2350 
    PUBLIC
        hardware_i2c
        hardware_gpio
        hardware_flash
)