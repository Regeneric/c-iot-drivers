add_library(hkk_drivers_sgp30 
    STATIC EXCLUDE_FROM_ALL
        sgp30.cpp
        commands.cpp
)

add_library(hkk::drivers::sgp30 ALIAS hkk_drivers_sgp30)

target_include_directories(hkk_drivers_sgp30 
    PUBLIC
        ${HKK_IOT_DRIVERS_ROOT}/include
)

# target_link_libraries(hkk_drivers_sgp30 
#     PUBLIC
#         hardware_i2c
#         hardware_gpio
# )