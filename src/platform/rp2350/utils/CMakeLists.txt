add_library(hkk_utils_rp2350 
    STATIC EXCLUDE_FROM_ALL
        utils.cpp
)

add_library(hkk::rp2350::utils ALIAS hkk_utils_rp2350)

target_include_directories(hkk_utils_rp2350 
    PUBLIC
        ${HKK_IOT_DRIVERS_ROOT}/include
)

target_link_libraries(hkk_utils_rp2350 
    PUBLIC
        pico_stdlib
)