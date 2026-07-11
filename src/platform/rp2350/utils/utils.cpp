#include <hkk/defines.h>
#include <hkk/utils/utils.hpp>

#include <hkk/logger/logger.h>

#include <pico/stdlib.h>
#include <hardware/gpio.h>

namespace hkk::utils {

void sleep_us(uint64 us) {::sleep_us(us);}
void sleep_ms(uint64 ms) {::sleep_ms(ms);}

bool8 repeating_timer_us(int64 us, void *callback, void *data) {
    HTRACE("utils.cpp -> repeating_timer_us(int64, void*, void*):bool8");
    
    static repeating_timer_t rt;
    auto cb = static_cast<repeating_timer_callback_t>(callback);

    return ::add_repeating_timer_us(us, cb, data, &rt);

}

bool8 repeating_timer_ms(int64 ms, void *callback, void *data) {
    HTRACE("utils.cpp -> repeating_timer_ms(int64, void*, void*):bool8");

    static repeating_timer_t rt;
    auto cb = static_cast<repeating_timer_callback_t>(callback);

    return ::add_repeating_timer_ms(ms, cb, data, &rt);
}

bool8 alarm_us(uint32 us, void *callback, void *data, bool8 fire_if_past = true) {
    HTRACE("utils.cpp -> alarm_us(uint32, void*, void*, bool8 = true):int8");

    auto cb = static_cast<alarm_callback_t>(callback);
    return static_cast<int8>(::add_alarm_in_us(us, cb, data, fire_if_past));
}

int8 alarm_ms(uint32 ms, void *callback, void *data, bool8 fire_if_past = true) {
    HTRACE("utils.cpp -> alarm_ms(uint32, void*, void*, bool8 = true):int8");

    auto cb = static_cast<alarm_callback_t>(callback);
    return static_cast<int8>(::add_alarm_in_ms(ms, cb, data, fire_if_past));
}

}