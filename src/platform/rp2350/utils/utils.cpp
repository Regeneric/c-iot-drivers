#include <hkk/defines.h>
#include <hkk/utils/utils.hpp>

#include <pico/stdlib.h>
#include <hardware/gpio.h>

namespace hkk::utils {

void sleep_us(uint64 us) {::sleep_us(us);}
void sleep_ms(uint64 ms) {::sleep_ms(ms);}

bool8 repeating_timer_us(int64 us, void *callback, void *data) {
    // static repeating_timer_t rt;
    // return ::add_repeating_timer_us(us, static_cast<repeating_timer_callback_t>(callback), data, &rt);
    return true;
}

bool8 repeating_timer_ms(int64 ms, void *callback, void *data) {
    // static repeating_timer_t rt;
    // return ::add_repeating_timer_ms(ms, static_cast<repeating_timer_callback_t>(callback), data, &rt);
    return true;
}

}