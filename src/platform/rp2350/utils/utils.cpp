#include <hkk/defines.h>
#include <hkk/utils/utils.hpp>

#include <hkk/logger/logger.h>

#include <pico/stdlib.h>
#include <hardware/gpio.h>

namespace hkk::utils {

void sleep_us(uint64 us) {::sleep_us(us);}
void sleep_ms(uint64 ms) {::sleep_ms(ms);}

static repeating_timer_t rt1;
static repeating_timer_t rt2;

bool8 repeating_timer_us(int64 us, void *callback, void *data) {
    HTRACE("utils.cpp -> repeating_timer_us(int64, void*, void*):bool8");
    
    auto cb = (repeating_timer_callback_t)(callback);
    return ::add_repeating_timer_us(us, cb, data, &rt1);

}

bool8 repeating_timer_ms(int64 ms, void *callback, void *data) {
    HTRACE("utils.cpp -> repeating_timer_ms(int64, void*, void*):bool8");

    auto cb = (repeating_timer_callback_t)(callback);
    return ::add_repeating_timer_ms(ms, cb, data, &rt2);
}


static int64 alarm_adapter(alarm_id_t id, void *user_data) {
    HTRACE("utils.cpp -> alarm_adapter(alarm_id_t, void*):int64");
    
    if(!user_data) {
        HERROR("[UTILS  ] Null context passed to function");
        return 0;
    }
    auto *ctx = static_cast<AlarmContext *>(user_data);

    if(ctx && ctx->callback) ctx->callback(ctx->data);
    return 0;
}

int32 alarm_us(uint32 us, AlarmContext *ctx, bool8 fire_if_past) {
    HTRACE("utils.cpp -> alarm_us(uint32, AlarmContext*, void*, bool8 = true):int8");

    return static_cast<int32>(::add_alarm_in_ms(us, alarm_adapter, ctx, fire_if_past));
}

int32 alarm_ms(uint32 ms, AlarmContext *ctx, bool8 fire_if_past) {
    HTRACE("utils.cpp -> alarm_ms(uint32, AlarmContext*, void*, bool8 = true):int8");

    return static_cast<int32>(::add_alarm_in_ms(ms, alarm_adapter, ctx, fire_if_past));
}

}