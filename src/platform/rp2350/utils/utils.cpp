#include <hkk/defines.h>
#include <hkk/utils/utils.hpp>

#include <hkk/logger/logger.h>

#include <pico/stdlib.h>
#include <hardware/gpio.h>

namespace hkk::utils {

void sleep_us(uint64 us) {::sleep_us(us);}
void sleep_ms(uint64 ms) {::sleep_ms(ms);}


static bool repeating_timer_adapter(::repeating_timer_t *rt) {
    HTRACE("utils.cpp -> s:repeating_timer_adapter(::repeating_timer_t*):bool");
    
    if(!rt) {
        HERROR("[UTILS  ] Null context passed to function");
        return false;
    }
    auto *ctx = static_cast<TimerContext*>(rt->user_data);

    if(!ctx || !ctx->callback) return false;
    return ctx->callback(ctx->data);
}

bool repeating_timer_us(int64 us, TimerContext *ctx) {
    HTRACE("utils.cpp -> repeating_timer_us(int64, TimerContext*):bool");

    if(!ctx) return false;
    if(!ctx->timer) return false;
    return ::add_repeating_timer_us(us, repeating_timer_adapter, ctx, static_cast<::repeating_timer_t*>(ctx->timer));
}

bool repeating_timer_ms(int64 ms, TimerContext *ctx) {
    HTRACE("utils.cpp -> repeating_timer_ms(int64, TimerContext*):bool");
    
    if(!ctx) return false;
    if(!ctx->timer) return false;
    return ::add_repeating_timer_ms(ms, repeating_timer_adapter, ctx, static_cast<::repeating_timer_t*>(ctx->timer));
}


static int64 alarm_adapter(::alarm_id_t id, void *user_data) {
    HTRACE("utils.cpp -> s:alarm_adapter(::alarm_id_t, void*):int64");
    
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