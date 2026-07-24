#pragma once
#include <hkk/defines.h>

#include <hkk/utils/time.hpp>

namespace hkk::utils {

constexpr uint8 msb(const uint16 data) {
    return static_cast<uint8>(static_cast<uint16>(data) >> 8);
}

constexpr uint8 lsb(const uint16 data) {
    return static_cast<uint8>(static_cast<uint16>(data) & 0xFF);
}

void sleep_us(uint64 us);
void sleep_ms(uint64 ms);


using TimerCallback = bool8 (*)(void *);
struct TimerContext {
    TimerCallback callback;
    void *data;
    void *timer;
};

bool repeating_timer_us(int64 us, TimerContext *ctx);
bool repeating_timer_ms(int64 ms, TimerContext *ctx);
bool cancel_repeating_timer(TimerContext *ctx);


using AlarmCallback = bool8 (*)(void *);
struct AlarmContext {
    AlarmCallback callback;
    void *data;
};

int32 alarm_us(uint32 us, AlarmContext *ctx, bool8 fire_if_past = true);
int32 alarm_ms(uint32 ms, AlarmContext *ctx, bool8 fire_if_past = true);
bool cancel_alarm(int32 alarm_id);

};