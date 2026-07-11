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

bool8 repeating_timer_us(int64 us, void *callback, void *data);
bool8 repeating_timer_ms(int64 ms, void *callback, void *data);

int8 alarm_us(uint32 us, void *callback, void *data, bool8 fire_if_past = true);
int8 alarm_ms(uint32 ms, void *callback, void *data, bool8 fire_if_past = true);

};