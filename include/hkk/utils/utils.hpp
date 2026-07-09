#pragma once
#include <hkk/defines.h>

namespace hkk::utils {

constexpr uint8 msb(const uint16 cmd) {
    return static_cast<uint8>(static_cast<uint16>(cmd) >> 8);
}

constexpr uint8 lsb(const uint16 cmd) {
    return static_cast<uint8>(static_cast<uint16>(cmd) & 0xFF);
}

void sleep_us(uint32 us);
void sleep_ms(uint32 ms);

};