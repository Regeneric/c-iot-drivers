#pragma once
#include <hkk/defines.h>

#include <drivers/sgp30/device.hpp>

namespace hkk::sgp30 {

constexpr uint8 msb(const Command cmd) {
    return static_cast<uint8>(static_cast<uint16>(cmd) >> 8);
}

constexpr uint8 lsb(const Command cmd) {
    return static_cast<uint8>(static_cast<uint16>(cmd) & 0xFF);
}

};