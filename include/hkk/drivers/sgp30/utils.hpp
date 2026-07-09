#pragma once
#include <hkk/defines.h>


namespace hkk::sgp30 {

int8 crc_calculate(uint8 &checksum, uint8 *data, size_t len);

template <size_t N>
int8 crc_calculate(uint8 &checksum, uint8 (&data)[N]) {
    return crc_calculate(checksum, data, N);
}

template <size_t N>
int8 crc_calculate(uint8 (&data)[N]) {
    return crc_calculate(nullptr, data, N);
}


int8 crc_validate(uint8 *data, size_t len);

template <size_t N>
int8 crc_validate(uint8 (&data)[N]) {
    return crc_validate(data, N);
}

}