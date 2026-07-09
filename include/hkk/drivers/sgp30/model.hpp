#pragma once
#include <hkk/defines.h>

namespace hkk::sgp30 {

struct Config {
    bool8       enable; 
    bool8       humid_compensation;
    bool8       use_dht;
    bool8       use_bme;
    uint8       address;
    const char  *name;
    const char  *location;
};

}