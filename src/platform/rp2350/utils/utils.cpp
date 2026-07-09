#include <hkk/defines.h>
#include <hkk/utils/utils.hpp>

#include <pico/stdlib.h>

namespace hkk::utils {

void sleep_us(uint64 us) {::sleep_us(us);}
void sleep_ms(uint64 ms) {::sleep_us(ms);}

}