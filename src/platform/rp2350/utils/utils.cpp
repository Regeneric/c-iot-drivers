#include <hkk/defines.h>
#include <hkk/utils/utils.hpp>

#include <pico/stdlib.h>

namespace hkk::utils {

inline void sleep_us(uint32 us) {::sleep_us(us);}
inline void sleep_ms(uint32 ms) {::sleep_us(ms);}

}