#include <hkk/defines.h>

#include <hkk/utils/utils.hpp>
#include <hkk/logger/logger.h>
#include <hkk/bus/i2c/i2c.hpp>

#include <hkk/drivers/sgp30/sgp30.hpp>

namespace hkk::sgp30 {

int8 SGP30::iaq_init() {
    HTRACE("commands.cpp -> SGP30::iaq_init(-):int8");

    int8 status = SGP30::send_command(IaqInit);
    if(status < SGP30_OK) return status;

    hkk::utils::sleep_ms(10);
}

}