#include <hkk/defines.h>

#include <hkk/utils/utils.hpp>
#include <hkk/bus/uart/uart.hpp>

#include <hkk/drivers/pms5003/pms5003.hpp>

#include <hardware/gpio.h>

#include <cstring>

namespace hkk::pms5003 {

int8 PMS5003::measure(void) {
    HTRACE("commands.cpp -> PMS5003::measure(-):int8");
    return this->measure(this->ctx);
}

int8 PMS5003::measure(Context &res) {
    HTRACE("commands.cpp -> PMS5003::measure(Context&):int8");
    if(int8 status = this->sensor_enabled(); status < PMS5003_OK) return res.status = status;

    int8 status = PMS5003_OK;

    if(res.operation_mode == Mode::Passive) {
        status = this->send_command(Command::Read, 0x00);
        if(status < PMS5003_OK) {
            HERROR("[PMS5003] Could not send measure command");
            return res.status = status;
        }
    }

    status = this->read_raw_data(res.raw_data, DATA_FRAME_LENGTH);
    if(status < PMS5003_OK) {
        HERROR("[PMS5003] Read sensor data fail");
        return res.status = status;
    }

    status = PMS5003::checksum_validate(res.raw_data, DATA_FRAME_LENGTH);
    if(status < PMS5003_OK) {
        HERROR("[PMS5003] Read sensor data fail");
        return res.status = status;
    }

    res.pm1_ref   = ((res.raw_data[4] << 8) | res.raw_data[5]);
    res.pm2_5_ref = ((res.raw_data[6] << 8) | res.raw_data[7]);
    res.pm10_ref  = ((res.raw_data[8] << 8) | res.raw_data[9]);

    res.pm1       = ((res.raw_data[10] << 8) | res.raw_data[11]);
    res.pm2_5     = ((res.raw_data[12] << 8) | res.raw_data[13]);
    res.pm10      = ((res.raw_data[14] << 8) | res.raw_data[15]);

    std::memcpy(res.raw_pc_0_3,  (res.raw_data + 16), SECTION_FRAME_LENGTH);
    std::memcpy(res.raw_pc_0_5,  (res.raw_data + 18), SECTION_FRAME_LENGTH);
    std::memcpy(res.raw_pc_1_0,  (res.raw_data + 20), SECTION_FRAME_LENGTH);
    std::memcpy(res.raw_pc_2_5,  (res.raw_data + 22), SECTION_FRAME_LENGTH);
    std::memcpy(res.raw_pc_5_0,  (res.raw_data + 24), SECTION_FRAME_LENGTH);
    std::memcpy(res.raw_pc_10_0, (res.raw_data + 26), SECTION_FRAME_LENGTH);

    return res.status = status;
}

int8 PMS5003::mode(const Mode mode) {
    HTRACE("commands.cpp -> PMS5003::mode(-):int8");
    
    this->ctx.operation_mode = mode;
    return this->mode(this->ctx);
}

int8 PMS5003::mode(Context &res) {
    HTRACE("commands.cpp -> PMS5003::mode(Context&):int8");
    if(int8 status = this->sensor_enabled(); status < PMS5003_OK) return res.status = status;

    int8 status = PMS5003_OK;

    status = this->send_command(Command::ChangeMode, res.operation_mode);
    if(status < PMS5003_OK) {
        HERROR("[PMS5003] Could not change sensor operation mode");
        HDEBUG("[PMS5003] Mode : 0x%02X", Command::ChangeMode);
        HDEBUG("[PMS5003] Value: 0x%02X", res.operation_mode);
    } 

    HDEBUG("[PMS5003] Operation mode set to 0x%02X", res.operation_mode);
    return res.status = status;
}
    
int8 PMS5003::sleep(const Power mode) {
    HTRACE("commands.cpp -> PMS5003::sleep(-):int8");

    this->ctx.sleep_mode = mode;
    return this->sleep(this->ctx);
}

int8 PMS5003::sleep(Context &res) {
    HTRACE("commands.cpp -> PMS5003::sleep(Context&):int8");
    if(int8 status = this->sensor_enabled(); status < PMS5003_OK) return res.status = status;

    int8 status = PMS5003_OK;

    status = this->send_command(Command::SetSleep, res.sleep_mode);
    if(status < PMS5003_OK) {
        HERROR("[PMS5003] Could not change sensor power mode");
        HDEBUG("[PMS5003] Mode : 0x%02X", Command::SetSleep);
        HDEBUG("[PMS5003] Value: 0x%02X", res.sleep_mode);
    } 

    HDEBUG("[PMS5003] Power mode set to 0x%02X", res.sleep_mode);
    return res.status = status;
}

int8 PMS5003::hard_reset(void) {
    HTRACE("commands.cpp -> PMS5003:hard_reset(-):int8");
    return this->hard_reset(this->ctx);
}

int8 PMS5003::hard_reset(Context &res) {
    HTRACE("commands.cpp -> PMS5003:hard_reset(-):int8");
    if(int8 status = this->sensor_enabled(); status < PMS5003_OK) return res.status = status;

    if(this->cfg.gpio_reset < 0) {
        HWARN("[PMS5003] No reset GPIO pin defined");
        return res.status = PMS5003_ERROR_GENERIC;
    }

    constexpr uint8 GPIO_LOW  = 0;
    constexpr uint8 GPIO_HIGH = 1;

    ::gpio_put(this->cfg.gpio_reset, GPIO_LOW);
    hkk::utils::sleep_ms(100);
    ::gpio_put(this->cfg.gpio_reset, GPIO_HIGH);

    HINFO("[PMS5003] Hard reset success");
    return PMS5003_OK;
}

}