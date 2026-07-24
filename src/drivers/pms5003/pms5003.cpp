#include <hkk/defines.h>

#include <hkk/utils/utils.hpp>
#include <hkk/bus/uart/uart.hpp>

#include <hkk/drivers/pms5003/pms5003.hpp>

#include <cmath>

namespace hkk::pms5003{

int8 PMS5003::setup(void) {
    HTRACE("pms5003.cpp -> PMS5003::setup(-):int8");
    return this->setup(this->ctx);
}

int8 PMS5003::setup(Context &res) {
    HTRACE("pms5003.cpp -> PMS5003::setup(Context&):int8");
    if(int8 status = this->sensor_enabled(); status < PMS5003_OK) return res.status = status;

    int8 status = PMS5003_OK;

    res.operation_mode = this->cfg.operation_mode;
    res.sleep_mode = this->cfg.sleep_mode;

    status = this->sleep(res.sleep_mode);
    if(status < PMS5003_OK) {
        HERROR("[PMS5003] Sensor sleep mode change fail");
        
        this->cfg.enable = false;
        return res.status = status;
    }

    status = this->mode(res.operation_mode);
    if(status < PMS5003_OK) {
        HERROR("[PMS5003] Sensor operation mode change fail");

        this->cfg.enable = false;
        return res.status = status;
    }

    HINFO ("[PMS5003] Sensor initialized");
    return res.status = status;
}

int8 PMS5003::process(void) {
    HTRACE("pms5003.cpp -> PMS5003::process(-):int8");
    return this->process(this->ctx);
}

int8 PMS5003::process(Context &res) {
    HTRACE("pms5003.cpp -> PMS5003::process(Context&):int8");
    if(int8 status = this->sensor_enabled(); status < PMS5003_OK) return res.status = status;

    int8 status = PMS5003_OK;

    status = this->measure(res);
    if(status < PMS5003_OK) return res.status = status;

    // if(this->cfg.compensate_humidity == true) {
    //     status = this->compensate_humidity();
    // }

    return res.status = status;
}

int8 PMS5003::send_command(Command command, uint8 value) {
    HTRACE("pms5003.cpp -> send_command(Command):int8");
    if(int8 status = this->sensor_enabled(); status < PMS5003_OK) return status;

    int8 status = PMS5003_OK;

    uint16 crc = 0;
    uint8 payload[COMMAND_FRAME_LENGTH] = {START_BYTE0, START_BYTE1, command, 0x00, value};

    status = PMS5003::checksum_calculate(crc, payload, COMMAND_FRAME_LENGTH);
    if(status < PMS5003_OK) {
        HERROR("[PMS5003] Could not calculate data payload checksum");
        return status;
    }

    payload[COMMAND_FRAME_LENGTH - 2] = hkk::utils::msb(crc);
    payload[COMMAND_FRAME_LENGTH - 1] = hkk::utils::lsb(crc);

    status = uart.write(payload);
    if(status < hkk::bus::uart::UART_OK) return status;

    HTRACE("[PMS5003] Command: 0x%02X", command);
    return status;
}

int8 PMS5003::read_raw_data(uint8 *data, size_t len) {
    HTRACE("pms5003.cpp -> PMS5003::read_raw_data(uint8*, size_t):int8");
    if(int8 status = this->sensor_enabled(); status < PMS5003_OK) return status;

    int32 status = PMS5003_OK;

    if(!data) {
        HERROR("[PMS5003] Null data pointer passed to function");
        return PMS5003_ERROR_NULL_DATA;
    }

    if(len == 0) {
        HERROR("[PMS5003] Data length is 0");
        return PMS5003_ERROR_ZERO_LENGTH;
    }

    status = uart.read(data, len);
    if(status < PMS5003_OK) return this->validate_uart_error(status);

    return (status > PMS5003_OK) ? PMS5003_OK : static_cast<int8>(status);
}
 
int8 PMS5003::compensate_humidity(Context &res, float64 humidity, float64 temperature) {
    HTRACE("pms5003.cpp -> PMS5003::compensate_humidity(Context&, float64):int8");
    if(int8 status = this->sensor_enabled(); status < PMS5003_OK) return res.status = status;

    // TODO: it's not that easy to compensate atmospheric values with a simple math formula...

    int8 status = PMS5003_OK;

    // float32 hygroscopicity = 0.25f;
    // TODO: i need real time clock
    // Winter-autumn: K ~ 0.05-0.10
    // Spring-summer: K ~ 0.15-0.25
    // switch(month) {
    //     case 12:  case 1:  case 2: hygroscopicity = 0.08f; break;
    //     case  3:  case 4:  case 5: hygroscopicity = 0.13f; break;
    //     case  6:  case 7:  case 8: hygroscopicity = 0.25f; break;
    //     case  9: case 10: case 11: hygroscopicity = 0.15f; break;
    //     default: hygroscopicity = 0.20f; break; 
    // }

    // bool8 pm10_compensate = true;
    // if(humidity <= 10.0) {
    //     HTRACE("[PMS5003] Humidity is less than 10 %RH. Data will not be compensated");
    //     return res.status = status;
    // }
    // if(humidity <= 50.0) {
    //     HTRACE("[PMS5003] Humidity is less than 50 %RH. PM10 will not be compensated");
    //     pm10_compensate = false;
    // }
    // if(humidity >= 85.0) {
    //     HTRACE("[PMS5003] Humidity is more than 85 %RH. PM10 will not be compensated");
    //     pm10_compensate = false;
    // }
    // if(humidity >= 90.0) {
    //     HTRACE("[PMS5003] Humidity is more than 90 %RH. Data will not be compensated");
    //     return res.status = status;
    // }


    // if(pm10_compensate == true) {
    //     float32 relative_humidity = (humidity / 100.0f);
    //     float32 grow_factor = std::cbrtf(1.0f + hygroscopicity * (relative_humidity / (1 - relative_humidity)));  // aka grow_factor == value^1/3

    //     float32 pm10     = (static_cast<float32>(res.pm10)     / grow_factor);
    //     float32 pm10_ref = (static_cast<float32>(res.pm10_ref) / grow_factor);

    //     res.pm10     = static_cast<uint16>(pm10     + ROUNDING_BIAS);
    //     res.pm10_ref = static_cast<uint16>(pm10_ref + ROUNDING_BIAS);
    // }


    // float32 pm1       = ((PM_GAIN * static_cast<float32>(res.pm1))     - (RH_GAIN * humidity) + (TEMP_GAIN * temperature) + PM_OFFSET);
    // float32 pm1_ref   = ((PM_GAIN * static_cast<float32>(res.pm1_ref)) - (RH_GAIN * humidity) + (TEMP_GAIN * temperature) + PM_OFFSET);

    // res.pm1       = static_cast<uint16>(pm1     + ROUNDING_BIAS);
    // res.pm1_ref   = static_cast<uint16>(pm1_ref + ROUNDING_BIAS);

    // Correction values based on https://www.mdpi.com/1424-8220/22/24/9669 and https://www.epa.gov/system/files/documents/2024-06/particulate-matter-pm2.5-sensor-loan-program-qapp-aasb-qapp-004-r1.1.pdf
    // float32 pm2_5     = ((PM_GAIN * static_cast<float32>(res.pm2_5))     - (RH_GAIN * humidity) + (TEMP_GAIN * temperature) + PM_OFFSET);
    // res.pm2_5     = static_cast<uint16>(pm2_5     + ROUNDING_BIAS);

    float32 pm2_5_ref = ((PM_GAIN * static_cast<float32>(res.pm2_5_ref)) - (RH_GAIN * humidity) + (TEMP_GAIN * temperature) + PM_OFFSET);
    res.pm2_5_ref = static_cast<uint16>(pm2_5_ref + ROUNDING_BIAS);

    return res.status = status;
}

// TODO:
int8 PMS5003::validate_uart_error(int8 status) {
    HTRACE("pms5003.cpp -> PMS5003::validate_uart_error(int8):int8");
    return PMS5003_OK;
}


uint16 PMS5003::pm1(bool8 ref) {
    HTRACE("pms5003.cpp -> PMS5003::pm1(bool8 = false):uint16");
    return ref ? this->ctx.pm1_ref : this->ctx.pm1;
}

void PMS5003::pm1(Context &res, bool8 ref) {
    HTRACE("pms5003.cpp -> PMS5003::pm1(Contex&, bool8 = false):void");
    
    if(ref) res.pm1_ref = this->ctx.pm1_ref;
    else res.pm1 = this->ctx.pm1;
}

uint16 PMS5003::pm2_5(bool8 ref) {
    HTRACE("pms5003.cpp -> PMS5003::pm2_5(bool8 = false):uint16");
    return ref ? this->ctx.pm2_5_ref : this->ctx.pm2_5;
}

void PMS5003::pm2_5(Context &res, bool8 ref) {
    HTRACE("pms5003.cpp -> PMS5003::pm2_5(Contex&, bool8 = false):void");
    
    if(ref) res.pm2_5_ref = this->ctx.pm2_5_ref;
    else res.pm2_5 = this->ctx.pm2_5;
}

uint16 PMS5003::pm10(bool8 ref) {
    HTRACE("pms5003.cpp -> PMS5003::pm10(bool8 = false):uint16");
    return ref ? this->ctx.pm10_ref : this->ctx.pm10;
}

void PMS5003::pm10(Context &res, bool8 ref) {
    HTRACE("pms5003.cpp -> PMS5003::pm10(Contex&, bool8 = false):void");
    
    if(ref) res.pm10_ref = this->ctx.pm10_ref;
    else res.pm10 = this->ctx.pm10;
}

}