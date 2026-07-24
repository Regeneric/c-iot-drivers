#include <hkk/defines.h>

#include <hkk/utils/utils.hpp>
#include <hkk/logger/logger.h>
#include <hkk/bus/i2c/i2c.hpp>

#include <hkk/drivers/bme280/bme280.hpp>

#include <cstring>

namespace hkk::bme280 {

int8 BME280::init(void) {
    HTRACE("commands.cpp -> BME280::init(-):int8");
    return this->init(this->ctx);
}

int8 BME280::init(Context &res) {
    HTRACE("commands.cpp -> BME280::init(Context&):int8");
    if(int8 status = this->sensor_enabled(); status < BME280_OK) return res.status = status;

    int8 status = BME280_OK;

    status = this->get_sensor_id(res);
    if(status < BME280_OK) {
        HERROR("[BME280 ] Could not get sensor ID; %s (%s)", this->cfg.name, this->cfg.location);
        return res.status = status;
    }

    status = this->soft_reset(res);
    if(status < BME280_OK) {
        HERROR("[BME280 ] Soft reset fail; %s (%s)", this->cfg.name, this->cfg.location);
        return res.status = status;
    }

    status = this->calibrate(res);
    if(status < BME280_OK) {
        HERROR("[BME280 ] Could not read sensor calbration data; %s (%s)", this->cfg.name, this->cfg.location);
        return res.status = status;
    }

    
    Register reg = Register::CtrlHum;
    uint8 value  = this->cfg.humidity_sampling;
    uint8 read   = 0x00;

    status = this->write_register(reg, value);
    if(status < BME280_OK) {
        HERROR("[BME280 ] Could not write sensor calbration data; %s (%s)", this->cfg.name, this->cfg.location);
        HDEBUG("[BME280 ] Register: 0x%02X", reg);
        HDEBUG("[BME280 ] Value   : 0x%02X", value);

        return res.status = status;
    }

    status = this->read_register(reg, read);
    if(status < BME280_OK || value != read) {
        HERROR("[BME280 ] Register value doesn't match; %s (%s)", this->cfg.name, this->cfg.location);
        HDEBUG("[BME280 ] Expected: 0x%02X", value);
        HDEBUG("[BME280 ] Got     : 0x%02X", read);

        return res.status = status;
    }

    HDEBUG("[BME280 ] Humidity oversampling set to: 0x%02X", read);


    reg   = Register::ConfigReg;
    value = this->cfg.iir_coefficient;
    read  = 0x00;

    status = this->write_register(reg, value);
    if(status < BME280_OK) {
        HERROR("[BME280 ] Could not write sensor calbration data; %s (%s)", this->cfg.name, this->cfg.location);
        HDEBUG("[BME280 ] Register: 0x%02X", reg);
        HDEBUG("[BME280 ] Value   : 0x%02X", value);

        return res.status = status;
    }

    status = this->read_register(reg, read);
    if(status < BME280_OK || value != read) {
        HERROR("[BME280 ] Register value doesn't match; %s (%s)", this->cfg.name, this->cfg.location);
        HDEBUG("[BME280 ] Expected: 0x%02X", value);
        HDEBUG("[BME280 ] Got     : 0x%02X", read);

        return res.status = status;
    }

    HDEBUG("[BME280 ] IIR Coefficient set to: 0x%02X", read);


    reg   = Register::CtrlMeas;
    value = this->cfg.temperature_pressure_mode;
    read  = 0x00;

    status = this->write_register(reg, value);
    if(status < BME280_OK) {
        HERROR("[BME280 ] Could not write sensor calbration data; %s (%s)", this->cfg.name, this->cfg.location);
        HDEBUG("[BME280 ] Register: 0x%02X", reg);
        HDEBUG("[BME280 ] Value   : 0x%02X", value);

        return res.status = status;
    }

    status = this->read_register(reg, read);
    if(status < BME280_OK || value != read) {
        HERROR("[BME280 ] Register value doesn't match; %s (%s)", this->cfg.name, this->cfg.location);
        HDEBUG("[BME280 ] Expected: 0x%02X", value);
        HDEBUG("[BME280 ] Got     : 0x%02X", read);

        return res.status = status;
    }

    HDEBUG("[BME280 ] Temperature and presure mode set to: 0x%02X", read);

    uint8 mode = (this->cfg.temperature_pressure_mode & OPERATON_MODE_MASK);
    switch(mode) {
        case Command::NormalMode:
            res.operation_mode = Command::NormalMode;
            HDEBUG("[BME280 ] Sensor operation mode: NORMAL");
        break;
        case Command::SleepMode:
            res.operation_mode = Command::SleepMode;
            HDEBUG("[BME280 ] Sensor operation mode: SLEEP");
        break;
        case Command::ForceMode:
            res.operation_mode = Command::ForceMode;
            HDEBUG("[BME280 ] Sensor operation mode: FORCE");
        break;
        default:
            res.operation_mode = Command::NormalMode;
            HWARN("[BME280 ] Unknown operation mode: 0x%02X", Command::NormalMode);
            HWARN("[BME280 ] Sensor operation mode: NORMAL");
        break;
    }

    HINFO ("[BME280 ] Sensor initialized; %s (%s)", this->cfg.name, this->cfg.location);
    if(i2c) {
        HDEBUG("[BME280 ] I2C bus: I2C%d", i2c->index());
        HDEBUG("[BME280 ] Address: 0x%02X", this->cfg.address);
    }

    return res.status = status;
}

int8 BME280::measure(void) {
    HTRACE("commands.cpp -> BME280::measure(-):int8");
    return this->measure(this->ctx);
}

int8 BME280::measure(Context &res) {
    HTRACE("commands.cpp -> BME280::measure(Context&):int8");
    if(int8 status = this->sensor_enabled(); status < BME280_OK) return res.status = status;

    int8 status = BME280_OK;

    // TODO: sleep mode operations
    if(res.operation_mode == Command::ForceMode) {
        Register reg = Register::CtrlMeas;
        uint8 value  = this->cfg.temperature_pressure_mode;

        status = this->write_register(reg, value);
        if(status < BME280_OK) {
            HERROR("[BME280 ] Could not init sensor measurement procedure; %s (%s)", this->cfg.name, this->cfg.location);
            HDEBUG("[BME280 ] Register: 0x%02X", reg);
            HDEBUG("[BME280 ] Value   : 0x%02X", value);

            return res.status = status;
        }
    }


    this->sensor_timeout = false;
    this->sensor_timeout_alarm.callback = BME280::sensor_timeout_callback;
    this->sensor_timeout_alarm.data = this;

    int32 alarm_id = hkk::utils::alarm_us(this->cfg.sensor_timeout_us, &this->sensor_timeout_alarm, true);
    HTRACE("[BME280 ] Sensor timeout alarm start");
    HTRACE("[BME280 ] ID     : %d", alarm_id);
    HTRACE("[BME280 ] Timeout: %d us", this->cfg.sensor_timeout_us);

    uint8 sensor_status = Status::Measuring;
    while((sensor_status & Status::Measuring) == Status::Measuring) {
        HTRACE("[BME280 ] Measurement in progress");

        if(this->sensor_timeout == true) {
            HERROR("[DME280 ] Sensor timeout while data read; %s (%s)", this->cfg.name, this->cfg.location);

            hkk::utils::cancel_alarm(alarm_id);
            return res.status = BME280_ERROR_TIMEOUT;
        }
        
        status = this->read_register(Register::Status, sensor_status);
        if(status < BME280_OK) {
            HERROR("[BME280 ] Could not read status register 0x%02X; %s (%s)", Register::Status, this->cfg.name, this->cfg.location);
            
            hkk::utils::cancel_alarm(alarm_id);
            return res.status = status;
        }

        hkk::utils::sleep_ms(1);
    }

    hkk::utils::cancel_alarm(alarm_id);


    uint8 sensor_raw_data[PRESSURE_DATA_FRAME_LENGTH + TEMPERATURE_DATA_FRAME_LENGTH + HUMIDITY_DATA_FRAME_LENGTH];
    status = read_register(Register::PressMsb, sensor_raw_data);
    if(status < BME280_OK) {
        HERROR("[BME280 ] Could not read sensor measurement data; %s (%s)", this->cfg.name, this->cfg.location);
        HDEBUG("[BME280 ] Register: 0x%02X", Register::PressMsb);

        return res.status = status;
    }

    std::memcpy(res.pressure_raw_data   , sensor_raw_data                                                               , PRESSURE_DATA_FRAME_LENGTH);
    std::memcpy(res.temperature_raw_data, (sensor_raw_data + PRESSURE_DATA_FRAME_LENGTH)                                , TEMPERATURE_DATA_FRAME_LENGTH);
    std::memcpy(res.humidity_raw_data   , (sensor_raw_data + PRESSURE_DATA_FRAME_LENGTH + TEMPERATURE_DATA_FRAME_LENGTH), HUMIDITY_DATA_FRAME_LENGTH);

    return res.status = status;
}

int8 BME280::calibrate(void) {
    HTRACE("commands.cpp -> BME280::calibrate(-):int8");
    return this->calibrate(this->ctx);
}

int8 BME280::calibrate(Context &res) {
    HTRACE("commands.cpp -> BME280::calibrate(Context &res):int8");
    if(int8 status =  this->sensor_enabled(); status < BME280_OK) return res.status = status;

    int8 status = BME280_OK;
    uint8 data[CALIB00_DATA_FRAME];

    status = this->read_register(Register::Calib00, data, CALIB00_DATA_FRAME);
    if(status < BME280_OK) {
        HERROR("[BME280 ] Could not read calibration register 0x%02X; %s (%s)", Register::Calib00, this->cfg.name, this->cfg.location);
        return res.status = status;
    }

    res.calibration.dig_T1 = ((data[1]   << 8) | data[0]);
    res.calibration.dig_T2 = ((data[3]   << 8) | data[2]);
    res.calibration.dig_T3 = ((data[5]   << 8) | data[4]);

    res.calibration.dig_P1 = ((data[7]   << 8) | data[6]);
    res.calibration.dig_P2 = ((data[9]   << 8) | data[8]);
    res.calibration.dig_P3 = ((data[11]  << 8) | data[10]);
    res.calibration.dig_P4 = ((data[13]  << 8) | data[12]);
    res.calibration.dig_P5 = ((data[15]  << 8) | data[14]);
    res.calibration.dig_P6 = ((data[17]  << 8) | data[16]);
    res.calibration.dig_P7 = ((data[19]  << 8) | data[18]);
    res.calibration.dig_P8 = ((data[21]  << 8) | data[20]);
    res.calibration.dig_P9 = ((data[23]  << 8) | data[22]);

    res.calibration.dig_H1 = data[25];

    status = this->read_register(Register::Calib26, data, CALIB26_DATA_FRAME);
    if(status < BME280_OK) {
        HERROR("[BME280 ] Could not read calibration register 0x%02X; %s (%s)", Register::Calib26, this->cfg.name, this->cfg.location);
        return res.status = status;
    }

    res.calibration.dig_H2 = ((data[1] << 8) | data[0]);
    res.calibration.dig_H3 =   data[2];
    res.calibration.dig_H4 = ((data[3] << 4) | data[4] & 0x0F); // H4 and H5 shares the same byte
    res.calibration.dig_H5 = ((data[5] << 4) | data[4] >> 4);   // H4 and H5 shares the same byte
    res.calibration.dig_H6 = static_cast<int8>(data[6]);

    HDEBUG("[BME280 ] Calibration process complete");
    return res.status = status;
}

int8 BME280::get_sensor_id(void) {
    HTRACE("commands.cpp -> BME280::get_sensor_id(-):int8");
    return this->get_sensor_id(this->ctx);
}

int8 BME280::get_sensor_id(Context &res) {
    HTRACE("commands.cpp -> BME280::get_sensor_id(Context&):int8");
    if(int8 status = this->sensor_enabled(); status < BME280_OK) return res.status = status;

    int8 status = BME280_OK;
    uint8 ready_status;

    status = this->read_register(Register::Id, ready_status);
    if(status < BME280_OK) {
        HERROR("[BME280 ] Could not read ID register 0x%02X", Register::Id);
        return res.status = status;
    }

    if(ready_status != Status::Ready) {
        HERROR("[BME280 ] ID register value doesn't match; %s (%s)", this->cfg.name, this->cfg.location); 
        HDEBUG("[BME280 ] Expected: 0x%02X", Status::Ready);
        HDEBUG("[BME280 ] Got     : 0x%02X", ready_status);

        return res.status = BME280_ERROR_GENERIC;
    }

    HDEBUG("[BME280 ] ID register value: 0x%02X", ready_status);
    return res.status = status;
}

int8 BME280::soft_reset(void) {
    HTRACE("commands.cpp -> soft_reset(-):int8");
    return this->soft_reset(this->ctx);
}

int8 BME280::soft_reset(Context &res) {
    HTRACE("commands.cpp -> soft_reset(Context&):int8");
    if(int8 status = this->sensor_enabled(); status < BME280_OK);

    int8 status = BME280_OK;

    status = this->write_register(Register::Reset, Command::SoftReset);
    if(status < BME280_OK) {
        HERROR("[BME280 ] Could not soft reset sensor using register 0x%02X", Register::Reset);
        return res.status = status;
    }

    uint8 reset_status = Status::ImUpdate;
    while((reset_status & Status::ImUpdate) == Status::ImUpdate) {
        HDEBUG("[BME280 ] Internal memory update in progress");
        
        status = this->read_register(Register::Status, reset_status);
        if(status < BME280_OK) {
            HERROR("[BME280 ] Could not read status register 0x%02X", Register::Status);
            return res.status = status;
        }

        hkk::utils::sleep_ms(1);
    }

    HDEBUG("[BME280 ] Soft reset complete");
    return res.status = status;
}


bool8 BME280::sensor_timeout_callback(void *data) {
    if(!data) return false;

    auto *self = static_cast<BME280*>(data);
    self->sensor_timeout = true;

    return false;
} 

}