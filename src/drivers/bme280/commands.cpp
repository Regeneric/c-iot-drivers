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
        HERROR("[BME280 ] Could not get sensor ID");
        return res.status = status;
    }

    status = this->soft_reset(res);
    if(status < BME280_OK) {
        HERROR("[BME280 ] Soft reset fail");
        return res.status = status;
    }

    status = this->calibrate(res);
    if(status < BME280_OK) {
        HERROR("[BME280 ] Could not read sensor calbration data");
        return res.status = status;
    }

    
    Register reg = Register::CtrlHum;
    uint8 value  = this->cfg.humidity_sampling;
    uint8 read   = 0x00;

    status = this->write_register(reg, value);
    if(status < BME280_OK) {
        HERROR("[BME280 ] Could not write sensor calbration data");
        HDEBUG("[BME280 ] Register: 0x%02X", reg);
        HDEBUG("[BME280 ] Value   : 0x%02X", value);

        return res.status = status;
    }

    status = this->read_register(reg, read);
    if(status < BME280_OK) {
        HERROR("[BME280 ] Register value doesn't match");
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
        HERROR("[BME280 ] Could not write sensor calbration data");
        HDEBUG("[BME280 ] Register: 0x%02X", reg);
        HDEBUG("[BME280 ] Value   : 0x%02X", value);

        return res.status = status;
    }

    status = this->read_register(reg, read);
    if(status < BME280_OK) {
        HERROR("[BME280 ] Register value doesn't match");
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
        HERROR("[BME280 ] Could not write sensor calbration data");
        HDEBUG("[BME280 ] Register: 0x%02X", reg);
        HDEBUG("[BME280 ] Value   : 0x%02X", value);

        return res.status = status;
    }

    status = this->read_register(reg, read);
    if(status < BME280_OK) {
        HERROR("[BME280 ] Register value doesn't match");
        HDEBUG("[BME280 ] Expected: 0x%02X", value);
        HDEBUG("[BME280 ] Got     : 0x%02X", read);

        return res.status = status;
    }

    HDEBUG("[BME280 ] Temperature are presure mode set to: 0x%02X", read);


    HINFO ("[BME280 ] Sensor initialized");
    if(i2c) {
        HDEBUG("[BME280 ] I2C bus: I2C%d", i2c->index());
        HDEBUG("[BME280 ] Address: 0x%02X", this->cfg.address);
    }

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
        HERROR("[BME280 ] Could not read calibration register 0x%02X", Register::Calib00);
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
        HERROR("[BME280 ] Could not read calibration register 0x%02X", Register::Calib26);
        return res.status = status;
    }

    res.calibration.dig_H2 = ((data[1] << 8) | data[0]);
    res.calibration.dig_H2 =   data[2];
    res.calibration.dig_H3 = ((data[3] << 4) | data[4] & 0x0F); // H4 and H5 shares the same byte
    res.calibration.dig_H4 = ((data[5] << 4) | data[4] >> 4);   // H4 and H5 shares the same byte
    res.calibration.dig_H5 = static_cast<int8>(data[6]);

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
        HERROR("[BME280 ] ID register value doesn't match"); 
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
    while(reset_status == Status::ImUpdate) {
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

}