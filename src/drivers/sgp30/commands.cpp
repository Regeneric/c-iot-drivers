#include <cstring>

#include <hkk/defines.h>

#include <hkk/utils/utils.hpp>
#include <hkk/logger/logger.h>
#include <hkk/bus/i2c/i2c.hpp>

#include <hkk/drivers/sgp30/sgp30.hpp>

namespace hkk::sgp30 {

int8 SGP30::iaq_init(void) {
    HTRACE("commands.cpp -> SGP30::iaq_init(-):int8");
    return this->iaq_init(this->ctx);
}

int8 SGP30::iaq_init(Context &res) {
    HTRACE("commands.cpp -> SGP30::iaq_init(Context&):int8");
    if(int8 status = this->sensor_enabled(); status < SGP30_OK) return res.status = status;

    int8 status = SGP30_OK;

    status = this->send_command(Command::IaqInit);
    if(status < SGP30_OK) return res.status = status;

    // Table 10 Measurement commands
    hkk::utils::sleep_ms(10);
    
    HINFO ("[SGP30  ] Sensor initialized");
    HDEBUG("[SGP30  ] I2C bus: I2C%d", i2c.index());
    HDEBUG("[SGP30  ] Address: 0x%02X", this->cfg.address);

    return res.status = status;
}

int8 SGP30::measure_iaq(void) {
    HTRACE("commands.cpp -> SGP30::measure_iaq(-):int8");
    return this->measure_iaq(this->ctx);
}

int8 SGP30::measure_iaq(Context &res) {
    HTRACE("commands.cpp -> SGP30::measure_iaq(Context&):int8");
    if(int8 status = this->sensor_enabled(); status < SGP30_OK) return res.status = status;
    
    int8 status = SGP30_OK;
    
    status = this->send_command(Command::MeasureIaq);
    if(status < SGP30_OK) return res.status = status;

    // Table 12 Measurement commands
    hkk::utils::sleep_ms(12);

    uint8 data[DATA_FRAME_LENGTH];
    status = this->read_raw_data(data, DATA_FRAME_LENGTH);
    if(status < SGP30_OK) return res.status = status;

    int8 crc_eco2_status = crc_validate((data + 0), 3);     // data[0:2] - eCO2_MSB, eCO2_LSB, eCO2_CRC
    if(crc_eco2_status < SGP30_OK) return res.status = crc_eco2_status;

    int8 crc_tvoc_status = crc_validate((data + 3), 3);     // data[3:6] - TVOC_MSB, TVOC_LSB, TVOC_CRC
    if(crc_tvoc_status < SGP30_OK) return res.status = crc_tvoc_status; 

    res.eco2 = ((static_cast<uint16>(data[0]) << 8) | static_cast<uint16>(data[1]));   // ((MSB << 8) | LSB)
    res.tvoc = ((static_cast<uint16>(data[3]) << 8) | static_cast<uint16>(data[4]));   // ((MSB << 8) | LSB)
    std::memcpy(res.raw_data, data, DATA_FRAME_LENGTH);

    HTRACE("[SGP30  ] Measure IAQ command sent");
    HTRACE("[SGP30  ] I2C bus: I2C%d", i2c.index());
    HTRACE("[SGP30  ] Address: 0x%02X", this->cfg.address);
    HTRACE("[SGP30  ] eCO2: %d", res.eco2);
    HTRACE("[SGP30  ] TVOC: %d", res.tvoc);

    return res.status = status;
}

int8 SGP30::get_iaq_baseline(void) {
    HTRACE("commands.cpp -> SGP30::get_iaq_baseline(-):int8");
    return this->get_iaq_baseline(this->ctx);
}

int8 SGP30::get_iaq_baseline(Context &res) {
    HTRACE("command.cpp -> SGP30::get_iaq_baseline(Context&):int8");
    if(int8 status = this->sensor_enabled(); status < SGP30_OK) return res.status = status;

    int8 status = SGP30_OK;

    status = this->send_command(Command::GetIaqBaseline);
    if(status < SGP30_OK) return res.status = status;

    // Table 10 Measurement commands
    hkk::utils::sleep_ms(10);

    uint8 data[DATA_FRAME_LENGTH];
    status = this->read_raw_data(data, DATA_FRAME_LENGTH);
    if(status < SGP30_OK) return res.status = status;

    int8 crc_eco2_status = crc_validate((data + 0), 3);     // data[0:2] - eCO2_MSB, eCO2_LSB, eCO2_CRC
    if(crc_eco2_status < SGP30_OK) return res.status = crc_eco2_status;

    int8 crc_tvoc_status = crc_validate((data + 3), 3);     // data[3:6] - TVOC_MSB, TVOC_LSB, TVOC_CRC
    if(crc_tvoc_status < SGP30_OK) return res.status = crc_tvoc_status;

    // 6 bytes: (eCO2_MSB, eCO2_LSB, eCO2_CRC, TVOC_MSB, TVOC_LSB, TVOC_CRC)
    std::memcpy(res.baseline, data, DATA_FRAME_LENGTH);

    HTRACE("[SGP30  ] Get IAQ baseline command sent");
    HTRACE("[SGP30  ] I2C bus : I2C%d", i2c.index());
    HTRACE("[SGP30  ] Address : 0x%02X", this->cfg.address);
    HTRACE("[SGP30  ] Baseline: {0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X}", res.baseline[0], res.baseline[1], res.baseline[2], res.baseline[3], res.baseline[4], res.baseline[5]);

    return res.status = status;
}

int8 SGP30::set_iaq_baseline(uint8 *baseline) {
    HTRACE("commands.cpp -> SGP30::set_iaq_baseline(uint8*):int8");
    
    if(!baseline) {
        HERROR("[SGP30  ] Null data pointer passed to function");
        return SGP30_ERROR_NULL_DATA;
    }

    std::memcpy(this->ctx.baseline, baseline, DATA_FRAME_LENGTH);
    
    return this->set_iaq_baseline(this->ctx);
}

int8 SGP30::set_iaq_baseline(Context &res) {
    HTRACE("commands.cpp -> SGP30::set_iaq_baseline(Context&):int8");
    if(int8 status = this->sensor_enabled(); status < SGP30_OK) return res.status = status;

    int8 status = SGP30_OK;

    int8 crc_eco2_status = crc_validate((res.baseline + 0), HALF_DATA_FRAME_LENGTH);     // data[0:2] - eCO2_MSB, eCO2_LSB, eCO2_CRC
    if(crc_eco2_status < SGP30_OK) return res.status = crc_eco2_status;

    int8 crc_tvoc_status = crc_validate((res.baseline + 3), HALF_DATA_FRAME_LENGTH);     // data[3:6] - TVOC_MSB, TVOC_LSB, TVOC_CRC
    if(crc_tvoc_status < SGP30_OK) return res.status = crc_tvoc_status; 

    uint8 data[COMMAND_FRAME_LENGTH + DATA_FRAME_LENGTH];
    uint8 command[COMMAND_FRAME_LENGTH] = {hkk::utils::msb(Command::SetIaqBaseline), hkk::utils::lsb(Command::SetIaqBaseline)};

    // 6 bytes: (TVOC_MSB, TVOC_LSB, TVOC_CRC, eCO2_MSB, eCO2_LSB, eCO2_CRC)
    std::memcpy(data, command, COMMAND_FRAME_LENGTH);
    std::memcpy((data + COMMAND_FRAME_LENGTH), (res.baseline + 3), HALF_DATA_FRAME_LENGTH);                             // data[3:6] - TVOC_MSB, TVOC_LSB, TVOC_CRC
    std::memcpy((data + COMMAND_FRAME_LENGTH + HALF_DATA_FRAME_LENGTH), (res.baseline + 0), HALF_DATA_FRAME_LENGTH);    // data[0:2] - eCO2_MSB, eCO2_LSB, eCO2_CRC
    
    status = this->send_payload(data);
    if(status < SGP30_OK) return res.status = status;

    // Table 10 Measurement commands
    hkk::utils::sleep_ms(10);

    HDEBUG("[SGP30  ] Set IAQ baseline command sent");
    HTRACE("[SGP30  ] I2C bus: I2C%d", i2c.index());
    HTRACE("[SGP30  ] Address: 0x%02X", this->cfg.address);

    return res.status = status;
}

int8 SGP30::set_absolute_humidity(uint8 *humidity) {
    HTRACE("commands.cpp -> SGP30::set_absolute_humidity(uint8*):int8");
    
    if(!humidity) {
        HERROR("[SGP30  ] Null data pointer passed to function");
        return SGP30_ERROR_NULL_DATA;
    }

    std::memcpy(this->ctx.raw_absolute_humidity, humidity, (HALF_DATA_FRAME_LENGTH - 1));

    return this->set_absolute_humidity(this->ctx);
}

int8 SGP30::set_absolute_humidity(Context &res) {
    HTRACE("commands.cpp -> SGP30::set_absolute_humidity(Context&):int8");
    if(int8 status = this->sensor_enabled(); status < SGP30_OK) return res.status = status;

    int8 status = SGP30_OK;

    // TODO: data_frame() function to encapsulate this code block
    uint8 crc = 0;
    status = crc_calculate(crc, res.raw_absolute_humidity, (HALF_DATA_FRAME_LENGTH - 1));
    if(status < SGP30_OK) return res.status = status;

    uint8 data[COMMAND_FRAME_LENGTH + HALF_DATA_FRAME_LENGTH];
    uint8 command[COMMAND_FRAME_LENGTH] = {hkk::utils::msb(SetAbsoluteHumidity), hkk::utils::lsb(SetAbsoluteHumidity)};

    std::memcpy(data, command, COMMAND_FRAME_LENGTH);
    std::memcpy((data + COMMAND_FRAME_LENGTH), res.raw_absolute_humidity, (HALF_DATA_FRAME_LENGTH - 1));
    data[COMMAND_FRAME_LENGTH + (HALF_DATA_FRAME_LENGTH - 1)] = crc;

    status = this->send_payload(data);
    if(status < SGP30_OK) return res.status = status;

    // Table 10 Measurement commands
    hkk::utils::sleep_ms(10);

    HTRACE("[SGP30  ] Set absolute humidity command sent");
    HTRACE("[SGP30  ] I2C bus: I2C%d", i2c.index());
    HTRACE("[SGP30  ] Address: 0x%02X", this->cfg.address);

    return res.status = status;
}

// Page 10 Measure Test
int8 SGP30::measure_test(void) {
    HTRACE("commands.cpp -> SGP30::measure_test(-):int8");
    return this->measure_test(this->ctx);
}

int8 SGP30::measure_test(Context &res) {
    HTRACE("commands.cpp -> SGP30::measure_test(Context&):int8");
    if(int8 status = this->sensor_enabled(); status < SGP30_OK) return res.status = status;

    int8 status = SGP30_OK;

    status = this->send_command(Command::MeasureTest);
    if(status < SGP30_OK) return res.status = status;

    // Table 10 Measurement commands
    hkk::utils::sleep_ms(220);

    uint8 data[HALF_DATA_FRAME_LENGTH];
    status = this->read_raw_data(data, HALF_DATA_FRAME_LENGTH);
    if(status < SGP30_OK) return res.status = status;

    uint16 test_value = ((static_cast<uint16>(data[0]) << 8) | static_cast<uint16>(data[1]));
    if(test_value != TestValue) {
        HERROR("[SGP30  ] Measure test returned unexpected value");
        HERROR("[SGP30  ] Expected: 0x%02X", TestValue);
        HERROR("[SGP30  ] Got     : 0x%02X", test_value);

        return res.status = SGP30_ERROR_GENERIC;
    }

    int8 crc_status = crc_validate(data);
    if(crc_status < SGP30_OK) return res.status = crc_status;

    HDEBUG("[SGP30  ] Measure test command sent");
    HTRACE("[SGP30  ] I2C bus: I2C%d", i2c.index());
    HTRACE("[SGP30  ] Address: 0x%02X", this->cfg.address);

    return res.status = status;
}

int8 SGP30::feature_set(void) {
    HTRACE("commands.cpp -> SGP30::feature_set(-):int8");
    return this->feature_set(this->ctx);
}

int8 SGP30::feature_set(Context &res) {
    HTRACE("commands.cpp -> SGP30::feature_set(Context&):int8");
    if(int8 status = this->sensor_enabled(); status < SGP30_OK) return res.status = status;

    int8 status = SGP30_OK;

    status = this->send_command(Command::GetFeatureSet);
    if(status < SGP30_OK) return res.status = status;

    // Table 10 Measurement commands
    hkk::utils::sleep_ms(10);

    uint8 data[HALF_DATA_FRAME_LENGTH];
    status = this->read_raw_data(data, HALF_DATA_FRAME_LENGTH);
    if(status < SGP30_OK) return res.status = status;

    uint16 test_value = ((static_cast<uint16>(data[0]) << 8) | static_cast<uint16>(data[1]));
    if(test_value != FeatureSet) {
        HERROR("[SGP30  ] Feature set returned unexpected value");
        HERROR("[SGP30  ] Expected: 0x%02X", FeatureSet);
        HERROR("[SGP30  ] Got     : 0x%02X", test_value);

        return res.status = SGP30_ERROR_GENERIC;
    }

    int8 crc_status = crc_validate(data);
    if(crc_status < SGP30_OK) return res.status = crc_status;

    HDEBUG("[SGP30  ] Feature set command sent");
    HTRACE("[SGP30  ] I2C bus: I2C%d", i2c.index());
    HTRACE("[SGP30  ] Address: 0x%02X", this->cfg.address);

    return res.status = status;
}

int8 SGP30::measure_raw(void) {
    HTRACE("commands.cpp -> SGP30::measure_raw(-):int8");
    return this->measure_raw(this->ctx);
}

int8 SGP30::measure_raw(Context &res) {
    HTRACE("commands.cpp -> measure_raw(Context&):int8");
    if(int8 status = this->sensor_enabled(); status < SGP30_OK) return res.status = status;

    int8 status = SGP30_OK;

    status = this->send_command(Command::MeasureRaw);
    if(status < SGP30_OK) return res.status = status;

    // Table 10 Measurement commands
    hkk::utils::sleep_ms(25);

    uint8 data[DATA_FRAME_LENGTH];
    status = this->read_raw_data(data, DATA_FRAME_LENGTH);
    if(status < SGP30_OK) return res.status = status;

    int8 crc_h2_status = crc_validate((data + 0), 3);           // data[0:2] - H2_MSB, H2_LSB, H2_CRC
    if(crc_h2_status < SGP30_OK) return res.status = crc_h2_status;

    int8 crc_c2h60_status = crc_validate((data + 3), 3);        // data[3:6] - C2H6O_MSB, C2H6O_LSB, C2H6O_CRC
    if(crc_c2h60_status < SGP30_OK) return res.status = crc_c2h60_status;     

    res.h2    = ((static_cast<uint16>(data[0]) << 8) | static_cast<uint16>(data[1]));   // ((MSB << 8) | LSB)
    res.c2h6o = ((static_cast<uint16>(data[3]) << 8) | static_cast<uint16>(data[4]));   // ((MSB << 8) | LSB)
    std::memcpy(res.raw_data, data, DATA_FRAME_LENGTH);

    HTRACE("[SGP30  ] Measure raw command sent");
    HTRACE("[SGP30  ] I2C bus: I2C%d", i2c.index());
    HTRACE("[SGP30  ] Address: 0x%02X", this->cfg.address);
    HTRACE("[SGP30  ] H2     : %d", res.h2);
    HTRACE("[SGP30  ] C2H6O  : %d", res.c2h6o);

    return res.status = status;
}

int8 SGP30::get_tvoc_inceptive_baseline(void) {
    HTRACE("commands.cpp -> SGP30::get_tvoc_inceptive_baseline(-):int8");
    return this->get_tvoc_inceptive_baseline(this->ctx);
}

int8 SGP30::get_tvoc_inceptive_baseline(Context &res) {
    HTRACE("command.cpp -> SGP30::get_tvoc_inceptive_baseline(Context&):int8");
    if(int8 status = this->sensor_enabled(); status < SGP30_OK) return res.status = status;

    int8 status = SGP30_OK;

    status = this->send_command(Command::GetTvocInceptiveBaseline);
    if(status < SGP30_OK) return res.status = status;

    // Table 10 Measurement commands
    hkk::utils::sleep_ms(10);

    uint8 data[HALF_DATA_FRAME_LENGTH];
    status = this->read_raw_data(data, HALF_DATA_FRAME_LENGTH);
    if(status < SGP30_OK) return res.status = status;

    int8 crc_status = crc_validate(data, HALF_DATA_FRAME_LENGTH);
    if(crc_status < SGP30_OK) return res.status = crc_status;

    std::memcpy(res.tvoc_baseline, data, HALF_DATA_FRAME_LENGTH);

    HDEBUG("[SGP30  ] Get TVOC inceptive baseline command sent");
    HTRACE("[SGP30  ] I2C bus: I2C%d", i2c.index());
    HTRACE("[SGP30  ] Address: 0x%02X", this->cfg.address);

    return res.status = status;
}

int8 SGP30::set_tvoc_baseline(uint8 *tvoc_baseline) {
    HTRACE("commands.cpp -> SGP30::set_tvoc_baseline(uint8*):int8");

    if(!tvoc_baseline) {
        HERROR("[SGP30  ] Null data pointer passed to function");
        return SGP30_ERROR_NULL_DATA;
    }

    std::memcpy(this->ctx.tvoc_baseline, tvoc_baseline, (HALF_DATA_FRAME_LENGTH - 1));

    return this->set_tvoc_baseline(this->ctx);
}

int8 SGP30::set_tvoc_baseline(Context &res) {
    HTRACE("commands.cpp -> SGP30::set_tvoc_baseline(Context&):int8");
    if(int8 status = this->sensor_enabled(); status < SGP30_OK) return res.status = status;

    int8 status = SGP30_OK;

    // TODO: data_frame() function to encapsulate this code block
    uint8 crc = 0;
    status = crc_calculate(crc, res.tvoc_baseline, (HALF_DATA_FRAME_LENGTH - 1));
    if(status < SGP30_OK) return res.status = status;

    uint8 data[COMMAND_FRAME_LENGTH + HALF_DATA_FRAME_LENGTH];    
    uint8 command[COMMAND_FRAME_LENGTH] = {hkk::utils::msb(Command::SetTvocBaseline), hkk::utils::lsb(Command::SetTvocBaseline)};

    std::memcpy(data, command, COMMAND_FRAME_LENGTH);
    std::memcpy((data + COMMAND_FRAME_LENGTH), res.tvoc_baseline, (HALF_DATA_FRAME_LENGTH - 1));
    data[COMMAND_FRAME_LENGTH + (HALF_DATA_FRAME_LENGTH - 1)] = crc;

    status = this->send_payload(data);
    if(status < SGP30_OK) return res.status = status;

    // Table 10 Measurement commands
    hkk::utils::sleep_ms(10);

    HDEBUG("[SGP30  ] Set TVOC baseline command sent");
    HTRACE("[SGP30  ] I2C bus: I2C%d", i2c.index());
    HTRACE("[SGP30  ] Address: 0x%02X", this->cfg.address);

    return res.status = status;
}
 
int8 SGP30::soft_reset(void) {
    HTRACE("commands.cpp -> SGP30::soft_reset(-):int8");
    if(int8 status = this->sensor_enabled(); status < SGP30_OK) return status;

    int8 status = SGP30_OK;

    {
        auto tx = i2c.transaction(this);
        if(tx.status < hkk::bus::i2c::I2C_OK) {
            return this->validate_i2c_error(tx.status);
        }

        const uint8 commands[] = {hkk::utils::msb(Command::SoftReset), hkk::utils::lsb(Command::SoftReset)};
        status = i2c.write(SOFT_RESET_ADDRESS, commands);
    }

    if(status < hkk::bus::i2c::I2C_OK) {
        return this->validate_i2c_error(static_cast<int8>(status));
    }

    // Table 10 Measurement commands
    hkk::utils::sleep_ms(10);
    
    HINFO ("[SGP30  ] Sensor soft reset");
    HDEBUG("[SGP30  ] I2C bus: I2C%d", i2c.index());
    HDEBUG("[SGP30  ] Address: 0x%02X", this->cfg.address);

    return status;
}

int8 SGP30::get_serial_number(void) {
    HTRACE("commands.cpp -> SGP30::get_serial_number(-):int8");
    return this->get_serial_number(this->ctx);
}

int8 SGP30::get_serial_number(Context &res) {
    HTRACE("commands.cpp -> SGP30::get_serial_number(Context&):int8");
    if(int8 status = this->sensor_enabled(); status < SGP30_OK) return res.status = status;

    int8 status = SGP30_OK;

    status = this->send_command(Command::GetSerialId);
    if(status < SGP30_OK) return res.status = status;

    // Table 10 Measurement commands
    hkk::utils::sleep_ms(1);

    uint8 data[JUMBO_DATA_FRAME_LENGTH];
    status = this->read_raw_data(data, JUMBO_DATA_FRAME_LENGTH);
    if(status < SGP30_OK) return res.status = status;

    int8 crc1_status = crc_validate((data + 0), 3);     // data[0:2]
    if(crc1_status < SGP30_OK) return res.status = crc1_status;

    int8 crc2_status = crc_validate((data + 3), 3);     // data[3:6]
    if(crc2_status < SGP30_OK) return res.status = crc2_status;

    int8 crc3_status = crc_validate((data + 6), 3);     // data[7:9]
    if(crc3_status < SGP30_OK) return res.status = crc3_status;

    std::memcpy(res.serial_number, data, JUMBO_DATA_FRAME_LENGTH);

    HDEBUG("[SGP30  ] Get serial number command sent");
    HTRACE("[SGP30  ] I2C bus: I2C%d", i2c.index());
    HTRACE("[SGP30  ] Address: 0x%02X", this->cfg.address);

    return res.status = status;
}

}