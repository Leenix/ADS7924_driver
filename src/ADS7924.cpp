#include "ADS7924.h"

void ADS7924::set_address(uint8_t address) {
    address &= 1001001;
    device_address = address;
}

void ADS7924::write_config(ads7924_config_t new_config) {
    config = new_config;
    write((uint8_t *)&config.int_config, ADS7924_REGISTER::INT_CONFIG);
    write((uint8_t *)&config.power_config, ADS7924_REGISTER::POWER_CONFIG);
    write((uint8_t *)&config.sleep_config, ADS7924_REGISTER::SLEEP_CONFIG);
    write((uint8_t *)&config.acquire_time, ADS7924_REGISTER::ACQUIRE_TIME_CONFIG);
}

void ADS7924::set_mode(ads7924_mode_t mode) { write((uint8_t *)&mode, ADS7924_REGISTER::MODE_CONTROL); }

void ADS7924::write_interrupt_control(ads7924_int_control_t control_settings) {
    write((uint8_t *)&control_settings, ADS7924_REGISTER::INT_CONTROL);
}

ads7924_int_control_t ADS7924::read_interrupt_control() {
    ads7924_int_control_t control;
    read((uint8_t *)&control, ADS7924_REGISTER::INT_CONTROL);
    return control;
}

uint16_t ADS7924::get_reading(uint8_t channel) {
    uint16_t reading;
    channel = constrain(channel, 0, 3);
    uint8_t channel_address = ADS7924_REGISTER::DATA0_U + (channel * 2);
    read((uint8_t *)&reading, (ads7924_reg_t)channel_address, 2);
    reading = reading >> 4;
    return reading;
}

void ADS7924::get_readings(uint16_t *readings) {
    read((uint8_t *)&readings, ADS7924_REGISTER::DATA0_U, 8);
    for (size_t i = 0; i < 4; i++) {
        readings[i] = readings[i] >> 4;
    }
}

void ADS7924::set_limit(uint16_t limit, uint8_t channel, bool high) {
    uint8_t low_res_limit = limit >> 8;
    channel = constrain(channel, 0, 3);
    uint8_t channel_address = ADS7924_REGISTER::UPPER_LIMIT_0 + (channel * 2);
    if (not high) channel_address += 1;
    write(&low_res_limit, (ads7924_reg_t)channel_address);
}

void ADS7924::set_limits(uint16_t *limits) {
    if (sizeof(limits) == 16) {
        uint8_t low_res_limits[8];
        for (size_t i = 0; i < 4; i++) {
            low_res_limits[i] = limits[i] >> 8;
        }
        write(low_res_limits, ADS7924_REGISTER::UPPER_LIMIT_0, 8);
    }
}

uint16_t ADS7924::get_limit(uint8_t channel, bool high) {
    uint16_t limit;
    channel = constrain(channel, 0, 3);
    uint8_t channel_address = ADS7924_REGISTER::UPPER_LIMIT_0 + (channel * 2);
    if (not high) channel_address += 1;
    read((uint8_t *)&limit, (ads7924_reg_t)channel_address);
    return limit << 8;
}

uint16_t ADS7924::get_limits(uint16_t *limits) {
    read((uint8_t *)limits, ADS7924_REGISTER::DATA0_U, 8);
    for (size_t i = 0; i < 4; i++) {
        limits[i] = limits[i] >> 4;
    }
}