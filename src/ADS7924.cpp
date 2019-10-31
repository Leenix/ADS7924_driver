#include "ADS7924.h"

///////////////////////////////////////////////////////////////////////////////
// Startup

/**
 * Start communication with the ADC.
 */
bool ADS7924::begin(uint8_t address) {
    set_address(address);
    return comms_check();
}

/**
 * Set the I2C address of the ADC.
 * Address is set on the ADC by A0 pin configuration.
 */
void ADS7924::set_address(uint8_t address) {
    address &= 1001001;
    _device_address = address;
}

/**
 * Check that I2C communications are working with the ADC.
 * The manufacturer's ID is read from the chip.
 * @return: True if the manufacturer's ID matches the expected value and communications are working.
 */
bool ADS7924::comms_check() {
    ads7924_reset_config_t id;
    read_config(id);

    // LSB is dependent on address configuration. Ignoring that bit.
    id.raw |= 1;
    return id.raw == ADS7924_ID;
}

/**
 * Write a value to a register using I2C
 *
 * @param input: Byte to write to the register
 * @param address: Address of register to write to
 * @param length: Number of bytes to be written
 * @return: Success/error result of the write.
 */
bool ADS7924::write(uint8_t *input, ads7924_reg_t address, uint8_t length) {
    bool result = true;
    Wire.beginTransmission(_device_address);
    Wire.write(address);

    for (size_t i = 0; i < length; i++) {
        Wire.write(input[i]);
    }

    if (Wire.endTransmission() != 0) {
        result = false;
    }
    return result;
}

/**
 * Read a specified number of bytes using the I2C bus.
 * @param output: The buffer in which to store the read values.
 * @param address: Register address to read (or starting address in burst reads)
 * @param length: Number of bytes to read.
 */
bool ADS7924::read(uint8_t *output, ads7924_reg_t address, uint8_t length) {
    bool result = true;
    Wire.beginTransmission(_device_address);
    Wire.write(address);
    if (Wire.endTransmission() != 0)
        result = false;

    else  // OK, all worked, keep going
    {
        Wire.requestFrom(_device_address, length);
        for (size_t i = 0; (i < length) and Wire.available(); i++) {
            uint8_t c = Wire.read();
            output[i] = c;
        }
    }
    return result;
}

/**
 * Perform a software reset of the ADS7924.
 */
void ADS7924::reset() {
    ads7924_reset_config_t reset;
    reset.raw = ADS7924_RESET_SEQUENCE;
    write_config(reset);
}

///////////////////////////////////////////////////////////////////////////////
// Configuration

/**
 * Write a configuration to the ADC's registers.
 * @param config: Configuration to be written.
 */
void ADS7924::write_config(ads7924_mode_control_t config) { write((uint8_t *)&config, MODE_CONTROL); }

/**
 * Write a configuration to the ADC's registers.
 * @param config: Configuration to be written.
 */
void ADS7924::write_config(ads7924_interrupt_control_t config) { write((uint8_t *)&config, INT_CONTROL); }

/**
 * Write a configuration to the ADC's registers.
 * @param config: Configuration to be written.
 */
void ADS7924::write_config(ads7924_interrupt_config_t config) { write((uint8_t *)&config, INT_CONFIG); }

/**
 * Write a configuration to the ADC's registers.
 * @param config: Configuration to be written.
 */
void ADS7924::write_config(ads7924_sleep_config_t config) { write((uint8_t *)&config, SLEEP_CONFIG); }

/**
 * Write a configuration to the ADC's registers.
 * @param config: Configuration to be written.
 */
void ADS7924::write_config(ads7924_acquire_config_t config) { write((uint8_t *)&config, ACQUIRE_TIME_CONFIG); }

/**
 * Write a configuration to the ADC's registers.
 * @param config: Configuration to be written.
 */
void ADS7924::write_config(ads7924_power_config_t config) { write((uint8_t *)&config, POWER_CONFIG); }

/**
 * Write a configuration to the ADC's registers.
 * @param config: Configuration to be written.
 */
void ADS7924::write_config(ads7924_reset_config_t config) { write((uint8_t *)&config, RESET); }

/**
 * Read a configuration in from the ADC's registers.
 * @param config: Object for the configuration to be read into.
 */
void ADS7924::read_config(ads7924_mode_control_t &config) { read((uint8_t *)&config, MODE_CONTROL); }

/**
 * Read a configuration in from the ADC's registers.
 * @param config: Object for the configuration to be read into.
 */
void ADS7924::read_config(ads7924_interrupt_control_t &config) { read((uint8_t *)&config, INT_CONTROL); }

/**
 * Read a configuration in from the ADC's registers.
 * @param config: Object for the configuration to be read into.
 */
void ADS7924::read_config(ads7924_interrupt_config_t &config) { read((uint8_t *)&config, INT_CONFIG); }

/**
 * Read a configuration in from the ADC's registers.
 * @param config: Object for the configuration to be read into.
 */
void ADS7924::read_config(ads7924_sleep_config_t &config) { read((uint8_t *)&config, SLEEP_CONFIG); }

/**
 * Read a configuration in from the ADC's registers.
 * @param config: Object for the configuration to be read into.
 */
void ADS7924::read_config(ads7924_acquire_config_t &config) { read((uint8_t *)&config, ACQUIRE_TIME_CONFIG); }

/**
 * Read a configuration in from the ADC's registers.
 * @param config: Object for the configuration to be read into.
 */
void ADS7924::read_config(ads7924_power_config_t &config) { read((uint8_t *)&config, POWER_CONFIG); }

/**
 * Read a configuration in from the ADC's registers.
 * @param config: Object for the configuration to be read into.
 */
void ADS7924::read_config(ads7924_reset_config_t &config) { read((uint8_t *)&config, RESET); }

///////////////////////////////////////////////////////////////////////////////
// Data

/**
 * Read in data from the ADC.
 * Data is given in a 16-bit format, though readings only have a 12-bit resolution.
 * @param data: Variable for the data to be read into.
 * @param channel: Data channel to read (0-3)
 */
void ADS7924::read_data(uint16_t &data, uint8_t channel) {
    channel = constrain(channel, 0, 4);
    uint8_t channel_address = ADS7924_REGISTER::DATA0_U + (channel * 2);

    uint8_t buffer[2];
    read(buffer, (ads7924_reg_t)channel_address, 2);
    data = (uint16_t(buffer[0]) << 4) | (buffer[1] >> 4);
}

/**
 * Get read data from the ADC.
 * Data is given in a 16-bit format, though readings only have a 12-bit resolution.
 * @param channel: Data channel to read (0-3)
 * @return: Data reading from the ADC.
 */
uint16_t ADS7924::get_data(uint8_t channel) {
    uint16_t data;
    read_data(data, channel);
    return data;
}

/**
 * Get read data from the ADC.
 * Data is given in a 16-bit format, though readings only have a 12-bit resolution.
 * All channels are read at once, which is faster than reading each channel separately.
 * @param data: Buffer to read data into. Must be at least 8 bytes.
 */
void ADS7924::read_data(uint16_t *data) {
    read((uint8_t *)&data, ADS7924_REGISTER::DATA0_U, ADS7924_NUM_CHANNELS * 2);
    for (size_t i = 0; i < ADS7924_NUM_CHANNELS; i++) {
        data[i] = data[i] >> 4;
    }
}

///////////////////////////////////////////////////////////////////////////////
// Limits

/**
 * Write a data alarm limit for the ADC.
 * An alarm is generated if the read value on the specified channel falls outside the set limits.
 * Depending on how the alarms are configured, alarms will trigger an external interrupt if enough alarms occur.
 * @param limit: Threshold for the limit. Limits are given as 12-bit values, but only the 8 MSBs are used.
 * @param channel: Channel to set the threshold for. (0-3)
 * @param is_high_limit: True if the high limit is being set. False for the low limit.
 */
void ADS7924::write_limit(uint16_t limit, uint8_t channel, bool is_high_limit) {
    uint8_t low_res_limit = uint8_t(limit >> 4);

    channel = constrain(channel, 0, 3);
    uint8_t address = UPPER_LIMIT_0 + (channel * 2);
    if (not is_high_limit) address += 1;

    write(&low_res_limit, ads7924_reg_t(address));
}

/**
 * Write a data alarm high limit for the ADC.
 * An alarm is generated if the read value on the specified channel is above the set limit.
 * Depending on how the alarms are configured, alarms will trigger an external interrupt if enough alarms occur.
 * @param limit: Threshold for the limit. Limits are given as 12-bit values, but only the 8 MSBs are used.
 * @param channel: Channel to set the threshold for. (0-3)
 */
void ADS7924::write_high_limit(uint16_t limit, uint8_t channel) { write_limit(limit, channel, true); }

/**
 * Write a data alarm low limit for the ADC.
 * An alarm is generated if the read value on the specified channel is below the set limit.
 * Depending on how the alarms are configured, alarms will trigger an external interrupt if enough alarms occur.
 * @param limit: Threshold for the limit. Limits are given as 12-bit values, but only the 8 MSBs are used.
 * @param channel: Channel to set the threshold for. (0-3)
 */
void ADS7924::write_low_limit(uint16_t limit, uint8_t channel) { write_limit(limit, channel, false); }

/**
 * Read a data alarm limit from the ADC.
 * @param limit: Threshold for the limit. The limit is given as a 12-bit value, shifted up from the 8-bit threshold.
 * @param channel: Channel to read the threshold from. (0-3)
 * @param is_high_limit: True if the high limit is being read. False for the low limit.
 */
void ADS7924::read_limit(uint16_t &limit, uint8_t channel, bool is_high_limit) {
    channel = constrain(channel, 0, 3);
    uint8_t channel_address = ADS7924_REGISTER::UPPER_LIMIT_0 + (channel * 2);
    if (not is_high_limit) channel_address += 1;
    read((uint8_t *)&limit, (ads7924_reg_t)channel_address);
    limit = limit << 4;
}

/**
 * Read a data alarm high limit from the ADC.
 * @param limit: Threshold for the limit. The limit is given as a 12-bit value, shifted up from the 8-bit threshold.
 * @param channel: Channel to read the threshold from. (0-3)
 */
void ADS7924::read_high_limit(uint16_t &limit, uint8_t channel) { read_limit(limit, channel, true); }

/**
 * Read a data alarm low limit from the ADC.
 * @param limit: Threshold for the limit. The limit is given as a 12-bit value, shifted up from the 8-bit threshold.
 * @param channel: Channel to read the threshold from. (0-3)
 * @param is_high_limit: True if the high limit is being read. False for the low limit.
 */
void ADS7924::read_low_limit(uint16_t &limit, uint8_t channel) { read_limit(limit, channel, false); }
