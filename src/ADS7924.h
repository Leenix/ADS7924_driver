#ifndef ADS7924_ADC_H
#define ADS7924_ADC_H

///////////////////////////////////////////////////////////////////////////////
// Imports

#include <Arduino.h>
#include <Wire.h>

///////////////////////////////////////////////////////////////////////////////
// Types and constants

const uint8_t DEFAULT_ADS7924_ADDRESS = 0b1001000;
const uint8_t ADS7924_ID = 0b00011000;
const uint8_t ADS7924_NUM_CHANNELS = 4;

///////////////////////////////////////////////////////////////////////////////
// MODECNTRL

/**
 * Mode explanation
 *
 * Manual-single mode
 * This mode converts the selected channel once. After the ADC Mode Control register is written, the power-up time (tPU)
 * and acquisition time (tACQ) are allowed to elapse. tPU can be set to '0' to effectively bypass if not needed. tACQ
 * time is programmable through the ACQCONFIG register, bits[4:0]. Sleep time (tSLEEP) is not used in this mode. After
 * the conversion completes, the device waits for a new mode to be set. This mode can be set to Idle to save power. When
 * tPU and tACQ are very short, the very short conversion time needed allows a read register operation to be issued on
 * the I2C bus immediately after the write operation that initiates this mode. If multiple conversions are needed, the
 * manual-single mode can be reissued without requiring the awake mode to be issued in between. Consecutive
 * manual-single commands have no tPU period.
 *
 * Manual-scan mode
 * This mode converts all of the channels once, starting with the selected channel, as illustrated in Figure 23. After
 * the ADC Mode Control register is written, the power-up time (tPU) is allowed to elapse. This value can be set to '0'
 * to effectively bypass if not needed. Before each conversion, an acquisition time (tACQ) is allowed to elapse. tACK
 * time is programmable through the ACQCONFIG register, bits[4:0]. Sleep time (tSLEEP) is not used in this mode. The
 * input multiplexer is automatically incremented as the conversions complete. If, for example, the initial selected
 * channel is CH2, the conversion order is CH2, CH3, CH0, and CH1. Data from the conversions are always put into the
 * data register that corresponds to a particular channel. For example, CH2 data always goes in register DATA2_H and
 * DATA2_L regardless of conversion order. After all four conversions complete, the device waits for a new mode to be
 * set. This mode can be set to Idle afterwards to save power. The INT pin can be configured to indicate the completion
 * of each individual conversion or it can wait until all four finish. In either case, the appropriate data register is
 * updated after each conversion. These registers can be read at any time afterwards. If multiple scan are needed, the
 * manual-scan mode can be reissued without requiring the Awake mode to be issued in between.
 *
 * Auto-single mode
 * This mode automatically converts the selected channel continuously. After the ADC Mode Control
 * register is written, the power-up time (tPU) is allowed to elapse. This value can be set to '0' to effectively bypass
 * if not needed. Before the conversion, an acquisition time (tACQ) is allowed to elapse. tACQ time is programmable
 * through the ACQCONFIG register, bits[4:0]. Sleep time (tSLEEP) is not used in this mode. After the conversion
 * completes the cycle is repeated.
 * This mode can be used with the onboard digital comparator to monitor the status of an input signal with little
 * support needed from a host microcontroller. The conversion time is less than the I2C data retrieval time. TI
 * suggests stopping this mode by setting the mode to Idle or stopping the conversion by configuring the alarm to
 * do so, before retrieving data. The alarm can also be configured to continue the conversion even after an interrupt
 * is generated.
 *
 * Auto-scan mode
 * This mode automatically converts all the channels continuously, starting with the selected channel. After the ADC
 * Mode Control register is written, the power-up time (tPU) is allowed to elapse. This value can be set to '0' to
 * effectively bypass if not needed. Before the conversion, an acquisition time (tACQ) is allowed to elapse. tACQ time
 * is programmable through the ACQCONFIG register, bits[4:0]. Sleep time (tSLEEP) is not used in this mode. The input
 * multiplexer is automatically incremented as the conversions complete. If, for example, the initial selected channel
 * is CH2, the conversion order is CH2, CH3, CH0, CH1, CH2, CH3, and so forth. until the mode is stopped. Data from the
 * conversions are always put into the data register that corresponds to a particular channel. For example, CH2 data
 * always go in register DATA2_H and DATA2_L regardless of conversion order. This mode can be used with the onboard
 * digital comparator to monitor the status of the input signals with little support needed from a host microcontroller.
 * TI suggests interrupting this mode and stopping the automatic conversions, either by setting the mode to Idle or
 * configuring the alarm to do so, before retrieving data.
 *
 * Auto-scan with sleep mode
 * This mode automatically converts all the channels repeatedly with a sleep interval between conversions, as
 * illustrated in Figure 27. After the ADC Mode Control register is written, the power-up time (tPU) is allowed to
 * elapse. This value can be set to '0' to effectively bypass if not needed. Before the first conversion of the selected
 * input, an acquisition time (tACQ) is allowed to elapse. tACQ time is programmable through the ACQCONFIG
 * register, bits[4:0]. After the conversion, a sleep time (tSLEEP) is allowed to elapse and then the cycle repeats. The
 * length of the sleep time is controlled by register bits. During the sleep mode, power dissipation is minimal and the
 * PWRCON output is always disabled. The input multiplexer is automatically incremented as the conversions
 * complete. If, for example, the initial selected channel is CH2, the conversion order is CH2, CH3, CH0, CH1, CH2,
 * CH3, and so forth until the mode is stopped. Data from the conversions are always put into the data register that
 * corresponds to a particular channel. For example, CH2 data always goes in register DATA2_H and DATA2_L
 * regardless of conversion order.
 * This mode can be used with the onboard digital comparator to periodically monitor the status of the input signals
 * while saving power between conversions. Little support is needed from a host microcontroller. TI suggests
 * stopping this mode by setting it to Idle or stopping the conversion by configuring the alarm to do so, before
 * retrieving data.
 *
 * Auto-burst scan with sleeo mode
 * This mode automatically converts all the channels without delay followed by a sleep interval before the cycle
 * repeats, as illustrated in Figure 28. After the ADC Mode Control register is written, the power-up time (tPU) is
 * allowed to elapse. This value can be set to '0' to effectively bypass if not needed. Before the first conversion of
 * the selected input, an acquisition time (tACQ) is allowed to elapse. tACQ time is programmable through the
 * ACQCONFIG register, bits[4:0]. Afterwards, all four inputs are measured without delay. The input multiplexer is
 * automatically incremented as the conversions complete. If, for example, the initial selected channel is CH2, the
 * conversion order is CH2, CH3, CH0, and CH1. After the four conversions, a sleep time (tSLEEP) is allowed to
 * elapse and then the cycle repeats. The length of the sleep time is controlled by register bits. During the sleep
 * mode, power dissipation is minimal and the PWRCON output is always disabled. Data from the conversions are
 * always put into the data register that corresponds to a particular channel. For example, CH2 data always goes in
 * register DATA2_H and DATA2_L regardless of conversion order.
 * This mode can be used with the onboard digital comparator to periodically monitor the status of the input signals
 * while saving power between conversions. Little support is needed from a host microcontroller. TI suggests
 * interrupting this mode and stop the automatic conversions, either by setting the mode to Idle or configuring the
 * alarm to do so, before retrieving data.
 */
enum ADS7924_MODE {
    ADS7924_IDLE = 0,                  // All circuits shut down. Lowest power setting.
    ADS7924_AWAKE = 0b100000,          // All circuits awake and ready to convert.
    ADS7924_MANUAL_SINGLE = 0b110000,  // Select input channel is converted once
    ADS7924_MANUAL_SCAN = 0b110010,    // All input channels are converted once
    ADS7924_AUTO_SINGLE = 0b110001,    // One input channel is continuously converted
    ADS7924_AUTO_SCAN = 0b110011,      // All input channels are continuously converted
    ADS7924_AUTO_SINGLE_SLEEP =
        0b111001,  // One input channel is continuously converted with a programmable sleep time between conversions
    ADS7924_AUTO_SCAN_SLEEP =
        0b111011,  // All input channels are continuously converted with a programmable sleep time between conversions
    ADS7924_AUTO_BURST_SCAN_SLEEP =
        0b111111,  // All input channels are converted with minimal delay followed by a programmable sleep time
};

typedef union {
    uint8_t raw;  // Control register as raw byte.
    struct {
        uint8_t channel : 2;  // Channel selected for conversions or first channel to be converted in scan mode.
        uint8_t mode : 6;     // ADC conversion mode. See ADS7924_MODE for available options.
    };

} ads7924_mode_control_t;

///////////////////////////////////////////////////////////////////////////////
// INTCNTRL

typedef union {
    uint8_t raw;  // Register as a raw byte
    struct {
        bool ch0_alarm_enabled : 1;    // True to enable alarm interrupts on channel 0
        bool ch1_alarm_enabled : 1;    // True to enable alarm interrupts on channel 1
        bool ch2_alarm_enabled : 1;    // True to enable alarm interrupts on channel 2
        bool ch3_alarm_enabled : 1;    // True to enable alarm interrupts on channel 3
        bool ch0_alarm_triggered : 1;  // True if alarm was triggered on channel 0
        bool ch1_alarm_triggered : 1;  // True if alarm was triggered on channel 1
        bool ch2_alarm_triggered : 1;  // True if alarm was triggered on channel 2
        bool ch3_alarm_triggered : 1;  // True if alarm was triggered on channel 3
    };
    struct {
        uint8_t alarm_enable : 4;     // Contains the 4 alarm enable flags as one 4-bit container
        uint8_t alarm_triggered : 4;  //  Contains the 4 alarm trigger flags as one 4-bit container
    };
} ads7924_interrupt_control_t;

///////////////////////////////////////////////////////////////////////////////
// INTCONFIG

enum ADS7924_INTERRUPT_OUTPUT {
    ADS7924_ALARM = 0,                   // Interrupt on alarm condition after exceeding the threshold limit
    ADS7924_BUSY_ALARM = 1,              // Interrupt on busy signal. Conversion control linked to alarm status
    ADS7924_DATA_READY_SINGLE = 2,       // Interrupt on every converted conversion
    ADS7924_BUSY_SINGLE_CONVERSION = 3,  // Interrupt on busy signal. Conversion control linked to conversion complete
    ADS7924_DATA_READY_ALL = 6,          // Interrupt on all 4 conversions completed
    ADS7924_BUSY_ALL_CONVERSION = 7,  // Interrupt on busy signal. Conversion control linked to all conversion complete
};

typedef union {
    uint8_t raw;  // Configuration register as a raw byte
    struct {
        bool pulse_interrupt_enabled : 1;  // True for an intermittent pulse interrupt. False (default) for latched
                                           // interrupt.
        bool interrupt_active_high_enabled : 1;   // True for a logic high on interrupt. False (default) for logic low.
        uint8_t alarm_interrupt_output_mode : 3;  // Interrupt mode. See ADS7924_INTERRUPT_MODE for available options.
        uint8_t alarm_threshold : 3;  // Number of times that thresholds are exceeded before an alarm is generated.
    };
} ads7924_interrupt_config_t;

///////////////////////////////////////////////////////////////////////////////
// SLPCONFIG

enum ADS7924_SLEEP_TIME {
    ADS7924_SLEEP_2MS = 0,
    ADS7924_SLEEP_5MS = 1,
    ADS7924_SLEEP_10MS = 2,
    ADS7924_SLEEP_20MS = 3,
    ADS7924_SLEEP_40MS = 4,
    ADS7924_SLEEP_80MS = 5,
    ADS7924_SLEEP_160MS = 6,
    ADS7924_SLEEP_320MS = 7,
};

typedef union {
    uint8_t raw;
    struct {
        /*
        Time between conversions (or scans) which the sensor is sleeping.
        See ADS7924_SLEEP_TIME for available options.
        */
        uint8_t sleep_time : 3;
        bool _reserved1 : 1;
        bool sleep_multiplier_x8_enabled : 1;  // True to multiply sleep time by a factor of 8.
        bool sleep_divider_x4_enabled : 1;     // True to divide the sleep time by a factor of 4.
        /*
        True to stop conversions after a control event.
        Control events are dictated by the interrupt configuration mode.
        */
        bool conversion_stop_on_control_event_enabled : 1;
        bool _reserved0 : 1;
    };
} ads7924_sleep_config_t;

///////////////////////////////////////////////////////////////////////////////
// ACQCONFIG

typedef union {
    uint8_t raw;
    struct {
        uint8_t acquire_time_x2us : 4;  // Acquire time of conversions (x2 + 6us)
        uint8_t _reserved : 4;
    };
} ads7924_acquire_config_t;

///////////////////////////////////////////////////////////////////////////////
// PWRCONFIG

typedef union {
    uint8_t raw;
    struct {
        uint8_t power_up_time_x2_us : 5;                    // Power up time of the ADS7924 (x2us)
        bool power_control_pin_enabled : 1;                 // True to always enable the PWRCON pin; False to disable
        uint8_t power_control_pin_active_high_enabled : 1;  // True enables a logic high for the power control pin

        /*
        True to connects the MUXOUT pin to AGND when CH3 is selected in the mode
        configuration register (gives 0V for calibration purposes);
        False (default) for normal operation when CH3 is selected.
        */
        bool ch3_to_agnd_enabled : 1;
    };
} ads7924_power_config_t;

///////////////////////////////////////////////////////////////////////////////
// Reset

const uint8_t ADS7924_RESET_SEQUENCE = 0b10101010;

typedef union {
    uint8_t raw;
} ads7924_reset_config_t;

///////////////////////////////////////////////////////////////////////////////
// Class definition

class ADS7924 {
   public:
    bool begin(uint8_t address = DEFAULT_ADS7924_ADDRESS);
    void set_address(uint8_t address);
    bool comms_check();

    void write_config(ads7924_mode_control_t config);
    void write_config(ads7924_interrupt_control_t config);
    void write_config(ads7924_interrupt_config_t config);
    void write_config(ads7924_sleep_config_t config);
    void write_config(ads7924_acquire_config_t config);
    void write_config(ads7924_power_config_t config);
    void write_config(ads7924_reset_config_t config);

    void read_config(ads7924_mode_control_t &config);
    void read_config(ads7924_interrupt_control_t &config);
    void read_config(ads7924_interrupt_config_t &config);
    void read_config(ads7924_sleep_config_t &config);
    void read_config(ads7924_acquire_config_t &config);
    void read_config(ads7924_power_config_t &config);
    void read_config(ads7924_reset_config_t &config);

    void read_data(uint16_t &data, uint8_t channel);
    void read_data(uint16_t *data);

    uint16_t get_data(uint8_t channel);

    void write_high_limit(uint16_t limit, uint8_t channel);
    void write_low_limit(uint16_t limit, uint8_t channel);

    void read_high_limit(uint16_t &limit, uint8_t channel);
    void read_low_limit(uint16_t &limit, uint8_t channel);

    void reset();

   private:
    typedef enum ADS7924_REGISTER {
        MODE_CONTROL = 0x00,
        INT_CONTROL = 0x01,
        DATA0_U = 0x02,
        DATA0_L = 0x03,
        DATA1_U = 0x04,
        DATA1_L = 0x05,
        DATA2_U = 0x06,
        DATA2_L = 0x07,
        DATA3_U = 0x08,
        DATA3_L = 0x09,
        UPPER_LIMIT_0 = 0x0A,
        LOWER_LIMIT_0 = 0x0B,
        UPPER_LIMIT_1 = 0x0C,
        LOWER_LIMIT_1 = 0x0D,
        UPPER_LIMIT_2 = 0x0E,
        LOWER_LIMIT_2 = 0x0F,
        UPPER_LIMIT_3 = 0x10,
        LOWER_LIMIT_3 = 0x11,
        INT_CONFIG = 0x12,
        SLEEP_CONFIG = 0x13,
        ACQUIRE_TIME_CONFIG = 0x14,
        POWER_CONFIG = 0x15,
        RESET = 0x16,
    } ads7924_reg_t;

    uint8_t _device_address;

    bool read(uint8_t *output, ads7924_reg_t address, uint8_t length = 1);
    bool write(uint8_t *input, ads7924_reg_t address, uint8_t length = 1);

    void write_limit(uint16_t limit, uint8_t channel, bool is_high_limit);
    void read_limit(uint16_t &limit, uint8_t channel, bool is_high_limit);
};

#endif