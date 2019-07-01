#include <Arduino.h>
#include <Wire.h>

const uint8_t DEFAULT_ADS7924_ADDRESS = 0b1001000;

enum ADS7924_MODE {
    ADS7924_IDLE = 0,                          //
    ADS7924_AWAKE = 0b100000,                  //
    ADS7924_MANUAL_SINGLE = 0b110000,          //
    ADS7924_MANUAL_SCAN = 0b110010,            //
    ADS7924_AUTO_SINGLE = 0b110001,            //
    ADS7924_AUTO_SCAN = 0b110011,              //
    ADS7924_AUTO_SINGLE_SLEEP = 0b111001,      //
    ADS7924_AUTO_SCAN_SLEEP = 0b111011,        //
    ADS7924_AUTO_BURST_SCAN_SLEEP = 0b111111,  //
};

typedef union {
    uint8_t raw;
    struct {
        uint8_t mode : 6;
        uint8_t channel : 2;
    };
} ads7924_mode_t;

typedef union {
    uint8_t raw;
    struct {
        uint8_t ch0_enable : 1;
        uint8_t ch1_enable : 1;
        uint8_t ch2_enable : 1;
        uint8_t ch3_enable : 1;
        uint8_t ch0_status : 1;
        uint8_t ch1_status : 1;
        uint8_t ch2_status : 1;
        uint8_t ch3_status : 1;
    };
} ads7924_int_control_t;

///////////////////////////////////////////////////////////////////////////////
// Interrupt config

enum ADS7924_INTERRUPT_POLARITY {
    ADS7924_ACTIVE_HIGH = 1,
    ADS7924_ACTIVE_LOW = 0,
};

enum ADS7924_INTERRUPT_MODE {
    ADS7924_STATIC_INTERRUPT = 1,
    ADS7924_PULSE_INTERRUPT = 0,
};

enum ADS7924_INTERRUPT_OUTPUT {
    ADS7924_ALARM = 0,
    ADS7924_ALARM_BUSY = 1,
    ADS7924_DATA_READY_SINGLE = 2,
    ADS7924_SINGLE_CONVERSION_BUSY = 3,
    ADS7924_DATA_READY_ALL = 6,
    ADS7924_ALL_CONVERSION_BUSY = 7,
};

typedef union {
    uint8_t raw;
    struct {
        uint8_t int_trigger_mode : 1;
        uint8_t int_polarity : 1;
        uint8_t int_output : 3;
        uint8_t alarm_threshold : 3;
    };
} ads7924_int_config_t;

///////////////////////////////////////////////////////////////////////////////
// Sleep config

enum ADS7924_CONVERSION_CONTROL {
    ADS7924_CONTROL_INDEPENDENT = 0,
    ADS7924_CONTROL_DEPENDENT = 1,
};

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
        uint8_t sleep_time : 2;
        uint8_t _reserved1 : 1;
        uint8_t sleep_divider_x8 : 1;
        uint8_t sleep_divider_x4 : 1;
        uint8_t conversion_control : 1;
        uint8_t _reserved0 : 1;
    };
} ads7924_sleep_config_t;

enum ADS7924_CALIBRATION_CONTROL {
    ADS7924_MUXOUT_TO_CH3 = 0,
    ADS7924_MUXOUT_TO_AGND = 1,
};

typedef union {
    uint8_t raw;
    struct {
        uint8_t power_up_time : 5;
        uint8_t power_control_enable : 1;
        uint8_t power_control_polarity : 1;
        uint8_t calibration_control : 1;
    };
} ads7924_power_config_t;

typedef struct {
    ads7924_int_config_t int_config;
    ads7924_power_config_t power_config;
    ads7924_sleep_config_t sleep_config;
    uint8_t acquire_time;
} ads7924_config_t;

class ADS7924 {
   public:
    ADS7924();
    static ads7924_config_t config;
    void set_address(uint8_t address);
    void write_config(ads7924_config_t new_config = config);
    void set_mode(ads7924_mode_t mode);
    void write_interrupt_control(ads7924_int_control_t control_settings);
    ads7924_int_control_t read_interrupt_control();
    uint16_t get_reading(uint8_t channel);
    void get_readings(uint16_t *readings);
    void set_limit(uint16_t limit, uint8_t channel, bool high = false);
    void set_limits(uint16_t *limits);
    uint16_t get_limit(uint8_t channel, bool high = false);
    uint16_t get_limits(uint16_t *limits);

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

    uint8_t device_address = DEFAULT_ADS7924_ADDRESS;

    // Read from the sensor's registers
    bool read(uint8_t *output, ads7924_reg_t address, uint8_t length = 1);

    // Write to the sensor's registers
    bool write(uint8_t *input, ads7924_reg_t address, uint8_t length = 1);
}