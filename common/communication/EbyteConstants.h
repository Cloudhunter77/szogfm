#ifndef SZOGFM_EBYTECONSTANTS_H
#define SZOGFM_EBYTECONSTANTS_H

namespace szogfm {
    namespace communication {

// UART baud rates for the E49-400T20D module
        enum UartBaudRate {
            UART_1200 = 0b000,     // 1200 baud
            UART_2400 = 0b001,     // 2400 baud
            UART_4800 = 0b010,     // 4800 baud
            UART_9600 = 0b011,     // 9600 baud (default)
            UART_19200 = 0b100,    // 19200 baud
            UART_38400 = 0b101,    // 38400 baud
            UART_57600 = 0b110,    // 57600 baud
            UART_115200 = 0b111    // 115200 baud
        };

// Air data rates for the E49-400T20D module
        enum AirDataRate {
            AIR_1K2 = 0b000,   // 1.2k baud
            AIR_2K4 = 0b001,   // 2.4k baud (default)
            AIR_4K8 = 0b010,   // 4.8k baud
            AIR_9K6 = 0b011,   // 9.6k baud
            AIR_19K2 = 0b100,  // 19.2k baud
            AIR_50K = 0b101,   // 50k baud
            AIR_100K = 0b110,  // 100k baud
            AIR_200K = 0b111   // 200k baud
        };

// Operating modes for the E49-400T20D module
        enum OperatingMode {
            MODE_NORMAL = 0,    // Normal mode (transmit/receive)
            MODE_WAKEUP = 1,    // Wake-up mode
            MODE_POWERDOWN = 2, // Power saving mode (actually configuration mode in E49)
            MODE_SLEEP = 3      // Sleep mode
        };

// Parameter types for module configuration
        enum ParamSaveType {
            TEMPORARY = 0,  // Temporary storage (lost on reset)
            PERMANENT = 1   // Save to EEPROM (persists on reset)
        };

    } // namespace communication
} // namespace szogfm

#endif // SZOGFM_EBYTECONSTANTS_H