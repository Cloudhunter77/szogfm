#ifndef SZOGFM_RDA5807RADIO_H
#define SZOGFM_RDA5807RADIO_H

#include "IFmRadio.h"
#include <Wire.h>

namespace szogfm {
    namespace radio {

/**
 * Implementation of the FM radio interface for RDA5807M modules
 */
        class RDA5807Radio : public IFmRadio {
        public:
            /**
             * Constructor
             * @param sda SDA pin for I2C communication
             * @param scl SCL pin for I2C communication
             */
            RDA5807Radio(int sda = -1, int scl = -1);

            /**
             * Destructor
             */
            ~RDA5807Radio() override = default;

            // Interface implementations
            bool initialize() override;
            bool setFrequency(uint16_t frequency, bool initializing = false) override;
            uint16_t getFrequency() const override;
            bool setVolume(uint8_t volume, bool initializing = false) override;
            uint8_t getVolume() const override;
            bool setMute(bool mute) override;
            bool isMuted() const override;
            bool setMono(bool mono) override;
            bool isStereo() const override;
            int getRssi() const override;
            bool seek(bool seekUp) override;
            bool isTunedToStation() const override;
            String getLastError() const override;
            void update() override;

        private:
            // RDA5807M registers and constants
            static constexpr uint8_t RDA5807M_ADDR = 0x11;  // I2C address
            static constexpr uint8_t REG_CHIP_ID = 0x00;    // Chip ID register
            static constexpr uint16_t CHIP_ID = 0x5801;     // Expected chip ID

            // Register addresses
            static constexpr uint8_t REG_CONFIG = 0x02;     // Configuration register
            static constexpr uint8_t REG_CHAN = 0x03;       // Channel register
            static constexpr uint8_t REG_CARRIER = 0x04;    // Carrier register
            static constexpr uint8_t REG_VOLUME = 0x05;     // Volume register
            static constexpr uint8_t REG_INT_MODE = 0x06;   // INT mode register
            static constexpr uint8_t REG_POWER_CFG = 0x07;  // Power configuration register
            static constexpr uint8_t REG_STATUS = 0x0A;     // Status register
            static constexpr uint8_t REG_RSSI = 0x0B;       // RSSI register

            // Configuration bits
            static constexpr uint16_t CONFIG_DHIZ = 0x8000;      // Audio output high-z disable
            static constexpr uint16_t CONFIG_DMUTE = 0x4000;     // Mute disable
            static constexpr uint16_t CONFIG_MONO = 0x2000;      // Mono select
            static constexpr uint16_t CONFIG_BASS = 0x1000;      // Bass boost
            static constexpr uint16_t CONFIG_RCLK_EN = 0x0800;   // RCLK enable
            static constexpr uint16_t CONFIG_RCLK_DIR = 0x0400;  // RCLK direct input
            static constexpr uint16_t CONFIG_SEEKUP = 0x0200;    // Seek direction
            static constexpr uint16_t CONFIG_SEEK = 0x0100;      // Seek enable
            static constexpr uint16_t CONFIG_SKMODE = 0x0080;    // Seek mode
            static constexpr uint16_t CONFIG_RDS_EN = 0x0008;    // RDS enable
            static constexpr uint16_t CONFIG_NEW_METHOD = 0x0004; // New demodulation method
            static constexpr uint16_t CONFIG_SOFT_RESET = 0x0002; // Soft reset
            static constexpr uint16_t CONFIG_ENABLE = 0x0001;    // Power up enable

            // Channel register bits
            static constexpr uint16_t CHAN_TUNE = 0x0010;        // Tune enable
            static constexpr uint16_t CHAN_BAND_MASK = 0x000C;   // Band mask
            static constexpr uint16_t CHAN_BAND_87_108 = 0x0000; // 87-108 MHz band
            static constexpr uint16_t CHAN_SPACE_MASK = 0x0003;  // Channel spacing mask
            static constexpr uint16_t CHAN_SPACE_100K = 0x0000;  // 100 kHz spacing

            // Status register bits
            static constexpr uint16_t STATUS_RDSR = 0x8000;      // RDS ready
            static constexpr uint16_t STATUS_STC = 0x4000;       // Seek/tune complete
            static constexpr uint16_t STATUS_SF = 0x2000;        // Seek fail
            static constexpr uint16_t STATUS_RDSS = 0x1000;      // RDS synchronized
            static constexpr uint16_t STATUS_STEREO = 0x0400;    // Stereo indicator

            // Member variables
            int _sda;
            int _scl;
            bool _initialized;
            uint16_t _frequency;
            uint8_t _volume;
            bool _muted;
            bool _mono;
            mutable String _lastError;
            unsigned long _lastUpdateTime;

            // Register cache to avoid unnecessary I2C operations
            uint16_t _regCache[16];

            /**
             * Write a 16-bit value to a register
             * @param reg Register address
             * @param value Value to write
             * @return true if successful, false otherwise
             */
            bool writeRegister(uint8_t reg, uint16_t value);

            /**
             * Read a 16-bit value from a register
             * @param reg Register address
             * @param[out] value Variable to store the read value
             * @return true if successful, false otherwise
             */
            bool readRegister(uint8_t reg, uint16_t& value) const;

            /**
             * Read multiple 16-bit values from consecutive registers
             * @param startReg Starting register address
             * @param values Array to store the read values
             * @param count Number of registers to read
             * @return true if successful, false otherwise
             */
            bool readRegisters(uint8_t startReg, uint16_t* values, uint8_t count) const;

            /**
             * Write multiple 16-bit values to consecutive registers
             * @param startReg Starting register address
             * @param values Array of values to write
             * @param count Number of registers to write
             * @return true if successful, false otherwise
             */
            bool writeRegisters(uint8_t startReg, const uint16_t* values, uint8_t count);

            /**
             * Update the configuration register with the current settings
             * @return true if successful, false otherwise
             */
            bool updateConfig();

            /**
             * Check if the module is present and responding
             * @return true if present, false otherwise
             */
            bool isModulePresent() const;
        };

    } // namespace radio
} // namespace szogfm

#endif // SZOGFM_RDA5807RADIO_H