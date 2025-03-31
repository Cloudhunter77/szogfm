#include "RDA5807Radio.h"

namespace szogfm {
    namespace radio {

        RDA5807Radio::RDA5807Radio(int sda, int scl)
                : _sda(sda), _scl(scl), _initialized(false), _frequency(8750), _volume(8),
                  _muted(false), _mono(false), _lastUpdateTime(0) {
            // Initialize register cache
            memset(_regCache, 0, sizeof(_regCache));
        }

        bool RDA5807Radio::initialize() {
            // Initialize I2C if pins are specified
            if (_sda >= 0 && _scl >= 0) {
                Wire.begin(_sda, _scl);
            } else {
                Wire.begin(); // Use default I2C pins
            }

            // Check if the module is present
            if (!isModulePresent()) {
                _lastError = "RDA5807M module not found";
                return false;
            }

            // Reset the module
            uint16_t config = CONFIG_ENABLE | CONFIG_SOFT_RESET;
            if (!writeRegister(REG_CONFIG, config)) {
                _lastError = "Failed to reset module";
                return false;
            }

            // Wait for the reset to complete
            delay(50);

            // Initialize with default settings
            config = CONFIG_ENABLE | CONFIG_DHIZ | CONFIG_DMUTE;
            if (!writeRegister(REG_CONFIG, config)) {
                _lastError = "Failed to initialize module";
                return false;
            }

            // Set initial band and channel spacing
            uint16_t chan = CHAN_BAND_87_108 | CHAN_SPACE_100K;
            if (!writeRegister(REG_CHAN, chan)) {
                _lastError = "Failed to set band and spacing";
                return false;
            }

            // Set initial volume
            if (!setVolume(_volume)) {
                return false;
            }

            // Set initial frequency
            if (!setFrequency(_frequency)) {
                return false;
            }

            _initialized = true;
            return true;
        }

        bool RDA5807Radio::setFrequency(uint16_t frequency) {
            if (!_initialized) {
                _lastError = "Module not initialized";
                return false;
            }

            // Validate frequency range (87.5 MHz to 108.0 MHz)
            if (frequency < 8750 || frequency > 10800) {
                _lastError = "Frequency out of range";
                return false;
            }

            // Calculate channel number
            // Channel = (Frequency - 87.5 MHz) / Channel Spacing
            uint16_t channel = (frequency - 8750) / 10;

            // Read current channel register value
            uint16_t chanReg;
            if (!readRegister(REG_CHAN, chanReg)) {
                _lastError = "Failed to read channel register";
                return false;
            }

            // Clear channel bits and set new channel
            chanReg &= ~0xFFC0; // Clear the channel bits (bits 6-15)
            chanReg |= (channel << 6); // Set channel (bits 6-15)
            chanReg |= CHAN_TUNE; // Set tune bit

            // Write the new channel
            if (!writeRegister(REG_CHAN, chanReg)) {
                _lastError = "Failed to set frequency";
                return false;
            }

            // Wait for tuning to complete
            unsigned long startTime = millis();
            bool tuneComplete = false;
            while (millis() - startTime < 100) { // Timeout after 100ms
                uint16_t status;
                if (readRegister(REG_STATUS, status) && (status & STATUS_STC)) {
                    tuneComplete = true;
                    break;
                }
                delay(5);
            }

            // Clear the tune bit
            chanReg &= ~CHAN_TUNE;
            if (!writeRegister(REG_CHAN, chanReg)) {
                _lastError = "Failed to clear tune bit";
                return false;
            }

            if (!tuneComplete) {
                _lastError = "Tuning timeout";
                return false;
            }

            // Store the new frequency
            _frequency = frequency;

            return true;
        }

        uint16_t RDA5807Radio::getFrequency() const {
            return _frequency;
        }

        bool RDA5807Radio::setVolume(uint8_t volume) {
            if (!_initialized) {
                _lastError = "Module not initialized";
                return false;
            }

            // Validate volume range (0-15)
            if (volume > 15) {
                volume = 15;
            }

            // Read current volume register
            uint16_t volReg;
            if (!readRegister(REG_VOLUME, volReg)) {
                _lastError = "Failed to read volume register";
                return false;
            }

            // Clear volume bits and set new volume
            volReg &= ~0x000F; // Clear volume bits (bits 0-3)
            volReg |= volume;  // Set new volume

            // Write the new volume
            if (!writeRegister(REG_VOLUME, volReg)) {
                _lastError = "Failed to set volume";
                return false;
            }

            // Store the new volume
            _volume = volume;

            return true;
        }

        uint8_t RDA5807Radio::getVolume() const {
            return _volume;
        }

        bool RDA5807Radio::setMute(bool mute) {
            if (!_initialized) {
                _lastError = "Module not initialized";
                return false;
            }

            // Read current config
            uint16_t config;
            if (!readRegister(REG_CONFIG, config)) {
                _lastError = "Failed to read config register";
                return false;
            }

            // Set or clear DMUTE bit (setting DMUTE disables mute)
            if (mute) {
                config &= ~CONFIG_DMUTE; // Clear DMUTE bit to enable mute
            } else {
                config |= CONFIG_DMUTE;  // Set DMUTE bit to disable mute
            }

            // Write the new config
            if (!writeRegister(REG_CONFIG, config)) {
                _lastError = "Failed to set mute state";
                return false;
            }

            // Store the new mute state
            _muted = mute;

            return true;
        }

        bool RDA5807Radio::isMuted() const {
            return _muted;
        }

        bool RDA5807Radio::setMono(bool mono) {
            if (!_initialized) {
                _lastError = "Module not initialized";
                return false;
            }

            // Read current config
            uint16_t config;
            if (!readRegister(REG_CONFIG, config)) {
                _lastError = "Failed to read config register";
                return false;
            }

            // Set or clear MONO bit
            if (mono) {
                config |= CONFIG_MONO;  // Set MONO bit
            } else {
                config &= ~CONFIG_MONO; // Clear MONO bit
            }

            // Write the new config
            if (!writeRegister(REG_CONFIG, config)) {
                _lastError = "Failed to set mono/stereo mode";
                return false;
            }

            // Store the new mono state
            _mono = mono;

            return true;
        }

        bool RDA5807Radio::isStereo() const {
            if (!_initialized) {
                return false;
            }

            // Read status register
            uint16_t status;
            if (!readRegister(REG_STATUS, status)) {
                _lastError = "Failed to read status register";
                return false;
            }

            // Check stereo indicator bit
            return (status & STATUS_STEREO) != 0;
        }

        int RDA5807Radio::getRssi() const {
            if (!_initialized) {
                return 0;
            }

            // Read status register
            uint16_t status;
            if (!readRegister(REG_STATUS, status)) {
                _lastError = "Failed to read status register";
                return 0;
            }

            // Extract RSSI value (bits 9-15 in the RSSI register)
            uint16_t rssiReg;
            if (!readRegister(REG_RSSI, rssiReg)) {
                _lastError = "Failed to read RSSI register";
                return 0;
            }

            return (rssiReg >> 9) & 0x7F;
        }

        bool RDA5807Radio::seek(bool seekUp) {
            if (!_initialized) {
                _lastError = "Module not initialized";
                return false;
            }

            // Read current config
            uint16_t config;
            if (!readRegister(REG_CONFIG, config)) {
                _lastError = "Failed to read config register";
                return false;
            }

            // Set seek direction and enable seek
            if (seekUp) {
                config |= CONFIG_SEEKUP; // Set seek up bit
            } else {
                config &= ~CONFIG_SEEKUP; // Clear seek up bit for seek down
            }
            config |= CONFIG_SEEK; // Set seek enable bit

            // Write the new config
            if (!writeRegister(REG_CONFIG, config)) {
                _lastError = "Failed to start seek";
                return false;
            }

            // Wait for seek to complete
            unsigned long startTime = millis();
            bool seekComplete = false;
            bool seekFailed = false;

            while (millis() - startTime < 5000) { // 5 second timeout
                uint16_t status;
                if (readRegister(REG_STATUS, status)) {
                    if (status & STATUS_STC) { // Seek/tune complete
                        seekComplete = true;
                        seekFailed = (status & STATUS_SF) != 0; // Seek failed flag
                        break;
                    }
                }
                delay(50);
            }

            // Read the new frequency if seek was successful
            if (seekComplete && !seekFailed) {
                uint16_t chanReg;
                if (readRegister(REG_CHAN, chanReg)) {
                    uint16_t channel = (chanReg >> 6) & 0x03FF; // Extract channel bits
                    _frequency = 8750 + (channel * 10); // Convert channel to frequency
                }
            }

            // Clear seek bit
            config &= ~CONFIG_SEEK;
            writeRegister(REG_CONFIG, config);

            if (!seekComplete) {
                _lastError = "Seek timeout";
                return false;
            }

            if (seekFailed) {
                _lastError = "Seek failed - no stations found";
                return false;
            }

            return true;
        }

        bool RDA5807Radio::isTunedToStation() const {
            if (!_initialized) {
                return false;
            }

            // Read status register
            uint16_t status;
            if (!readRegister(REG_STATUS, status)) {
                _lastError = "Failed to read status register";
                return false;
            }

            // Check RSSI level to determine if a station is tuned
            int rssi = getRssi();
            return rssi > 20; // Threshold for a valid station
        }

        String RDA5807Radio::getLastError() const {
            return _lastError;
        }

        void RDA5807Radio::update() {
            // Perform periodic updates
            if (!_initialized) {
                return;
            }

            // Only update once per second to avoid excessive I2C traffic
            unsigned long currentTime = millis();
            if (currentTime - _lastUpdateTime > 1000) {
                _lastUpdateTime = currentTime;

                // Check signal strength and stereo status
                uint16_t status;
                if (readRegister(REG_STATUS, status)) {
                    // Could log or store this information if needed
                }
            }
        }

        bool RDA5807Radio::writeRegister(uint8_t reg, uint16_t value) {
            // Start I2C transmission
            Wire.beginTransmission(RDA5807M_ADDR);
            Wire.write(reg);
            Wire.write(value >> 8);    // High byte
            Wire.write(value & 0xFF);  // Low byte

            // Check for errors
            uint8_t result = Wire.endTransmission();
            if (result != 0) {
                _lastError = "I2C write error: " + String(result);
                return false;
            }

            // Update cache
            _regCache[reg] = value;

            return true;
        }

        bool RDA5807Radio::readRegister(uint8_t reg, uint16_t& value) const {
            // Start I2C transmission
            Wire.beginTransmission(RDA5807M_ADDR);
            Wire.write(reg);
            uint8_t result = Wire.endTransmission(false); // Don't send stop condition

            if (result != 0) {
                _lastError = "I2C write error: " + String(result);
                return false;
            }

            // Request 2 bytes from the register
            if (Wire.requestFrom(RDA5807M_ADDR, 2) != 2) {
                _lastError = "I2C read error: not enough bytes";
                return false;
            }

            // Read the data
            uint8_t highByte = Wire.read();
            uint8_t lowByte = Wire.read();
            value = (highByte << 8) | lowByte;

            return true;
        }

        bool RDA5807Radio::readRegisters(uint8_t startReg, uint16_t* values, uint8_t count) const {
            // Start I2C transmission
            Wire.beginTransmission(RDA5807M_ADDR);
            Wire.write(startReg);
            uint8_t result = Wire.endTransmission(false); // Don't send stop condition

            if (result != 0) {
                _lastError = "I2C write error: " + String(result);
                return false;
            }

            // Request 2*count bytes from the registers
            if (Wire.requestFrom(RDA5807M_ADDR, 2 * count) != 2 * count) {
                _lastError = "I2C read error: not enough bytes";
                return false;
            }

            // Read the data
            for (uint8_t i = 0; i < count; i++) {
                uint8_t highByte = Wire.read();
                uint8_t lowByte = Wire.read();
                values[i] = (highByte << 8) | lowByte;
            }

            return true;
        }

        bool RDA5807Radio::writeRegisters(uint8_t startReg, const uint16_t* values, uint8_t count) {
            // Start I2C transmission
            Wire.beginTransmission(RDA5807M_ADDR);
            Wire.write(startReg);

            for (uint8_t i = 0; i < count; i++) {
                Wire.write(values[i] >> 8);    // High byte
                Wire.write(values[i] & 0xFF);  // Low byte

                // Update cache
                _regCache[startReg + i] = values[i];
            }

            // Check for errors
            uint8_t result = Wire.endTransmission();
            if (result != 0) {
                _lastError = "I2C write error: " + String(result);
                return false;
            }

            return true;
        }

        bool RDA5807Radio::updateConfig() {
            // Read current config
            uint16_t config;
            if (!readRegister(REG_CONFIG, config)) {
                _lastError = "Failed to read config register";
                return false;
            }

            // Update with current settings
            if (_muted) {
                config &= ~CONFIG_DMUTE;
            } else {
                config |= CONFIG_DMUTE;
            }

            if (_mono) {
                config |= CONFIG_MONO;
            } else {
                config &= ~CONFIG_MONO;
            }

            // Write updated config
            return writeRegister(REG_CONFIG, config);
        }

        bool RDA5807Radio::isModulePresent() const {
            // Try to read the chip ID register
            uint16_t chipId;
            if (!readRegister(REG_CHIP_ID, chipId)) {
                return false;
            }

            // Check if the chip ID matches
            return (chipId & 0xFF00) == 0x5800; // RDA5807M chip ID should be 0x58xx
        }

    } // namespace radio
} // namespace szogfm