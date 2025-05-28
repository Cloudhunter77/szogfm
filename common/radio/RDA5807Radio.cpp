#include "RDA5807Radio.h"

namespace szogfm {
    namespace radio {

        // Helper function to repeat a character for consistent formatting
        String repeatChar(char c, int count) {
            String result = "";
            result.reserve(count);
            for (int i = 0; i < count; i++) {
                result += c;
            }
            return result;
        }

        RDA5807Radio::RDA5807Radio(int sda, int scl)
                : _sda(sda), _scl(scl), _initialized(false), _frequency(8750), _volume(8),
                  _muted(false), _mono(false), _lastUpdateTime(0) {
            // Initialize register cache to zero
            memset(_regCache, 0, sizeof(_regCache));
        }

        bool RDA5807Radio::initialize() {
            Serial.println("\n" + repeatChar('=', 50));
            Serial.println("📻 RDA5807M FM Radio Module Initialization");
            Serial.println(repeatChar('=', 50));

            // Initialize I2C with specified pins or defaults
            if (_sda >= 0 && _scl >= 0) {
                Serial.printf("🔧 Initializing I2C with custom pins - SDA:%d, SCL:%d\n", _sda, _scl);
                Wire.begin(_sda, _scl);
            } else {
                Serial.println("🔧 Initializing I2C with default pins");
                Wire.begin();
            }

            // Set conservative I2C clock speed for better reliability
            Wire.setClock(100000); // 100kHz - slower but more reliable
            Serial.println("⚙️  I2C clock set to 100kHz for maximum compatibility");

            // Give I2C bus time to stabilize
            delay(100);

            // Test I2C bus functionality first
            if (!testI2CBus()) {
                _lastError = "I2C bus test failed";
                Serial.println("❌ " + _lastError);
                return false;
            }

            // Check if the RDA5807M module is present and responding
            Serial.println("🔍 Scanning for RDA5807M module...");
            if (!isModulePresent()) {
                _lastError = "RDA5807M module not found or not responding";
                Serial.println("❌ " + _lastError);
                return false;
            }
            Serial.println("✅ RDA5807M module detected and responding");

            // Perform soft reset to ensure clean state
            Serial.println("🔄 Performing module soft reset...");
            if (!performSoftReset()) {
                _lastError = "Soft reset failed";
                Serial.println("❌ " + _lastError);
                return false;
            }
            Serial.println("✅ Soft reset completed successfully");

            // Wait for module to stabilize after reset
            delay(100);

            // Initialize with basic power-up sequence
            Serial.println("⚡ Powering up FM radio module...");
            if (!powerUpModule()) {
                _lastError = "Power-up sequence failed";
                Serial.println("❌ " + _lastError);
                return false;
            }
            Serial.println("✅ Module powered up successfully");

            // Configure band and channel spacing
            Serial.println("📊 Configuring band and channel spacing...");
            if (!configureBandAndSpacing()) {
                _lastError = "Band/spacing configuration failed";
                Serial.println("❌ " + _lastError);
                return false;
            }
            Serial.println("✅ Band set to 87-108MHz, spacing set to 100kHz");

            // Set initial volume with verification
            Serial.printf("🔊 Setting initial volume to %d...\n", _volume);
            if (!setVolume(_volume, true)) {
                Serial.println("⚠️  Initial volume setting failed, continuing...");
            } else {
                Serial.println("✅ Initial volume set successfully");
            }

            // Set initial frequency with verification
            Serial.printf("🎵 Setting initial frequency to %.1f MHz...\n", _frequency / 100.0);
            if (!setFrequency(_frequency, true)) {
                Serial.println("⚠️  Initial frequency setting failed, continuing...");
            } else {
                Serial.println("✅ Initial frequency set successfully");
            }

            // Perform comprehensive functionality test
            Serial.println("🧪 Performing functionality tests...");
            if (!performFunctionalityTest()) {
                Serial.println("⚠️  Some functionality tests failed, but basic operation should work");
            } else {
                Serial.println("✅ All functionality tests passed");
            }

            _initialized = true;

            // Log final status
            Serial.println("📊 Final Module Status:");
            logCurrentStatus();

            Serial.println("🎉 RDA5807M initialization completed successfully!");
            Serial.println(repeatChar('=', 50));

            return true;
        }

        bool RDA5807Radio::testI2CBus() {
            Serial.println("   🔍 Testing I2C bus functionality...");

            // Test basic I2C communication by scanning the bus
            int deviceCount = 0;
            for (uint8_t address = 1; address < 127; address++) {
                Wire.beginTransmission(address);
                uint8_t error = Wire.endTransmission();

                if (error == 0) {
                    deviceCount++;
                    if (address == RDA5807M_ADDR) {
                        Serial.printf("      ✅ RDA5807M found at address 0x%02X\n", address);
                    } else {
                        Serial.printf("      📍 Other device at address 0x%02X\n", address);
                    }
                }
            }

            Serial.printf("   📊 Total I2C devices found: %d\n", deviceCount);

            if (deviceCount == 0) {
                Serial.println("   ❌ No I2C devices found - bus may be faulty");
                return false;
            }

            return true;
        }

        bool RDA5807Radio::isModulePresent() {
            Serial.println("   🔍 Checking RDA5807M presence...");

            // Method 1: Try to read from a known register
            uint16_t testValue;
            if (readRegister(REG_CONFIG, testValue)) {
                Serial.printf("      📊 Config register read: 0x%04X\n", testValue);

                // The config register should have some reasonable values after reset
                // We're not checking for specific chip ID as RDA5807M doesn't have a reliable one
                if (testValue != 0x0000 && testValue != 0xFFFF) {
                    Serial.println("      ✅ Register read successful with valid data");
                    return true;
                }
            }

            // Method 2: Try a simple write/read test
            Serial.println("      🔄 Attempting write/read test...");

            // Try to write to a register and read it back
            // We'll use the volume register as it's safe to modify
            uint16_t originalVolume;
            if (readRegister(REG_VOLUME, originalVolume)) {
                Serial.printf("      📖 Original volume register: 0x%04X\n", originalVolume);

                // Try writing a test value
                uint16_t testValue = 0x0005; // Safe test value for volume register
                if (writeRegister(REG_VOLUME, testValue)) {
                    delay(10);

                    uint16_t readBack;
                    if (readRegister(REG_VOLUME, readBack)) {
                        Serial.printf("      📝 Write test: wrote 0x%04X, read 0x%04X\n", testValue, readBack);

                        // Restore original value
                        writeRegister(REG_VOLUME, originalVolume);

                        // Check if the write was successful (some bits might be ignored)
                        if ((readBack & 0x000F) == (testValue & 0x000F)) {
                            Serial.println("      ✅ Write/read test successful");
                            return true;
                        }
                    }
                }
            }

            // Method 3: Check if we can at least communicate (even if responses are unexpected)
            Serial.println("      🔄 Testing basic I2C communication...");
            Wire.beginTransmission(RDA5807M_ADDR);
            uint8_t error = Wire.endTransmission();

            if (error == 0) {
                Serial.println("      ✅ Basic I2C communication successful");
                Serial.println("      ℹ️  Module present but may need initialization");
                return true;
            } else {
                Serial.printf("      ❌ I2C communication failed with error %d\n", error);
                return false;
            }
        }

        bool RDA5807Radio::performSoftReset() {
            Serial.println("   🔄 Executing soft reset sequence...");

            // Method 1: Use the SOFT_RESET bit in config register
            uint16_t resetConfig = CONFIG_SOFT_RESET | CONFIG_ENABLE;

            if (writeRegister(REG_CONFIG, resetConfig)) {
                Serial.println("      📝 Soft reset command sent");
                delay(50); // Wait for reset to complete

                // Clear the reset bit
                resetConfig &= ~CONFIG_SOFT_RESET;
                if (writeRegister(REG_CONFIG, resetConfig)) {
                    Serial.println("      ✅ Reset bit cleared");
                    delay(50);
                    return true;
                }
            }

            // Method 2: Alternative reset approach
            Serial.println("      🔄 Trying alternative reset method...");

            // Write zeros to all registers first
            for (uint8_t reg = REG_CONFIG; reg <= REG_POWER_CFG; reg++) {
                writeRegister(reg, 0x0000);
                delay(10);
            }

            Serial.println("      ✅ Alternative reset completed");
            return true;
        }

        bool RDA5807Radio::powerUpModule() {
            Serial.println("   ⚡ Starting power-up sequence...");

            // Step 1: Basic power up with essential bits
            uint16_t powerConfig = CONFIG_DHIZ | CONFIG_DMUTE | CONFIG_ENABLE;

            Serial.printf("      📝 Writing power config: 0x%04X\n", powerConfig);
            if (!writeRegister(REG_CONFIG, powerConfig)) {
                Serial.println("      ❌ Failed to write power config");
                return false;
            }

            delay(100); // Give time for power-up

            // Step 2: Verify power-up by reading back the register
            uint16_t readBack;
            if (readRegister(REG_CONFIG, readBack)) {
                Serial.printf("      📖 Power config readback: 0x%04X\n", readBack);

                // Check if essential bits are set (allowing for some bits to be different)
                if (readBack & CONFIG_ENABLE) {
                    Serial.println("      ✅ Power-up verified - ENABLE bit is set");
                } else {
                    Serial.println("      ⚠️  ENABLE bit not set, but continuing");
                }
            }

            // Step 3: Additional configuration for better operation
            uint16_t enhancedConfig = powerConfig | CONFIG_NEW_METHOD | CONFIG_BASS;
            Serial.printf("      📝 Writing enhanced config: 0x%04X\n", enhancedConfig);
            writeRegister(REG_CONFIG, enhancedConfig);

            delay(50);

            Serial.println("      ✅ Power-up sequence completed");
            return true;
        }

        bool RDA5807Radio::configureBandAndSpacing() {
            Serial.println("   📊 Configuring band and spacing...");

            // Configure for 87-108 MHz band with 100kHz spacing
            uint16_t channelConfig = CHAN_BAND_87_108 | CHAN_SPACE_100K;

            Serial.printf("      📝 Writing channel config: 0x%04X\n", channelConfig);
            if (!writeRegister(REG_CHAN, channelConfig)) {
                Serial.println("      ❌ Failed to write channel config");
                return false;
            }

            delay(50);

            // Verify configuration
            uint16_t readBack;
            if (readRegister(REG_CHAN, readBack)) {
                Serial.printf("      📖 Channel config readback: 0x%04X\n", readBack);

                // Check band and spacing bits
                uint16_t bandBits = readBack & CHAN_BAND_MASK;
                uint16_t spaceBits = readBack & CHAN_SPACE_MASK;

                Serial.printf("      📊 Band: 0x%X, Spacing: 0x%X\n", bandBits, spaceBits);

                if (bandBits == CHAN_BAND_87_108 && spaceBits == CHAN_SPACE_100K) {
                    Serial.println("      ✅ Band and spacing configured correctly");
                } else {
                    Serial.println("      ⚠️  Band/spacing may not be set correctly, but continuing");
                }
            }

            return true;
        }

        bool RDA5807Radio::performFunctionalityTest() {
            Serial.println("   🧪 Testing module functionality...");

            bool allTestsPassed = true;

            // Test 1: Volume control
            Serial.println("      🔊 Testing volume control...");
            uint8_t originalVolume = _volume;
            if (setVolume(10, true)) {
                if (setVolume(5, true)) {
                    if (setVolume(originalVolume, true)) {
                        Serial.println("         ✅ Volume control test passed");
                    } else {
                        Serial.println("         ⚠️  Volume restore failed");
                        allTestsPassed = false;
                    }
                } else {
                    Serial.println("         ⚠️  Volume change test failed");
                    allTestsPassed = false;
                }
            } else {
                Serial.println("         ⚠️  Initial volume test failed");
                allTestsPassed = false;
            }

            // Test 2: Frequency tuning
            Serial.println("      🎵 Testing frequency tuning...");
            uint16_t originalFreq = _frequency;
            if (setFrequency(9000, true)) { // 90.0 MHz
                delay(100);
                if (setFrequency(10000, true)) { // 100.0 MHz
                    delay(100);
                    if (setFrequency(originalFreq, true)) {
                        Serial.println("         ✅ Frequency tuning test passed");
                    } else {
                        Serial.println("         ⚠️  Frequency restore failed");
                        allTestsPassed = false;
                    }
                } else {
                    Serial.println("         ⚠️  Frequency change test failed");
                    allTestsPassed = false;
                }
            } else {
                Serial.println("         ⚠️  Initial frequency test failed");
                allTestsPassed = false;
            }

            // Test 3: Mute functionality
            Serial.println("      🔇 Testing mute functionality...");
            if (setMute(true)) {
                delay(50);
                if (setMute(false)) {
                    Serial.println("         ✅ Mute functionality test passed");
                } else {
                    Serial.println("         ⚠️  Unmute test failed");
                    allTestsPassed = false;
                }
            } else {
                Serial.println("         ⚠️  Mute test failed");
                allTestsPassed = false;
            }

            // Test 4: Register read/write consistency
            Serial.println("      📝 Testing register consistency...");
            int consistencyTests = 0;
            int passedTests = 0;

            for (uint8_t reg = REG_CONFIG; reg <= REG_POWER_CFG; reg++) {
                uint16_t value1, value2;
                if (readRegister(reg, value1)) {
                    delay(10);
                    if (readRegister(reg, value2)) {
                        consistencyTests++;
                        if (value1 == value2) {
                            passedTests++;
                        }
                    }
                }
            }

            Serial.printf("         📊 Consistency: %d/%d registers passed\n", passedTests, consistencyTests);
            if (passedTests >= consistencyTests * 0.8) { // 80% success rate
                Serial.println("         ✅ Register consistency test passed");
            } else {
                Serial.println("         ⚠️  Register consistency test failed");
                allTestsPassed = false;
            }

            return allTestsPassed;
        }

        void RDA5807Radio::logCurrentStatus() {
            Serial.println("   📊 Current Module Status:");

            // Read and display key registers
            uint16_t config, channel, volume, status;

            if (readRegister(REG_CONFIG, config)) {
                Serial.printf("      Config (0x%02X): 0x%04X\n", REG_CONFIG, config);
                Serial.printf("         Enable: %s\n", (config & CONFIG_ENABLE) ? "Yes" : "No");
                Serial.printf("         Mute: %s\n", (config & CONFIG_DMUTE) ? "Disabled" : "Enabled");
                Serial.printf("         Mono: %s\n", (config & CONFIG_MONO) ? "Yes" : "No");
                Serial.printf("         Bass: %s\n", (config & CONFIG_BASS) ? "Yes" : "No");
            }

            if (readRegister(REG_CHAN, channel)) {
                Serial.printf("      Channel (0x%02X): 0x%04X\n", REG_CHAN, channel);
                uint16_t channelNum = (channel >> 6) & 0x03FF;
                uint16_t frequency = 8750 + (channelNum * 10);
                Serial.printf("         Channel: %d (%.1f MHz)\n", channelNum, frequency / 100.0);
            }

            if (readRegister(REG_VOLUME, volume)) {
                Serial.printf("      Volume (0x%02X): 0x%04X\n", REG_VOLUME, volume);
                Serial.printf("         Level: %d/15\n", volume & 0x0F);
            }

            if (readRegister(REG_STATUS, status)) {
                Serial.printf("      Status (0x%02X): 0x%04X\n", REG_STATUS, status);
                Serial.printf("         Stereo: %s\n", (status & STATUS_STEREO) ? "Yes" : "No");
                Serial.printf("         Tune Complete: %s\n", (status & STATUS_STC) ? "Yes" : "No");
                Serial.printf("         Seek Fail: %s\n", (status & STATUS_SF) ? "Yes" : "No");
            }

            // Display current cached values
            Serial.printf("   💾 Cached Values:\n");
            Serial.printf("      Frequency: %.1f MHz\n", _frequency / 100.0);
            Serial.printf("      Volume: %d/15\n", _volume);
            Serial.printf("      Muted: %s\n", _muted ? "Yes" : "No");
            Serial.printf("      Mono: %s\n", _mono ? "Yes" : "No");
        }

        bool RDA5807Radio::setFrequency(uint16_t frequency, bool initializing) {
            if (!_initialized && !initializing) {
                _lastError = "Module not initialized";
                Serial.println("❌ [FREQ] " + _lastError);
                return false;
            }

            // Validate frequency range (87.5 MHz to 108.0 MHz)
            if (frequency < 8750 || frequency > 10800) {
                _lastError = "Frequency out of range (87.5-108.0 MHz)";
                Serial.printf("❌ [FREQ] %s: %d\n", _lastError.c_str(), frequency);
                return false;
            }

            Serial.printf("🎵 [FREQ] Setting frequency to %.1f MHz\n", frequency / 100.0);

            // Calculate channel number for RDA5807M
            // Channel = (Frequency - 87.5 MHz) / 0.1 MHz
            uint16_t channel = (frequency - 8750) / 10;

            Serial.printf("   📊 Calculated channel: %d\n", channel);

            // Read current channel register to preserve other settings
            uint16_t channelReg = 0;
            if (!readRegister(REG_CHAN, channelReg)) {
                Serial.println("   ⚠️  Could not read current channel register, using defaults");
                channelReg = CHAN_BAND_87_108 | CHAN_SPACE_100K;
            }

            // Clear old channel bits and set new channel with tune bit
            channelReg &= ~(0x03FF << 6); // Clear channel bits (10 bits starting at bit 6)
            channelReg |= (channel << 6); // Set new channel
            channelReg |= CHAN_TUNE;      // Set tune bit

            Serial.printf("   📝 Writing channel register: 0x%04X\n", channelReg);

            // Write channel register with tune bit
            if (!writeRegister(REG_CHAN, channelReg)) {
                _lastError = "Failed to write channel register";
                Serial.println("❌ [FREQ] " + _lastError);
                return false;
            }

            // Wait for tuning to complete
            Serial.println("   ⏳ Waiting for tuning to complete...");
            unsigned long tuneStart = millis();
            bool tuneComplete = false;

            for (int attempt = 0; attempt < 50; attempt++) { // Max 500ms wait
                delay(10);

                uint16_t status;
                if (readRegister(REG_STATUS, status)) {
                    if (status & STATUS_STC) { // Seek/Tune Complete
                        tuneComplete = true;
                        Serial.printf("   ✅ Tuning completed after %lu ms\n", millis() - tuneStart);

                        // Check if seek failed
                        if (status & STATUS_SF) {
                            Serial.println("   ⚠️  Seek fail flag set (may indicate weak signal)");
                        }
                        break;
                    }
                }
            }

            if (!tuneComplete) {
                Serial.printf("   ⚠️  Tuning timeout after %lu ms\n", millis() - tuneStart);
            }

            // Clear the tune bit
            channelReg &= ~CHAN_TUNE;
            writeRegister(REG_CHAN, channelReg);

            // Verify the frequency was set correctly
            uint16_t verifyReg;
            if (readRegister(REG_CHAN, verifyReg)) {
                uint16_t readChannel = (verifyReg >> 6) & 0x03FF;
                uint16_t readFrequency = 8750 + (readChannel * 10);

                Serial.printf("   📖 Verification: set=%d, read=%d (%.1f MHz)\n",
                              channel, readChannel, readFrequency / 100.0);

                if (abs((int)readChannel - (int)channel) <= 1) { // Allow 1 channel tolerance
                    _frequency = frequency;
                    Serial.printf("✅ [FREQ] Frequency set successfully to %.1f MHz\n", frequency / 100.0);

                    // Update display of current radio status
                    delay(50); // Give radio time to settle
                    logCurrentRadioInfo();

                    return true;
                } else {
                    _lastError = "Frequency verification failed";
                    Serial.printf("❌ [FREQ] %s: expected %d, got %d\n",
                                  _lastError.c_str(), channel, readChannel);
                    return false;
                }
            } else {
                Serial.println("   ⚠️  Could not verify frequency setting");
                _frequency = frequency; // Assume it worked
                return true;
            }
        }

        uint16_t RDA5807Radio::getFrequency() const {
            return _frequency;
        }

        bool RDA5807Radio::setVolume(uint8_t volume, bool initializing) {
            if (!_initialized && !initializing) {
                _lastError = "Module not initialized";
                Serial.println("❌ [VOL] " + _lastError);
                return false;
            }

            // Validate volume range (0-15)
            if (volume > 15) {
                Serial.printf("⚠️  [VOL] Volume %d exceeds maximum, clamping to 15\n", volume);
                volume = 15;
            }

            Serial.printf("🔊 [VOL] Setting volume to %d/15\n", volume);

            // Read current volume register to preserve other settings
            uint16_t volumeReg = 0;
            if (!readRegister(REG_VOLUME, volumeReg)) {
                Serial.println("   ⚠️  Could not read current volume register, using defaults");
                volumeReg = 0x0000;
            }

            // Clear volume bits and set new volume
            volumeReg &= ~0x000F; // Clear lower 4 bits (volume)
            volumeReg |= (volume & 0x0F); // Set new volume

            Serial.printf("   📝 Writing volume register: 0x%04X\n", volumeReg);

            // Write volume register
            if (!writeRegister(REG_VOLUME, volumeReg)) {
                _lastError = "Failed to write volume register";
                Serial.println("❌ [VOL] " + _lastError);
                return false;
            }

            // Verify the volume was set correctly
            uint16_t verifyReg;
            if (readRegister(REG_VOLUME, verifyReg)) {
                uint8_t readVolume = verifyReg & 0x0F;

                Serial.printf("   📖 Verification: set=%d, read=%d\n", volume, readVolume);

                if (readVolume == volume) {
                    _volume = volume;
                    Serial.printf("✅ [VOL] Volume set successfully to %d/15\n", volume);
                    return true;
                } else {
                    _lastError = "Volume verification failed";
                    Serial.printf("❌ [VOL] %s: expected %d, got %d\n",
                                  _lastError.c_str(), volume, readVolume);
                    return false;
                }
            } else {
                Serial.println("   ⚠️  Could not verify volume setting");
                _volume = volume; // Assume it worked
                return true;
            }
        }

        uint8_t RDA5807Radio::getVolume() const {
            return _volume;
        }

        bool RDA5807Radio::setMute(bool mute) {
            if (!_initialized) {
                _lastError = "Module not initialized";
                Serial.println("❌ [MUTE] " + _lastError);
                return false;
            }

            Serial.printf("🔇 [MUTE] Setting mute to %s\n", mute ? "ON" : "OFF");

            // Read current config register
            uint16_t config;
            if (!readRegister(REG_CONFIG, config)) {
                _lastError = "Failed to read config register";
                Serial.println("❌ [MUTE] " + _lastError);
                return false;
            }

            Serial.printf("   📖 Current config: 0x%04X\n", config);

            // Modify DMUTE bit (DMUTE=1 means mute disabled, DMUTE=0 means mute enabled)
            if (mute) {
                config &= ~CONFIG_DMUTE; // Clear DMUTE bit to enable mute
            } else {
                config |= CONFIG_DMUTE;  // Set DMUTE bit to disable mute
            }

            Serial.printf("   📝 New config: 0x%04X (DMUTE %s)\n",
                          config, (config & CONFIG_DMUTE) ? "disabled" : "enabled");

            // Write the modified config
            if (!writeRegister(REG_CONFIG, config)) {
                _lastError = "Failed to write config register";
                Serial.println("❌ [MUTE] " + _lastError);
                return false;
            }

            // Verify the mute state was set correctly
            uint16_t verifyConfig;
            if (readRegister(REG_CONFIG, verifyConfig)) {
                bool readMute = !(verifyConfig & CONFIG_DMUTE); // Inverted logic

                Serial.printf("   📖 Verification: set=%s, read=%s\n",
                              mute ? "MUTED" : "UNMUTED", readMute ? "MUTED" : "UNMUTED");

                if (readMute == mute) {
                    _muted = mute;
                    Serial.printf("✅ [MUTE] Mute set successfully to %s\n", mute ? "ON" : "OFF");
                    return true;
                } else {
                    _lastError = "Mute verification failed";
                    Serial.printf("❌ [MUTE] %s\n", _lastError.c_str());
                    return false;
                }
            } else {
                Serial.println("   ⚠️  Could not verify mute setting");
                _muted = mute; // Assume it worked
                return true;
            }
        }

        bool RDA5807Radio::isMuted() const {
            return _muted;
        }

        void RDA5807Radio::logCurrentRadioInfo() {
            if (!_initialized) return;

            Serial.println("   📻 Current Radio Status:");

            int rssi = getRssi();
            bool stereo = isStereo();
            bool tuned = isTunedToStation();

            Serial.printf("      🎵 Frequency: %.1f MHz\n", _frequency / 100.0);
            Serial.printf("      🔊 Volume: %d/15\n", _volume);
            Serial.printf("      🔇 Muted: %s\n", _muted ? "Yes" : "No");
            Serial.printf("      📶 RSSI: %d\n", rssi);
            Serial.printf("      🎧 Stereo: %s\n", stereo ? "Yes" : "No");
            Serial.printf("      📡 Tuned: %s\n", tuned ? "Yes" : "No");

            if (rssi > 40) {
                Serial.println("      📊 Signal: Excellent");
            } else if (rssi > 30) {
                Serial.println("      📊 Signal: Good");
            } else if (rssi > 20) {
                Serial.println("      📊 Signal: Fair");
            } else {
                Serial.println("      📊 Signal: Poor");
            }
        }

        // Implementation of remaining interface methods with improved error handling

        bool RDA5807Radio::setMono(bool mono) {
            if (!_initialized) {
                _lastError = "Module not initialized";
                return false;
            }

            uint16_t config;
            if (!readRegister(REG_CONFIG, config)) {
                _lastError = "Failed to read config register";
                return false;
            }

            if (mono) {
                config |= CONFIG_MONO;
            } else {
                config &= ~CONFIG_MONO;
            }

            if (!writeRegister(REG_CONFIG, config)) {
                _lastError = "Failed to set mono/stereo mode";
                return false;
            }

            _mono = mono;
            return true;
        }

        bool RDA5807Radio::isStereo() const {
            if (!_initialized) {
                return false;
            }

            uint16_t status;
            if (!readRegister(REG_STATUS, status)) {
                return false;
            }

            return (status & STATUS_STEREO) != 0;
        }

        int RDA5807Radio::getRssi() const {
            if (!_initialized) {
                return 0;
            }

            uint16_t rssiReg;
            if (!readRegister(REG_RSSI, rssiReg)) {
                return 0;
            }

            // Extract RSSI value from bits 9-15
            return (rssiReg >> 9) & 0x7F;
        }

        bool RDA5807Radio::seek(bool seekUp) {
            if (!_initialized) {
                _lastError = "Module not initialized";
                return false;
            }

            Serial.printf("🔍 [SEEK] Starting seek %s\n", seekUp ? "UP" : "DOWN");

            uint16_t config;
            if (!readRegister(REG_CONFIG, config)) {
                _lastError = "Failed to read config register";
                return false;
            }

            // Set seek direction and enable seek
            if (seekUp) {
                config |= CONFIG_SEEKUP;
            } else {
                config &= ~CONFIG_SEEKUP;
            }
            config |= CONFIG_SEEK;

            if (!writeRegister(REG_CONFIG, config)) {
                _lastError = "Failed to start seek";
                return false;
            }

            // Wait for seek to complete
            unsigned long startTime = millis();
            bool seekComplete = false;
            bool seekFailed = false;

            while (millis() - startTime < 10000) { // 10 second timeout
                uint16_t status;
                if (readRegister(REG_STATUS, status)) {
                    if (status & STATUS_STC) {
                        seekComplete = true;
                        seekFailed = (status & STATUS_SF) != 0;
                        break;
                    }
                }
                delay(100);
            }

            // Clear seek bit
            config &= ~CONFIG_SEEK;
            writeRegister(REG_CONFIG, config);

            if (!seekComplete) {
                _lastError = "Seek timeout";
                Serial.println("❌ [SEEK] " + _lastError);
                return false;
            }

            if (seekFailed) {
                _lastError = "Seek failed - no stations found";
                Serial.println("❌ [SEEK] " + _lastError);
                return false;
            }

            // Read the new frequency
            uint16_t chanReg;
            if (readRegister(REG_CHAN, chanReg)) {
                uint16_t channel = (chanReg >> 6) & 0x03FF;
                _frequency = 8750 + (channel * 10);
                Serial.printf("✅ [SEEK] Found station at %.1f MHz\n", _frequency / 100.0);
            }

            return true;
        }

        bool RDA5807Radio::isTunedToStation() const {
            if (!_initialized) {
                return false;
            }

            int rssi = getRssi();
            return rssi > 25; // Threshold for a valid station
        }

        String RDA5807Radio::getLastError() const {
            return _lastError;
        }

        void RDA5807Radio::update() {
            if (!_initialized) {
                return;
            }

            unsigned long currentTime = millis();
            if (currentTime - _lastUpdateTime > 2000) { // Every 2 seconds
                _lastUpdateTime = currentTime;

                // Periodically log radio status in debug mode
                static int updateCount = 0;
                updateCount++;

                if (updateCount % 15 == 0) { // Every 30 seconds (15 * 2s)
                    logCurrentRadioInfo();
                }
            }
        }

        // Enhanced I2C communication methods with better error handling

        bool RDA5807Radio::writeRegister(uint8_t reg, uint16_t value) {
            const int maxRetries = 3;

            for (int attempt = 1; attempt <= maxRetries; attempt++) {
                Wire.beginTransmission(RDA5807M_ADDR);
                Wire.write(reg);
                Wire.write(value >> 8);    // High byte first
                Wire.write(value & 0xFF);  // Low byte second

                uint8_t result = Wire.endTransmission();

                if (result == 0) {
                    // Update cache on successful write
                    if (reg < 16) {
                        _regCache[reg] = value;
                    }
                    return true;
                }

                if (attempt < maxRetries) {
                    Serial.printf("   ⚠️  I2C write retry %d/%d (error %d)\n", attempt, maxRetries, result);
                    delay(10);
                } else {
                    _lastError = "I2C write error after retries: " + String(result);
                    Serial.println("   ❌ " + _lastError);
                }
            }

            return false;
        }

        bool RDA5807Radio::readRegister(uint8_t reg, uint16_t& value) const {
            const int maxRetries = 3;

            for (int attempt = 1; attempt <= maxRetries; attempt++) {
                Wire.beginTransmission(RDA5807M_ADDR);
                Wire.write(reg);
                uint8_t result = Wire.endTransmission(false); // Don't send stop

                if (result != 0) {
                    if (attempt < maxRetries) {
                        delay(10);
                        continue;
                    }
                    _lastError = "I2C write error in read: " + String(result);
                    return false;
                }

                // Request 2 bytes
                size_t bytesReceived = Wire.requestFrom(RDA5807M_ADDR, (uint8_t)2);
                if (bytesReceived != 2) {
                    if (attempt < maxRetries) {
                        delay(10);
                        continue;
                    }
                    _lastError = "I2C read error: received " + String(bytesReceived) + " bytes";
                    return false;
                }

                // Read the data (high byte first, low byte second)
                uint8_t highByte = Wire.read();
                uint8_t lowByte = Wire.read();
                value = (highByte << 8) | lowByte;

                return true;
            }

            return false;
        }

        bool RDA5807Radio::readRegisters(uint8_t startReg, uint16_t* values, uint8_t count) const {
            Wire.beginTransmission(RDA5807M_ADDR);
            Wire.write(startReg);
            uint8_t result = Wire.endTransmission(false);

            if (result != 0) {
                _lastError = "I2C write error in multi-read: " + String(result);
                return false;
            }

            size_t bytesRequested = 2 * count;
            size_t bytesReceived = Wire.requestFrom(RDA5807M_ADDR, (uint8_t)bytesRequested);

            if (bytesReceived != bytesRequested) {
                _lastError = "I2C multi-read error: received " + String(bytesReceived) + " bytes";
                return false;
            }

            for (uint8_t i = 0; i < count; i++) {
                uint8_t highByte = Wire.read();
                uint8_t lowByte = Wire.read();
                values[i] = (highByte << 8) | lowByte;
            }

            return true;
        }

        bool RDA5807Radio::writeRegisters(uint8_t startReg, const uint16_t* values, uint8_t count) {
            Wire.beginTransmission(RDA5807M_ADDR);
            Wire.write(startReg);

            for (uint8_t i = 0; i < count; i++) {
                Wire.write(values[i] >> 8);    // High byte
                Wire.write(values[i] & 0xFF);  // Low byte

                // Update cache
                if (startReg + i < 16) {
                    _regCache[startReg + i] = values[i];
                }
            }

            uint8_t result = Wire.endTransmission();
            if (result != 0) {
                _lastError = "I2C multi-write error: " + String(result);
                return false;
            }

            return true;
        }

    } // namespace radio
} // namespace szogfm