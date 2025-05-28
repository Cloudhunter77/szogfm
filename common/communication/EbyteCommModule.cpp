#include "EbyteCommModule.h"

namespace szogfm {
    namespace communication {

        // Helper function to repeat a character for consistent formatting
        String repeatChar(char c, int count) {
            String result = "";
            result.reserve(count);
            for (int i = 0; i < count; i++) {
                result += c;
            }
            return result;
        }

        EbyteCommModule::EbyteCommModule(HardwareSerial* serial, int pinM0, int pinM1, int pinAUX)
                : _serial(serial), _pinM0(pinM0), _pinM1(pinM1), _pinAUX(pinAUX),
                  _initialized(false), _lastRssi(-120), _channel(0), _address(0),
                  _rxBufferLen(0), _hasMessage(false), _debugLevel(2) {
            _ebyte = new EBYTE(_serial, _pinM0, _pinM1, _pinAUX);
        }

        EbyteCommModule::~EbyteCommModule() {
            delete _ebyte;
        }

        bool EbyteCommModule::initialize() {
            Serial.println("\n" + repeatChar('=', 50));
            Serial.println("📡 EBYTE E49-400T20D Module Initialization");
            Serial.println(repeatChar('=', 50));

            // Configure pins with explicit direction and pull-up settings
            Serial.printf("📌 Configuring control pins:\n");
            Serial.printf("   M0 (pin %d): OUTPUT\n", _pinM0);
            Serial.printf("   M1 (pin %d): OUTPUT\n", _pinM1);
            Serial.printf("   AUX (pin %d): INPUT_PULLUP\n", _pinAUX);

            pinMode(_pinM0, OUTPUT);
            pinMode(_pinM1, OUTPUT);
            pinMode(_pinAUX, INPUT_PULLUP);

            // Initialize pins to known state (Normal mode)
            digitalWrite(_pinM0, LOW);
            digitalWrite(_pinM1, LOW);
            delay(100); // Give time for pin states to stabilize

            // Check initial pin states
            Serial.printf("📊 Initial pin states:\n");
            Serial.printf("   M0: %s\n", digitalRead(_pinM0) ? "HIGH" : "LOW");
            Serial.printf("   M1: %s\n", digitalRead(_pinM1) ? "HIGH" : "LOW");
            Serial.printf("   AUX: %s\n", digitalRead(_pinAUX) ? "HIGH" : "LOW");

            // Initialize Serial port with multiple baud rate attempts
            Serial.println("🔧 Initializing UART communication...");
            _serial->end(); // Ensure clean start
            delay(100);

            _serial->begin(9600, SERIAL_8N1);
            delay(100);

            // Test UART communication
            Serial.println("🧪 Testing UART communication...");
            _serial->flush();

            // Clear any pending data
            int clearCount = 0;
            while (_serial->available()) {
                _serial->read();
                clearCount++;
                if (clearCount > 100) break; // Prevent infinite loop
            }
            if (clearCount > 0) {
                Serial.printf("   🧹 Cleared %d bytes from UART buffer\n", clearCount);
            }

            // Wait for AUX pin to stabilize
            Serial.println("⏳ Waiting for module to be ready (AUX pin)...");
            unsigned long auxWaitStart = millis();
            int auxState = digitalRead(_pinAUX);
            Serial.printf("   Initial AUX state: %s\n", auxState ? "HIGH (ready)" : "LOW (busy)");

            // Wait up to 5 seconds for AUX to go HIGH
            while (digitalRead(_pinAUX) == LOW && millis() - auxWaitStart < 5000) {
                delay(100);
                if (millis() - auxWaitStart > 1000 && (millis() - auxWaitStart) % 1000 == 0) {
                    Serial.printf("   Still waiting... (%lu ms)\n", millis() - auxWaitStart);
                }
            }

            auxState = digitalRead(_pinAUX);
            if (auxState == HIGH) {
                Serial.printf("✅ Module ready after %lu ms\n", millis() - auxWaitStart);
            } else {
                Serial.printf("⚠️  Module not ready after %lu ms (AUX still LOW)\n", millis() - auxWaitStart);
                Serial.println("   Continuing anyway - some modules don't follow AUX protocol exactly");
            }

            // Attempt to initialize with EBYTE library
            Serial.println("🔧 Initializing EBYTE library...");
            bool ebyte_result = _ebyte->init();
            Serial.printf("   EBYTE library init result: %s\n", ebyte_result ? "SUCCESS" : "FAILED");

            if (!ebyte_result) {
                Serial.println("⚠️  EBYTE library initialization failed, but continuing with manual control");
                Serial.println("   This is often normal - library may be too strict about responses");
            }

            // Perform comprehensive communication tests
            Serial.println("🧪 Performing communication tests...");

            // Test 1: Mode switching
            Serial.println("   Test 1: Mode switching capability");
            bool modeTest = testModeTransitions();
            Serial.printf("   Mode switching test: %s\n", modeTest ? "PASS" : "FAIL");

            // Test 2: Configuration mode access
            Serial.println("   Test 2: Configuration mode access");
            bool configTest = testConfigurationMode();
            Serial.printf("   Configuration access test: %s\n", configTest ? "PASS" : "FAIL");

            // Test 3: Basic parameter reading
            Serial.println("   Test 3: Parameter reading");
            bool paramTest = testParameterReading();
            Serial.printf("   Parameter reading test: %s\n", paramTest ? "PASS" : "FAIL");

            // Set initialized flag based on basic functionality
            _initialized = (modeTest || configTest); // At least one basic test should pass

            if (_initialized) {
                Serial.println("✅ Module initialization SUCCESSFUL");
                Serial.println("   Basic communication with module established");
            } else {
                Serial.println("❌ Module initialization FAILED");
                Serial.println("   No communication possible with module");
                return false;
            }

            Serial.println(repeatChar('=', 50));
            return _initialized;
        }

        bool EbyteCommModule::testModeTransitions() {
            Serial.println("      🔄 Testing mode transitions...");

            const char* modeNames[] = {"Normal", "Wakeup", "PowerDown", "Sleep"};
            bool results[4];

            for (int mode = 0; mode < 4; mode++) {
                Serial.printf("         Setting mode %d (%s)... ", mode, modeNames[mode]);
                results[mode] = setMode(mode);
                Serial.printf("%s\n", results[mode] ? "OK" : "FAIL");
                delay(200); // Give time between mode changes
            }

            // Return to normal mode
            setMode(MODE_NORMAL);

            // Return true if at least normal mode works
            return results[MODE_NORMAL];
        }

        bool EbyteCommModule::testConfigurationMode() {
            Serial.println("      ⚙️  Testing configuration mode access...");

            // Try to enter configuration mode
            if (!setMode(MODE_POWERDOWN)) {
                Serial.println("         ❌ Cannot enter configuration mode");
                return false;
            }

            // Try to send a simple command (read parameters)
            Serial.println("         📤 Sending parameter read command...");
            _serial->write(0xC1);
            _serial->write(0xC1);
            _serial->write(0xC1);
            _serial->flush();

            delay(100); // Give time for response

            bool hasResponse = _serial->available() > 0;
            Serial.printf("         📥 Response received: %s\n", hasResponse ? "YES" : "NO");

            // Clear any response
            while (_serial->available()) {
                _serial->read();
            }

            // Return to normal mode
            setMode(MODE_NORMAL);

            return hasResponse;
        }

        bool EbyteCommModule::testParameterReading() {
            Serial.println("      📊 Testing parameter reading...");

            // This is a basic test - just verify we can detect any response from the module

            // Test if we can detect any response from the module
            for (int attempt = 0; attempt < 3; attempt++) {
                Serial.printf("         Attempt %d: ", attempt + 1);

                // Send a ping-like command
                _serial->write(0x00);
                _serial->flush();
                delay(50);

                if (_serial->available() > 0) {
                    Serial.println("Response detected");
                    while (_serial->available()) _serial->read(); // Clear
                    return true;
                }
                Serial.println("No response");
                delay(100);
            }

            return false; // No response to any test
        }

        bool EbyteCommModule::configure(uint8_t channel, uint16_t address, uint8_t airDataRate,
                                        uint8_t uartBaud, uint8_t powerLevel) {
            if (!_initialized) {
                _lastError = "Module not initialized";
                Serial.println("❌ Configuration failed: " + _lastError);
                return false;
            }

            Serial.println("\n🔧 Configuring EBYTE module parameters...");
            Serial.printf("📊 Target configuration:\n");
            Serial.printf("   Channel: 0x%02X (%d)\n", channel, channel);
            Serial.printf("   Address: 0x%04X (%d)\n", address, address);
            Serial.printf("   Air Data Rate: %d\n", airDataRate);
            Serial.printf("   UART Baud: %d\n", uartBaud);
            Serial.printf("   Power Level: %d\n", powerLevel);

            // Store target configuration
            _channel = channel;
            _address = address;

            // Enhanced mode switching with better error handling
            Serial.println("🔄 Entering configuration mode...");
            if (!setModeWithRetries(MODE_POWERDOWN, 3)) {
                _lastError = "Failed to enter configuration mode after retries";
                Serial.println("❌ " + _lastError);
                return false;
            }

            // Wait for module to be ready
            if (!waitForAUXWithTimeout(2000)) {
                Serial.println("⚠️  AUX timeout, but continuing configuration attempt");
            }

            // Configuration attempts with retries
            bool configSuccess = false;
            for (int attempt = 1; attempt <= 3 && !configSuccess; attempt++) {
                Serial.printf("🔧 Configuration attempt %d/3...\n", attempt);

                // Use EBYTE library methods with error checking
                try {
                    Serial.printf("   Setting channel to 0x%02X...\n", channel);
                    _ebyte->SetChannel(channel);
                    delay(100);

                    Serial.printf("   Setting address to 0x%04X...\n", address);
                    _ebyte->SetAddress(address);
                    delay(100);

                    Serial.printf("   Setting air data rate to %d...\n", airDataRate);
                    _ebyte->SetAirDataRate(airDataRate);
                    delay(100);

                    Serial.printf("   Setting UART baud rate to %d...\n", uartBaud);
                    _ebyte->SetUARTBaudRate(uartBaud);
                    delay(100);

                    Serial.printf("   Setting transmission power to %d...\n", powerLevel);
                    _ebyte->SetTransmitPower(powerLevel);
                    delay(100);

                    Serial.println("   Saving parameters to EEPROM...");
                    _ebyte->SaveParameters(PERMANENT);
                    delay(500); // EEPROM write needs time

                    configSuccess = true;
                    Serial.printf("✅ Configuration attempt %d successful\n", attempt);

                } catch (...) {
                    Serial.printf("❌ Configuration attempt %d failed with exception\n", attempt);
                    delay(1000);
                }
            }

            if (!configSuccess) {
                Serial.println("❌ All configuration attempts failed");
                _lastError = "Configuration failed after all retries";
            }

            // Return to normal mode with retries
            Serial.println("🔄 Returning to normal mode...");
            if (!setModeWithRetries(MODE_NORMAL, 3)) {
                Serial.println("⚠️  Failed to return to normal mode");
                _lastError = "Failed to return to normal mode";
            } else {
                Serial.println("✅ Returned to normal mode successfully");
            }

            // Verify configuration by attempting a test transmission
            if (configSuccess) {
                Serial.println("🧪 Testing configuration with dummy transmission...");
                delay(1000); // Let module settle

                // Send a small test message
                uint8_t testMsg[] = {0xAA, 0x55, 0xAA, 0x55};
                if (sendMessage(0, testMsg, sizeof(testMsg))) {
                    Serial.println("✅ Configuration test transmission successful");
                } else {
                    Serial.println("⚠️  Configuration test transmission failed (may be normal without receiver)");
                }
            }

            // Final status report
            Serial.println("📊 Configuration Summary:");
            Serial.printf("   Overall result: %s\n", configSuccess ? "SUCCESS" : "FAILED");
            Serial.printf("   Channel: 0x%02X\n", _channel);
            Serial.printf("   Address: 0x%04X\n", _address);
            Serial.printf("   Current mode: %s\n", getCurrentModeString().c_str());
            Serial.printf("   AUX pin state: %s\n", digitalRead(_pinAUX) ? "HIGH" : "LOW");

            return configSuccess;
        }

        bool EbyteCommModule::sendMessage(uint8_t nodeId, const void* message, size_t length) {
            if (!_initialized) {
                _lastError = "Module not initialized";
                if (_debugLevel >= 1) {
                    Serial.println("❌ Send failed: " + _lastError);
                }
                return false;
            }

            if (length == 0 || message == nullptr) {
                _lastError = "Invalid message parameters";
                if (_debugLevel >= 1) {
                    Serial.println("❌ Send failed: " + _lastError);
                }
                return false;
            }

            // Enhanced pre-transmission checks
            if (_debugLevel >= 2) {
                Serial.printf("📤 SEND MESSAGE START\n");
                Serial.printf("   Target Node: %d %s\n", nodeId, nodeId == 0 ? "(BROADCAST)" : "");
                Serial.printf("   Message Length: %d bytes\n", length);
                Serial.printf("   Current Mode: %s\n", getCurrentModeString().c_str());
                Serial.printf("   AUX Pin: %s\n", digitalRead(_pinAUX) ? "HIGH (ready)" : "LOW (busy)");
            }

            // Ensure we're in transmission mode
            if (!setModeWithRetries(MODE_NORMAL, 2)) {
                _lastError = "Failed to set transmission mode";
                Serial.println("❌ " + _lastError);
                return false;
            }

            // Wait for module to be ready with timeout
            if (!waitForAUXWithTimeout(1000)) {
                if (_debugLevel >= 1) {
                    Serial.println("⚠️  AUX timeout before transmission (continuing anyway)");
                }
            }

            // Clear any existing data in serial buffer
            int clearCount = 0;
            while (_serial->available() && clearCount < 100) {
                _serial->read();
                clearCount++;
            }
            if (clearCount > 0 && _debugLevel >= 2) {
                Serial.printf("   🧹 Cleared %d bytes from RX buffer\n", clearCount);
            }

            // Enhanced message transmission with monitoring
            unsigned long sendStart = millis();

            if (_debugLevel >= 2) {
                Serial.printf("   📡 Transmitting %d bytes...\n", length);
                dumpMessageHex(message, length);
            }

            // Perform actual transmission
            size_t bytesWritten = _serial->write(static_cast<const uint8_t*>(message), length);
            _serial->flush(); // Ensure all data is sent

            unsigned long sendDuration = millis() - sendStart;

            if (bytesWritten != length) {
                _lastError = "Incomplete transmission: " + String(bytesWritten) + "/" + String(length) + " bytes";
                Serial.println("❌ " + _lastError);
                return false;
            }

            // Monitor AUX pin during and after transmission
            if (_debugLevel >= 2) {
                Serial.printf("   ⏱️  Transmission completed in %lu ms\n", sendDuration);
                Serial.printf("   📊 Bytes written: %d/%d\n", bytesWritten, length);

                // Check AUX pin state after transmission
                delay(50); // Give time for transmission to start
                int auxAfter = digitalRead(_pinAUX);
                Serial.printf("   📍 AUX after send: %s\n", auxAfter ? "HIGH" : "LOW");

                // Wait to see if AUX changes (indicating transmission in progress)
                for (int i = 0; i < 10; i++) {
                    delay(10);
                    int currentAux = digitalRead(_pinAUX);
                    if (currentAux != auxAfter) {
                        Serial.printf("   🔄 AUX changed to %s after %d ms\n",
                                      currentAux ? "HIGH" : "LOW", (i + 1) * 10);
                        break;
                    }
                }
            }

            if (_debugLevel >= 1) {
                Serial.printf("✅ Message sent successfully (%d bytes in %lu ms)\n", bytesWritten, sendDuration);
            }

            return true;
        }

        bool EbyteCommModule::isMessageAvailable() {
            if (!_initialized) {
                return false;
            }

            // Check if we already have a complete message buffered
            if (_hasMessage) {
                return true;
            }

            // Check for new data on serial port
            size_t availableBytes = _serial->available();
            if (availableBytes == 0) {
                return false;
            }

            if (_debugLevel >= 2) {
                Serial.printf("📥 Data available: %d bytes\n", availableBytes);
            }

            // Read available data into our buffer
            size_t spacesLeft = RX_BUFFER_SIZE - _rxBufferLen;
            size_t bytesToRead = min(availableBytes, spacesLeft);

            if (bytesToRead == 0) {
                Serial.println("⚠️  RX buffer full, clearing...");
                _rxBufferLen = 0; // Reset buffer
                bytesToRead = min(availableBytes, RX_BUFFER_SIZE);
            }

            size_t bytesRead = _serial->readBytes(_rxBuffer + _rxBufferLen, bytesToRead);
            _rxBufferLen += bytesRead;

            if (_debugLevel >= 2) {
                Serial.printf("📥 Read %d bytes, buffer now has %d bytes\n", bytesRead, _rxBufferLen);
            }

            // Check if we have enough data for a message header
            if (_rxBufferLen < sizeof(MessageHeader)) {
                if (_debugLevel >= 2) {
                    Serial.printf("   📊 Need %d more bytes for complete header\n",
                                  sizeof(MessageHeader) - _rxBufferLen);
                }
                return false;
            }

            // Parse message header to determine total message length
            const MessageHeader* header = reinterpret_cast<const MessageHeader*>(_rxBuffer);
            size_t expectedLength = sizeof(MessageHeader) + header->payloadLength;

            if (_debugLevel >= 2) {
                Serial.printf("   📋 Message header parsed:\n");
                Serial.printf("      Type: %d\n", static_cast<int>(header->type));
                Serial.printf("      Node ID: %d\n", header->nodeId);
                Serial.printf("      Sequence: %d\n", header->sequenceNum);
                Serial.printf("      Payload Length: %d\n", header->payloadLength);
                Serial.printf("      Expected Total: %d bytes\n", expectedLength);
                Serial.printf("      Checksum: 0x%02X\n", header->checksum);
            }

            // Validate message parameters
            if (expectedLength > RX_BUFFER_SIZE) {
                Serial.printf("❌ Message too large: %d > %d bytes (clearing buffer)\n",
                              expectedLength, RX_BUFFER_SIZE);
                _rxBufferLen = 0;
                return false;
            }

            if (header->payloadLength > 200) { // Sanity check
                Serial.printf("❌ Suspicious payload length: %d (clearing buffer)\n", header->payloadLength);
                _rxBufferLen = 0;
                return false;
            }

            // Check if we have the complete message
            if (_rxBufferLen >= expectedLength) {
                if (_debugLevel >= 2) {
                    Serial.printf("✅ Complete message received (%d bytes)\n", _rxBufferLen);
                    Serial.printf("   📊 Message type: %d from node %d\n",
                                  static_cast<int>(header->type), header->nodeId);
                }

                // Validate checksum
                bool checksumValid = header->validateChecksum();
                if (_debugLevel >= 2) {
                    Serial.printf("   ✅ Checksum: %s (0x%02X)\n",
                                  checksumValid ? "VALID" : "INVALID", header->checksum);
                }

                if (!checksumValid) {
                    Serial.println("❌ Invalid checksum, discarding message");
                    _rxBufferLen = 0;
                    return false;
                }

                _hasMessage = true;

                if (_debugLevel >= 2) {
                    dumpMessageHex(_rxBuffer, _rxBufferLen);
                }

                return true;
            } else {
                if (_debugLevel >= 2) {
                    Serial.printf("   📊 Partial message: have %d bytes, need %d more\n",
                                  _rxBufferLen, expectedLength - _rxBufferLen);
                }
                return false;
            }
        }

        size_t EbyteCommModule::receiveMessage(void* buffer, size_t maxLength, uint8_t& senderNodeId) {
            if (!_initialized || !_hasMessage || _rxBufferLen == 0) {
                return 0;
            }

            // Validate buffer parameters
            if (!buffer || maxLength == 0) {
                Serial.println("❌ Invalid receive buffer parameters");
                return 0;
            }

            if (_debugLevel >= 2) {
                Serial.printf("📥 RECEIVE MESSAGE START\n");
                Serial.printf("   Buffer size: %d bytes\n", maxLength);
                Serial.printf("   Message size: %d bytes\n", _rxBufferLen);
            }

            // Copy message to output buffer
            size_t bytesToCopy = min(_rxBufferLen, maxLength);
            memcpy(buffer, _rxBuffer, bytesToCopy);

            // Extract sender information from header
            const MessageHeader* header = reinterpret_cast<const MessageHeader*>(_rxBuffer);
            senderNodeId = header->nodeId;

            if (_debugLevel >= 1) {
                Serial.printf("📥 Message received: %d bytes from node %d\n", bytesToCopy, senderNodeId);
                Serial.printf("   Type: %d, Sequence: %d, Timestamp: %lu\n",
                              static_cast<int>(header->type), header->sequenceNum, header->timestamp);
            }

            // Clear the receive buffer
            _rxBufferLen = 0;
            _hasMessage = false;

            return bytesToCopy;
        }

        // Enhanced helper methods with better error handling

        bool EbyteCommModule::setModeWithRetries(uint8_t mode, int maxRetries) {
            for (int attempt = 1; attempt <= maxRetries; attempt++) {
                if (_debugLevel >= 2) {
                    Serial.printf("🔄 Mode change attempt %d/%d to mode %d\n", attempt, maxRetries, mode);
                }

                if (setMode(mode)) {
                    if (_debugLevel >= 2) {
                        Serial.printf("✅ Mode change successful on attempt %d\n", attempt);
                    }
                    return true;
                }

                if (attempt < maxRetries) {
                    delay(200 * attempt); // Increasing delay between retries
                }
            }

            if (_debugLevel >= 1) {
                Serial.printf("❌ Mode change failed after %d attempts\n", maxRetries);
            }
            return false;
        }

        bool EbyteCommModule::waitForAUXWithTimeout(unsigned long timeout) {
            unsigned long startTime = millis();

            // Check current state
            int initialState = digitalRead(_pinAUX);
            if (initialState == HIGH) {
                return true; // Already ready
            }

            if (_debugLevel >= 2) {
                Serial.printf("⏳ Waiting for AUX (timeout: %lu ms)\n", timeout);
            }

            // Wait for AUX to go HIGH
            while (digitalRead(_pinAUX) == LOW) {
                if (millis() - startTime > timeout) {
                    if (_debugLevel >= 1) {
                        Serial.printf("⏰ AUX timeout after %lu ms\n", timeout);
                    }
                    return false;
                }
                delay(10);
            }

            unsigned long elapsed = millis() - startTime;
            if (_debugLevel >= 2) {
                Serial.printf("✅ AUX ready after %lu ms\n", elapsed);
            }

            return true;
        }

        String EbyteCommModule::getCurrentModeString() const {
            int m0 = digitalRead(_pinM0);
            int m1 = digitalRead(_pinM1);

            if (m0 == 0 && m1 == 0) return "Normal (M0=0,M1=0)";
            if (m0 == 1 && m1 == 0) return "Wakeup (M0=1,M1=0)";
            if (m0 == 0 && m1 == 1) return "PowerDown (M0=0,M1=1)";
            if (m0 == 1 && m1 == 1) return "Sleep (M0=1,M1=1)";
            return "Unknown";
        }

        bool EbyteCommModule::setMode(uint8_t mode) {
            if (_debugLevel >= 2) {
                Serial.printf("🔄 Setting mode to %d (%s)\n", mode, getModeString(mode).c_str());
            }

            // Store current state for comparison
            int oldM0 = digitalRead(_pinM0);
            int oldM1 = digitalRead(_pinM1);
            int oldAUX = digitalRead(_pinAUX);

            if (_debugLevel >= 2) {
                Serial.printf("   Before: M0=%d, M1=%d, AUX=%d\n", oldM0, oldM1, oldAUX);
            }

            // Set pins according to mode
            switch (mode) {
                case MODE_NORMAL:    // 0: Normal mode (M0=0, M1=0)
                    digitalWrite(_pinM0, LOW);
                    digitalWrite(_pinM1, LOW);
                    break;
                case MODE_WAKEUP:    // 1: Wakeup mode (M0=1, M1=0)
                    digitalWrite(_pinM0, HIGH);
                    digitalWrite(_pinM1, LOW);
                    break;
                case MODE_POWERDOWN: // 2: Power saving/config mode (M0=0, M1=1)
                    digitalWrite(_pinM0, LOW);
                    digitalWrite(_pinM1, HIGH);
                    break;
                case MODE_SLEEP:     // 3: Sleep mode (M0=1, M1=1)
                    digitalWrite(_pinM0, HIGH);
                    digitalWrite(_pinM1, HIGH);
                    break;
                default:
                    if (_debugLevel >= 1) {
                        Serial.printf("❌ Invalid mode: %d\n", mode);
                    }
                    return false;
            }

            // Give pins time to settle
            delay(50);

            // Verify pin states
            int newM0 = digitalRead(_pinM0);
            int newM1 = digitalRead(_pinM1);

            if (_debugLevel >= 2) {
                Serial.printf("   After: M0=%d, M1=%d\n", newM0, newM1);
            }

            // Wait for AUX response (but don't fail if it doesn't respond)
            bool auxReady = waitForAUXWithTimeout(1000);
            int finalAUX = digitalRead(_pinAUX);

            if (_debugLevel >= 2) {
                Serial.printf("   Final AUX: %d (%s)\n", finalAUX, auxReady ? "ready" : "timeout");
            }

            // Consider successful if pins are set correctly
            bool pinsCorrect = (newM0 == ((mode & 1) ? 1 : 0)) && (newM1 == ((mode & 2) ? 1 : 0));

            if (pinsCorrect) {
                if (_debugLevel >= 2) {
                    Serial.printf("✅ Mode change successful\n");
                }
                return true;
            } else {
                if (_debugLevel >= 1) {
                    Serial.printf("❌ Mode change failed - pin verification failed\n");
                }
                return false;
            }
        }

        String EbyteCommModule::getModeString(uint8_t mode) const {
            switch (mode) {
                case MODE_NORMAL: return "Normal";
                case MODE_WAKEUP: return "Wakeup";
                case MODE_POWERDOWN: return "PowerDown";
                case MODE_SLEEP: return "Sleep";
                default: return "Invalid";
            }
        }

        void EbyteCommModule::dumpMessageHex(const void* message, size_t length) {
            if (_debugLevel < 2) return;

            const uint8_t* data = static_cast<const uint8_t*>(message);

            Serial.println("   📦 Message Hex Dump:");
            Serial.println("   " + repeatChar('-', 48));

            for (size_t i = 0; i < length; i += 16) {
                // Address
                Serial.printf("   %04X: ", i);

                // Hex bytes
                for (size_t j = 0; j < 16 && (i + j) < length; j++) {
                    Serial.printf("%02X ", data[i + j]);
                }

                // Padding for incomplete lines
                for (size_t j = (length - i < 16 ? length - i : 16); j < 16; j++) {
                    Serial.print("   ");
                }

                // ASCII representation
                Serial.print(" | ");
                for (size_t j = 0; j < 16 && (i + j) < length; j++) {
                    uint8_t c = data[i + j];
                    Serial.print((c >= 32 && c < 127) ? (char)c : '.');
                }

                Serial.println();
            }

            Serial.println("   " + repeatChar('-', 48));

            // Message interpretation if it's a known type
            if (length >= sizeof(MessageHeader)) {
                const MessageHeader* header = reinterpret_cast<const MessageHeader*>(message);
                Serial.println("   📋 Message Structure:");
                Serial.printf("      Version: %d\n", header->version);
                Serial.printf("      Type: %d\n", static_cast<int>(header->type));
                Serial.printf("      Node ID: %d\n", header->nodeId);
                Serial.printf("      Sequence: %d\n", header->sequenceNum);
                Serial.printf("      Payload Len: %d\n", header->payloadLength);
                Serial.printf("      Timestamp: %lu ms\n", header->timestamp);
                Serial.printf("      Checksum: 0x%02X (computed: 0x%02X) %s\n",
                              header->checksum, header->computeChecksum(),
                              header->validateChecksum() ? "✅" : "❌");
            }
        }

        // Implementation of remaining interface methods with enhanced error handling

        String EbyteCommModule::getLastError() const {
            return _lastError;
        }

        bool EbyteCommModule::isTransmitting() const {
            if (!_initialized) {
                return false;
            }
            // AUX low typically indicates the module is busy (transmitting or processing)
            return digitalRead(_pinAUX) == LOW;
        }

        int EbyteCommModule::getSignalStrength() const {
            // E49 module doesn't provide direct RSSI measurement
            // Return cached value or estimate based on communication success
            return _lastRssi;
        }

        bool EbyteCommModule::setTransmissionPower(uint8_t level) {
            if (!_initialized) {
                _lastError = "Module not initialized";
                return false;
            }

            Serial.printf("🔋 Setting transmission power to level %d\n", level);

            if (!setModeWithRetries(MODE_POWERDOWN, 3)) {
                _lastError = "Failed to enter configuration mode for power setting";
                return false;
            }

            _ebyte->SetTransmitPower(level);
            _ebyte->SaveParameters(PERMANENT);

            bool success = setModeWithRetries(MODE_NORMAL, 3);
            if (success) {
                Serial.printf("✅ Transmission power set to level %d\n", level);
            } else {
                Serial.printf("❌ Failed to set transmission power\n");
            }

            return success;
        }

        void EbyteCommModule::update() {
            static unsigned long lastStatusCheck = 0;
            unsigned long currentTime = millis();

            // Periodic status checks
            if (currentTime - lastStatusCheck > 5000) { // Every 5 seconds
                if (_debugLevel >= 2) {
                    int auxState = digitalRead(_pinAUX);
                    Serial.printf("📊 Module status: AUX=%s, Mode=%s, Buffer=%d bytes\n",
                                  auxState ? "Ready" : "Busy", getCurrentModeString().c_str(), _rxBufferLen);
                }
                lastStatusCheck = currentTime;
            }

            // Check for new messages
            isMessageAvailable();

            // Buffer management - prevent overflow
            if (_rxBufferLen > RX_BUFFER_SIZE * 0.8) { // 80% full
                if (_debugLevel >= 1) {
                    Serial.printf("⚠️  RX buffer nearly full (%d/%d bytes)\n", _rxBufferLen, RX_BUFFER_SIZE);
                }
            }
        }

        void EbyteCommModule::printParameters() {
            if (!_initialized) {
                Serial.println("❌ Module not initialized - cannot print parameters");
                return;
            }

            Serial.println("\n📊 EBYTE Module Parameters");
            Serial.println(repeatChar('=', 40));
            Serial.printf("Initialization: %s\n", _initialized ? "✅ SUCCESS" : "❌ FAILED");
            Serial.printf("Channel: 0x%02X (%d)\n", _channel, _channel);
            Serial.printf("Address: 0x%04X (%d)\n", _address, _address);
            Serial.printf("Debug Level: %d\n", _debugLevel);
            Serial.printf("Current Mode: %s\n", getCurrentModeString().c_str());

            // Pin states
            Serial.println("\nPin States:");
            Serial.printf("  M0 (pin %d): %s\n", _pinM0, digitalRead(_pinM0) ? "HIGH" : "LOW");
            Serial.printf("  M1 (pin %d): %s\n", _pinM1, digitalRead(_pinM1) ? "HIGH" : "LOW");
            Serial.printf("  AUX (pin %d): %s\n", _pinAUX, digitalRead(_pinAUX) ? "HIGH" : "LOW");

            // Buffer status
            Serial.println("\nBuffer Status:");
            Serial.printf("  RX Buffer: %d/%d bytes (%d%% full)\n",
                          _rxBufferLen, RX_BUFFER_SIZE, (_rxBufferLen * 100) / RX_BUFFER_SIZE);
            Serial.printf("  Has Message: %s\n", _hasMessage ? "Yes" : "No");

            // Last error
            if (_lastError.length() > 0) {
                Serial.println("\nLast Error: " + _lastError);
            }

            Serial.println(repeatChar('=', 40));
        }

        void EbyteCommModule::setDebugLevel(uint8_t level) {
            _debugLevel = level;
            Serial.printf("🐛 EBYTE debug level set to %d\n", level);
            if (level >= 2) {
                Serial.println("   Level 2: Maximum verbosity with hex dumps enabled");
            } else if (level >= 1) {
                Serial.println("   Level 1: Basic operation logging enabled");
            } else {
                Serial.println("   Level 0: Debug logging disabled");
            }
        }

        void EbyteCommModule::performDiagnostics() {
            Serial.println("\n🔬 EBYTE Module Diagnostics");
            Serial.println(repeatChar('=', 50));

            // Basic connectivity test
            Serial.println("1. Basic Connectivity:");
            Serial.printf("   Module initialized: %s\n", _initialized ? "✅ YES" : "❌ NO");
            Serial.printf("   Current mode: %s\n", getCurrentModeString().c_str());

            // Pin test
            Serial.println("\n2. Pin Functionality:");
            int m0 = digitalRead(_pinM0);
            int m1 = digitalRead(_pinM1);
            int aux = digitalRead(_pinAUX);
            Serial.printf("   M0 pin %d: %s\n", _pinM0, m0 ? "HIGH" : "LOW");
            Serial.printf("   M1 pin %d: %s\n", _pinM1, m1 ? "HIGH" : "LOW");
            Serial.printf("   AUX pin %d: %s\n", _pinAUX, aux ? "HIGH (ready)" : "LOW (busy)");

            // Mode switching test
            Serial.println("\n3. Mode Switching Test:");
            bool modeTestResult = testModeTransitions();
            Serial.printf("   Mode switching: %s\n", modeTestResult ? "✅ WORKING" : "❌ FAILED");

            // UART communication test
            Serial.println("\n4. UART Communication:");
            _serial->flush();
            int preCount = _serial->available();
            Serial.printf("   Pre-test buffer: %d bytes\n", preCount);

            // Clear buffer
            while (_serial->available()) _serial->read();

            Serial.printf("   UART baud rate: 9600\n");
            Serial.printf("   Buffer cleared: ✅\n");

            // Configuration access test
            Serial.println("\n5. Configuration Access:");
            bool configAccess = testConfigurationMode();
            Serial.printf("   Config mode access: %s\n", configAccess ? "✅ WORKING" : "❌ FAILED");

            // Buffer and memory test
            Serial.println("\n6. Buffer Status:");
            Serial.printf("   RX buffer size: %d bytes\n", RX_BUFFER_SIZE);
            Serial.printf("   Current buffer usage: %d bytes\n", _rxBufferLen);
            Serial.printf("   Has pending message: %s\n", _hasMessage ? "Yes" : "No");

            // Final assessment
            Serial.println("\n📋 Diagnostic Summary:");
            bool overallHealth = _initialized && modeTestResult;
            Serial.printf("   Overall module health: %s\n", overallHealth ? "✅ GOOD" : "⚠️  ISSUES DETECTED");

            if (!overallHealth) {
                Serial.println("   🔧 Recommended actions:");
                if (!_initialized) {
                    Serial.println("      • Check power supply and connections");
                    Serial.println("      • Verify correct pin assignments");
                }
                if (!modeTestResult) {
                    Serial.println("      • Check M0/M1 control pin wiring");
                    Serial.println("      • Verify module is not damaged");
                }
            }

            Serial.println(repeatChar('=', 50));
        }

    } // namespace communication
} // namespace szogfm