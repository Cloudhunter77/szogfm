#include "EbyteCommModule.h"

namespace szogfm {
    namespace communication {

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
            // Initialize the EBYTE module
            Serial.println("[EBYTE] Initializing EBYTE transceiver module...");

            // Configure pins
            pinMode(_pinM0, OUTPUT);
            pinMode(_pinM1, OUTPUT);
            pinMode(_pinAUX, INPUT);

            Serial.printf("[EBYTE] Pins configured - M0: %d, M1: %d, AUX: %d\n", _pinM0, _pinM1, _pinAUX);

            // Initialize Serial port for the module
            _serial->begin(9600);
            Serial.printf("[EBYTE] Serial port initialized at 9600 baud\n");

            // Wait for serial to stabilize
            delay(50);

            // Check AUX pin initial state
            int auxState = digitalRead(_pinAUX);
            Serial.printf("[EBYTE] Initial AUX pin state: %d (should be HIGH when ready)\n", auxState);

            // Call init but don't rely on its return value since some modules don't respond correctly
            bool initResult = _ebyte->init();
            Serial.printf("[EBYTE] Module init result: %s\n", initResult ? "SUCCESS" : "FAILED");

            // Check if module is responding to basic commands
            if (testCommunication()) {
                Serial.println("[EBYTE] Communication test PASSED");
            } else {
                Serial.println("[EBYTE] Communication test FAILED - module may not be responding correctly");
                // We'll continue anyway and try to use the module
            }

            // Set initialized flag true regardless of result
            // We'll rely on actual message transmission/reception to determine if it's working
            _initialized = true;
            Serial.println("[EBYTE] Module initialized (basic initialization)");

            return true;
        }

        bool EbyteCommModule::testCommunication() {
            // Try to set mode to normal and check if it works
            Serial.println("[EBYTE] Testing communication with module...");

            // Set module to normal mode
            digitalWrite(_pinM0, LOW);
            digitalWrite(_pinM1, LOW);
            delay(50);

            // Check if AUX pin goes high (indicating module is ready)
            unsigned long startTime = millis();
            while (digitalRead(_pinAUX) == LOW) {
                if (millis() - startTime > 1000) {
                    Serial.println("[EBYTE] ERROR: AUX pin did not go HIGH within timeout");
                    return false;
                }
                delay(10);
            }

            Serial.printf("[EBYTE] AUX pin went HIGH after %lu ms\n", millis() - startTime);
            return true;
        }

        bool EbyteCommModule::configure(uint8_t channel, uint16_t address, uint8_t airDataRate,
                                        uint8_t uartBaud, uint8_t powerLevel) {
            if (!_initialized) {
                _lastError = "Module not initialized";
                return false;
            }

            Serial.println("[EBYTE] Configuring EBYTE module...");

            // Log current pin states
            Serial.printf("[EBYTE] Current pin states - M0: %d, M1: %d, AUX: %d\n",
                          digitalRead(_pinM0), digitalRead(_pinM1), digitalRead(_pinAUX));

            // Set the module to configuration mode
            if (!setMode(MODE_POWERDOWN)) { // Use power down mode (mode 2) for configuration
                _lastError = "Failed to set configuration mode";
                Serial.println("[EBYTE] ERROR: " + _lastError);
                return false;
            }

            // Wait for AUX to be HIGH before proceeding
            if (!waitForAUX(2000)) {
                Serial.println("[EBYTE] ERROR: AUX pin not going HIGH after setting config mode");
            }

            // Configure module parameters
            _channel = channel;
            _address = address;

            // Set channel
            _ebyte->SetChannel(channel);
            Serial.println("[EBYTE] Channel set to: " + String(channel));

            // Set address
            _ebyte->SetAddress(address);
            Serial.println("[EBYTE] Address set to: 0x" + String(address, HEX));

            // Set air data rate
            // Directly use the numerical values based on E49-400T20D manual
            switch (airDataRate) {
                case AIR_1K2:  _ebyte->SetAirDataRate(0b000); break; // 1.2k baud
                case AIR_2K4:  _ebyte->SetAirDataRate(0b001); break; // 2.4k baud
                case AIR_4K8:  _ebyte->SetAirDataRate(0b010); break; // 4.8k baud
                case AIR_9K6:  _ebyte->SetAirDataRate(0b011); break; // 9.6k baud
                case AIR_19K2: _ebyte->SetAirDataRate(0b100); break; // 19.2k baud
                case AIR_50K:  _ebyte->SetAirDataRate(0b101); break; // 50k baud
                case AIR_100K: _ebyte->SetAirDataRate(0b110); break; // 100k baud
                case AIR_200K: _ebyte->SetAirDataRate(0b111); break; // 200k baud
                default:       _ebyte->SetAirDataRate(0b001); break; // 2.4k baud (default)
            }
            Serial.println("[EBYTE] Air data rate set to: " + String(airDataRate));

            // Set UART baud rate
            // Directly use the numerical values based on E49-400T20D manual
            switch (uartBaud) {
                case UART_1200:   _ebyte->SetUARTBaudRate(0b000); break; // 1200 baud
                case UART_2400:   _ebyte->SetUARTBaudRate(0b001); break; // 2400 baud
                case UART_4800:   _ebyte->SetUARTBaudRate(0b010); break; // 4800 baud
                case UART_9600:   _ebyte->SetUARTBaudRate(0b011); break; // 9600 baud
                case UART_19200:  _ebyte->SetUARTBaudRate(0b100); break; // 19200 baud
                case UART_38400:  _ebyte->SetUARTBaudRate(0b101); break; // 38400 baud
                case UART_57600:  _ebyte->SetUARTBaudRate(0b110); break; // 57600 baud
                case UART_115200: _ebyte->SetUARTBaudRate(0b111); break; // 115200 baud
                default:          _ebyte->SetUARTBaudRate(0b011); break; // 9600 baud (default)
            }
            Serial.println("[EBYTE] UART baud rate set to: " + String(uartBaud));

            // Set transmission power
            _ebyte->SetTransmitPower(powerLevel);
            Serial.println("[EBYTE] Transmit power set to: " + String(powerLevel));

            // Save parameters to module
            _ebyte->SaveParameters(PERMANENT); // Using the constant from the library
            Serial.println("[EBYTE] Parameters saved to EEPROM");

            // Return to normal mode
            bool success = setMode(MODE_NORMAL);
            Serial.println("[EBYTE] Module returned to normal mode: " + String(success ? "SUCCESS" : "FAILED"));

            // Log current pin states after configuration
            Serial.printf("[EBYTE] Pin states after config - M0: %d, M1: %d, AUX: %d\n",
                          digitalRead(_pinM0), digitalRead(_pinM1), digitalRead(_pinAUX));

            return success;
        }

        bool EbyteCommModule::sendMessage(uint8_t nodeId, const void* message, size_t length) {
            if (!_initialized) {
                _lastError = "Module not initialized";
                Serial.println("[EBYTE] ERROR: " + _lastError);
                return false;
            }

            // Set the module to transmission mode
            if (!setMode(MODE_NORMAL)) {
                _lastError = "Failed to set transmission mode";
                Serial.println("[EBYTE] ERROR: " + _lastError);
                return false;
            }

            // Wait for module to be ready
            if (!waitForAUX()) {
                _lastError = "Module not ready for transmission";
                Serial.println("[EBYTE] ERROR: " + _lastError);
                return false;
            }

            // Debug: dump message contents if debug level is high enough
            if (_debugLevel >= 2) {
                Serial.printf("[EBYTE] Sending message to node %d (%d bytes):\n", nodeId, length);
                dumpMessageHex(message, length);
            }

            // Clear any existing data in the buffer
            while (_serial->available()) {
                _serial->read();
            }

            // Send message
            size_t bytesWritten = _serial->write(static_cast<const uint8_t*>(message), length);

            if (bytesWritten != length) {
                _lastError = "Failed to send complete message";
                Serial.printf("[EBYTE] ERROR: Only sent %d of %d bytes\n", bytesWritten, length);
                return false;
            }

            Serial.printf("[EBYTE] Message sent: %d bytes\n", bytesWritten);
            return true;
        }

        bool EbyteCommModule::isMessageAvailable() {
            if (!_initialized) {
                return false;
            }

            // Check if we already have a message in the buffer
            if (_hasMessage) {
                return true;
            }

            // Check if there's data available on the serial port
            if (_serial->available() > 0) {
                // Get the number of bytes available
                size_t availableBytes = _serial->available();
                Serial.printf("[EBYTE] Data available: %d bytes\n", availableBytes);

                // Read available data into buffer
                size_t bytesToRead = min(availableBytes, RX_BUFFER_SIZE - _rxBufferLen);

                if (bytesToRead > 0) {
                    size_t bytesRead = _serial->readBytes(_rxBuffer + _rxBufferLen, bytesToRead);
                    _rxBufferLen += bytesRead;

                    Serial.printf("[EBYTE] Read %d bytes into buffer (total: %d)\n", bytesRead, _rxBufferLen);

                    // Very basic check if we have a complete message
                    // In a real implementation, you would need proper message framing
                    if (_rxBufferLen >= sizeof(MessageHeader)) {
                        const MessageHeader* header = reinterpret_cast<const MessageHeader*>(_rxBuffer);
                        size_t expectedLength = sizeof(MessageHeader) + header->payloadLength;

                        if (_rxBufferLen >= expectedLength) {
                            Serial.printf("[EBYTE] Complete message received (%d bytes, from node %d)\n",
                                          _rxBufferLen, header->nodeId);
                            _hasMessage = true;

                            // Debug message contents
                            if (_debugLevel >= 2) {
                                dumpMessageHex(_rxBuffer, _rxBufferLen);
                            }

                            return true;
                        } else {
                            Serial.printf("[EBYTE] Partial message: Have %d bytes, need %d more\n",
                                          _rxBufferLen, expectedLength - _rxBufferLen);
                        }
                    }
                }
            }

            return false;
        }

        size_t EbyteCommModule::receiveMessage(void* buffer, size_t maxLength, uint8_t& senderNodeId) {
            if (!_initialized || !_hasMessage || _rxBufferLen == 0) {
                return 0;
            }

            // Debug - check header validity
            if (_rxBufferLen >= sizeof(MessageHeader)) {
                const MessageHeader* header = reinterpret_cast<const MessageHeader*>(_rxBuffer);
                bool validChecksum = header->validateChecksum();
                Serial.printf("[EBYTE] Message checksum: %s\n", validChecksum ? "VALID" : "INVALID");
            }

            // Copy the message from the buffer
            size_t bytesToCopy = min(_rxBufferLen, maxLength);
            memcpy(buffer, _rxBuffer, bytesToCopy);

            // Extract sender node ID from message (assuming it's in the header)
            // This would depend on your message format
            const MessageHeader* header = reinterpret_cast<const MessageHeader*>(_rxBuffer);
            senderNodeId = header->nodeId;

            // Log message details
            Serial.printf("[EBYTE] Message received: %d bytes from node %d (type: %d, seq: %d)\n",
                          bytesToCopy, senderNodeId,
                          static_cast<int>(header->type),
                          header->sequenceNum);

            // Clear the buffer
            _rxBufferLen = 0;
            _hasMessage = false;

            return bytesToCopy;
        }

        String EbyteCommModule::getLastError() const {
            return _lastError;
        }

        bool EbyteCommModule::isTransmitting() const {
            if (!_initialized) {
                return false;
            }

            // Check the AUX pin - low means transmitting
            return digitalRead(_pinAUX) == LOW;
        }

        int EbyteCommModule::getSignalStrength() const {
            return _lastRssi;
        }

        bool EbyteCommModule::setTransmissionPower(uint8_t level) {
            if (!_initialized) {
                _lastError = "Module not initialized";
                return false;
            }

            // Set to configuration mode
            if (!setMode(MODE_POWERDOWN)) {
                _lastError = "Failed to set configuration mode";
                return false;
            }

            // Set transmission power
            _ebyte->SetTransmitPower(level);
            Serial.println("[EBYTE] Transmission power set to: " + String(level));

            // Save parameters
            _ebyte->SaveParameters(PERMANENT);

            // Return to normal mode
            return setMode(MODE_NORMAL);
        }

        void EbyteCommModule::update() {
            // Process any pending operations or maintenance tasks
            // Check if there's data available to read
            if (_serial->available() > 0 && !_hasMessage) {
                isMessageAvailable();
            }

            // Check AUX pin periodically
            static unsigned long lastAuxCheck = 0;
            if (millis() - lastAuxCheck > 5000) {
                lastAuxCheck = millis();
                int auxState = digitalRead(_pinAUX);

                if (_debugLevel >= 2) {
                    Serial.printf("[EBYTE] AUX pin state: %d\n", auxState);
                }
            }
        }

        bool EbyteCommModule::setMode(uint8_t mode) {
            // Mode 0: Normal mode (transmit/receive)
            // Mode 1: Wake-up mode
            // Mode 2: Power saving mode (configuration mode in E49)
            // Mode 3: Sleep mode

            Serial.printf("[EBYTE] Setting mode to %d\n", mode);
            Serial.printf("[EBYTE] Before: M0=%d, M1=%d\n", digitalRead(_pinM0), digitalRead(_pinM1));

            // Set module to the specified mode - manual pin control for better control
            switch (mode) {
                case MODE_NORMAL:
                    digitalWrite(_pinM0, LOW);
                    digitalWrite(_pinM1, LOW);
                    break;
                case MODE_WAKEUP:
                    digitalWrite(_pinM0, HIGH);
                    digitalWrite(_pinM1, LOW);
                    break;
                case MODE_POWERDOWN:
                    digitalWrite(_pinM0, LOW);
                    digitalWrite(_pinM1, HIGH);
                    break;
                case MODE_SLEEP:
                    digitalWrite(_pinM0, HIGH);
                    digitalWrite(_pinM1, HIGH);
                    break;
                default:
                    Serial.printf("[EBYTE] Invalid mode: %d\n", mode);
                    return false;
            }

            // Delay to allow mode change to take effect
            delay(50);

            Serial.printf("[EBYTE] After: M0=%d, M1=%d\n", digitalRead(_pinM0), digitalRead(_pinM1));

            // Wait for module to be ready
            return waitForAUX(1000);
        }

        bool EbyteCommModule::waitForAUX(unsigned long timeout) {
            unsigned long startTime = millis();

            // Give the module some time to change state
            delay(50);

            Serial.printf("[EBYTE] Waiting for AUX pin to go HIGH (timeout: %lu ms)\n", timeout);

            // Initial state
            int auxState = digitalRead(_pinAUX);
            Serial.printf("[EBYTE] Initial AUX state: %d\n", auxState);

            // If already high, we're good
            if (auxState == HIGH) {
                Serial.println("[EBYTE] AUX already HIGH");
                return true;
            }

            // Wait for AUX to go high
            while (digitalRead(_pinAUX) == LOW) {
                if (millis() - startTime > timeout) {
                    _lastError = "AUX pin timeout";
                    Serial.println("[EBYTE] ERROR: AUX pin timeout - pin never went HIGH");
                    return false;
                }

                // Add a small delay to avoid hammering CPU
                delay(10);

                // Log progress periodically
                if ((millis() - startTime) % 200 == 0) {
                    Serial.printf("[EBYTE] Still waiting for AUX, elapsed: %lu ms\n", millis() - startTime);
                }
            }

            unsigned long elapsed = millis() - startTime;
            Serial.printf("[EBYTE] AUX went HIGH after %lu ms\n", elapsed);
            return true;
        }

        void EbyteCommModule::printParameters() {
            if (!_initialized) {
                Serial.println("[EBYTE] Module not initialized");
                return;
            }

            Serial.println("\n[EBYTE] Module Parameters");
            Serial.println("---------------------");
            Serial.printf("Channel: %d (0x%02X)\n", _channel, _channel);
            Serial.printf("Address: 0x%04X\n", _address);
            Serial.printf("Pin States - M0: %d, M1: %d, AUX: %d\n",
                          digitalRead(_pinM0), digitalRead(_pinM1), digitalRead(_pinAUX));

            // For E49-400T20D, we won't try to read from the module directly
            // since some methods are private/protected in the EBYTE library
            Serial.println("[EBYTE] Using cached parameter values (reading from module not available)");
        }

        void EbyteCommModule::dumpMessageHex(const void* message, size_t length) {
            const uint8_t* data = static_cast<const uint8_t*>(message);
            char hexStr[16*3+1]; // Each line shows 16 bytes, each byte is 2 hex chars + space

            Serial.println("[EBYTE] Message Hex Dump:");
            Serial.println("------------------------------------------------------");

            for (size_t i = 0; i < length; i += 16) {
                char* hexPtr = hexStr;
                *hexPtr = '\0';

                // Address
                Serial.printf("%04X: ", i);

                // Hex representation
                for (size_t j = 0; j < 16 && (i+j) < length; j++) {
                    sprintf(hexPtr, "%02X ", data[i+j]);
                    hexPtr += 3;
                }

                // Pad if incomplete line
                for (size_t j = (length - i < 16 ? length - i : 16); j < 16; j++) {
                    strcpy(hexPtr, "   ");
                    hexPtr += 3;
                }
                *hexPtr = '\0';

                Serial.print(hexStr);

                // ASCII representation
                Serial.print(" | ");
                for (size_t j = 0; j < 16 && (i+j) < length; j++) {
                    uint8_t c = data[i+j];
                    if (c >= 32 && c < 127) {
                        Serial.write(c);
                    } else {
                        Serial.print('.');
                    }
                }

                Serial.println();
            }

            Serial.println("------------------------------------------------------");

            // If this is a known message type, try to interpret its header
            if (length >= sizeof(MessageHeader)) {
                const MessageHeader* header = reinterpret_cast<const MessageHeader*>(message);
                Serial.println("[EBYTE] Message Header Interpretation:");
                Serial.printf("  Version:      %d\n", header->version);
                Serial.printf("  Type:         %d\n", static_cast<int>(header->type));
                Serial.printf("  Node ID:      %d\n", header->nodeId);
                Serial.printf("  Sequence:     %d\n", header->sequenceNum);
                Serial.printf("  Payload Len:  %d\n", header->payloadLength);
                Serial.printf("  Timestamp:    %lu\n", header->timestamp);
                Serial.printf("  Checksum:     0x%02X (Calculated: 0x%02X - %s)\n",
                              header->checksum, header->computeChecksum(),
                              (header->checksum == header->computeChecksum() ? "VALID" : "INVALID"));
            }
        }

        void EbyteCommModule::setDebugLevel(uint8_t level) {
            _debugLevel = level;
            Serial.printf("[EBYTE] Debug level set to %d\n", level);
        }

        void EbyteCommModule::performDiagnostics() {
            Serial.println("[EBYTE] Performing diagnostics...");

            // Check physical connections
            Serial.printf("[EBYTE] Pin status - M0: %d, M1: %d, AUX: %d\n",
                          digitalRead(_pinM0), digitalRead(_pinM1), digitalRead(_pinAUX));

            // Try to enter and exit all operating modes
            Serial.println("[EBYTE] Testing mode transitions...");

            const char* modeNames[] = {"Normal", "Wakeup", "PowerDown", "Sleep"};
            bool modeResults[4];

            for (int mode = 0; mode < 4; mode++) {
                Serial.printf("[EBYTE] Setting mode to %s (%d)...\n", modeNames[mode], mode);
                modeResults[mode] = setMode(mode);
                Serial.printf("[EBYTE] Mode change result: %s\n", modeResults[mode] ? "SUCCESS" : "FAILED");
                delay(200);
            }

            // Return to normal mode for operation
            setMode(MODE_NORMAL);

            // Check UART communication
            Serial.println("[EBYTE] Testing UART communication...");
            _serial->flush();

            // Read any pending data
            int availableCount = 0;
            while (_serial->available()) {
                _serial->read();
                availableCount++;
            }

            if (availableCount > 0) {
                Serial.printf("[EBYTE] Cleared %d bytes from UART buffer\n", availableCount);
            }

            // Diagnostic summary
            Serial.println("[EBYTE] Diagnostics summary:");
            Serial.printf("  Normal mode transition:     %s\n", modeResults[0] ? "PASS" : "FAIL");
            Serial.printf("  Wakeup mode transition:     %s\n", modeResults[1] ? "PASS" : "FAIL");
            Serial.printf("  PowerDown mode transition:  %s\n", modeResults[2] ? "PASS" : "FAIL");
            Serial.printf("  Sleep mode transition:      %s\n", modeResults[3] ? "PASS" : "FAIL");

            // Try a loopback test if possible
            if (modeResults[0]) { // If normal mode works
                Serial.println("[EBYTE] A basic loopback test would require two modules - skipping");
            }

            Serial.println("[EBYTE] Diagnostics complete");
        }

    } // namespace communication
} // namespace szogfm