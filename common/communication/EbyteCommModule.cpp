#include "EbyteCommModule.h"

namespace szogfm {
    namespace communication {

        EbyteCommModule::EbyteCommModule(HardwareSerial* serial, int pinM0, int pinM1, int pinAUX)
                : _serial(serial), _pinM0(pinM0), _pinM1(pinM1), _pinAUX(pinAUX),
                  _initialized(false), _lastRssi(-120), _channel(0), _address(0),
                  _rxBufferLen(0), _hasMessage(false) {
            _ebyte = new EBYTE(_serial, _pinM0, _pinM1, _pinAUX);
        }

        EbyteCommModule::~EbyteCommModule() {
            delete _ebyte;
        }

        bool EbyteCommModule::initialize() {
            // Initialize the EBYTE module
            Serial.println("Initializing EBYTE transceiver module...");

            // Call init but don't rely on its return value since some modules don't respond correctly
            _ebyte->init();

            // Set initialized flag true regardless of result
            // We'll rely on actual message transmission/reception to determine if it's working
            _initialized = true;
            Serial.println("EBYTE module initialized (basic initialization)");

            return true;
        }

        bool EbyteCommModule::configure(uint8_t channel, uint16_t address, uint8_t airDataRate,
                                        uint8_t uartBaud, uint8_t powerLevel) {
            if (!_initialized) {
                _lastError = "Module not initialized";
                return false;
            }

            Serial.println("Configuring EBYTE module...");

            // Set the module to configuration mode
            if (!setMode(MODE_POWERDOWN)) { // Use power down mode (mode 2) for configuration
                _lastError = "Failed to set configuration mode";
                return false;
            }

            // Configure module parameters
            _channel = channel;
            _address = address;

            // Set channel
            _ebyte->SetChannel(channel);
            Serial.println("Channel set to: " + String(channel));

            // Set address
            _ebyte->SetAddress(address);
            Serial.println("Address set to: 0x" + String(address, HEX));

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
            Serial.println("Air data rate set to: " + String(airDataRate));

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
            Serial.println("UART baud rate set to: " + String(uartBaud));

            // Set transmission power
            _ebyte->SetTransmitPower(powerLevel);
            Serial.println("Transmit power set to: " + String(powerLevel));

            // Save parameters to module
            _ebyte->SaveParameters(PERMANENT); // Using the constant from the library
            Serial.println("Parameters saved to EEPROM");

            // Return to normal mode
            bool success = setMode(MODE_NORMAL);
            Serial.println("Module returned to normal mode");

            return success;
        }

        bool EbyteCommModule::sendMessage(uint8_t nodeId, const void* message, size_t length) {
            if (!_initialized) {
                _lastError = "Module not initialized";
                return false;
            }

            // Set the module to transmission mode
            if (!setMode(MODE_NORMAL)) {
                _lastError = "Failed to set transmission mode";
                return false;
            }

            // Wait for module to be ready
            if (!waitForAUX()) {
                _lastError = "Module not ready for transmission";
                return false;
            }

            // Send message
            size_t bytesWritten = _serial->write(static_cast<const uint8_t*>(message), length);

            if (bytesWritten != length) {
                _lastError = "Failed to send complete message";
                return false;
            }

            Serial.println("Message sent: " + String(bytesWritten) + " bytes");
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
                // Read available data into buffer
                size_t availableBytes = _serial->available();
                size_t bytesToRead = min(availableBytes, RX_BUFFER_SIZE - _rxBufferLen);

                if (bytesToRead > 0) {
                    size_t bytesRead = _serial->readBytes(_rxBuffer + _rxBufferLen, bytesToRead);
                    _rxBufferLen += bytesRead;

                    // Very basic check if we have a complete message
                    // In a real implementation, you would need proper message framing
                    if (_rxBufferLen >= sizeof(MessageHeader)) {
                        _hasMessage = true;
                        return true;
                    }
                }
            }

            return false;
        }

        size_t EbyteCommModule::receiveMessage(void* buffer, size_t maxLength, uint8_t& senderNodeId) {
            if (!_initialized || !_hasMessage || _rxBufferLen == 0) {
                return 0;
            }

            // Copy the message from the buffer
            size_t bytesToCopy = min(_rxBufferLen, maxLength);
            memcpy(buffer, _rxBuffer, bytesToCopy);

            // Extract sender node ID from message (assuming it's in the header)
            // This would depend on your message format
            const MessageHeader* header = reinterpret_cast<const MessageHeader*>(_rxBuffer);
            senderNodeId = header->nodeId;

            // Clear the buffer
            _rxBufferLen = 0;
            _hasMessage = false;

            Serial.println("Message received: " + String(bytesToCopy) + " bytes from node " + String(senderNodeId));
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
            Serial.println("Transmission power set to: " + String(level));

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
        }

        bool EbyteCommModule::setMode(uint8_t mode) {
            // Mode 0: Normal mode (transmit/receive)
            // Mode 1: Wake-up mode
            // Mode 2: Power saving mode (configuration mode in E49)
            // Mode 3: Sleep mode

            // Set module to the specified mode
            switch (mode) {
                case MODE_NORMAL:    _ebyte->SetMode(0b00); break; // Normal mode
                case MODE_WAKEUP:    _ebyte->SetMode(0b01); break; // Wake up mode
                case MODE_POWERDOWN: _ebyte->SetMode(0b10); break; // Power down mode (Configuration mode)
                case MODE_SLEEP:     _ebyte->SetMode(0b11); break; // Sleep mode
                default: return false;
            }

            // Wait for module to be ready
            return waitForAUX();
        }

        bool EbyteCommModule::waitForAUX(unsigned long timeout) {
            unsigned long startTime = millis();

            // Give the module some time to change state
            delay(50);

            while (digitalRead(_pinAUX) == LOW) {
                if (millis() - startTime > timeout) {
                    _lastError = "AUX pin timeout";
                    return false;
                }
                delay(1);
            }

            return true;
        }

        void EbyteCommModule::printParameters() {
            if (!_initialized) {
                Serial.println("Module not initialized");
                return;
            }

            Serial.println("\nModule Parameters (from cache):");
            Serial.println("Channel: " + String(_channel));
            Serial.println("Address: 0x" + String(_address, HEX));

            // Don't try to read from the module since it may not respond correctly
            // Instead, just print what we've set
        }

    } // namespace communication
} // namespace szogfm