#include "NodeApplication.h"
#include <Arduino.h>

#ifdef ENABLE_DHT_SENSOR
#include <DHT.h>
#endif

namespace szogfm {
    namespace node {

// Static instances for sensor (if enabled)
#ifdef ENABLE_DHT_SENSOR
        DHT* dht = nullptr;
#endif

        NodeApplication::NodeApplication()
                : _initialized(false), _connected(false),
                  _lastStatusTime(0), _lastDisplayUpdateTime(0),
                  _lastHeartbeatTime(0), _lastReceivedMessageTime(0),
                  _connectionSignalStrength(0), _messageSequence(0),
                  _hasSensors(false), _temperature(0.0f), _humidity(0.0f),

                // Add distinct pin values for I2C:
                  _pinUserButton(34),        // Analog input for buttons
                  _pinRelayControl(27),      // Control pin for relay
                  _pinSDASensor(21),         // SDA pin for I2C (FM radio)
                  _pinSCLSensor(22),         // SCL pin for I2C (FM radio)
                  _pinM0(4),                 // M0 pin for EBYTE module
                  _pinM1(32),                // M1 pin for EBYTE module
                  _pinAUX(33) {              // AUX pin for EBYTE module

            _commModule = nullptr;
            _radio = nullptr;
            _display = nullptr;
            _buttonHandler = nullptr;
        }

        bool NodeApplication::initialize() {
            Serial.println("Initializing NodeApplication...");

            // Load configuration
            if (!_config.load()) {
                Serial.println("Failed to load configuration, using defaults");
                _config.reset(true);
            }

            // Initialize components

            // Initialize button handler
            Serial.println("Initializing button handler...");
            _buttonHandler = new input::ButtonHandler(_pinUserButton);
            _buttonHandler->initialize();
            setupButtonCallbacks();
            Serial.println("Button handler initialized");

            // Initialize radio module
            Serial.println("Scanning I2C bus for devices...");
            Wire.begin(_pinSDASensor, _pinSCLSensor);
            byte error, address;
            int deviceCount = 0;

            for (address = 1; address < 127; address++) {
                Wire.beginTransmission(address);
                error = Wire.endTransmission();

                if (error == 0) {
                    Serial.print("I2C device found at address 0x");
                    if (address < 16) Serial.print("0");
                    Serial.print(address, HEX);
                    Serial.println("!");
                    deviceCount++;
                }
            }

            if (deviceCount == 0) {
                Serial.println("No I2C devices found - check wiring!");
            } else {
                Serial.print("Found ");
                Serial.print(deviceCount);
                Serial.println(" device(s)");
            }

            Serial.println("Initializing radio module...");
            _radio = new radio::RDA5807Radio(_pinSDASensor, _pinSCLSensor);
            if (!_radio->initialize()) {
                handleError("Failed to initialize radio module: " + _radio->getLastError());
                return false;
            }
            Serial.println("Radio module initialized successfully");

            // Set initial radio parameters
            _radio->setFrequency(_config.getFmFrequency());
            _radio->setVolume(_config.getVolume());
            _radio->setMute(_config.isMuted());
            Serial.println("Radio parameters set");

            // Initialize ST7789 TFT display
            Serial.println("Initializing ST7789 TFT display...");
            // CLK: GPIO18, MISO: GPIO19, MOSI: GPIO23, CS: GPIO35, RST: GPIO26, DC: GPIO25
            _display = new display::ST7789Display(240, 240, 18, 19, 23, 12, 25, 26);
            if (!_display->initialize()) {
                handleError("Failed to initialize display");
                return false;
            }
            Serial.println("ST7789 TFT display initialized successfully");

            // Set display rotation if needed
            _display->setRotation(2);  // Adjust based on your mounting orientation


            // Add this after display initialization in NodeApplication::initialize()
            if (_display) {
                Serial.println("Drawing test pattern on display");

                // Display test message
                _display->clear();
                _display->displayMessage("TEST RED", 20, 20, 2);
                _display->displayMessage("Display Check", 20, 60, 2);
                _display->update();
                delay(2000);

                // Try another message
                _display->clear();
                _display->displayMessage("TEST GREEN", 20, 20, 2);
                _display->displayMessage("If you see this", 20, 60, 2);
                _display->displayMessage("display is working", 20, 100, 2);
                _display->update();
                delay(2000);

                // Return to normal display state
                _display->clear();
                _display->update();

                Serial.println("Test pattern complete");
            }

            // Initialize communication module
            Serial.println("Initializing communication module...");
            _commModule = new communication::EbyteCommModule(&Serial2, _pinM0, _pinM1, _pinAUX);
            if (!_commModule->initialize()) {
                handleError("Failed to initialize communication module");
                return false;
            }
            Serial.println("Communication module initialized successfully");

            // Configure communication module
            Serial.println("Configuring communication module...");
            // Use the proper constants for air data rate and UART baud rate
            // Air data rate: 1 = 2.4k baud (AIR_2K4)
            // UART baud rate: 3 = 9600 baud (UART_9600)
            // Power level: 0 = 20dBm (max power)
            if (!_commModule->configure(_config.getRadioChannel(), _config.getRadioAddress(),
                                        communication::AIR_2K4, communication::UART_9600, 0)) {
                Serial.println("Warning: Configuration issues with communication module");
                Serial.println("Last error: " + _commModule->getLastError());
                Serial.println("Continuing despite configuration issues...");
                // Don't return false - module may still work for basic messaging
            } else {
                Serial.println("Communication module configured successfully");
            }

            // Set high debug level for communication module
            _commModule->setDebugLevel(2);
            // Perform diagnostics on the module
            _commModule->performDiagnostics();
            // Print current module parameters
            _commModule->printParameters();

            // Initialize relay control pin
            Serial.println("Initializing relay control...");
            pinMode(_pinRelayControl, OUTPUT);

            // Set initial relay state
            Serial.println("Setting initial relay state");
            applyRelayState();

            // Initialize sensors if available
            Serial.println("Checking for sensors...");
            _hasSensors = initializeSensors();
            if (_hasSensors) {
                Serial.println("Sensors detected and initialized");
            } else {
                Serial.println("No sensors detected");
            }

            _initialized = true;

            // Initial display update
            updateDisplay();

            Serial.println("NodeApplication initialized successfully");
            return true;
        }

        void NodeApplication::update() {
            if (!_initialized) {
                return;
            }

            // Process any received messages
            processMessages();

            // Read sensors if available
            if (_hasSensors) {
                readSensors();
            }

            // Update button handler
            _buttonHandler->update();

            // Check if we should send a status update
            unsigned long currentTime = millis();
            if (currentTime - _lastStatusTime > STATUS_INTERVAL) {
                sendStatusMessage();
                _lastStatusTime = currentTime;
            }

            // Check if we should send a heartbeat
            if (currentTime - _lastHeartbeatTime > HEARTBEAT_INTERVAL) {
                sendHeartbeat();
                _lastHeartbeatTime = currentTime;
            }

            // Check connection status
            checkConnection();

            // Update display
            if (currentTime - _lastDisplayUpdateTime > DISPLAY_UPDATE_INTERVAL) {
                updateDisplay();
                _lastDisplayUpdateTime = currentTime;
            }

            // Update radio module
            _radio->update();

            // Update communication module
            _commModule->update();
        }

        bool NodeApplication::handleCommand(const CommandMessage& message) {
            // Add detailed logging
            Serial.printf("\n[CMD] Received command: %d, Seq: %d from node %d\n",
                          static_cast<int>(message.command),
                          message.header.sequenceNum,
                          message.header.nodeId);

            // Process the command based on type
            bool success = false;

            switch (message.command) {
                case Command::SET_VOLUME:
                {
                    uint8_t volume = message.data[0];
                    Serial.printf("[CMD] SET_VOLUME command: volume=%d\n", volume);

                    // First update display to confirm command receipt
                    if (_display) {
                        Serial.println("[CMD] Updating display with new volume");
                        _display->displayVolume(volume, _config.isMuted());
                        _display->update();
                    }

                    // Actually process the command
                    success = processSetVolumeCommand(volume);
                    Serial.printf("[CMD] SET_VOLUME result: %s\n", success ? "SUCCESS" : "FAILED");
                    if (success) {
                        Serial.printf("[CMD] New volume set: %d\n", _config.getVolume());
                    } else {
                        Serial.printf("[CMD] Current volume unchanged: %d\n", _config.getVolume());
                    }
                }
                    break;

                case Command::SET_FREQUENCY:
                {
                    // Extract frequency from data bytes
                    uint16_t frequency = message.data[0] | (message.data[1] << 8);
                    Serial.printf("[CMD] SET_FREQUENCY command: frequency=%d (%.1f MHz)\n",
                                  frequency, frequency/100.0);

                    // First update display to confirm command receipt
                    if (_display) {
                        Serial.println("[CMD] Updating display with new frequency");
                        _display->displayFrequency(frequency);
                        _display->update();
                    }

                    // Actually process the command
                    success = processSetFrequencyCommand(frequency);
                    Serial.printf("[CMD] SET_FREQUENCY result: %s\n", success ? "SUCCESS" : "FAILED");
                    if (success) {
                        Serial.printf("[CMD] New frequency set: %d (%.1f MHz)\n",
                                      _config.getFmFrequency(), _config.getFmFrequency()/100.0);
                    } else {
                        Serial.printf("[CMD] Current frequency unchanged: %d (%.1f MHz)\n",
                                      _config.getFmFrequency(), _config.getFmFrequency()/100.0);
                    }
                }
                    break;

                case Command::TOGGLE_RELAY:
                {
                    bool state = message.data[0] != 0;
                    Serial.printf("[CMD] TOGGLE_RELAY command: relay=%s\n", state ? "ON" : "OFF");

                    // First update display to confirm command receipt
                    if (_display) {
                        Serial.println("[CMD] Updating display with new relay state");
                        _display->displayNodeInfo(_config.getNodeId(), state);
                        _display->update();
                    }

                    // Actually process the command
                    success = processToggleRelayCommand(state);
                    Serial.printf("[CMD] TOGGLE_RELAY result: %s\n", success ? "SUCCESS" : "FAILED");
                    Serial.printf("[CMD] Current relay state: %s\n",
                                  _config.getRelayState() ? "ON" : "OFF");
                }
                    break;

                case Command::MUTE:
                    Serial.println("[CMD] MUTE command received");

                    // First update display to confirm command receipt
                    if (_display) {
                        Serial.println("[CMD] Updating display with mute state");
                        _display->displayVolume(_config.getVolume(), true);
                        _display->update();
                    }

                    // Actually process the command
                    success = processMuteCommand(true);
                    Serial.printf("[CMD] MUTE result: %s\n", success ? "SUCCESS" : "FAILED");
                    Serial.printf("[CMD] Current mute state: %s\n",
                                  _config.isMuted() ? "MUTED" : "UNMUTED");
                    break;

                case Command::UNMUTE:
                    Serial.println("[CMD] UNMUTE command received");

                    // First update display to confirm command receipt
                    if (_display) {
                        Serial.println("[CMD] Updating display with unmute state");
                        _display->displayVolume(_config.getVolume(), false);
                        _display->update();
                    }

                    // Actually process the command
                    success = processMuteCommand(false);
                    Serial.printf("[CMD] UNMUTE result: %s\n", success ? "SUCCESS" : "FAILED");
                    Serial.printf("[CMD] Current mute state: %s\n",
                                  _config.isMuted() ? "MUTED" : "UNMUTED");
                    break;

                case Command::RESET:
                    // Reset the node
                    Serial.println("[CMD] RESET command received - Restarting ESP32...");

                    // Send acknowledgement before resetting
                    sendAcknowledgement(message.header.sequenceNum, true);

                    delay(500);  // Small delay to allow ACK to be sent

                    ESP.restart();
                    success = true; // This will never be reached
                    break;

                case Command::GET_STATUS:
                    Serial.println("[CMD] GET_STATUS command received");
                    success = sendStatusMessage();
                    Serial.printf("[CMD] Status message sent: %s\n", success ? "SUCCESS" : "FAILED");
                    break;

                default:
                    Serial.printf("[CMD] UNKNOWN command code: %d\n", static_cast<int>(message.command));
                    success = false;
                    break;
            }

            // Check if radio module is responding
            if (_radio) {
                int rssi = _radio->getRssi();
                bool stereo = _radio->isStereo();
                bool tuned = _radio->isTunedToStation();
                Serial.printf("[CMD] Radio status - RSSI: %d, Stereo: %s, Tuned: %s\n",
                              rssi, stereo ? "Yes" : "No", tuned ? "Yes" : "No");
            } else {
                Serial.println("[CMD] WARNING: Radio module pointer is null!");
            }

            // Send acknowledgement (unless it was a RESET command, which already sent the ACK)
            if (message.command != Command::RESET) {
                Serial.printf("[CMD] Sending ACK for sequence %d, success=%s\n",
                              message.header.sequenceNum, success ? "true" : "false");
                sendAcknowledgement(message.header.sequenceNum, success);
            }

            return success;
        }

        bool NodeApplication::handleStatusRequest(const MessageHeader& message) {
            // Send a status message in response to the request
            return sendStatusMessage();
        }

        bool NodeApplication::sendStatusMessage() {
            if (!_initialized || !_commModule) {
                return false;
            }

            // Prepare status message
            StatusMessage statusMsg;
            statusMsg.header.version = 1;
            statusMsg.header.type = MessageType::STATUS_RESPONSE;
            statusMsg.header.nodeId = _config.getNodeId();
            statusMsg.header.sequenceNum = _messageSequence++;
            statusMsg.header.payloadLength = sizeof(StatusMessage) - sizeof(MessageHeader);
            statusMsg.header.timestamp = millis();

            // Fill status data
            statusMsg.status = NodeStatus::OK;
            statusMsg.volume = _config.getVolume();
            statusMsg.frequency = _config.getFmFrequency();
            statusMsg.relayState = _config.getRelayState();
            statusMsg.rssi = _radio->getRssi();
            statusMsg.isStereo = _radio->isStereo();
            statusMsg.uptime = millis();

            // Add sensor data if available
            if (_hasSensors) {
                statusMsg.temperature = _temperature;
                statusMsg.humidity = _humidity;
            } else {
                statusMsg.temperature = -100.0f; // Invalid value to indicate no sensor
                statusMsg.humidity = -1.0f;      // Invalid value to indicate no sensor
            }

            // Compute header checksum
            statusMsg.header.setChecksum();

            // Send message
            return _commModule->sendMessage(0, &statusMsg, sizeof(StatusMessage));
        }

        void NodeApplication::handleError(const String& message) {
            Serial.println("ERROR: " + message);

            if (_display) {
                _display->displayError(message);
            }
        }

        void NodeApplication::setupButtonCallbacks() {
            // Add buttons with their analog value ranges
            // These ranges are for the resistor network described in the documentation
            // with 4 buttons connected to a single analog pin

            // Button: Frequency Up (1600-2200)
            _buttonHandler->addButton(input::ButtonType::FREQUENCY_UP, 1600, 2200, [this]() {
                uint16_t freq = _config.getFmFrequency();
                freq += 10; // Increase by 0.1 MHz
                if (freq > 10800) freq = 8750; // Wrap around
                processSetFrequencyCommand(freq);
            });

            // Button: Frequency Down (2300-2900)
            _buttonHandler->addButton(input::ButtonType::FREQUENCY_DOWN, 2300, 2900, [this]() {
                uint16_t freq = _config.getFmFrequency();
                freq -= 10; // Decrease by 0.1 MHz
                if (freq < 8750) freq = 10800; // Wrap around
                processSetFrequencyCommand(freq);
            });

            // Button: Volume Up (3700-4095)
            _buttonHandler->addButton(input::ButtonType::VOLUME_UP, 3700, 4095, [this]() {
                uint8_t vol = _config.getVolume();
                if (vol < 15) vol++; // Increase volume if not at maximum
                processSetVolumeCommand(vol);
            });

            // Button: Volume Down (1000-1400)
            _buttonHandler->addButton(input::ButtonType::VOLUME_DOWN, 1000, 1400, [this]() {
                uint8_t vol = _config.getVolume();
                if (vol > 0) vol--; // Decrease volume if not at minimum
                processSetVolumeCommand(vol);
            });
        }

        void NodeApplication::updateDisplay() {
            if (!_display) {
                return;
            }

            // Update frequency display
            _display->displayFrequency(_config.getFmFrequency());

            // Update volume display
            _display->displayVolume(_config.getVolume(), _config.isMuted());

            // Update connection status
            _display->displayConnectionStatus(_connected, _connectionSignalStrength);

            // Update node info
            _display->displayNodeInfo(_config.getNodeId(), _config.getRelayState());

            // Update sensor data if available
            if (_hasSensors) {
                _display->displaySensorData(_temperature, _humidity);
            }

            // Refresh display
            _display->update();
        }

        bool NodeApplication::initializeSensors() {
#ifdef ENABLE_DHT_SENSOR
            // Initialize DHT sensor
    dht = new DHT(_pinSDASensor, DHT22);
    dht->begin();

    // Test if sensor is responding
    float test = dht->readTemperature();
    if (isnan(test)) {
        // Sensor failed
        delete dht;
        dht = nullptr;
        return false;
    }

    return true;
#else
            return false;
#endif
        }

        void NodeApplication::readSensors() {
#ifdef ENABLE_DHT_SENSOR
            if (dht) {
        // Read temperature and humidity
        float newTemp = dht->readTemperature();
        float newHumidity = dht->readHumidity();

        // Check if readings are valid
        if (!isnan(newTemp) && !isnan(newHumidity)) {
            _temperature = newTemp;
            _humidity = newHumidity;
        }
    }
#endif
        }

        void NodeApplication::applyVolume() {
            if (_radio) {
                _radio->setVolume(_config.getVolume());
                _radio->setMute(_config.isMuted());
            }
        }

        void NodeApplication::applyRelayState() {
            digitalWrite(_pinRelayControl, _config.getRelayState() ? HIGH : LOW);
        }

        void NodeApplication::checkConnection() {
            unsigned long currentTime = millis();

            // Check if we've received a message recently
            bool wasConnected = _connected;
            _connected = (currentTime - _lastReceivedMessageTime < CONNECTION_TIMEOUT);

            // If connection state changed, update the display
            if (wasConnected != _connected) {
                updateDisplay();
            }
        }

        bool NodeApplication::processMessages() {
            if (!_commModule || !_commModule->isMessageAvailable()) {
                return false;
            }

            // Buffer for received message
            uint8_t buffer[256];
            uint8_t senderNodeId;

            // Receive message
            size_t bytesReceived = _commModule->receiveMessage(buffer, sizeof(buffer), senderNodeId);
            if (bytesReceived == 0) {
                return false;
            }

            // Update connection status
            _lastReceivedMessageTime = millis();
            _connectionSignalStrength = _commModule->getSignalStrength();
            _connected = true;

            // Parse the message header
            if (bytesReceived < sizeof(MessageHeader)) {
                Serial.println("Received message too short");
                return false;
            }

            const MessageHeader* header = reinterpret_cast<const MessageHeader*>(buffer);

            // Validate checksum
            if (!header->validateChecksum()) {
                Serial.println("Received message has invalid checksum");
                return false;
            }

            // Process message based on type
            switch (header->type) {
                case MessageType::COMMAND:
                    if (bytesReceived >= sizeof(CommandMessage)) {
                        const CommandMessage* command = reinterpret_cast<const CommandMessage*>(buffer);
                        return handleCommand(*command);
                    }
                    break;

                case MessageType::STATUS_REQUEST:
                    return handleStatusRequest(*header);

                case MessageType::ACK:
                    // Process acknowledgement if needed
                    // Not implementing here as nodes typically just receive commands
                    break;

                case MessageType::ERROR:
                    // Process error message if needed
                    break;

                default:
                    Serial.println("Received unknown message type");
                    break;
            }

            return true;
        }

        bool NodeApplication::sendHeartbeat() {
            return sendStatusMessage(); // Simply send a status message as a heartbeat
        }

        bool NodeApplication::processSetVolumeCommand(uint8_t volume) {
            if (!_radio) {
                return false;
            }

            // Validate volume range
            if (volume > 15) {
                volume = 15;
            }

            // Set volume
            if (_radio->setVolume(volume)) {
                // Update configuration
                _config.setVolume(volume);
                _config.save();

                return true;
            }

            return false;
        }

        bool NodeApplication::processSetFrequencyCommand(uint16_t frequency) {
            if (!_radio) {
                return false;
            }

            // Validate frequency range (87.5 MHz to 108.0 MHz)
            if (frequency < 8750 || frequency > 10800) {
                return false;
            }

            // Set frequency
            if (_radio->setFrequency(frequency)) {
                // Update configuration
                _config.setFmFrequency(frequency);
                _config.save();

                return true;
            }

            return false;
        }

        bool NodeApplication::processToggleRelayCommand(bool state) {
            // Set relay state
            _config.setRelayState(state);
            applyRelayState();
            _config.save();

            return true;
        }

        bool NodeApplication::processMuteCommand(bool mute) {
            if (!_radio) {
                return false;
            }

            // Set mute state
            if (_radio->setMute(mute)) {
                // Update configuration
                _config.setMuted(mute);
                _config.save();

                return true;
            }

            return false;
        }

        bool NodeApplication::sendAcknowledgement(uint16_t sequenceNum, bool success, uint8_t errorCode) {
            if (!_initialized || !_commModule) {
                return false;
            }

            // Prepare acknowledgement message
            AckMessage ackMsg;
            ackMsg.header.version = 1;
            ackMsg.header.type = MessageType::ACK;
            ackMsg.header.nodeId = _config.getNodeId();
            ackMsg.header.sequenceNum = _messageSequence++;
            ackMsg.header.payloadLength = sizeof(AckMessage) - sizeof(MessageHeader);
            ackMsg.header.timestamp = millis();

            // Fill acknowledgement data
            ackMsg.acknowledgedSeq = sequenceNum;
            ackMsg.success = success;
            ackMsg.errorCode = errorCode;

            // Compute header checksum
            ackMsg.header.setChecksum();

            // Send acknowledgement
            return _commModule->sendMessage(0, &ackMsg, sizeof(AckMessage));
        }

    } // namespace node
} // namespace szogfm