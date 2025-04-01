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

                  _pinAUX(33) {           // AUX pin for EBYTE module

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
            Serial.println("Initializing radio module...");
            _radio = new radio::RDA5807Radio(_pinSDASensor, _pinSCLSensor);
            if (!_radio->initialize()) {
                handleError("Failed to initialize radio module");
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
            _display = new display::ST7789Display(240, 240, 18, 19, 23, 35, 25, 26);
            if (!_display->initialize()) {
                handleError("Failed to initialize display");
                return false;
            }
            Serial.println("ST7789 TFT display initialized successfully");

            // Set display rotation if needed
            _display->setRotation(2);  // Adjust based on your mounting orientation

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
            // Process the command based on type
            bool success = false;

            switch (message.command) {
                case Command::SET_VOLUME:
                    success = processSetVolumeCommand(message.data[0]);
                    break;

                case Command::SET_FREQUENCY:
                {
                    // Extract frequency from data bytes
                    uint16_t frequency = message.data[0] | (message.data[1] << 8);
                    success = processSetFrequencyCommand(frequency);
                }
                    break;

                case Command::TOGGLE_RELAY:
                    success = processToggleRelayCommand(message.data[0] != 0);
                    break;

                case Command::MUTE:
                    success = processMuteCommand(true);
                    break;

                case Command::UNMUTE:
                    success = processMuteCommand(false);
                    break;

                case Command::RESET:
                    // Reset the node
                    ESP.restart();
                    success = true; // This will never be reached
                    break;

                case Command::GET_STATUS:
                    success = sendStatusMessage();
                    break;

                default:
                    success = false;
                    break;
            }

            // Send acknowledgement
            sendAcknowledgement(message.header.sequenceNum, success);

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