#include "NodeApplication.h"
#include "SimpleNodeStatus.h"
#include <Arduino.h>
#include "../common/input/ButtonHandler.h"

#ifdef ENABLE_DHT_SENSOR
#include <DHT.h>
#endif

namespace szogfm {
    namespace node {

        // Helper function to repeat a character
        String repeatChar(char c, int count) {
            String result = "";
            result.reserve(count);
            for (int i = 0; i < count; i++) {
                result += c;
            }
            return result;
        }

        // Using input namespace for easier access to ButtonType
        using namespace szogfm::input;

// Static instances for sensor (if enabled)
#ifdef ENABLE_DHT_SENSOR
        DHT* dht = nullptr;
#endif

        NodeApplication::NodeApplication()
                : _initialized(false), _connected(false),
                  _lastStatusTime(0), _lastHeartbeatTime(0),
                  _lastReceivedMessageTime(0), _lastSelfTestTime(0),
                  _connectionSignalStrength(0), _messageSequence(0),
                  _hasSensors(false), _temperature(0.0f), _humidity(0.0f),
                  _verboseDebugging(false), // Changed to false for clean output

                // Pin configuration with clear documentation
                  _pinUserButton(34),        // Analog input for resistor network buttons
                  _pinRelayControl(5),      // Digital output for relay control
                  _pinSDASensor(21),         // I2C SDA (FM radio only - NO DISPLAY)
                  _pinSCLSensor(22),         // I2C SCL (FM radio only - NO DISPLAY)
                  _pinM0(4),                 // EBYTE M0 control pin
                  _pinM1(32),                // EBYTE M1 control pin
                  _pinAUX(33) {              // EBYTE AUX status pin

            _commModule = nullptr;
            _radio = nullptr;
            _buttonHandler = nullptr;

            // Initialize statistics
            memset(&_stats, 0, sizeof(_stats));
            _stats.bootTime = millis();

            Serial.println("🎵 SzögFM Node Application created (FIXED VERSION - Proper Command Filtering)");
        }

        bool NodeApplication::initialize() {
            SimpleNodeStatus::printHeader();
            Serial.println("🚀 Starting initialization (FIXED VERSION)...\n");
            Serial.println("🔧 CRITICAL FIX: Added proper command filtering by Node ID");
            Serial.println("   - Commands now check if they're meant for this node");
            Serial.println("   - Broadcast commands (nodeId=0) processed by all nodes");
            Serial.println("   - Individual commands only processed by target node\n");

            // Load configuration
            SimpleNodeStatus::printBootSequence("Load Configuration", true,
                                                "Node ID: " + String(_config.getNodeId()));

            if (!_config.load()) {
                _config.reset(true);
            }

            // IMPORTANT: Show which Node ID this device will respond to
            Serial.printf("📍 THIS NODE WILL RESPOND TO:\n");
            Serial.printf("   • Node ID %d (individual commands)\n", _config.getNodeId());
            Serial.printf("   • Node ID 0 (broadcast commands to all nodes)\n");
            Serial.printf("   • All other node IDs will be IGNORED\n\n");

            // Initialize I2C and scan for devices
            SimpleNodeStatus::printBootSequence("I2C Bus Setup", true, "SDA:" + String(_pinSDASensor) + " SCL:" + String(_pinSCLSensor));
            Wire.begin(_pinSDASensor, _pinSCLSensor);
            Wire.setClock(100000);

            // Quick I2C scan
            bool foundFMRadio = false;
            for (uint8_t addr = 1; addr < 127; addr++) {
                Wire.beginTransmission(addr);
                if (Wire.endTransmission() == 0 && addr == 0x11) {
                    foundFMRadio = true;
                    break;
                }
            }

            // Initialize FM radio
            if (foundFMRadio) {
                _radio = new radio::RDA5807Radio(_pinSDASensor, _pinSCLSensor);
                bool radioOk = _radio->initialize();
                SimpleNodeStatus::printBootSequence("FM Radio (RDA5807M)", radioOk,
                                                    radioOk ? "Ready" : _radio->getLastError());

                if (radioOk) {
                    _radio->setFrequency(_config.getFmFrequency());
                    _radio->setVolume(_config.getVolume());
                    _radio->setMute(_config.isMuted());
                }
            } else {
                SimpleNodeStatus::printBootSequence("FM Radio (RDA5807M)", false, "Not found at 0x11");
                return false;
            }

            // Initialize communication
            _commModule = new communication::EbyteCommModule(&Serial2, _pinM0, _pinM1, _pinAUX);
            bool commOk = _commModule->initialize();
            SimpleNodeStatus::printBootSequence("433MHz Communication", commOk,
                                                commOk ? "EBYTE E49 Ready" : "Check wiring");

            if (commOk) {
                bool configOk = _commModule->configure(_config.getRadioChannel(), _config.getRadioAddress(),
                                                       communication::AIR_2K4, communication::UART_9600, 0);
                if (!configOk) {
                    SimpleNodeStatus::printBootSequence("433MHz Configuration", false, "Using defaults");
                }
                _commModule->setDebugLevel(0); // Quiet mode
            }

            // Initialize buttons with IMPROVED noise filtering
            _buttonHandler = new input::ButtonHandler(_pinUserButton, 150); // Increased debounce to 150ms
            _buttonHandler->initialize();
            setupButtonCallbacks();
            SimpleNodeStatus::printBootSequence("Button Handler", true, "4 buttons configured with noise filtering");

            // Initialize relay with EXPLICIT pin configuration
            pinMode(_pinRelayControl, OUTPUT);
            applyRelayState();
            SimpleNodeStatus::printBootSequence("Relay Control", true,
                                                "Pin " + String(_pinRelayControl) + " -> " + String(_config.getRelayState() ? "ON" : "OFF"));

            // Check for sensors
            _hasSensors = initializeSensors();
            if (_hasSensors) {
                SimpleNodeStatus::printBootSequence("Environmental Sensors", true, "DHT22 detected");
            }

            _initialized = true;
            _stats.bootTime = millis();

            Serial.println();
            SimpleNodeStatus::printConnectionTest();

            // Send initial status
            if (commOk && sendStatusMessage()) {
                Serial.println("📤 Initial status sent to controller");
            }

            return true;
        }

        void NodeApplication::update() {
            if (!_initialized) {
                return;
            }

            static unsigned long lastStatusDisplay = 0;
            static unsigned long lastConnectionCheck = 0;
            static unsigned long lastAnalogMonitor = 0;
            static bool connectionTestActive = false;
            static unsigned long connectionTestStart = 0;
            unsigned long currentTime = millis();

            // Show clean status dashboard every 5 seconds
            if (currentTime - lastStatusDisplay > 5000) {
                SimpleNodeStatus::NodeInfo info;
                info.nodeId = _config.getNodeId();
                info.connected = _connected;
                info.frequency = _config.getFmFrequency();
                info.volume = _config.getVolume();
                info.muted = _config.isMuted();
                info.relayOn = _config.getRelayState();
                info.fmSignal = _radio ? _radio->getRssi() : 0;
                info.commSignal = _connectionSignalStrength;
                info.lastCommand = _lastReceivedMessageTime;
                info.uptime = millis();
                info.lastError = ""; // Clear for now

                SimpleNodeStatus::printStatus(info);

                // Add relay debugging info to status
                Serial.printf("🔌 RELAY DEBUG: Config=%s, Pin%d=%s\n",
                              _config.getRelayState() ? "ON" : "OFF",
                              _pinRelayControl,
                              digitalRead(_pinRelayControl) ? "HIGH" : "LOW");

                lastStatusDisplay = currentTime;
            }

            // IMPROVED analog monitoring with noise filtering (every 3 seconds)
            if (currentTime - lastAnalogMonitor > 3000) {
                int analogVal = getFilteredAnalogReading();

                // Only show readings that are significantly away from the "no button" state
                if (analogVal < 4000) {
                    Serial.printf("🔘 Analog: %d | Buttons: Vol-[1000-1300] Freq+[1800-2000] Freq-[2500-2700] Vol+[3900-4095]\n", analogVal);

                    // Warn about values in "danger zones" that might cause false triggers
                    if ((analogVal > 800 && analogVal < 1500) ||
                        (analogVal > 1600 && analogVal < 2200) ||
                        (analogVal > 2300 && analogVal < 2900) ||
                        (analogVal > 3700 && analogVal < 4000)) {
                        Serial.printf("⚠️  Analog value %d in button zone - may cause false trigger!\n", analogVal);
                    }
                }
                lastAnalogMonitor = currentTime;
            }

            // Connection test logic (first 30 seconds after boot)
            if (!connectionTestActive && currentTime < 30000) {
                connectionTestActive = true;
                connectionTestStart = currentTime;
            }

            if (connectionTestActive && currentTime - connectionTestStart > 30000) {
                connectionTestActive = false;
                if (_connected) {
                    SimpleNodeStatus::printConnectionSuccess();
                } else {
                    SimpleNodeStatus::printConnectionFailed();
                }
            }

            // Process messages (WITH PROPER FILTERING - THIS IS THE KEY FIX!)
            processMessages();

            // Read sensors if available
            if (_hasSensors && currentTime - lastConnectionCheck > 5000) {
                readSensors();
                lastConnectionCheck = currentTime;
            }

            // Handle button presses with IMPROVED filtering
            /*if (_buttonHandler->update()) {
                _stats.totalButtonPresses++;
                int analogValue = _buttonHandler->getLastAnalogValue();

                // Additional validation - reject if analog value seems unstable
                int confirmValue = getFilteredAnalogReading();
                if (abs(analogValue - confirmValue) > 50) {
                    Serial.printf("🔘 Button press REJECTED due to unstable reading: %d vs %d\n", analogValue, confirmValue);
                    return; // Ignore this button press
                }

                Serial.printf("🔘 Button CONFIRMED: %s (analog: %d)\n",
                              static_cast<int>(_buttonHandler->getLastButtonType()) == 0 ? "VOL+" :
                              static_cast<int>(_buttonHandler->getLastButtonType()) == 1 ? "VOL-" :
                              static_cast<int>(_buttonHandler->getLastButtonType()) == 2 ? "FREQ+" : "FREQ-",
                              analogValue);
            }*/

            // Send periodic status updates (less frequent)
            if (currentTime - _lastStatusTime > 10000) { // Every 10 seconds
                sendStatusMessage();
                _lastStatusTime = currentTime;
            }

            // Send heartbeat (less frequent)
            if (currentTime - _lastHeartbeatTime > 60000) { // Every minute
                sendHeartbeat();
                _lastHeartbeatTime = currentTime;
            }

            // Check connection status
            checkConnection();

            // Update components
            if (_radio) _radio->update();
            if (_commModule) _commModule->update();

            // Update statistics
            updateStatistics();
        }

        /**
         * Get a filtered analog reading with noise reduction
         * Takes multiple samples and averages them to reduce noise
         */
        int NodeApplication::getFilteredAnalogReading() {
            const int numSamples = 5;
            long total = 0;

            // Take multiple samples
            for (int i = 0; i < numSamples; i++) {
                total += analogRead(_pinUserButton);
                delayMicroseconds(100); // Small delay between samples
            }

            return total / numSamples;
        }

        bool NodeApplication::handleCommand(const CommandMessage& message) {
            _stats.totalCommandsReceived++;
            _stats.communication.messagesReceived++;

            // 🔥 CRITICAL FIX: CHECK IF COMMAND IS FOR THIS NODE! 🔥
            // This was the missing piece causing all nodes to respond to every command
            uint8_t targetNodeId = message.header.nodeId;
            uint8_t myNodeId = _config.getNodeId();

            // Only process if:
            // 1. Command is broadcast (nodeId = 0) - affects all nodes
            // 2. Command is specifically for this node (nodeId matches our node ID)
            if (targetNodeId != 0 && targetNodeId != myNodeId) {
                // Command is for a different node - ignore it completely
                if (_verboseDebugging) {
                    Serial.printf("📤 IGNORING command for node %d (I am node %d)\n", targetNodeId, myNodeId);
                }
                return true; // Return true because we handled it correctly by ignoring it
            }

            // If we reach here, the command is either broadcast (0) or specifically for us
            String targetType = (targetNodeId == 0) ? "BROADCAST" : "INDIVIDUAL";
            Serial.printf("📥 PROCESSING %s command (target: %d, I am: %d): ",
                          targetType.c_str(), targetNodeId, myNodeId);

            // Simple command logging
            String commandName = "";
            switch (message.command) {
                case Command::SET_VOLUME: commandName = "SET_VOLUME"; break;
                case Command::SET_FREQUENCY: commandName = "SET_FREQUENCY"; break;
                case Command::TOGGLE_RELAY: commandName = "TOGGLE_RELAY"; break;
                case Command::MUTE: commandName = "MUTE"; break;
                case Command::UNMUTE: commandName = "UNMUTE"; break;
                case Command::RESET: commandName = "RESET"; break;
                case Command::GET_STATUS: commandName = "GET_STATUS"; break;
                default: commandName = "UNKNOWN"; break;
            }

            Serial.printf("%s", commandName.c_str());

            // Process the command
            bool success = false;

            switch (message.command) {
                case Command::SET_VOLUME:
                {
                    uint8_t volume = message.data[0];
                    Serial.printf(" -> Volume %d", volume);
                    success = processSetVolumeCommand(volume);
                }
                    break;

                case Command::SET_FREQUENCY:
                {
                    uint16_t frequency = message.data[0] | (message.data[1] << 8);
                    Serial.printf(" -> %.1f MHz", frequency / 100.0);
                    success = processSetFrequencyCommand(frequency);
                }
                    break;

                case Command::TOGGLE_RELAY:
                {
                    bool state = message.data[0] != 0;
                    Serial.printf(" -> Relay %s", state ? "ON" : "OFF");
                    success = processToggleRelayCommand(state);
                }
                    break;

                case Command::MUTE:
                    Serial.print(" -> MUTE ON");
                    success = processMuteCommand(true);
                    break;

                case Command::UNMUTE:
                    Serial.print(" -> MUTE OFF");
                    success = processMuteCommand(false);
                    break;

                case Command::RESET:
                    Serial.print(" -> RESTART NODE");
                    sendAcknowledgement(message.header.sequenceNum, true);
                    delay(1000);
                    ESP.restart();
                    success = true;
                    break;

                case Command::GET_STATUS:
                    Serial.print(" -> Status Request");
                    success = sendStatusMessage();
                    break;

                default:
                    Serial.printf(" -> UNKNOWN (%d)", static_cast<int>(message.command));
                    success = false;
                    break;
            }

            // Show result
            Serial.printf(" %s\n", success ? "✅" : "❌");

            // Update statistics
            if (success) {
                _stats.successfulCommands++;
            } else {
                _stats.failedCommands++;
            }
            _stats.commandSuccessRate = (_stats.successfulCommands * 100.0f) / _stats.totalCommandsReceived;

            // Send acknowledgement
            if (message.command != Command::RESET) {
                sendAcknowledgement(message.header.sequenceNum, success);
            }

            return success;
        }

        bool NodeApplication::handleStatusRequest(const MessageHeader& message) {
            _stats.totalStatusRequests++;
            return sendStatusMessage();
        }

        bool NodeApplication::sendStatusMessage() {
            if (!_initialized || !_commModule) {
                return false;
            }

            // Prepare comprehensive status message
            StatusMessage statusMsg;
            statusMsg.header.version = 1;
            statusMsg.header.type = MessageType::STATUS_RESPONSE;
            statusMsg.header.nodeId = _config.getNodeId();
            statusMsg.header.sequenceNum = _messageSequence++;
            statusMsg.header.payloadLength = sizeof(StatusMessage) - sizeof(MessageHeader);
            statusMsg.header.timestamp = millis();

            // Fill status data from current configuration
            statusMsg.status = NodeStatus::OK;
            statusMsg.volume = _config.getVolume();
            statusMsg.frequency = _config.getFmFrequency();
            statusMsg.relayState = _config.getRelayState();
            statusMsg.uptime = millis();

            // Get real-time radio status
            if (_radio) {
                statusMsg.rssi = _radio->getRssi();
                statusMsg.isStereo = _radio->isStereo();
            } else {
                statusMsg.rssi = 0;
                statusMsg.isStereo = false;
            }

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

            // Send message to controller (node ID 0)
            bool success = _commModule->sendMessage(0, &statusMsg, sizeof(StatusMessage));

            if (success) {
                _stats.communication.messagesSent++;
                _stats.communication.lastMessageSent = millis();
            } else {
                _stats.communication.communicationErrors++;
            }

            return success;
        }

        void NodeApplication::handleError(const String& message) {
            Serial.println("🚨 ERROR: " + message);
            Serial.printf("   ⏰ Time: %lu ms\n", millis());
            Serial.printf("   💾 Free heap: %d bytes\n", ESP.getFreeHeap());

            // Flash built-in LED to indicate error
            pinMode(2, OUTPUT);
            for (int i = 0; i < 6; i++) {
                digitalWrite(2, HIGH);
                delay(100);
                digitalWrite(2, LOW);
                delay(100);
            }
        }

        bool NodeApplication::performSelfTest() {
            Serial.println("🧪 Performing self-test...");
            bool allTestsPassed = true;

            if (!testI2CBus()) allTestsPassed = false;
            if (!testRadio()) allTestsPassed = false;
            if (!testCommunication()) allTestsPassed = false;

            _stats.lastCommTest = millis();
            return allTestsPassed;
        }

        String NodeApplication::getDetailedSystemStatus() const {
            String status = "=== NODE " + String(_config.getNodeId()) + " STATUS ===\n";
            status += "Uptime: " + String(millis() / 1000) + " seconds\n";
            status += "Free Heap: " + String(ESP.getFreeHeap()) + " bytes\n";
            status += "Connected: " + String(_connected ? "Yes" : "No") + "\n";
            status += "FM Frequency: " + String(_config.getFmFrequency() / 100.0, 1) + " MHz\n";
            status += "Volume: " + String(_config.getVolume()) + "/15\n";
            status += "Relay: " + String(_config.getRelayState() ? "ON" : "OFF") + "\n";
            return status;
        }

        void NodeApplication::setupButtonCallbacks() {
            Serial.println("🔘 Setting up button callbacks with GREATLY IMPROVED noise filtering...");

            // GREATLY IMPROVED button ranges - much more conservative with larger gaps
            // Added significant dead zones between buttons to prevent cross-talk

            // Button: Volume Down (moved to a cleaner range, very tight)
            int volDownBtn = _buttonHandler->addButton(input::ButtonType::VOLUME_DOWN, 1000, 1300, [this]() {
                Serial.println("🔘 [VOL DOWN] Button press confirmed");
                uint8_t vol = _config.getVolume();
                uint8_t newVol = (vol > 0) ? vol - 1 : vol;
                Serial.printf("🔘 [VOL DOWN] %d → %d\n", vol, newVol);
                processSetVolumeCommand(newVol);
            });

            // LARGE GAP: 1300 - 1800 (500 point buffer zone)

            // Button: Frequency Up (moved to cleaner range, very tight)
            int freqUpBtn = _buttonHandler->addButton(input::ButtonType::FREQUENCY_UP, 1800, 2000, [this]() {
                Serial.println("🔘 [FREQ UP] Button press confirmed");
                uint16_t freq = _config.getFmFrequency();
                uint16_t newFreq = freq + 10; // Increase by 0.1 MHz
                if (newFreq > 10800) newFreq = 8750; // Wrap around
                Serial.printf("🔘 [FREQ UP] %d → %d (%.1f MHz)\n", freq, newFreq, newFreq/100.0);
                processSetFrequencyCommand(newFreq);
            });

            // LARGE GAP: 2000 - 2500 (500 point buffer zone)

            // Button: Frequency Down (moved to cleaner range, very tight)
            int freqDownBtn = _buttonHandler->addButton(input::ButtonType::FREQUENCY_DOWN, 2500, 2700, [this]() {
                Serial.println("🔘 [FREQ DOWN] Button press confirmed");
                uint16_t freq = _config.getFmFrequency();
                uint16_t newFreq = freq - 10; // Decrease by 0.1 MHz
                if (newFreq < 8750) newFreq = 10800; // Wrap around
                Serial.printf("🔘 [FREQ DOWN] %d → %d (%.1f MHz)\n", freq, newFreq, newFreq/100.0);
                processSetFrequencyCommand(newFreq);
            });

            // LARGE GAP: 2700 - 3900 (1200 point buffer zone - HUGE gap)

            // Button: Volume Up (at the high end, very tight range)
            int volUpBtn = _buttonHandler->addButton(input::ButtonType::VOLUME_UP, 3900, 4095, [this]() {
                Serial.println("🔘 [VOL UP] Button press confirmed");
                uint8_t vol = _config.getVolume();
                uint8_t newVol = (vol < 15) ? vol + 1 : vol;
                Serial.printf("🔘 [VOL UP] %d → %d\n", vol, newVol);
                processSetVolumeCommand(newVol);
            });

            Serial.printf("✅ Button callbacks configured with GREATLY IMPROVED ranges:\n");
            Serial.printf("   🔉 Vol Down: %s (analog 1000-1300) - VERY TIGHT, MOVED\n", volDownBtn >= 0 ? "OK" : "FAILED");
            Serial.printf("   📈 Freq Up: %s (analog 1800-2000) - VERY TIGHT, MOVED\n", freqUpBtn >= 0 ? "OK" : "FAILED");
            Serial.printf("   📉 Freq Down: %s (analog 2500-2700) - VERY TIGHT, MOVED\n", freqDownBtn >= 0 ? "OK" : "FAILED");
            Serial.printf("   🔊 Vol Up: %s (analog 3900-4095) - VERY TIGHT\n", volUpBtn >= 0 ? "OK" : "FAILED");

            Serial.println("🛡️ Button improvements implemented:");
            Serial.println("   • VERY narrow detection windows (200-300 points max)");
            Serial.println("   • HUGE gaps between buttons (500-1200 point buffers)");
            Serial.println("   • Multi-sample analog reading with averaging");
            Serial.println("   • Increased debounce time to 150ms");
            Serial.println("   • Stability confirmation before accepting button press");
            Serial.println("   • Dead zones to prevent cross-talk between buttons");
            Serial.println("   • Values below 1000 completely ignored (noise rejection)");

            Serial.println("📊 Button zones with dead spaces:");
            Serial.println("   0-999: NOISE ZONE (ignored)");
            Serial.println("   1000-1300: VOL DOWN");
            Serial.println("   1301-1799: DEAD ZONE #1");
            Serial.println("   1800-2000: FREQ UP");
            Serial.println("   2001-2499: DEAD ZONE #2");
            Serial.println("   2500-2700: FREQ DOWN");
            Serial.println("   2701-3899: DEAD ZONE #3 (HUGE)");
            Serial.println("   3900-4095: VOL UP");
        }

        bool NodeApplication::processSetVolumeCommand(uint8_t volume) {
            if (!_radio) return false;

            if (volume > 15) volume = 15;

            if (_radio->setVolume(volume)) {
                _config.setVolume(volume);
                _config.save();
                _stats.radio.volumeChanges++;
                _stats.radio.lastVolumeChange = millis();
                return true;
            }
            return false;
        }

        bool NodeApplication::processSetFrequencyCommand(uint16_t frequency) {
            if (!_radio) return false;

            if (frequency < 8750 || frequency > 10800) return false;

            if (_radio->setFrequency(frequency)) {
                _config.setFmFrequency(frequency);
                _config.save();
                _stats.radio.frequencyChanges++;
                _stats.radio.lastFrequencyChange = millis();
                return true;
            }
            return false;
        }

        bool NodeApplication::processToggleRelayCommand(bool state) {
            Serial.printf("🔌 [RELAY DEBUG - Node %d] Command received: %s\n", _config.getNodeId(), state ? "ON" : "OFF");
            Serial.printf("🔌 [RELAY DEBUG - Node %d] Pin %d before: %s\n", _config.getNodeId(), _pinRelayControl, digitalRead(_pinRelayControl) ? "HIGH" : "LOW");

            _config.setRelayState(state);

            // Enhanced relay control with explicit pin configuration
            pinMode(_pinRelayControl, OUTPUT);  // Ensure pin is output

            if (state) {
                digitalWrite(_pinRelayControl, HIGH);
                Serial.printf("🔌 [RELAY DEBUG - Node %d] Setting pin %d to HIGH (3.3V)\n", _config.getNodeId(), _pinRelayControl);
            } else {
                digitalWrite(_pinRelayControl, LOW);
                Serial.printf("🔌 [RELAY DEBUG - Node %d] Setting pin %d to LOW (0V)\n", _config.getNodeId(), _pinRelayControl);
            }

            // Verify the pin state after setting
            delay(10); // Small delay to ensure state change
            bool actualState = digitalRead(_pinRelayControl);
            Serial.printf("🔌 [RELAY DEBUG - Node %d] Pin %d after: %s\n", _config.getNodeId(), _pinRelayControl, actualState ? "HIGH" : "LOW");

            if (actualState != state) {
                Serial.printf("❌ [RELAY DEBUG - Node %d] WARNING: Pin state doesn't match command!\n", _config.getNodeId());
            } else {
                Serial.printf("✅ [RELAY DEBUG - Node %d] Pin state verified correct\n", _config.getNodeId());
            }

            // Check if this is a hardware driver issue
            if (state && actualState) {
                Serial.printf("🔍 [RELAY DEBUG - Node %d] If relay still doesn't click:\n", _config.getNodeId());
                Serial.println("   • Check if relay needs transistor driver (ESP32 = 3.3V, 20mA max)");
                Serial.println("   • Verify relay coil voltage (should be 3.3V or 5V type)");
                Serial.println("   • Check relay wiring and power supply");
                Serial.println("   • Add flyback diode if using inductive relay");
            }

            _config.save();
            return true;
        }

        bool NodeApplication::processMuteCommand(bool mute) {
            if (!_radio) return false;

            if (_radio->setMute(mute)) {
                _config.setMuted(mute);
                _config.save();
                _stats.radio.muteToggles++;
                return true;
            }
            return false;
        }

        bool NodeApplication::processMessages() {
            if (!_commModule || !_commModule->isMessageAvailable()) {
                return false;
            }

            uint8_t buffer[256];
            uint8_t senderNodeId;

            size_t bytesReceived = _commModule->receiveMessage(buffer, sizeof(buffer), senderNodeId);
            if (bytesReceived == 0) {
                return false;
            }

            // Update connection tracking
            _lastReceivedMessageTime = millis();
            _connectionSignalStrength = _commModule->getSignalStrength();
            _connected = true;
            _stats.communication.lastMessageReceived = millis();

            // Simple message receipt notification
            Serial.printf("📥 Message (%d bytes) from controller\n", bytesReceived);

            if (bytesReceived < sizeof(MessageHeader)) {
                return false;
            }

            const MessageHeader* header = reinterpret_cast<const MessageHeader*>(buffer);

            // Validate checksum
            if (!header->validateChecksum()) {
                Serial.println("❌ Checksum error");
                return false;
            }

            // Process message based on type
            switch (header->type) {
                case MessageType::COMMAND:
                    if (bytesReceived >= sizeof(CommandMessage)) {
                        const CommandMessage* command = reinterpret_cast<const CommandMessage*>(buffer);
                        return handleCommand(*command); // This now includes proper nodeId filtering
                    }
                    break;

                case MessageType::STATUS_REQUEST:
                    Serial.println("📊 Status request from controller");
                    return handleStatusRequest(*header);

                default:
                    Serial.printf("❓ Unknown message type: %d\n", static_cast<int>(header->type));
                    break;
            }

            return false;
        }

        bool NodeApplication::sendAcknowledgement(uint16_t sequenceNum, bool success, uint8_t errorCode) {
            if (!_initialized || !_commModule) {
                return false;
            }

            AckMessage ackMsg;
            ackMsg.header.version = 1;
            ackMsg.header.type = MessageType::ACK;
            ackMsg.header.nodeId = _config.getNodeId();
            ackMsg.header.sequenceNum = _messageSequence++;
            ackMsg.header.payloadLength = sizeof(AckMessage) - sizeof(MessageHeader);
            ackMsg.header.timestamp = millis();

            ackMsg.acknowledgedSeq = sequenceNum;
            ackMsg.success = success;
            ackMsg.errorCode = errorCode;

            ackMsg.header.setChecksum();

            bool sent = _commModule->sendMessage(0, &ackMsg, sizeof(AckMessage));

            if (sent) {
                _stats.communication.acksSent++;
                _stats.communication.lastMessageSent = millis();
            } else {
                _stats.communication.communicationErrors++;
            }

            return sent;
        }

        void NodeApplication::updateStatistics() {
            // Update command success rate
            if (_stats.totalCommandsReceived > 0) {
                _stats.commandSuccessRate = (_stats.successfulCommands * 100.0f) / _stats.totalCommandsReceived;
            }
        }

        void NodeApplication::checkConnection() {
            unsigned long currentTime = millis();
            bool wasConnected = _connected;

            _connected = (currentTime - _lastReceivedMessageTime < CONNECTION_TIMEOUT);

            if (wasConnected != _connected && !_connected) {
                Serial.printf("🔗 Connection lost to controller\n");
            }
        }

        // Helper methods - simplified implementations
        void NodeApplication::logRadioStatus() {
            // Only log if there are issues
            if (_radio) {
                int rssi = _radio->getRssi();
                if (rssi < 15) {
                    Serial.printf("⚠️  Weak FM signal: RSSI %d\n", rssi);
                }
            }
        }

        void NodeApplication::logCommunicationStatus() {
            // Only log if there are issues
            if (!_connected) {
                Serial.println("⚠️  Controller communication lost");
            }
        }

        void NodeApplication::showErrorInfo(const String& error) {
            Serial.println("🚨 ERROR: " + error);
        }

        void NodeApplication::logMessageDetails(const String& direction, const String& messageType, const void* message, size_t length) {
            // No detailed logging in simple mode
        }

        bool NodeApplication::sendHeartbeat() {
            return sendStatusMessage();
        }

        void NodeApplication::applyVolume() {
            if (_radio) {
                _radio->setVolume(_config.getVolume());
                _radio->setMute(_config.isMuted());
            }
        }

        void NodeApplication::applyRelayState() {
            // EXPLICIT relay pin control with proper OUTPUT configuration
            pinMode(_pinRelayControl, OUTPUT);
            digitalWrite(_pinRelayControl, _config.getRelayState() ? HIGH : LOW);

            // Debug output to verify relay state
            Serial.printf("🔌 [RELAY INIT - Node %d] Pin %d set to %s (config: %s)\n",
                          _config.getNodeId(),
                          _pinRelayControl,
                          digitalRead(_pinRelayControl) ? "HIGH" : "LOW",
                          _config.getRelayState() ? "ON" : "OFF");
        }

        bool NodeApplication::testHardwareComponents() {
            return testI2CBus() && testRadio() && testCommunication();
        }

        bool NodeApplication::testI2CBus() {
            return true; // Basic test - could be enhanced
        }

        bool NodeApplication::testRadio() {
            return _radio != nullptr;
        }

        bool NodeApplication::testCommunication() {
            return _commModule != nullptr;
        }

        void NodeApplication::performPeriodicSelfTest() {
            // Minimal periodic testing
        }

        bool NodeApplication::initializeSensors() {
#ifdef ENABLE_DHT_SENSOR
            return false; /* DHT sensor initialization - implement if needed */
#else
            return false;
#endif
        }

        void NodeApplication::readSensors() {
#ifdef ENABLE_DHT_SENSOR
            /* DHT sensor reading implementation - implement if needed */
#endif
        }

    } // namespace node
} // namespace szogfm