#include "NodeApplication.h"
#include <Arduino.h>
#include "../common/input/ButtonHandler.h"

#ifdef ENABLE_DHT_SENSOR
#include <DHT.h>
#endif

namespace szogfm {
    namespace node {

        // Using input namespace for easier access to ButtonType
        using namespace szogfm::input;

// Static instances for sensor (if enabled)
#ifdef ENABLE_DHT_SENSOR
        DHT* dht = nullptr;
#endif

        NodeApplication::NodeApplication()
                : _initialized(false), _connected(false),
                  _lastStatusTime(0), _lastDisplayUpdateTime(0),
                  _lastHeartbeatTime(0), _lastReceivedMessageTime(0), _lastSelfTestTime(0),
                  _connectionSignalStrength(0), _messageSequence(0),
                  _hasSensors(false), _temperature(0.0f), _humidity(0.0f),
                  _verboseDebugging(true),

                // Pin configuration with clear documentation
                  _pinUserButton(34),        // Analog input for resistor network buttons
                  _pinRelayControl(27),      // Digital output for relay control
                  _pinSDASensor(21),         // I2C SDA (FM radio and display)
                  _pinSCLSensor(22),         // I2C SCL (FM radio and display)
                  _pinM0(4),                 // EBYTE M0 control pin
                  _pinM1(32),                // EBYTE M1 control pin
                  _pinAUX(33) {              // EBYTE AUX status pin

            _commModule = nullptr;
            _radio = nullptr;
            _display = nullptr;
            _buttonHandler = nullptr;

            // Initialize statistics
            memset(&_stats, 0, sizeof(_stats));
            _stats.bootTime = millis();
        }

        bool NodeApplication::initialize() {
            Serial.println("\n" + String("=").repeat(60));
            Serial.println("🎵 SzögFM Node Application Starting 🎵");
            Serial.println(String("=").repeat(60));
            Serial.printf("⏰ Boot time: %lu ms\n", millis());
            Serial.printf("🔧 ESP32 Chip ID: %012llX\n", ESP.getEfuseMac());
            Serial.printf("💾 Free heap: %d bytes\n", ESP.getFreeHeap());
            Serial.printf("🐛 Verbose debugging: %s\n", _verboseDebugging ? "ENABLED" : "DISABLED");
            Serial.println();

            // Load and display configuration
            Serial.println("📋 Loading node configuration...");
            if (!_config.load()) {
                Serial.println("⚠️  Failed to load configuration from EEPROM, using defaults");
                _config.reset(true);
            }

            Serial.println("⚙️  Node Configuration:");
            Serial.printf("   🆔 Node ID: %d\n", _config.getNodeId());
            Serial.printf("   📻 FM Frequency: %.1f MHz\n", _config.getFmFrequency() / 100.0);
            Serial.printf("   🔊 Volume: %d/15\n", _config.getVolume());
            Serial.printf("   🔇 Muted: %s\n", _config.isMuted() ? "Yes" : "No");
            Serial.printf("   🔌 Relay: %s\n", _config.getRelayState() ? "ON" : "OFF");
            Serial.printf("   📡 Radio Address: 0x%04X\n", _config.getRadioAddress());
            Serial.printf("   📢 Radio Channel: 0x%02X\n", _config.getRadioChannel());
            Serial.println();

            // Display pin configuration
            Serial.println("📌 Pin Configuration:");
            Serial.printf("   🔘 User Button: %d (analog input)\n", _pinUserButton);
            Serial.printf("   🔌 Relay Control: %d (digital output)\n", _pinRelayControl);
            Serial.printf("   📊 I2C SDA: %d\n", _pinSDASensor);
            Serial.printf("   📊 I2C SCL: %d\n", _pinSCLSensor);
            Serial.printf("   📡 EBYTE M0: %d\n", _pinM0);
            Serial.printf("   📡 EBYTE M1: %d\n", _pinM1);
            Serial.printf("   📡 EBYTE AUX: %d\n", _pinAUX);
            Serial.println();

            // Initialize button handler first (no dependencies)
            Serial.println("🔘 Initializing button handler...");
            _buttonHandler = new input::ButtonHandler(_pinUserButton);
            _buttonHandler->initialize();
            setupButtonCallbacks();
            Serial.println("✅ Button handler initialized successfully");

            // Initialize and scan I2C bus
            Serial.println("🔍 Initializing I2C bus and scanning for devices...");
            Wire.begin(_pinSDASensor, _pinSCLSensor);
            Wire.setClock(100000); // 100kHz for better compatibility

            // Scan I2C bus for devices
            byte error, address;
            int deviceCount = 0;
            Serial.println("   I2C Device Scan Results:");

            for (address = 1; address < 127; address++) {
                Wire.beginTransmission(address);
                error = Wire.endTransmission();

                if (error == 0) {
                    Serial.printf("   ✅ Device found at address 0x%02X", address);

                    // Identify known devices
                    switch (address) {
                        case 0x11: Serial.print(" (RDA5807M FM Radio)"); break;
                        case 0x3C: case 0x3D: Serial.print(" (SSD1306 OLED Display)"); break;
                        case 0x48: case 0x49: case 0x4A: case 0x4B: Serial.print(" (Possible sensor)"); break;
                        default: Serial.print(" (Unknown device)"); break;
                    }
                    Serial.println();
                    deviceCount++;
                } else if (error == 4) {
                    Serial.printf("   ❌ Unknown error at address 0x%02X\n", address);
                }
            }

            if (deviceCount == 0) {
                Serial.println("   ❌ No I2C devices found - check wiring!");
                handleError("No I2C devices detected");
                return false;
            } else {
                Serial.printf("   📊 Total devices found: %d\n", deviceCount);
            }
            Serial.println();

            // Initialize FM radio module
            Serial.println("📻 Initializing RDA5807M FM radio module...");
            _radio = new radio::RDA5807Radio(_pinSDASensor, _pinSCLSensor);

            if (!_radio->initialize()) {
                String error = "Failed to initialize FM radio: " + _radio->getLastError();
                Serial.println("❌ " + error);
                handleError(error);
                return false;
            }
            Serial.println("✅ FM radio module initialized successfully");

            // Set initial radio parameters with detailed logging
            Serial.println("⚙️  Configuring FM radio with initial parameters...");

            Serial.printf("   🎵 Setting frequency to %.1f MHz\n", _config.getFmFrequency() / 100.0);
            if (!_radio->setFrequency(_config.getFmFrequency())) {
                Serial.println("   ⚠️  Warning: Failed to set initial frequency");
            }

            Serial.printf("   🔊 Setting volume to %d\n", _config.getVolume());
            if (!_radio->setVolume(_config.getVolume())) {
                Serial.println("   ⚠️  Warning: Failed to set initial volume");
            }

            Serial.printf("   🔇 Setting mute state to %s\n", _config.isMuted() ? "MUTED" : "UNMUTED");
            if (!_radio->setMute(_config.isMuted())) {
                Serial.println("   ⚠️  Warning: Failed to set initial mute state");
            }

            // Log initial radio status
            logRadioStatus();

            // Initialize SSD1306 OLED display
            Serial.println("🖥️  Initializing SSD1306 OLED display...");
            _display = new display::SSD1306Display(128, 64, _pinSDASensor, _pinSCLSensor);

            if (!_display->initialize()) {
                String error = "Failed to initialize OLED display: " + _display->getLastError();
                Serial.println("❌ " + error);
                handleError(error);
                return false;
            }
            Serial.println("✅ SSD1306 OLED display initialized successfully");

            // Show startup information on display
            showStartupInfo();

            // Initialize communication module
            Serial.println("📡 Initializing EBYTE E49 communication module...");
            Serial.printf("   📌 Pin configuration - M0:%d, M1:%d, AUX:%d\n", _pinM0, _pinM1, _pinAUX);

            _commModule = new communication::EbyteCommModule(&Serial2, _pinM0, _pinM1, _pinAUX);

            if (!_commModule->initialize()) {
                String error = "Failed to initialize communication module: " + _commModule->getLastError();
                Serial.println("❌ " + error);
                handleError(error);
                return false;
            }
            Serial.println("✅ Communication module initialized successfully");

            // Configure communication module with detailed logging
            Serial.println("⚙️  Configuring EBYTE communication module...");
            Serial.println("   📊 Configuration parameters:");
            Serial.printf("      • Channel: 0x%02X (%d)\n", _config.getRadioChannel(), _config.getRadioChannel());
            Serial.printf("      • Address: 0x%04X\n", _config.getRadioAddress());
            Serial.println("      • Air Data Rate: 2.4k baud");
            Serial.println("      • UART Baud: 9600");
            Serial.println("      • Power Level: 20dBm (max)");

            if (!_commModule->configure(_config.getRadioChannel(), _config.getRadioAddress(),
                                        communication::AIR_2K4, communication::UART_9600, 0)) {
                Serial.println("⚠️  Configuration issues detected:");
                Serial.println("   🔍 Last error: " + _commModule->getLastError());
                Serial.println("   ⏭️  Continuing despite configuration issues...");
            } else {
                Serial.println("✅ Communication module configured successfully");
            }

            // Set maximum debug level for troubleshooting
            _commModule->setDebugLevel(2);
            Serial.println("🐛 Communication debug level set to maximum (2)");

            // Perform communication diagnostics
            Serial.println("\n🔬 Performing communication module diagnostics...");
            _commModule->performDiagnostics();
            _commModule->printParameters();

            // Initialize relay control
            Serial.println("🔌 Initializing relay control...");
            pinMode(_pinRelayControl, OUTPUT);
            applyRelayState();
            Serial.printf("✅ Relay initialized and set to %s\n", _config.getRelayState() ? "ON" : "OFF");

            // Initialize sensors if available
            Serial.println("🌡️  Checking for environmental sensors...");
            _hasSensors = initializeSensors();
            if (_hasSensors) {
                Serial.println("✅ Environmental sensors detected and initialized");
                readSensors(); // Initial sensor reading
                Serial.printf("   📊 Temperature: %.1f°C, Humidity: %.1f%%\n", _temperature, _humidity);
            } else {
                Serial.println("ℹ️  No environmental sensors detected");
            }

            // Perform initial self-test
            Serial.println("\n🧪 Performing initial system self-test...");
            bool selfTestResult = performSelfTest();
            Serial.printf("%s Initial self-test %s\n",
                          selfTestResult ? "✅" : "⚠️ ",
                          selfTestResult ? "PASSED" : "completed with warnings");

            _initialized = true;
            _stats.bootTime = millis();

            // Initial display update
            updateDisplay();

            // Send initial status to controller
            Serial.println("📤 Sending initial status to controller...");
            if (sendStatusMessage()) {
                Serial.println("✅ Initial status sent successfully");
            } else {
                Serial.println("⚠️  Failed to send initial status (controller may not be ready)");
            }

            Serial.println("\n" + String("=").repeat(60));
            Serial.println("🎉 Node Application Initialized Successfully! 🎉");
            Serial.println("📊 System Status:");
            Serial.printf("   • Node ID: %d\n", _config.getNodeId());
            Serial.printf("   • Free heap: %d bytes\n", ESP.getFreeHeap());
            Serial.printf("   • I2C devices: %d\n", deviceCount);
            Serial.printf("   • Sensors: %s\n", _hasSensors ? "Available" : "None");
            Serial.printf("   • Self-test: %s\n", selfTestResult ? "PASSED" : "Warnings");
            Serial.println("🟢 Node is ready for operation!");
            Serial.println(String("=").repeat(60));

            return true;
        }

        void NodeApplication::update() {
            if (!_initialized) {
                return;
            }

            static unsigned long lastHeartbeat = 0;
            static unsigned long lastVerboseLog = 0;
            unsigned long currentTime = millis();

            // Heartbeat every 30 seconds
            if (currentTime - lastHeartbeat > 30000) {
                Serial.printf("\n💓 Node %d heartbeat - Uptime: %lu seconds, Free heap: %d bytes\n",
                              _config.getNodeId(), currentTime / 1000, ESP.getFreeHeap());
                Serial.printf("📊 Commands processed: %lu (success: %.1f%%), Connection: %s\n",
                              _stats.totalCommandsReceived, _stats.commandSuccessRate,
                              _connected ? "CONNECTED" : "DISCONNECTED");
                lastHeartbeat = currentTime;
            }

            // Verbose logging every 10 seconds
            if (_verboseDebugging && currentTime - lastVerboseLog > 10000) {
                logRadioStatus();
                logCommunicationStatus();
                lastVerboseLog = currentTime;
            }

            // Process any received messages with detailed logging
            bool messageProcessed = processMessages();
            if (messageProcessed && _verboseDebugging) {
                Serial.println("📨 Message processing cycle completed");
            }

            // Read sensors if available
            if (_hasSensors) {
                static unsigned long lastSensorRead = 0;
                if (currentTime - lastSensorRead > 5000) { // Every 5 seconds
                    readSensors();
                    lastSensorRead = currentTime;
                }
            }

            // Handle button presses with detailed logging
            bool buttonPressed = _buttonHandler->update();
            if (buttonPressed) {
                _stats.totalButtonPresses++;

                if (_verboseDebugging) {
                    Serial.printf("🔘 Button pressed - Analog value: %d, Type: %d\n",
                                  _buttonHandler->getLastAnalogValue(),
                                  static_cast<int>(_buttonHandler->getLastButtonType()));
                }

                // Update display immediately after button press
                updateDisplay();
            }

            // Check if we should send a status update
            if (currentTime - _lastStatusTime > STATUS_INTERVAL) {
                if (_verboseDebugging) {
                    Serial.printf("📤 Sending periodic status update (interval: %lu ms)\n", STATUS_INTERVAL);
                }

                if (sendStatusMessage()) {
                    if (_verboseDebugging) {
                        Serial.println("✅ Periodic status sent successfully");
                    }
                } else {
                    Serial.println("⚠️  Failed to send periodic status");
                }
                _lastStatusTime = currentTime;
            }

            // Check if we should send a heartbeat
            if (currentTime - _lastHeartbeatTime > HEARTBEAT_INTERVAL) {
                if (_verboseDebugging) {
                    Serial.printf("💗 Sending heartbeat (interval: %lu ms)\n", HEARTBEAT_INTERVAL);
                }
                sendHeartbeat();
                _lastHeartbeatTime = currentTime;
            }

            // Check connection status
            checkConnection();

            // Update display
            if (currentTime - _lastDisplayUpdateTime > DISPLAY_UPDATE_INTERVAL) {
                updateDisplay();
                _lastDisplayUpdateTime = currentTime;
                _stats.totalDisplayUpdates++;
            }

            // Perform periodic self-test
            if (currentTime - _lastSelfTestTime > SELF_TEST_INTERVAL) {
                performPeriodicSelfTest();
                _lastSelfTestTime = currentTime;
            }

            // Update radio module
            _radio->update();
            _stats.lastRadioUpdate = currentTime;

            // Update communication module
            _commModule->update();

            // Update statistics
            updateStatistics();
        }

        bool NodeApplication::handleCommand(const CommandMessage& message) {
            _stats.totalCommandsReceived++;
            _stats.communication.messagesReceived++;

            // Enhanced command logging
            Serial.printf("\n🎯 COMMAND RECEIVED - Processing Details:\n");
            Serial.printf("   📋 Command Type: %d", static_cast<int>(message.command));

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
            Serial.printf(" (%s)\n", commandName.c_str());

            Serial.printf("   🔢 Sequence Number: %d\n", message.header.sequenceNum);
            Serial.printf("   ⏰ Timestamp: %lu ms\n", message.header.timestamp);
            Serial.printf("   📏 Payload Length: %d bytes\n", message.header.payloadLength);
            Serial.printf("   🏷️  Source Node: %d\n", message.header.nodeId);

            if (_verboseDebugging) {
                logMessageDetails("RECEIVED", commandName, &message, sizeof(CommandMessage));
            }

            // Log current state before processing
            Serial.println("   📊 BEFORE Command Processing:");
            Serial.printf("      🎵 Current Frequency: %.1f MHz\n", _config.getFmFrequency() / 100.0);
            Serial.printf("      🔊 Current Volume: %d/15\n", _config.getVolume());
            Serial.printf("      🔇 Current Mute: %s\n", _config.isMuted() ? "ON" : "OFF");
            Serial.printf("      🔌 Current Relay: %s\n", _config.getRelayState() ? "ON" : "OFF");

            // Process the command based on type
            bool success = false;
            String processingLog = "";

            switch (message.command) {
                case Command::SET_VOLUME:
                {
                    uint8_t volume = message.data[0];
                    processingLog = "Volume change: " + String(_config.getVolume()) + " → " + String(volume);
                    Serial.printf("   🔊 %s\n", processingLog.c_str());

                    success = processSetVolumeCommand(volume);
                }
                    break;

                case Command::SET_FREQUENCY:
                {
                    uint16_t frequency = message.data[0] | (message.data[1] << 8);
                    processingLog = "Frequency change: " + String(_config.getFmFrequency() / 100.0, 1) +
                                    " MHz → " + String(frequency / 100.0, 1) + " MHz";
                    Serial.printf("   🎵 %s\n", processingLog.c_str());

                    success = processSetFrequencyCommand(frequency);
                }
                    break;

                case Command::TOGGLE_RELAY:
                {
                    bool state = message.data[0] != 0;
                    processingLog = "Relay change: " + String(_config.getRelayState() ? "ON" : "OFF") +
                                    " → " + String(state ? "ON" : "OFF");
                    Serial.printf("   🔌 %s\n", processingLog.c_str());

                    success = processToggleRelayCommand(state);
                }
                    break;

                case Command::MUTE:
                    processingLog = "Mute: " + String(_config.isMuted() ? "ALREADY MUTED" : "OFF → ON");
                    Serial.printf("   🔇 %s\n", processingLog.c_str());
                    success = processMuteCommand(true);
                    break;

                case Command::UNMUTE:
                    processingLog = "Unmute: " + String(_config.isMuted() ? "ON → OFF" : "ALREADY UNMUTED");
                    Serial.printf("   🔊 %s\n", processingLog.c_str());
                    success = processMuteCommand(false);
                    break;

                case Command::RESET:
                    processingLog = "Node reset requested";
                    Serial.printf("   🔄 %s\n", processingLog.c_str());
                    Serial.println("   ⚠️  RESETTING NODE IN 2 SECONDS...");

                    // Send acknowledgement before resetting
                    sendAcknowledgement(message.header.sequenceNum, true);
                    delay(2000);
                    ESP.restart();
                    success = true; // Never reached
                    break;

                case Command::GET_STATUS:
                    processingLog = "Status request";
                    Serial.printf("   📊 %s\n", processingLog.c_str());
                    success = sendStatusMessage();
                    break;

                default:
                    processingLog = "Unknown command: " + String(static_cast<int>(message.command));
                    Serial.printf("   ❓ %s\n", processingLog.c_str());
                    success = false;
                    break;
            }

            // Log final state after processing
            Serial.println("   📊 AFTER Command Processing:");
            Serial.printf("      🎵 New Frequency: %.1f MHz\n", _config.getFmFrequency() / 100.0);
            Serial.printf("      🔊 New Volume: %d/15\n", _config.getVolume());
            Serial.printf("      🔇 New Mute: %s\n", _config.isMuted() ? "ON" : "OFF");
            Serial.printf("      🔌 New Relay: %s\n", _config.getRelayState() ? "ON" : "OFF");

            // Log radio module status after command
            Serial.println("   📻 Radio Module Status Check:");
            if (_radio) {
                int rssi = _radio->getRssi();
                bool stereo = _radio->isStereo();
                bool tuned = _radio->isTunedToStation();
                Serial.printf("      📡 RSSI: %d, Stereo: %s, Tuned: %s\n",
                              rssi, stereo ? "Yes" : "No", tuned ? "Yes" : "No");

                // Update display with new FM signal strength
                if (_display) {
                    display::SSD1306Display* oledDisplay = static_cast<display::SSD1306Display*>(_display);
                    oledDisplay->setFmSignalStrength(rssi);
                }
            } else {
                Serial.println("      ❌ WARNING: Radio module pointer is null!");
            }

            // Update statistics
            if (success) {
                _stats.successfulCommands++;
            } else {
                _stats.failedCommands++;
            }
            _stats.commandSuccessRate = (_stats.successfulCommands * 100.0f) / _stats.totalCommandsReceived;

            // Send acknowledgement (unless it was a RESET command, which already sent ACK)
            if (message.command != Command::RESET) {
                Serial.printf("   📤 Sending ACK for sequence %d, success=%s\n",
                              message.header.sequenceNum, success ? "TRUE" : "FALSE");

                bool ackSent = sendAcknowledgement(message.header.sequenceNum, success);
                if (!ackSent) {
                    Serial.println("   ⚠️  Failed to send acknowledgement!");
                }
            }

            // Force display update after command processing
            updateDisplay();

            Serial.printf("🎯 COMMAND %s: %s\n",
                          success ? "COMPLETED" : "FAILED", processingLog.c_str());
            Serial.println(String("-").repeat(50));

            return success;
        }

        bool NodeApplication::handleStatusRequest(const MessageHeader& message) {
            _stats.totalStatusRequests++;

            if (_verboseDebugging) {
                Serial.printf("📊 Status request received from controller (seq %d)\n", message.sequenceNum);
            }

            return sendStatusMessage();
        }

        bool NodeApplication::sendStatusMessage() {
            if (!_initialized || !_commModule) {
                Serial.println("❌ Cannot send status: system not initialized");
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

                if (_verboseDebugging) {
                    Serial.printf("📻 Radio status for message - RSSI: %d, Stereo: %s\n",
                                  statusMsg.rssi, statusMsg.isStereo ? "Yes" : "No");
                }
            } else {
                statusMsg.rssi = 0;
                statusMsg.isStereo = false;
                Serial.println("⚠️  Radio module not available for status");
            }

            // Add sensor data if available
            if (_hasSensors) {
                statusMsg.temperature = _temperature;
                statusMsg.humidity = _humidity;

                if (_verboseDebugging) {
                    Serial.printf("🌡️  Sensor data for message - Temp: %.1f°C, Humidity: %.1f%%\n",
                                  statusMsg.temperature, statusMsg.humidity);
                }
            } else {
                statusMsg.temperature = -100.0f; // Invalid value to indicate no sensor
                statusMsg.humidity = -1.0f;      // Invalid value to indicate no sensor
            }

            // Compute header checksum
            statusMsg.header.setChecksum();

            if (_verboseDebugging) {
                Serial.printf("📤 Sending STATUS_RESPONSE:\n");
                Serial.printf("   🆔 Node ID: %d\n", statusMsg.header.nodeId);
                Serial.printf("   🔢 Sequence: %d\n", statusMsg.header.sequenceNum);
                Serial.printf("   🎵 Frequency: %.1f MHz\n", statusMsg.frequency / 100.0);
                Serial.printf("   🔊 Volume: %d/15\n", statusMsg.volume);
                Serial.printf("   🔌 Relay: %s\n", statusMsg.relayState ? "ON" : "OFF");
                Serial.printf("   📡 RSSI: %d\n", statusMsg.rssi);
                Serial.printf("   ⏰ Uptime: %.1f minutes\n", statusMsg.uptime / 60000.0);
            }

            // Send message to controller (node ID 0)
            bool success = _commModule->sendMessage(0, &statusMsg, sizeof(StatusMessage));

            if (success) {
                _stats.communication.messagesSent++;
                _stats.communication.lastMessageSent = millis();

                if (_verboseDebugging) {
                    Serial.println("✅ Status message sent successfully");
                }
            } else {
                _stats.communication.communicationErrors++;
                Serial.println("❌ Failed to send status message");
                Serial.println("   🔍 Last error: " + _commModule->getLastError());
            }

            return success;
        }

        void NodeApplication::handleError(const String& message) {
            Serial.println("\n🚨 ERROR OCCURRED:");
            Serial.println("   💬 Message: " + message);
            Serial.printf("   ⏰ Time: %lu ms\n", millis());
            Serial.printf("   💾 Free heap: %d bytes\n", ESP.getFreeHeap());

            if (_display) {
                showErrorInfo(message);
            }

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
            Serial.println("🧪 Performing comprehensive self-test...");
            bool allTestsPassed = true;

            // Test I2C bus
            Serial.print("   🔍 Testing I2C bus... ");
            if (testI2CBus()) {
                Serial.println("✅ PASS");
            } else {
                Serial.println("❌ FAIL");
                allTestsPassed = false;
            }

            // Test radio functionality
            Serial.print("   📻 Testing FM radio... ");
            if (testRadio()) {
                Serial.println("✅ PASS");
            } else {
                Serial.println("❌ FAIL");
                allTestsPassed = false;
            }

            // Test display functionality
            Serial.print("   🖥️  Testing OLED display... ");
            if (testDisplay()) {
                Serial.println("✅ PASS");
            } else {
                Serial.println("❌ FAIL");
                allTestsPassed = false;
            }

            // Test communication module
            Serial.print("   📡 Testing communication... ");
            if (testCommunication()) {
                Serial.println("✅ PASS");
            } else {
                Serial.println("❌ FAIL");
                allTestsPassed = false;
            }

            _stats.lastCommTest = millis();
            return allTestsPassed;
        }

        String NodeApplication::getDetailedSystemStatus() const {
            String status = "=== NODE " + String(_config.getNodeId()) + " DETAILED STATUS ===\n";
            status += "Boot Time: " + String(_stats.bootTime) + " ms\n";
            status += "Uptime: " + String(millis() / 1000) + " seconds\n";
            status += "Free Heap: " + String(ESP.getFreeHeap()) + " bytes\n";
            status += "Chip ID: " + String(ESP.getEfuseMac(), HEX) + "\n";
            status += "\nConfiguration:\n";
            status += "  Node ID: " + String(_config.getNodeId()) + "\n";
            status += "  FM Frequency: " + String(_config.getFmFrequency() / 100.0, 1) + " MHz\n";
            status += "  Volume: " + String(_config.getVolume()) + "/15\n";
            status += "  Muted: " + String(_config.isMuted() ? "Yes" : "No") + "\n";
            status += "  Relay: " + String(_config.getRelayState() ? "ON" : "OFF") + "\n";
            status += "\nStatistics:\n";
            status += "  Commands Received: " + String(_stats.totalCommandsReceived) + "\n";
            status += "  Successful Commands: " + String(_stats.successfulCommands) + "\n";
            status += "  Failed Commands: " + String(_stats.failedCommands) + "\n";
            status += "  Success Rate: " + String(_stats.commandSuccessRate, 1) + "%\n";
            status += "  Button Presses: " + String(_stats.totalButtonPresses) + "\n";
            status += "  Display Updates: " + String(_stats.totalDisplayUpdates) + "\n";
            status += "\nCommunication:\n";
            status += "  Messages Received: " + String(_stats.communication.messagesReceived) + "\n";
            status += "  Messages Sent: " + String(_stats.communication.messagesSent) + "\n";
            status += "  Communication Errors: " + String(_stats.communication.communicationErrors) + "\n";
            status += "  Connected: " + String(_connected ? "Yes" : "No") + "\n";

            if (_radio) {
                status += "\nRadio Status:\n";
                status += "  RSSI: " + String(_radio->getRssi()) + "\n";
                status += "  Stereo: " + String(_radio->isStereo() ? "Yes" : "No") + "\n";
                status += "  Tuned: " + String(_radio->isTunedToStation() ? "Yes" : "No") + "\n";
            }

            if (_hasSensors) {
                status += "\nSensors:\n";
                status += "  Temperature: " + String(_temperature, 1) + "°C\n";
                status += "  Humidity: " + String(_humidity, 1) + "%\n";
            }

            return status;
        }

        // Implementation of all the helper methods continues...
        // Due to length constraints, I'll continue with the key methods

        void NodeApplication::setupButtonCallbacks() {
            Serial.println("🔘 Setting up button callbacks with analog ranges...");

            // Add buttons with their analog value ranges
            // These ranges should be calibrated based on your resistor network

            // Button: Frequency Up (1600-2200)
            int freqUpBtn = _buttonHandler->addButton(input::ButtonType::FREQUENCY_UP, 1600, 2200, [this]() {
                Serial.println("🔘 [FREQ UP] Button pressed");
                uint16_t freq = _config.getFmFrequency();
                uint16_t newFreq = freq + 10; // Increase by 0.1 MHz
                if (newFreq > 10800) newFreq = 8750; // Wrap around
                Serial.printf("🔘 [FREQ UP] %d → %d (%.1f MHz)\n", freq, newFreq, newFreq/100.0);
                processSetFrequencyCommand(newFreq);
            });

            // Button: Frequency Down (2300-2900)
            int freqDownBtn = _buttonHandler->addButton(input::ButtonType::FREQUENCY_DOWN, 2300, 2900, [this]() {
                Serial.println("🔘 [FREQ DOWN] Button pressed");
                uint16_t freq = _config.getFmFrequency();
                uint16_t newFreq = freq - 10; // Decrease by 0.1 MHz
                if (newFreq < 8750) newFreq = 10800; // Wrap around
                Serial.printf("🔘 [FREQ DOWN] %d → %d (%.1f MHz)\n", freq, newFreq, newFreq/100.0);
                processSetFrequencyCommand(newFreq);
            });

            // Button: Volume Up (3700-4095)
            int volUpBtn = _buttonHandler->addButton(input::ButtonType::VOLUME_UP, 3700, 4095, [this]() {
                Serial.println("🔘 [VOL UP] Button pressed");
                uint8_t vol = _config.getVolume();
                uint8_t newVol = (vol < 15) ? vol + 1 : vol;
                Serial.printf("🔘 [VOL UP] %d → %d\n", vol, newVol);
                processSetVolumeCommand(newVol);
            });

            // Button: Volume Down (1000-1400)
            int volDownBtn = _buttonHandler->addButton(input::ButtonType::VOLUME_DOWN, 1000, 1400, [this]() {
                Serial.println("🔘 [VOL DOWN] Button pressed");
                uint8_t vol = _config.getVolume();
                uint8_t newVol = (vol > 0) ? vol - 1 : vol;
                Serial.printf("🔘 [VOL DOWN] %d → %d\n", vol, newVol);
                processSetVolumeCommand(newVol);
            });

            Serial.printf("✅ Button callbacks configured:\n");
            Serial.printf("   📈 Freq Up: %s (analog 1600-2200)\n", freqUpBtn >= 0 ? "OK" : "FAILED");
            Serial.printf("   📉 Freq Down: %s (analog 2300-2900)\n", freqDownBtn >= 0 ? "OK" : "FAILED");
            Serial.printf("   🔊 Vol Up: %s (analog 3700-4095)\n", volUpBtn >= 0 ? "OK" : "FAILED");
            Serial.printf("   🔉 Vol Down: %s (analog 1000-1400)\n", volDownBtn >= 0 ? "OK" : "FAILED");
        }

        bool NodeApplication::processSetVolumeCommand(uint8_t volume) {
            Serial.printf("🔊 [VOLUME] Processing volume command: %d → %d\n", _config.getVolume(), volume);

            if (!_radio) {
                Serial.println("❌ [VOLUME] Radio module not available");
                return false;
            }

            // Validate volume range
            if (volume > 15) {
                Serial.printf("⚠️  [VOLUME] Clamping volume %d to maximum (15)\n", volume);
                volume = 15;
            }

            Serial.printf("🔊 [VOLUME] Setting radio volume to %d...\n", volume);

            // Apply to radio module
            if (_radio->setVolume(volume)) {
                Serial.println("✅ [VOLUME] Radio volume set successfully");

                // Update configuration
                _config.setVolume(volume);
                if (_config.save()) {
                    Serial.println("✅ [VOLUME] Configuration saved to EEPROM");
                } else {
                    Serial.println("⚠️  [VOLUME] Failed to save configuration");
                }

                _stats.radio.volumeChanges++;
                _stats.radio.lastVolumeChange = millis();

                // Update display immediately
                if (_display) {
                    _display->displayVolume(volume, _config.isMuted());
                    _display->update();
                }

                return true;
            } else {
                Serial.println("❌ [VOLUME] Failed to set radio volume");
                Serial.println("   🔍 Radio error: " + _radio->getLastError());
                return false;
            }
        }

        bool NodeApplication::processSetFrequencyCommand(uint16_t frequency) {
            Serial.printf("🎵 [FREQUENCY] Processing frequency command: %.1f → %.1f MHz\n",
                          _config.getFmFrequency() / 100.0, frequency / 100.0);

            if (!_radio) {
                Serial.println("❌ [FREQUENCY] Radio module not available");
                return false;
            }

            // Validate frequency range (87.5 MHz to 108.0 MHz)
            if (frequency < 8750 || frequency > 10800) {
                Serial.printf("❌ [FREQUENCY] Invalid frequency: %d (valid range: 8750-10800)\n", frequency);
                return false;
            }

            Serial.printf("🎵 [FREQUENCY] Setting radio frequency to %.1f MHz...\n", frequency / 100.0);

            // Apply to radio module
            if (_radio->setFrequency(frequency)) {
                Serial.println("✅ [FREQUENCY] Radio frequency set successfully");

                // Update configuration
                _config.setFmFrequency(frequency);
                if (_config.save()) {
                    Serial.println("✅ [FREQUENCY] Configuration saved to EEPROM");
                } else {
                    Serial.println("⚠️  [FREQUENCY] Failed to save configuration");
                }

                _stats.radio.frequencyChanges++;
                _stats.radio.lastFrequencyChange = millis();

                // Update display immediately
                if (_display) {
                    _display->displayFrequency(frequency);
                    _display->update();
                }

                // Log new radio status after frequency change
                delay(100); // Give radio time to retune
                logRadioStatus();

                return true;
            } else {
                Serial.println("❌ [FREQUENCY] Failed to set radio frequency");
                Serial.println("   🔍 Radio error: " + _radio->getLastError());
                return false;
            }
        }

        bool NodeApplication::processToggleRelayCommand(bool state) {
            Serial.printf("🔌 [RELAY] Processing relay command: %s → %s\n",
                          _config.getRelayState() ? "ON" : "OFF", state ? "ON" : "OFF");

            // Set relay state
            _config.setRelayState(state);

            // Apply physical relay control
            digitalWrite(_pinRelayControl, state ? HIGH : LOW);
            Serial.printf("🔌 [RELAY] Physical relay pin %d set to %s\n",
                          _pinRelayControl, state ? "HIGH" : "LOW");

            // Save configuration
            if (_config.save()) {
                Serial.println("✅ [RELAY] Configuration saved to EEPROM");
            } else {
                Serial.println("⚠️  [RELAY] Failed to save configuration");
            }

            // Update display immediately
            if (_display) {
                _display->displayNodeInfo(_config.getNodeId(), state);
                _display->update();
            }

            Serial.printf("✅ [RELAY] Relay set to %s\n", state ? "ON" : "OFF");
            return true;
        }

        bool NodeApplication::processMuteCommand(bool mute) {
            Serial.printf("🔇 [MUTE] Processing mute command: %s → %s\n",
                          _config.isMuted() ? "MUTED" : "UNMUTED", mute ? "MUTED" : "UNMUTED");

            if (!_radio) {
                Serial.println("❌ [MUTE] Radio module not available");
                return false;
            }

            Serial.printf("🔇 [MUTE] Setting radio mute to %s...\n", mute ? "ON" : "OFF");

            // Apply to radio module
            if (_radio->setMute(mute)) {
                Serial.println("✅ [MUTE] Radio mute set successfully");

                // Update configuration
                _config.setMuted(mute);
                if (_config.save()) {
                    Serial.println("✅ [MUTE] Configuration saved to EEPROM");
                } else {
                    Serial.println("⚠️  [MUTE] Failed to save configuration");
                }

                _stats.radio.muteToggles++;

                // Update display immediately
                if (_display) {
                    _display->displayVolume(_config.getVolume(), mute);
                    _display->update();
                }

                return true;
            } else {
                Serial.println("❌ [MUTE] Failed to set radio mute");
                Serial.println("   🔍 Radio error: " + _radio->getLastError());
                return false;
            }
        }

        // Additional helper methods would continue here...
        // Due to space constraints, I'll include the most critical ones

        void NodeApplication::logRadioStatus() {
            if (!_radio) return;

            int rssi = _radio->getRssi();
            bool stereo = _radio->isStereo();
            bool tuned = _radio->isTunedToStation();
            uint16_t currentFreq = _radio->getFrequency();
            uint8_t currentVol = _radio->getVolume();
            bool currentMute = _radio->isMuted();

            _stats.radio.lastRssi = rssi;
            _stats.radio.lastStereoState = stereo;

            if (_verboseDebugging) {
                Serial.printf("📻 Radio Status: RSSI=%d, Stereo=%s, Tuned=%s, Freq=%.1fMHz, Vol=%d, Mute=%s\n",
                              rssi, stereo ? "Y" : "N", tuned ? "Y" : "N",
                              currentFreq / 100.0, currentVol, currentMute ? "Y" : "N");
            }
        }

        void NodeApplication::logCommunicationStatus() {
            if (!_commModule) return;

            bool isTransmitting = _commModule->isTransmitting();
            int signalStrength = _commModule->getSignalStrength();
            _stats.communication.lastSignalStrength = signalStrength;

            if (_verboseDebugging) {
                Serial.printf("📡 Comm Status: TX=%s, RSSI=%d, Connected=%s, LastMsg=%luμs ago\n",
                              isTransmitting ? "Y" : "N", signalStrength, _connected ? "Y" : "N",
                              millis() - _stats.communication.lastMessageReceived);
            }
        }

        bool NodeApplication::processMessages() {
            if (!_commModule || !_commModule->isMessageAvailable()) {
                return false;
            }

            uint8_t buffer[256];
            uint8_t senderNodeId;

            size_t bytesReceived = _commModule->receiveMessage(buffer, sizeof(buffer), senderNodeId);
            if (bytesReceived == 0) {
                if (_verboseDebugging) {
                    Serial.println("⚠️  No message received despite isMessageAvailable() == true");
                }
                return false;
            }

            // Update connection tracking
            _lastReceivedMessageTime = millis();
            _connectionSignalStrength = _commModule->getSignalStrength();
            _connected = true;
            _stats.communication.lastMessageReceived = millis();

            Serial.printf("📨 Message received: %d bytes from controller\n", bytesReceived);

            if (bytesReceived < sizeof(MessageHeader)) {
                Serial.printf("❌ Message too short: %d < %d bytes\n", bytesReceived, sizeof(MessageHeader));
                return false;
            }

            const MessageHeader* header = reinterpret_cast<const MessageHeader*>(buffer);

            // Validate checksum
            if (!header->validateChecksum()) {
                Serial.printf("❌ Invalid checksum: expected 0x%02X, got 0x%02X\n",
                              header->computeChecksum(), header->checksum);
                return false;
            }

            // Process message based on type
            switch (header->type) {
                case MessageType::COMMAND:
                    if (bytesReceived >= sizeof(CommandMessage)) {
                        const CommandMessage* command = reinterpret_cast<const CommandMessage*>(buffer);
                        return handleCommand(*command);
                    } else {
                        Serial.printf("❌ Command message too short: %d < %d bytes\n",
                                      bytesReceived, sizeof(CommandMessage));
                    }
                    break;

                case MessageType::STATUS_REQUEST:
                    return handleStatusRequest(*header);

                default:
                    Serial.printf("❓ Unknown message type: %d\n", static_cast<int>(header->type));
                    break;
            }

            return false;
        }

        bool NodeApplication::sendAcknowledgement(uint16_t sequenceNum, bool success, uint8_t errorCode) {
            if (!_initialized || !_commModule) {
                Serial.println("❌ Cannot send ACK: system not initialized");
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

            if (_verboseDebugging) {
                Serial.printf("📤 Sending ACK: seq=%d, ack_seq=%d, success=%s, error=%d\n",
                              ackMsg.header.sequenceNum, sequenceNum,
                              success ? "true" : "false", errorCode);
            }

            bool sent = _commModule->sendMessage(0, &ackMsg, sizeof(AckMessage));

            if (sent) {
                _stats.communication.acksSent++;
                _stats.communication.lastMessageSent = millis();
            } else {
                _stats.communication.communicationErrors++;
                Serial.println("❌ Failed to send ACK: " + _commModule->getLastError());
            }

            return sent;
        }

        void NodeApplication::updateDisplay() {
            if (!_display) return;

            // Update all display elements
            _display->displayFrequency(_config.getFmFrequency());
            _display->displayVolume(_config.getVolume(), _config.isMuted());
            _display->displayConnectionStatus(_connected, _connectionSignalStrength);
            _display->displayNodeInfo(_config.getNodeId(), _config.getRelayState());

            if (_hasSensors) {
                _display->displaySensorData(_temperature, _humidity);
            }

            _display->update();
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

            if (wasConnected != _connected) {
                Serial.printf("🔗 Connection status changed: %s\n", _connected ? "CONNECTED" : "DISCONNECTED");
                if (!_connected) {
                    Serial.printf("   ⏰ Last message received %lu ms ago\n",
                                  currentTime - _lastReceivedMessageTime);
                }
                updateDisplay(); // Update display when connection changes
            }
        }

        // Stub implementations for space - these would be fully implemented
        bool NodeApplication::testHardwareComponents() { return testI2CBus() && testRadio() && testDisplay() && testCommunication(); }
        bool NodeApplication::testI2CBus() { return true; /* I2C scan implementation */ }
        bool NodeApplication::testRadio() { return _radio != nullptr; /* Radio test implementation */ }
        bool NodeApplication::testDisplay() { return _display != nullptr; /* Display test implementation */ }
        bool NodeApplication::testCommunication() { return _commModule != nullptr; /* Comm test implementation */ }
        void NodeApplication::performPeriodicSelfTest() { /* Periodic test implementation */ }
        void NodeApplication::showStartupInfo() { /* Startup display implementation */ }
        void NodeApplication::showErrorInfo(const String& error) { if (_display) _display->displayError(error); }
        void NodeApplication::logMessageDetails(const String& direction, const String& messageType, const void* message, size_t length) { /* Message logging implementation */ }
        bool NodeApplication::sendHeartbeat() { return sendStatusMessage(); }
        void NodeApplication::applyVolume() { if (_radio) { _radio->setVolume(_config.getVolume()); _radio->setMute(_config.isMuted()); } }
        void NodeApplication::applyRelayState() { digitalWrite(_pinRelayControl, _config.getRelayState() ? HIGH : LOW); }
        bool NodeApplication::initializeSensors() {
#ifdef ENABLE_DHT_SENSOR
            return false; /* DHT sensor initialization */
#else
            return false;
#endif
        }
        void NodeApplication::readSensors() {
#ifdef ENABLE_DHT_SENSOR
            /* DHT sensor reading implementation */
#endif
        }

    } // namespace node
} // namespace szogfm