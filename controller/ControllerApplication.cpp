#include "ControllerApplication.h"
#include <ArduinoJson.h>

namespace szogfm {
    namespace controller {

        // Helper function to repeat a character for consistent formatting
        String repeatChar(char c, int count) {
            String result = "";
            result.reserve(count);
            for (int i = 0; i < count; i++) {
                result += c;
            }
            return result;
        }

        ControllerApplication::ControllerApplication()
                : _initialized(false), _messageSequence(0),
                  _lastStatusRequestTime(0), _lastWebUpdateTime(0), _lastDiscoveryTime(0),

                // Default pin assignments for EBYTE module
                  _pinM0(4),              // M0 pin for EBYTE module
                  _pinM1(32),             // M1 pin for EBYTE module
                  _pinAUX(33),            // AUX pin for EBYTE module

                // WiFi configuration (defaults to AP mode)
                  _wifiSsid("SzogFM_Controller"),
                  _wifiPassword("GombaSzog24"),
                  _wifiApMode(true) {

            _commModule = nullptr;
            _webServer = nullptr;

            // Initialize default configuration
            _config.wifiSsid = "SzogFM_Controller";
            _config.wifiPassword = "GombaSzog24";
            _config.accessPointMode = true;
            _config.radioChannel = 0x1A;
            _config.radioAddress = 0x1234;
            _config.transmissionPower = 0; // Maximum power
            _config.statusRequestInterval = 10000; // 10 seconds
            _config.discoveryInterval = 60000; // 60 seconds
            _config.messageTimeout = 3000; // 3 seconds
            _config.maxRetryCount = 5;
            _config.verboseDebugging = true;
            _config.communicationDebugLevel = 2;

            // Initialize communication statistics
            memset(&_commStats, 0, sizeof(_commStats));
            _commStats.lastResetTime = millis();
            _commStats.minimumResponseTime = ULONG_MAX;
        }

        ControllerApplication::~ControllerApplication() {
            if (_commModule) {
                delete _commModule;
            }
            if (_webServer) {
                delete _webServer;
            }
        }

        bool ControllerApplication::initialize() {
            Serial.println("\n" + repeatChar('=', 60));
            Serial.println("🎵 SzögFM Controller Application Starting 🎵");
            Serial.println(repeatChar('=', 60));
            Serial.printf("⏰ Startup time: %lu ms\n", millis());
            Serial.printf("🔧 ESP32 Chip ID: %012llX\n", ESP.getEfuseMac());
            Serial.printf("💾 Free heap: %d bytes\n", ESP.getFreeHeap());
            Serial.printf("📡 WiFi mode: %s\n", _config.accessPointMode ? "Access Point" : "Station");
            Serial.println();

            // Initialize communication module with enhanced debugging
            Serial.println("📻 Initializing EBYTE Communication Module...");
            Serial.printf("   📌 Pin configuration - M0:%d, M1:%d, AUX:%d\n", _pinM0, _pinM1, _pinAUX);

            _commModule = new communication::EbyteCommModule(&Serial2, _pinM0, _pinM1, _pinAUX);

            if (!_commModule->initialize()) {
                Serial.println("❌ FAILED to initialize communication module");
                Serial.println("   🔍 Last error: " + _commModule->getLastError());
                Serial.println("   🚨 This is a critical failure - controller cannot function");
                return false;
            }
            Serial.println("✅ Communication module initialized successfully");

            // Configure communication module with detailed logging
            Serial.println("⚙️  Configuring communication module...");
            Serial.println("   📊 Configuration parameters:");
            Serial.printf("      • Channel: 0x%02X (%d)\n", _config.radioChannel, _config.radioChannel);
            Serial.printf("      • Address: 0x%04X\n", _config.radioAddress);
            Serial.println("      • Air Data Rate: 2.4k baud");
            Serial.println("      • UART Baud: 9600");
            Serial.println("      • Power Level: 20dBm (max)");

            if (!_commModule->configure(_config.radioChannel, _config.radioAddress,
                                        communication::AIR_2K4, communication::UART_9600, _config.transmissionPower)) {
                Serial.println("⚠️  Configuration issues detected:");
                Serial.println("   🔍 Last error: " + _commModule->getLastError());
                Serial.println("   ⏭️  Continuing despite configuration issues...");
            } else {
                Serial.println("✅ Communication module configured successfully");
            }

            // Set debug level for detailed troubleshooting
            _commModule->setDebugLevel(_config.communicationDebugLevel);
            Serial.printf("🐛 Debug level set to %d for detailed logging\n", _config.communicationDebugLevel);

            // Perform comprehensive diagnostics
            Serial.println("\n🔬 Performing module diagnostics...");
            _commModule->performDiagnostics();
            _commModule->printParameters();

            // Initialize WiFi and web server with detailed logging
            Serial.println("\n🌐 Initializing WiFi and Web Server...");
            if (!initializeWebServer()) {
                Serial.println("❌ FAILED to initialize web server");
                return false;
            }
            Serial.println("✅ Web server initialized successfully");

            _initialized = true;

            // Initial node discovery
            Serial.println("\n🔍 Starting initial node discovery...");
            int discoveredNodes = discoverNodes();
            Serial.printf("📋 Initial discovery found %d nodes\n", discoveredNodes);

            Serial.println("\n" + repeatChar('=', 60));
            Serial.println("🎉 Controller Application Initialized Successfully! 🎉");
            Serial.println("📊 System Status:");
            Serial.printf("   • Free heap: %d bytes\n", ESP.getFreeHeap());
            Serial.printf("   • WiFi IP: %s\n", _config.accessPointMode ? WiFi.softAPIP().toString().c_str() : WiFi.localIP().toString().c_str());
            Serial.printf("   • Web interface: http://%s\n", _config.accessPointMode ? WiFi.softAPIP().toString().c_str() : WiFi.localIP().toString().c_str());
            Serial.println(repeatChar('=', 60));

            return true;
        }

        void ControllerApplication::update() {
            if (!_initialized) {
                return;
            }

            static unsigned long lastHeartbeat = 0;
            unsigned long currentTime = millis();

            // Heartbeat every 30 seconds
            if (currentTime - lastHeartbeat > 30000) {
                Serial.printf("\n💓 System heartbeat - Uptime: %lu seconds, Free heap: %d bytes\n",
                              currentTime / 1000, ESP.getFreeHeap());
                Serial.printf("📊 Communication stats - Sent: %lu, Received: %lu, Success rate: %.1f%%\n",
                              _commStats.totalMessagesSent, _commStats.totalMessagesReceived,
                              _commStats.messageSuccessRate);
                lastHeartbeat = currentTime;
            }

            // Process any received messages with detailed logging
            bool messageProcessed = processMessages();
            if (messageProcessed && _config.verboseDebugging) {
                Serial.println("📨 Message processing cycle completed");
            }

            // Process any pending messages that need to be retried
            processPendingMessages();

            // Update node connection status
            updateNodeConnectionStatus();

            // Check if we should request status updates from all nodes
            if (currentTime - _lastStatusRequestTime > _config.statusRequestInterval) {
                if (_config.verboseDebugging) {
                    Serial.printf("🔄 Requesting status from all nodes (interval: %lu ms)\n", _config.statusRequestInterval);
                }
                requestAllNodeStatus();
                _lastStatusRequestTime = currentTime;
            }

            // Periodic node discovery
            if (currentTime - _lastDiscoveryTime > _config.discoveryInterval) {
                Serial.println("🔍 Performing periodic node discovery...");
                int discoveredNodes = discoverNodes();
                Serial.printf("📋 Periodic discovery found %d nodes\n", discoveredNodes);
                _lastDiscoveryTime = currentTime;
            }

            // Handle web server requests
            _webServer->handleClient();

            // Update communication module
            _commModule->update();

            // Show pending messages count periodically in verbose mode
            if (_config.verboseDebugging && (currentTime % 10000 == 0)) {
                if (!_pendingMessages.empty()) {
                    Serial.printf("⏳ Pending messages: %d\n", _pendingMessages.size());
                }
            }
        }

        bool ControllerApplication::setNodeVolume(uint8_t nodeId, uint8_t volume) {
            // Validate volume range
            if (volume > 15) {
                Serial.printf("⚠️  Volume value %d exceeds maximum (15), clamping to 15\n", volume);
                volume = 15;
            }

            String description = "Set volume to " + String(volume) + " for node " + String(nodeId);
            Serial.printf("🔊 %s\n", description.c_str());

            // Send command to set volume
            bool success = sendCommandMessage(nodeId, Command::SET_VOLUME, &volume, 1, description);

            if (success) {
                Serial.printf("✅ Volume command queued successfully for node %d\n", nodeId);

                // Update local cache optimistically
                if (_nodeStatus.find(nodeId) != _nodeStatus.end()) {
                    _nodeStatus[nodeId].volume = volume;
                    _nodeStatus[nodeId].lastCommandTime = millis();
                    _nodeStatus[nodeId].lastCommand = "SET_VOLUME(" + String(volume) + ")";
                }
            } else {
                Serial.printf("❌ Failed to queue volume command for node %d\n", nodeId);
            }

            return success;
        }

        bool ControllerApplication::setNodeFrequency(uint8_t nodeId, uint16_t frequency) {
            // Validate frequency range
            if (frequency < 8750 || frequency > 10800) {
                Serial.printf("❌ Invalid frequency value: %d (must be between 8750-10800)\n", frequency);
                return false;
            }

            String description = "Set frequency to " + String(frequency/100.0, 1) + " MHz for node " + String(nodeId);
            Serial.printf("📻 %s\n", description.c_str());

            // Prepare data (little-endian: low byte first, high byte second)
            uint8_t data[2];
            data[0] = frequency & 0xFF;
            data[1] = (frequency >> 8) & 0xFF;

            if (_config.verboseDebugging) {
                Serial.printf("📦 Frequency data bytes: 0x%02X 0x%02X (little-endian)\n", data[0], data[1]);
            }

            // Send command to set frequency
            bool success = sendCommandMessage(nodeId, Command::SET_FREQUENCY, data, 2, description);

            if (success) {
                Serial.printf("✅ Frequency command queued successfully for node %d\n", nodeId);

                // Update local cache optimistically
                if (_nodeStatus.find(nodeId) != _nodeStatus.end()) {
                    _nodeStatus[nodeId].frequency = frequency;
                    _nodeStatus[nodeId].lastCommandTime = millis();
                    _nodeStatus[nodeId].lastCommand = "SET_FREQUENCY(" + String(frequency/100.0, 1) + "MHz)";
                }
            } else {
                Serial.printf("❌ Failed to queue frequency command for node %d\n", nodeId);
            }

            return success;
        }

        bool ControllerApplication::setNodeRelayState(uint8_t nodeId, bool state) {
            // Prepare data
            uint8_t data = state ? 1 : 0;

            String description = "Set relay " + String(state ? "ON" : "OFF") + " for node " + String(nodeId);
            Serial.printf("🔌 %s\n", description.c_str());

            // Send command to set relay state
            bool success = sendCommandMessage(nodeId, Command::TOGGLE_RELAY, &data, 1, description);

            if (success) {
                Serial.printf("✅ Relay command queued successfully for node %d\n", nodeId);

                // Update local cache optimistically
                if (_nodeStatus.find(nodeId) != _nodeStatus.end()) {
                    _nodeStatus[nodeId].relayState = state;
                    _nodeStatus[nodeId].lastCommandTime = millis();
                    _nodeStatus[nodeId].lastCommand = "TOGGLE_RELAY(" + String(state ? "ON" : "OFF") + ")";
                }
            } else {
                Serial.printf("❌ Failed to queue relay command for node %d\n", nodeId);
            }

            return success;
        }

        bool ControllerApplication::setNodeMute(uint8_t nodeId, bool mute) {
            String description = "Set mute " + String(mute ? "ON" : "OFF") + " for node " + String(nodeId);
            Serial.printf("🔇 %s\n", description.c_str());

            // Send command to set mute state
            Command cmd = mute ? Command::MUTE : Command::UNMUTE;
            bool success = sendCommandMessage(nodeId, cmd, nullptr, 0, description);

            if (success) {
                Serial.printf("✅ Mute command queued successfully for node %d\n", nodeId);

                // Update local cache optimistically
                if (_nodeStatus.find(nodeId) != _nodeStatus.end()) {
                    _nodeStatus[nodeId].muted = mute;
                    _nodeStatus[nodeId].lastCommandTime = millis();
                    _nodeStatus[nodeId].lastCommand = mute ? "MUTE" : "UNMUTE";
                }
            } else {
                Serial.printf("❌ Failed to queue mute command for node %d\n", nodeId);
            }

            return success;
        }

        // Broadcast methods (send to all nodes with nodeId = 0)
        bool ControllerApplication::setAllNodesVolume(uint8_t volume) {
            Serial.printf("🔊 Setting volume for ALL nodes to %d (broadcast)\n", volume);
            return setNodeVolume(0, volume);
        }

        bool ControllerApplication::setAllNodesFrequency(uint16_t frequency) {
            Serial.printf("📻 Setting frequency for ALL nodes to %.1f MHz (broadcast)\n", frequency/100.0);
            return setNodeFrequency(0, frequency);
        }

        bool ControllerApplication::setAllNodesRelayState(bool state) {
            Serial.printf("🔌 Setting relay for ALL nodes to %s (broadcast)\n", state ? "ON" : "OFF");
            return setNodeRelayState(0, state);
        }

        bool ControllerApplication::setAllNodesMute(bool mute) {
            Serial.printf("🔇 Setting mute for ALL nodes to %s (broadcast)\n", mute ? "ON" : "OFF");
            return setNodeMute(0, mute);
        }

        const NodeStatus* ControllerApplication::getNodeStatus(uint8_t nodeId) const {
            auto it = _nodeStatus.find(nodeId);
            if (it != _nodeStatus.end()) {
                return &(it->second);
            }
            return nullptr;
        }

        std::vector<NodeStatus> ControllerApplication::getAllNodeStatus() const {
            std::vector<NodeStatus> result;
            result.reserve(_nodeStatus.size());

            for (const auto& pair : _nodeStatus) {
                result.push_back(pair.second);
            }

            return result;
        }

        bool ControllerApplication::requestNodeStatus(uint8_t nodeId) {
            String description = "Request status from node " + String(nodeId);
            if (_config.verboseDebugging) {
                Serial.printf("📊 %s\n", description.c_str());
            }

            bool success = sendCommandMessage(nodeId, Command::GET_STATUS, nullptr, 0, description);

            if (success && _config.verboseDebugging) {
                Serial.printf("✅ Status request queued for node %d\n", nodeId);
            } else if (!success) {
                Serial.printf("❌ Failed to queue status request for node %d\n", nodeId);
            }

            return success;
        }

        bool ControllerApplication::requestAllNodeStatus() {
            if (_config.verboseDebugging) {
                Serial.println("📊 Requesting status from ALL nodes (broadcast)");
            }
            return requestNodeStatus(0);
        }

        bool ControllerApplication::resetNode(uint8_t nodeId) {
            String description = "RESET node " + String(nodeId);
            Serial.printf("🔄 %s\n", description.c_str());

            bool success = sendCommandMessage(nodeId, Command::RESET, nullptr, 0, description);

            if (success) {
                Serial.printf("✅ Reset command queued for node %d\n", nodeId);

                // Remove node from status cache as it will restart
                if (_nodeStatus.find(nodeId) != _nodeStatus.end()) {
                    _nodeStatus.erase(nodeId);
                    Serial.printf("🗑️  Removed node %d from cache (reset pending)\n", nodeId);
                }
            } else {
                Serial.printf("❌ Failed to queue reset command for node %d\n", nodeId);
            }

            return success;
        }

        int ControllerApplication::discoverNodes() {
            Serial.println("🔍 Starting node discovery process...");

            // Request status from all possible node IDs
            int discoveredCount = 0;
            for (uint8_t nodeId = 1; nodeId <= 20; nodeId++) {
                if (requestNodeStatus(nodeId)) {
                    discoveredCount++;
                }
                delay(50); // Small delay between requests
            }

            Serial.printf("📡 Discovery requests sent to %d potential nodes\n", discoveredCount);
            return discoveredCount;
        }

        bool ControllerApplication::testNodeCommunication(uint8_t nodeId) {
            Serial.printf("🧪 Testing communication with node %d...\n", nodeId);

            bool success = requestNodeStatus(nodeId);

            if (success) {
                Serial.printf("✅ Communication test initiated for node %d (response pending)\n", nodeId);
            } else {
                Serial.printf("❌ Communication test failed for node %d\n", nodeId);
            }

            return success;
        }

        void ControllerApplication::resetCommunicationStats() {
            Serial.println("🔄 Resetting communication statistics...");
            memset(&_commStats, 0, sizeof(_commStats));
            _commStats.lastResetTime = millis();
            _commStats.minimumResponseTime = ULONG_MAX;
            Serial.println("✅ Communication statistics reset");
        }

        String ControllerApplication::getDetailedSystemStatus() const {
            DynamicJsonDocument doc(4096);

            // System information
            doc["system"]["uptime"] = millis();
            doc["system"]["free_heap"] = ESP.getFreeHeap();
            doc["system"]["chip_id"] = String(ESP.getEfuseMac(), HEX);
            doc["system"]["verbose_debugging"] = _config.verboseDebugging;

            // Communication statistics
            JsonObject commStats = doc.createNestedObject("communication_stats");
            commStats["total_sent"] = _commStats.totalMessagesSent;
            commStats["total_received"] = _commStats.totalMessagesReceived;
            commStats["total_retries"] = _commStats.totalRetries;
            commStats["total_timeouts"] = _commStats.totalTimeouts;
            commStats["total_errors"] = _commStats.totalErrors;
            commStats["success_rate"] = _commStats.messageSuccessRate;
            commStats["avg_response_time"] = _commStats.averageResponseTime;
            commStats["last_reset"] = _commStats.lastResetTime;

            // Pending messages
            doc["pending_messages"]["count"] = _pendingMessages.size();
            JsonArray pending = doc.createNestedArray("pending_messages_list");
            for (const auto& msg : _pendingMessages) {
                JsonObject pendingMsg = pending.createNestedObject();
                pendingMsg["sequence"] = msg.sequenceNum;
                pendingMsg["node_id"] = msg.nodeId;
                pendingMsg["retry_count"] = msg.retryCount;
                pendingMsg["age_ms"] = millis() - msg.firstSentTime;
                pendingMsg["description"] = msg.commandDescription;
            }

            // Node information
            JsonArray nodes = doc.createNestedArray("nodes");
            for (const auto& pair : _nodeStatus) {
                JsonObject node = nodes.createNestedObject();
                const NodeStatus& status = pair.second;

                node["id"] = status.nodeId;
                node["connected"] = status.isConnected;
                node["last_seen"] = millis() - status.lastSeenTime;
                node["total_commands"] = status.totalCommands;
                node["successful_commands"] = status.successfulCommands;
                node["failed_commands"] = status.failedCommands;
                node["last_command"] = status.lastCommand;
                node["last_command_success"] = status.lastCommandSuccess;

                if (status.totalCommands > 0) {
                    node["command_success_rate"] = (float)status.successfulCommands / status.totalCommands * 100.0f;
                    node["avg_response_time"] = (float)status.totalResponseTime / status.totalCommands;
                }
            }

            String result;
            serializeJsonPretty(doc, result);
            return result;
        }

        bool ControllerApplication::setConfiguration(const ControllerConfig& config) {
            _config = config;

            // Apply communication debug level if changed
            if (_commModule) {
                _commModule->setDebugLevel(_config.communicationDebugLevel);
            }

            Serial.println("✅ Controller configuration updated");
            return true;
        }

        void ControllerApplication::setCommunicationDebugLevel(uint8_t level) {
            _config.communicationDebugLevel = level;
            if (_commModule) {
                _commModule->setDebugLevel(level);
            }
        }

        // Private helper methods implementation continues...

        bool ControllerApplication::initializeWebServer() {
            Serial.println("🌐 Setting up WiFi and Web Server...");

            // Initialize WiFi
            if (_config.accessPointMode) {
                Serial.printf("📡 Creating WiFi Access Point: %s\n", _config.wifiSsid.c_str());
                WiFi.softAP(_config.wifiSsid.c_str(), _config.wifiPassword.c_str());
                Serial.printf("✅ AP IP address: %s\n", WiFi.softAPIP().toString().c_str());
            } else {
                Serial.printf("📡 Connecting to WiFi network: %s\n", _config.wifiSsid.c_str());
                WiFi.begin(_config.wifiSsid.c_str(), _config.wifiPassword.c_str());

                unsigned long startTime = millis();
                while (WiFi.status() != WL_CONNECTED && millis() - startTime < 30000) {
                    delay(500);
                    Serial.print(".");
                }

                if (WiFi.status() != WL_CONNECTED) {
                    Serial.println("\n❌ Failed to connect to WiFi network");
                    return false;
                }

                Serial.printf("\n✅ Connected to %s\n", _config.wifiSsid.c_str());
                Serial.printf("📍 IP address: %s\n", WiFi.localIP().toString().c_str());
            }

            // Set up mDNS responder
            if (MDNS.begin("szogfm")) {
                Serial.println("🔍 mDNS responder started (szogfm.local)");
            }

            // Initialize web server
            _webServer = new WebServer(80);

            // Set up enhanced routes
            _webServer->on("/", HTTP_GET, [this]() { handleRoot(); });
            _webServer->on("/status", HTTP_GET, [this]() { handleStatus(); });
            _webServer->on("/detailed_status", HTTP_GET, [this]() { handleDetailedStatus(); });
            _webServer->on("/comm_stats", HTTP_GET, [this]() { handleCommStats(); });
            _webServer->on("/diagnostics", HTTP_GET, [this]() { handleSystemDiagnostics(); });
            _webServer->on("/node_diagnostics", HTTP_GET, [this]() { handleNodeDiagnostics(); });

            // Command endpoints
            _webServer->on("/set_volume", HTTP_POST, [this]() { handleSetVolume(); });
            _webServer->on("/set_frequency", HTTP_POST, [this]() { handleSetFrequency(); });
            _webServer->on("/set_relay", HTTP_POST, [this]() { handleSetRelay(); });
            _webServer->on("/set_mute", HTTP_POST, [this]() { handleSetMute(); });
            _webServer->on("/reset_node", HTTP_POST, [this]() { handleResetNode(); });

            // Utility endpoints
            _webServer->on("/discover_nodes", HTTP_POST, [this]() { handleDiscoverNodes(); });
            _webServer->on("/test_node", HTTP_POST, [this]() { handleTestNode(); });
            _webServer->on("/reset_stats", HTTP_POST, [this]() { handleResetStats(); });
            _webServer->on("/comm_test", HTTP_POST, [this]() { handleCommTest(); });

            // Start server
            _webServer->begin();
            Serial.println("🌍 HTTP server started on port 80");

            return true;
        }

        bool ControllerApplication::processMessages() {
            if (!_commModule || !_commModule->isMessageAvailable()) {
                return false;
            }

            uint8_t buffer[256];
            uint8_t senderNodeId;

            size_t bytesReceived = _commModule->receiveMessage(buffer, sizeof(buffer), senderNodeId);
            if (bytesReceived == 0) {
                if (_config.verboseDebugging) {
                    Serial.println("⚠️  No message received despite isMessageAvailable() == true");
                }
                return false;
            }

            Serial.printf("📨 Received %d bytes from node %d\n", bytesReceived, senderNodeId);

            return handleNodeMessage(buffer, bytesReceived, senderNodeId);
        }

        void ControllerApplication::processPendingMessages() {
            unsigned long currentTime = millis();
            bool hasTimeouts = false;

            for (auto it = _pendingMessages.begin(); it != _pendingMessages.end(); ) {
                if (currentTime - it->sentTime > _config.messageTimeout) {
                    hasTimeouts = true;

                    if (it->retryCount < _config.maxRetryCount) {
                        // Retry sending the message
                        Serial.printf("🔄 Retrying message to node %d (seq %d, attempt %d/%d) - %s\n",
                                      it->nodeId, it->sequenceNum, it->retryCount+1, _config.maxRetryCount,
                                      it->commandDescription.c_str());

                        if (_commModule->sendMessage(it->nodeId, it->messageData.data(), it->messageData.size())) {
                            it->sentTime = currentTime;
                            it->retryCount++;
                            _commStats.totalRetries++;
                            ++it;
                        } else {
                            Serial.printf("❌ Failed to retry message to node %d (seq %d)\n",
                                          it->nodeId, it->sequenceNum);
                            _commStats.totalErrors++;

                            // Update node stats
                            if (_nodeStatus.find(it->nodeId) != _nodeStatus.end()) {
                                _nodeStatus[it->nodeId].failedCommands++;
                                _nodeStatus[it->nodeId].lastCommandSuccess = false;
                            }

                            it = _pendingMessages.erase(it);
                        }
                    } else {
                        // Max retry count exceeded
                        Serial.printf("💀 Message to node %d FAILED after %d attempts (seq %d) - %s\n",
                                      it->nodeId, _config.maxRetryCount, it->sequenceNum,
                                      it->commandDescription.c_str());

                        _commStats.totalTimeouts++;

                        // Update node stats
                        if (_nodeStatus.find(it->nodeId) != _nodeStatus.end()) {
                            _nodeStatus[it->nodeId].failedCommands++;
                            _nodeStatus[it->nodeId].lastCommandSuccess = false;
                        }

                        it = _pendingMessages.erase(it);
                    }
                } else {
                    ++it;
                }
            }

            // Log pending messages status periodically
            static unsigned long lastPendingLog = 0;
            if (!_pendingMessages.empty() && currentTime - lastPendingLog > 5000) {
                Serial.printf("⏳ %d messages pending (oldest: %lu ms ago)\n",
                              _pendingMessages.size(),
                              _pendingMessages.empty() ? 0 : currentTime - _pendingMessages[0].firstSentTime);
                lastPendingLog = currentTime;
            }
        }

        bool ControllerApplication::sendCommandMessage(uint8_t nodeId, Command command,
                                                       const uint8_t* data, size_t dataLength,
                                                       const String& description) {
            if (!_initialized || !_commModule) {
                Serial.println("❌ Cannot send command: system not initialized");
                return false;
            }

            // Prepare command message
            CommandMessage cmdMsg;
            cmdMsg.header.version = 1;
            cmdMsg.header.type = MessageType::COMMAND;
            cmdMsg.header.nodeId = nodeId;
            cmdMsg.header.sequenceNum = _messageSequence++;
            cmdMsg.header.payloadLength = sizeof(Command) + dataLength;
            cmdMsg.header.timestamp = millis();
            cmdMsg.command = command;

            // Copy data if provided
            if (data && dataLength > 0) {
                size_t copyLength = std::min(dataLength, sizeof(cmdMsg.data));
                memcpy(cmdMsg.data, data, copyLength);

                if (_config.verboseDebugging && dataLength <= 4) {
                    Serial.printf("📦 Command data: ");
                    for (size_t i = 0; i < copyLength; i++) {
                        Serial.printf("0x%02X ", cmdMsg.data[i]);
                    }
                    Serial.println();
                }
            }

            // Compute header checksum
            cmdMsg.header.setChecksum();

            String cmdName = "";
            switch (command) {
                case Command::SET_VOLUME: cmdName = "SET_VOLUME"; break;
                case Command::SET_FREQUENCY: cmdName = "SET_FREQUENCY"; break;
                case Command::TOGGLE_RELAY: cmdName = "TOGGLE_RELAY"; break;
                case Command::MUTE: cmdName = "MUTE"; break;
                case Command::UNMUTE: cmdName = "UNMUTE"; break;
                case Command::RESET: cmdName = "RESET"; break;
                case Command::GET_STATUS: cmdName = "GET_STATUS"; break;
                default: cmdName = "UNKNOWN"; break;
            }

            Serial.printf("📤 Sending %s command to node %d (seq %d)\n",
                          cmdName.c_str(), nodeId, cmdMsg.header.sequenceNum);

            if (_config.verboseDebugging) {
                logMessageDetails("SENDING", nodeId, cmdName, &cmdMsg,
                                  sizeof(MessageHeader) + sizeof(Command) + dataLength);
            }

            // Send message
            size_t totalSize = sizeof(MessageHeader) + sizeof(Command) + dataLength;
            if (_commModule->sendMessage(nodeId, &cmdMsg, totalSize)) {
                _commStats.totalMessagesSent++;

                // Add to pending messages list for retry handling
                PendingMessage pending;
                pending.sequenceNum = cmdMsg.header.sequenceNum;
                pending.nodeId = nodeId;
                pending.sentTime = millis();
                pending.firstSentTime = pending.sentTime;
                pending.retryCount = 0;
                pending.commandDescription = description.length() > 0 ? description :
                                             (cmdName + " to node " + String(nodeId));

                // Copy message data
                pending.messageData.resize(totalSize);
                memcpy(pending.messageData.data(), &cmdMsg, totalSize);

                _pendingMessages.push_back(pending);

                // Update node stats
                if (_nodeStatus.find(nodeId) != _nodeStatus.end()) {
                    _nodeStatus[nodeId].totalCommands++;
                    _nodeStatus[nodeId].lastCommandTime = millis();
                    _nodeStatus[nodeId].lastCommand = pending.commandDescription;
                }

                Serial.printf("✅ Command queued successfully (%d pending total)\n",
                              _pendingMessages.size());
                return true;
            }

            Serial.printf("❌ Failed to send %s command to node %d\n", cmdName.c_str(), nodeId);
            _commStats.totalErrors++;
            return false;
        }

        // Additional methods continue with basic web interface...

        void ControllerApplication::handleRoot() {
            Serial.println("🌐 Web request: GET / (Main Interface)");

            String html = "<!DOCTYPE html><html><head><title>SzogFM Controller</title></head><body>";
            html += "<h1>SzogFM Controller</h1>";
            html += "<p>Controller is running. Use the API endpoints for control.</p>";
            html += "<h2>API Endpoints:</h2>";
            html += "<ul>";
            html += "<li>GET /status - Node status</li>";
            html += "<li>GET /detailed_status - Detailed system status</li>";
            html += "<li>POST /set_volume - Set node volume</li>";
            html += "<li>POST /set_frequency - Set node frequency</li>";
            html += "<li>POST /discover_nodes - Discover nodes</li>";
            html += "</ul>";
            html += "</body></html>";

            _webServer->send(200, "text/html", html);
        }

        void ControllerApplication::handleStatus() {
            if (_config.verboseDebugging) {
                Serial.println("🌐 Web request: GET /status");
            }

            DynamicJsonDocument doc(4096);
            JsonArray nodesArray = doc.createNestedArray("nodes");

            int nodeCount = 0;
            for (const auto& pair : _nodeStatus) {
                const NodeStatus& status = pair.second;
                nodeCount++;

                JsonObject nodeObj = nodesArray.createNestedObject();
                nodeObj["id"] = status.nodeId;
                nodeObj["connected"] = status.isConnected;
                nodeObj["frequency"] = status.frequency;
                nodeObj["volume"] = status.volume;
                nodeObj["muted"] = status.muted;
                nodeObj["relay"] = status.relayState;
                nodeObj["rssi"] = status.rssi;
                nodeObj["stereo"] = status.isStereo;
                nodeObj["uptime"] = status.uptime;
                nodeObj["last_seen"] = millis() - status.lastSeenTime;

                if (status.hasSensors) {
                    nodeObj["has_sensors"] = true;
                    nodeObj["temperature"] = status.temperature;
                    nodeObj["humidity"] = status.humidity;
                } else {
                    nodeObj["has_sensors"] = false;
                }

                if (!status.errorMessage.isEmpty()) {
                    nodeObj["error"] = status.errorMessage;
                }
            }

            String jsonResponse;
            serializeJson(doc, jsonResponse);

            _webServer->send(200, "application/json", jsonResponse);

            if (_config.verboseDebugging) {
                Serial.printf("✅ Status sent for %d nodes\n", nodeCount);
            }
        }

        // Stub implementations for remaining web handlers
        void ControllerApplication::handleDetailedStatus() {
            String response = getDetailedSystemStatus();
            _webServer->send(200, "application/json", response);
        }

        void ControllerApplication::handleCommStats() {
            DynamicJsonDocument doc(1024);
            doc["total_sent"] = _commStats.totalMessagesSent;
            doc["total_received"] = _commStats.totalMessagesReceived;
            doc["success_rate"] = _commStats.messageSuccessRate;
            doc["pending_messages"] = _pendingMessages.size();

            String response;
            serializeJson(doc, response);
            _webServer->send(200, "application/json", response);
        }

        void ControllerApplication::handleSystemDiagnostics() {
            _webServer->send(200, "text/plain", "System diagnostics not yet implemented");
        }

        void ControllerApplication::handleNodeDiagnostics() {
            _webServer->send(200, "text/plain", "Node diagnostics not yet implemented");
        }

        void ControllerApplication::handleSetVolume() {
            if (!_webServer->hasArg("node_id") || !_webServer->hasArg("volume")) {
                _webServer->send(400, "text/plain", "Missing parameters");
                return;
            }
            uint8_t nodeId = _webServer->arg("node_id").toInt();
            uint8_t volume = _webServer->arg("volume").toInt();
            bool success = setNodeVolume(nodeId, volume);
            _webServer->send(success ? 200 : 500, "text/plain", success ? "OK" : "Failed");
        }

        void ControllerApplication::handleSetFrequency() {
            if (!_webServer->hasArg("node_id") || !_webServer->hasArg("frequency")) {
                _webServer->send(400, "text/plain", "Missing parameters");
                return;
            }
            uint8_t nodeId = _webServer->arg("node_id").toInt();
            uint16_t frequency = _webServer->arg("frequency").toInt();
            bool success = setNodeFrequency(nodeId, frequency);
            _webServer->send(success ? 200 : 500, "text/plain", success ? "OK" : "Failed");
        }

        void ControllerApplication::handleSetRelay() {
            if (!_webServer->hasArg("node_id") || !_webServer->hasArg("state")) {
                _webServer->send(400, "text/plain", "Missing parameters");
                return;
            }
            uint8_t nodeId = _webServer->arg("node_id").toInt();
            bool state = _webServer->arg("state").toInt() != 0;
            bool success = setNodeRelayState(nodeId, state);
            _webServer->send(success ? 200 : 500, "text/plain", success ? "OK" : "Failed");
        }

        void ControllerApplication::handleSetMute() {
            if (!_webServer->hasArg("node_id") || !_webServer->hasArg("mute")) {
                _webServer->send(400, "text/plain", "Missing parameters");
                return;
            }
            uint8_t nodeId = _webServer->arg("node_id").toInt();
            bool mute = _webServer->arg("mute").toInt() != 0;
            bool success = setNodeMute(nodeId, mute);
            _webServer->send(success ? 200 : 500, "text/plain", success ? "OK" : "Failed");
        }

        void ControllerApplication::handleResetNode() {
            if (!_webServer->hasArg("node_id")) {
                _webServer->send(400, "text/plain", "Missing parameters");
                return;
            }
            uint8_t nodeId = _webServer->arg("node_id").toInt();
            bool success = resetNode(nodeId);
            _webServer->send(success ? 200 : 500, "text/plain", success ? "OK" : "Failed");
        }

        void ControllerApplication::handleDiscoverNodes() {
            int discovered = discoverNodes();
            _webServer->send(200, "text/plain", "Discovery initiated for " + String(discovered) + " nodes");
        }

        void ControllerApplication::handleTestNode() {
            if (!_webServer->hasArg("node_id")) {
                _webServer->send(400, "text/plain", "Missing parameters");
                return;
            }
            uint8_t nodeId = _webServer->arg("node_id").toInt();
            bool success = testNodeCommunication(nodeId);
            _webServer->send(200, "text/plain", success ? "Test initiated" : "Test failed");
        }

        void ControllerApplication::handleResetStats() {
            resetCommunicationStats();
            _webServer->send(200, "text/plain", "Statistics reset");
        }

        void ControllerApplication::handleCommTest() {
            _commModule->performDiagnostics();
            _webServer->send(200, "text/plain", "Communication test performed");
        }

        // Additional helper methods (simplified for compilation)
        bool ControllerApplication::handleNodeMessage(const void* message, size_t length, uint8_t senderNodeId) {
            // Basic message handling - full implementation would handle different message types
            Serial.printf("📨 Handling message from node %d (%d bytes)\n", senderNodeId, length);
            return true;
        }

        void ControllerApplication::updateNodeConnectionStatus() {
            // Update connection status based on timeouts
            unsigned long currentTime = millis();
            for (auto& pair : _nodeStatus) {
                NodeStatus& status = pair.second;
                if (status.isConnected && currentTime - status.lastSeenTime > 120000) { // 2 minutes
                    status.isConnected = false;
                }
            }
        }

        void ControllerApplication::logMessageDetails(const String& direction, uint8_t nodeId,
                                                      const String& messageType, const void* message, size_t length) {
            if (!_config.verboseDebugging) return;
            Serial.printf("📋 %s %s to/from node %d (%d bytes)\n",
                          direction.c_str(), messageType.c_str(), nodeId, length);
        }

        void ControllerApplication::updateCommStats(bool success, unsigned long responseTime) {
            if (success) {
                _commStats.totalMessagesReceived++;
                if (responseTime > 0) {
                    _commStats.totalResponseTime += responseTime;
                    _commStats.averageResponseTime = (float)_commStats.totalResponseTime / _commStats.totalMessagesReceived;

                    if (responseTime < _commStats.minimumResponseTime) {
                        _commStats.minimumResponseTime = responseTime;
                    }
                    if (responseTime > _commStats.peakResponseTime) {
                        _commStats.peakResponseTime = responseTime;
                    }
                }
            } else {
                _commStats.totalErrors++;
            }

            // Calculate success rate
            unsigned long totalAttempts = _commStats.totalMessagesSent;
            if (totalAttempts > 0) {
                _commStats.messageSuccessRate = (float)_commStats.totalMessagesReceived / totalAttempts * 100.0f;
            }
        }

    } // namespace controller
} // namespace szogfm