#include "ControllerApplication.h"
#include <ArduinoJson.h>

namespace szogfm {
    namespace controller {

        ControllerApplication::ControllerApplication()
                : _initialized(false), _messageSequence(0),
                  _lastStatusRequestTime(0), _lastWebUpdateTime(0), _lastDiscoveryTime(0),
                  _verboseDebugging(true),

                // Default pin assignments
                  _pinM0(4),              // M0 pin for EBYTE module
                  _pinM1(32),             // M1 pin for EBYTE module
                  _pinAUX(33),            // AUX pin for EBYTE module

                // WiFi configuration (defaults to AP mode)
                  _wifiSsid("SzogFM_Controller"),
                  _wifiPassword("GombaSzog24"),
                  _wifiApMode(true) {

            _commModule = nullptr;
            _webServer = nullptr;

            // Initialize communication statistics
            memset(&_commStats, 0, sizeof(_commStats));
            _commStats.lastResetTime = millis();
        }

        bool ControllerApplication::initialize() {
            Serial.println("\n" + String("=").repeat(60));
            Serial.println("🎵 SzögFM Controller Application Starting 🎵");
            Serial.println(String("=").repeat(60));
            Serial.printf("⏰ Startup time: %lu ms\n", millis());
            Serial.printf("🔧 ESP32 Chip ID: %012llX\n", ESP.getEfuseMac());
            Serial.printf("💾 Free heap: %d bytes\n", ESP.getFreeHeap());
            Serial.printf("📡 WiFi mode: %s\n", _wifiApMode ? "Access Point" : "Station");
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
            Serial.println("      • Channel: 0x1A (26)");
            Serial.println("      • Address: 0x1234");
            Serial.println("      • Air Data Rate: 2.4k baud");
            Serial.println("      • UART Baud: 9600");
            Serial.println("      • Power Level: 20dBm (max)");

            if (!_commModule->configure(0x1A, 0x1234,
                                        communication::AIR_2K4, communication::UART_9600, 0)) {
                Serial.println("⚠️  Configuration issues detected:");
                Serial.println("   🔍 Last error: " + _commModule->getLastError());
                Serial.println("   ⏭️  Continuing despite configuration issues...");
            } else {
                Serial.println("✅ Communication module configured successfully");
            }

            // Set maximum debug level for detailed troubleshooting
            _commModule->setDebugLevel(2);
            Serial.println("🐛 Debug level set to maximum (2) for detailed logging");

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

            Serial.println("\n" + String("=").repeat(60));
            Serial.println("🎉 Controller Application Initialized Successfully! 🎉");
            Serial.println("📊 System Status:");
            Serial.printf("   • Free heap: %d bytes\n", ESP.getFreeHeap());
            Serial.printf("   • WiFi IP: %s\n", _wifiApMode ? WiFi.softAPIP().toString().c_str() : WiFi.localIP().toString().c_str());
            Serial.printf("   • Web interface: http://%s\n", _wifiApMode ? WiFi.softAPIP().toString().c_str() : WiFi.localIP().toString().c_str());
            Serial.println(String("=").repeat(60));

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
            if (messageProcessed && _verboseDebugging) {
                Serial.println("📨 Message processing cycle completed");
            }

            // Process any pending messages that need to be retried
            processPendingMessages();

            // Update node connection status
            updateNodeConnectionStatus();

            // Check if we should request status updates from all nodes
            if (currentTime - _lastStatusRequestTime > STATUS_REQUEST_INTERVAL) {
                if (_verboseDebugging) {
                    Serial.printf("🔄 Requesting status from all nodes (interval: %lu ms)\n", STATUS_REQUEST_INTERVAL);
                }
                requestAllNodeStatus();
                _lastStatusRequestTime = currentTime;
            }

            // Periodic node discovery
            if (currentTime - _lastDiscoveryTime > DISCOVERY_INTERVAL) {
                Serial.println("🔍 Performing periodic node discovery...");
                int discoveredNodes = discoverNodes();
                Serial.printf("📋 Periodic discovery found %d nodes\n", discoveredNodes);
                _lastDiscoveryTime = currentTime;
            }

            // Handle web server requests
            _webServer->handleClient();

            // Update communication module
            _commModule->update();

            // Small delay to prevent CPU hogging
            if (_verboseDebugging && (currentTime % 10000 == 0)) {
                // Every 10 seconds in verbose mode, show pending messages count
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

            if (_verboseDebugging) {
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
            if (_verboseDebugging) {
                Serial.printf("📊 %s\n", description.c_str());
            }

            bool success = sendCommandMessage(nodeId, Command::GET_STATUS, nullptr, 0, description);

            if (success && _verboseDebugging) {
                Serial.printf("✅ Status request queued for node %d\n", nodeId);
            } else if (!success) {
                Serial.printf("❌ Failed to queue status request for node %d\n", nodeId);
            }

            return success;
        }

        bool ControllerApplication::requestAllNodeStatus() {
            if (_verboseDebugging) {
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

            unsigned long startTime = millis();
            bool success = requestNodeStatus(nodeId);

            if (success) {
                Serial.printf("✅ Communication test initiated for node %d (response pending)\n", nodeId);
            } else {
                Serial.printf("❌ Communication test failed for node %d\n", nodeId);
            }

            return success;
        }

        bool ControllerApplication::handleNodeMessage(const void* message, size_t length, uint8_t senderNodeId) {
            if (!message || length < sizeof(MessageHeader)) {
                Serial.printf("❌ Invalid message: %s (length: %d)\n",
                              !message ? "null pointer" : "too short", length);
                return false;
            }

            // Parse message header
            const MessageHeader* header = static_cast<const MessageHeader*>(message);

            if (_verboseDebugging) {
                logMessageDetails("RECEIVED", senderNodeId, "Unknown", message, length);
            }

            // Validate checksum
            if (!header->validateChecksum()) {
                Serial.printf("❌ Message from node %d has INVALID CHECKSUM\n", senderNodeId);
                Serial.printf("   📦 Expected: 0x%02X, Got: 0x%02X\n",
                              header->computeChecksum(), header->checksum);
                updateCommStats(false);
                return false;
            }

            updateCommStats(true);

            // Process message based on type
            switch (header->type) {
                case MessageType::STATUS_RESPONSE:
                    if (length >= sizeof(StatusMessage)) {
                        Serial.printf("📊 Received STATUS_RESPONSE from node %d\n", senderNodeId);
                        const StatusMessage* status = static_cast<const StatusMessage*>(message);
                        return handleStatusMessage(*status);
                    } else {
                        Serial.printf("❌ Status message from node %d too short (%d < %d bytes)\n",
                                      senderNodeId, length, sizeof(StatusMessage));
                    }
                    break;

                case MessageType::ACK:
                    if (length >= sizeof(AckMessage)) {
                        Serial.printf("✅ Received ACK from node %d\n", senderNodeId);
                        const AckMessage* ack = static_cast<const AckMessage*>(message);
                        return handleAckMessage(*ack);
                    } else {
                        Serial.printf("❌ ACK message from node %d too short (%d < %d bytes)\n",
                                      senderNodeId, length, sizeof(AckMessage));
                    }
                    break;

                case MessageType::ERROR:
                    Serial.printf("⚠️  Received ERROR message from node %d\n", senderNodeId);
                    return handleErrorMessage(*header);

                default:
                    Serial.printf("❓ Received UNKNOWN message type %d from node %d\n",
                                  static_cast<int>(header->type), senderNodeId);
                    break;
            }

            return false;
        }

        void ControllerApplication::resetCommunicationStats() {
            Serial.println("🔄 Resetting communication statistics...");
            memset(&_commStats, 0, sizeof(_commStats));
            _commStats.lastResetTime = millis();
            Serial.println("✅ Communication statistics reset");
        }

        String ControllerApplication::getDetailedSystemStatus() const {
            DynamicJsonDocument doc(4096);

            // System information
            doc["system"]["uptime"] = millis();
            doc["system"]["free_heap"] = ESP.getFreeHeap();
            doc["system"]["chip_id"] = String(ESP.getEfuseMac(), HEX);
            doc["system"]["verbose_debugging"] = _verboseDebugging;

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
                pendingMsg["description"] = msg.commandDescription.c_str();
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
                node["last_command"] = status.lastCommand.c_str();
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

        bool ControllerApplication::initializeWebServer() {
            Serial.println("🌐 Setting up WiFi and Web Server...");

            // Initialize WiFi
            if (_wifiApMode) {
                Serial.printf("📡 Creating WiFi Access Point: %s\n", _wifiSsid);
                WiFi.softAP(_wifiSsid, _wifiPassword);
                Serial.printf("✅ AP IP address: %s\n", WiFi.softAPIP().toString().c_str());
            } else {
                Serial.printf("📡 Connecting to WiFi network: %s\n", _wifiSsid);
                WiFi.begin(_wifiSsid, _wifiPassword);

                unsigned long startTime = millis();
                while (WiFi.status() != WL_CONNECTED && millis() - startTime < 30000) {
                    delay(500);
                    Serial.print(".");
                }

                if (WiFi.status() != WL_CONNECTED) {
                    Serial.println("\n❌ Failed to connect to WiFi network");
                    return false;
                }

                Serial.printf("\n✅ Connected to %s\n", _wifiSsid);
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
                if (_verboseDebugging) {
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
                if (currentTime - it->sentTime > MESSAGE_TIMEOUT) {
                    hasTimeouts = true;

                    if (it->retryCount < MAX_RETRY_COUNT) {
                        // Retry sending the message
                        Serial.printf("🔄 Retrying message to node %d (seq %d, attempt %d/%d) - %s\n",
                                      it->nodeId, it->sequenceNum, it->retryCount+1, MAX_RETRY_COUNT,
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
                                      it->nodeId, MAX_RETRY_COUNT, it->sequenceNum,
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

                if (_verboseDebugging && dataLength <= 4) {
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

            if (_verboseDebugging) {
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
                pending.commandDescription = description.length() > 0 ? description.c_str() :
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

        bool ControllerApplication::handleStatusMessage(const szogfm::StatusMessage& status) {
            uint8_t nodeId = status.header.nodeId;

            Serial.printf("📊 Processing STATUS from node %d:\n", nodeId);
            Serial.printf("   🎵 Frequency: %.1f MHz\n", status.frequency/100.0);
            Serial.printf("   🔊 Volume: %d/15\n", status.volume);
            Serial.printf("   🔌 Relay: %s\n", status.relayState ? "ON" : "OFF");
            Serial.printf("   📡 FM RSSI: %d\n", status.rssi);
            Serial.printf("   🎧 Stereo: %s\n", status.isStereo ? "Yes" : "No");
            Serial.printf("   ⏰ Uptime: %.1f minutes\n", status.uptime / 60000.0);

            // Create or update node status
            NodeStatus& nodeStatus = _nodeStatus[nodeId];

            // Update status information
            nodeStatus.nodeId = nodeId;
            nodeStatus.isConnected = true;
            nodeStatus.lastSeenTime = millis();
            nodeStatus.rssi = status.rssi;
            nodeStatus.volume = status.volume;
            nodeStatus.muted = false; // Status message doesn't include mute state
            nodeStatus.frequency = status.frequency;
            nodeStatus.relayState = status.relayState;
            nodeStatus.signalStrength = status.rssi;
            nodeStatus.isStereo = status.isStereo;
            nodeStatus.uptime = status.uptime;
            nodeStatus.errorMessage = ""; // No error

            // Update sensor data if available
            if (status.temperature > -50.0f && status.humidity >= 0.0f) {
                nodeStatus.temperature = status.temperature;
                nodeStatus.humidity = status.humidity;
                nodeStatus.hasSensors = true;
                Serial.printf("   🌡️  Temperature: %.1f°C\n", status.temperature);
                Serial.printf("   💧 Humidity: %.1f%%\n", status.humidity);
            } else {
                nodeStatus.hasSensors = false;
                if (_verboseDebugging) {
                    Serial.printf("   📊 No sensor data (T:%.1f H:%.1f)\n", status.temperature, status.humidity);
                }
            }

            Serial.printf("✅ Node %d status updated successfully\n", nodeId);
            return true;
        }

        bool ControllerApplication::handleAckMessage(const szogfm::AckMessage& ack) {
            uint16_t ackSeq = ack.acknowledgedSeq;

            // Find and remove the pending message that was acknowledged
            for (auto it = _pendingMessages.begin(); it != _pendingMessages.end(); ++it) {
                if (it->sequenceNum == ackSeq) {
                    unsigned long responseTime = millis() - it->firstSentTime;

                    Serial.printf("✅ ACK received for seq %d from node %d - %s (response time: %lu ms)\n",
                                  ackSeq, ack.header.nodeId,
                                  ack.success ? "SUCCESS" : "FAILED", responseTime);

                    if (!ack.success) {
                        Serial.printf("   ❌ Error code: %d\n", ack.errorCode);
                    }

                    // Update node stats
                    if (_nodeStatus.find(ack.header.nodeId) != _nodeStatus.end()) {
                        NodeStatus& nodeStatus = _nodeStatus[ack.header.nodeId];
                        if (ack.success) {
                            nodeStatus.successfulCommands++;
                        } else {
                            nodeStatus.failedCommands++;
                        }
                        nodeStatus.lastCommandSuccess = ack.success;
                        nodeStatus.totalResponseTime += responseTime;
                    }

                    // Update communication stats
                    updateCommStats(ack.success, responseTime);

                    _pendingMessages.erase(it);
                    return true;
                }
            }

            Serial.printf("⚠️  Received ACK for UNKNOWN message seq %d from node %d\n",
                          ackSeq, ack.header.nodeId);
            return false;
        }

        bool ControllerApplication::handleErrorMessage(const MessageHeader& error) {
            Serial.printf("⚠️  ERROR message from node %d (seq %d)\n", error.nodeId, error.sequenceNum);

            // Update node status to indicate error
            if (_nodeStatus.find(error.nodeId) != _nodeStatus.end()) {
                _nodeStatus[error.nodeId].errorMessage = "Error reported by node";
                _nodeStatus[error.nodeId].lastSeenTime = millis();
                Serial.printf("   📝 Node %d status updated with error\n", error.nodeId);
            }

            return true;
        }

        void ControllerApplication::updateNodeConnectionStatus() {
            unsigned long currentTime = millis();
            int disconnectedCount = 0;

            for (auto& pair : _nodeStatus) {
                NodeStatus& status = pair.second;

                if (status.isConnected && currentTime - status.lastSeenTime > NODE_TIMEOUT) {
                    status.isConnected = false;
                    disconnectedCount++;

                    Serial.printf("⚠️  Node %d DISCONNECTED (timeout after %.1f minutes)\n",
                                  status.nodeId, NODE_TIMEOUT/60000.0);
                }
            }

            if (disconnectedCount > 0 && _verboseDebugging) {
                Serial.printf("📊 Connection status: %d nodes disconnected due to timeout\n", disconnectedCount);
            }
        }

        void ControllerApplication::logMessageDetails(const String& direction, uint8_t nodeId,
                                                      const String& messageType, const void* message, size_t length) {
            if (!_verboseDebugging) return;

            Serial.printf("📋 %s message details:\n", direction.c_str());
            Serial.printf("   🏷️  Type: %s\n", messageType.c_str());
            Serial.printf("   📍 Node: %d\n", nodeId);
            Serial.printf("   📏 Length: %d bytes\n", length);

            if (length >= sizeof(MessageHeader)) {
                const MessageHeader* header = static_cast<const MessageHeader*>(message);
                Serial.printf("   🔢 Sequence: %d\n", header->sequenceNum);
                Serial.printf("   ⏰ Timestamp: %lu\n", header->timestamp);
                Serial.printf("   ✅ Checksum: 0x%02X (valid: %s)\n",
                              header->checksum, header->validateChecksum() ? "yes" : "no");
            }
        }

        void ControllerApplication::updateCommStats(bool success, unsigned long responseTime) {
            if (success) {
                _commStats.totalMessagesReceived++;
                if (responseTime > 0) {
                    _commStats.totalResponseTime += responseTime;
                    _commStats.averageResponseTime = (float)_commStats.totalResponseTime / _commStats.totalMessagesReceived;
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

        // Enhanced web server handlers will be implemented in the next part...
        void ControllerApplication::handleRoot() {
            Serial.println("🌐 Web request: GET / (Main Interface)");

            String html = R"(
<!DOCTYPE html>
<html>
<head>
    <title>SzögFM Controller - Enhanced Debug Interface</title>
    <meta name='viewport' content='width=device-width, initial-scale=1'>
    <style>
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            margin: 0; padding: 20px; background: #f5f5f5;
        }
        .header {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white; padding: 20px; border-radius: 10px; margin-bottom: 20px;
        }
        .stats-grid {
            display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 15px; margin-bottom: 20px;
        }
        .stat-card {
            background: white; padding: 15px; border-radius: 8px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }
        .stat-value { font-size: 2em; font-weight: bold; color: #667eea; }
        .stat-label { color: #666; font-size: 0.9em; margin-top: 5px; }
        .node {
            background: white; border-radius: 10px; padding: 20px;
            margin-bottom: 15px; box-shadow: 0 2px 8px rgba(0,0,0,0.1);
        }
        .connected { border-left: 5px solid #4CAF50; }
        .disconnected { border-left: 5px solid #f44336; }
        .node-header {
            display: flex; justify-content: space-between; align-items: center;
            margin-bottom: 15px;
        }
        .node-title { font-size: 1.3em; font-weight: bold; }
        .node-status {
            padding: 5px 15px; border-radius: 20px; color: white; font-size: 0.9em;
        }
        .status-connected { background: #4CAF50; }
        .status-disconnected { background: #f44336; }
        .controls {
            display: grid; grid-template-columns: 1fr 1fr; gap: 15px;
            margin-top: 15px;
        }
        .control-group {
            background: #f8f9fa; padding: 15px; border-radius: 8px;
        }
        .control-group h4 { margin: 0 0 10px 0; color: #333; }
        input[type="range"] { width: 100%; margin: 10px 0; }
        input[type="number"] { width: 80px; }
        button {
            background: #667eea; color: white; border: none;
            padding: 8px 16px; border-radius: 5px; cursor: pointer; margin: 2px;
        }
        button:hover { background: #5a67d8; }
        button.danger { background: #f44336; }
        button.danger:hover { background: #d32f2f; }
        .global-controls {
            background: white; padding: 20px; border-radius: 10px;
            margin-bottom: 20px; box-shadow: 0 2px 8px rgba(0,0,0,0.1);
        }
        .debug-info {
            background: #2d3748; color: #e2e8f0; padding: 15px;
            border-radius: 8px; font-family: monospace; font-size: 0.9em;
            white-space: pre-wrap; max-height: 300px; overflow-y: auto;
        }
        .refresh-indicator {
            display: inline-block; animation: spin 1s linear infinite;
        }
        @keyframes spin { 0% { transform: rotate(0deg); } 100% { transform: rotate(360deg); } }
    </style>
</head>
<body>
    <div class="header">
        <h1>🎵 SzögFM Controller - Enhanced Debug Interface</h1>
        <p>Real-time festival audio distribution control system</p>
    </div>

    <div class="stats-grid">
        <div class="stat-card">
            <div class="stat-value" id="total-nodes">-</div>
            <div class="stat-label">Total Nodes</div>
        </div>
        <div class="stat-card">
            <div class="stat-value" id="connected-nodes">-</div>
            <div class="stat-label">Connected</div>
        </div>
        <div class="stat-card">
            <div class="stat-value" id="success-rate">-</div>
            <div class="stat-label">Success Rate</div>
        </div>
        <div class="stat-card">
            <div class="stat-value" id="pending-msgs">-</div>
            <div class="stat-label">Pending Messages</div>
        </div>
    </div>

    <div class="global-controls">
        <h2>🌐 Global Controls</h2>
        <div class="controls">
            <div class="control-group">
                <h4>Volume Control</h4>
                <input type="range" min="0" max="15" value="8" id="global-volume">
                <span id="global-volume-value">8</span>
                <button onclick="setGlobalVolume()">Set All Volumes</button>
                                                            </div>
                                                              <div class="control-group">
                                                                         <h4>Frequency Control</h4>
                                                                                       <input type="number" min="87.5" max="108.0" step="0.1" value="88.5" id="global-frequency"> MHz
                                                                                                                                                              <button onclick="setGlobalFrequency()">Set All Frequencies</button>
                                                                                                                                                                                                             </div>
                                                                                                                                                                                                               </div>
                                                                                                                                                                                                                 <div style="margin-top: 15px;">
                                                                                                                                                                                                                            <button onclick="setGlobalRelay(true)">🔌 All Relays ON</button>
                                                                                                                                                                                                                                                                                <button onclick="setGlobalRelay(false)">🔌 All Relays OFF</button>
                                                                                                                                                                                                                                                                                                                                     <button onclick="setGlobalMute(true)">🔇 Mute All</button>
                                                                                                                                                                                                                                                                                                                                                                                  <button onclick="setGlobalMute(false)">🔊 Unmute All</button>
                                                                                                                                                                                                                                                                                                                                                                                                                                  <button onclick="discoverNodes()" style="background: #ff9800;">🔍 Discover Nodes</button>
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            <button onclick="resetStats()" class="danger">📊 Reset Stats</button>
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  </div>
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    </div>

                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      <h2>📡 Node Status <span class="refresh-indicator" id="refresh-indicator">🔄</span></h2>
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         <div id="nodes-container">
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 <p>Loading nodes...</p>
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      </div>

                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        <div style="margin-top: 30px;">
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   <h3>🐛 Debug Information</h3>
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               <div class="debug-info" id="debug-info">Loading debug information...</div>
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     </div>

                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       <script>
            const globalVolumeSlider = document.getElementById('global-volume');
            const globalVolumeValue = document.getElementById('global-volume-value');

            globalVolumeSlider.oninput = function() {
                globalVolumeValue.textContent = this.value;
            };

            function updateStatus() {
                fetch('/status')
                        .then(response => response.json())
                .then(data => {
                        updateStats(data);
                        updateNodes(data.nodes);
                })
                .catch(error => console.error('Error fetching status:', error));

                fetch('/detailed_status')
                        .then(response => response.text())
                .then(data => {
                        document.getElementById('debug-info').textContent = data;
                })
                .catch(error => console.error('Error fetching debug info:', error));
            }

            function updateStats(data) {
                const totalNodes = data.nodes ? data.nodes.length : 0;
                const connectedNodes = data.nodes ? data.nodes.filter(n => n.connected).length : 0;

                document.getElementById('total-nodes').textContent = totalNodes;
                document.getElementById('connected-nodes').textContent = connectedNodes;

                // We'll get success rate from detailed status
                fetch('/comm_stats')
                        .then(response => response.json())
                .then(stats => {
                        document.getElementById('success-rate').textContent = stats.success_rate.toFixed(1) + '%';
                        document.getElementById('pending-msgs').textContent = stats.pending_messages || 0;
                });
            }

            function updateNodes(nodes) {
                const container = document.getElementById('nodes-container');
                container.innerHTML = '';

                if (!nodes || nodes.length === 0) {
                    container.innerHTML = '<p style="text-align: center; color: #666;">No nodes discovered yet. Click "Discover Nodes" to search for active nodes.</p>';
                    return;
                }

                nodes.forEach(node => {
                        const nodeDiv = document.createElement('div');
                        nodeDiv.className = 'node ' + (node.connected ? 'connected' : 'disconnected');

                        nodeDiv.innerHTML = `
                        <div class="node-header">
                        <div class="node-title">Node ${node.id}</div>
                        <div class="node-status ${node.connected ? 'status-connected' : 'status-disconnected'}">
                        ${node.connected ? '🟢 Connected' : '🔴 Disconnected'}
                        </div>
                        </div>

                        <div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 10px; margin-bottom: 15px;">
                        <div><strong>🎵 Frequency:</strong> ${(node.frequency / 100).toFixed(1)} MHz</div>
                        <div><strong>🔊 Volume:</strong> ${node.volume}/15 ${node.muted ? '(MUTED)' : ''}</div>
                        <div><strong>🔌 Relay:</strong> ${node.relay ? '🟢 ON' : '🔴 OFF'}</div>
                        <div><strong>📡 RSSI:</strong> ${node.rssi || 'N/A'}</div>
                        ${node.has_sensors ? `
                            <div><strong>🌡️ Temp:</strong> ${node.temperature.toFixed(1)}°C</div>
                                                                                          <div><strong>💧 Humidity:</strong> ${node.humidity.toFixed(1)}%</div>
                            ` : ''}
                        </div>

                        <div class="controls">
                        <div class="control-group">
                        <h4>Volume</h4>
                        <input type="range" min="0" max="15" value="${node.volume}"
                        onchange="setVolume(${node.id}, this.value)">
                        <span>${node.volume}</span>
                        <br>
                        <button onclick="toggleMute(${node.id}, ${!node.muted})">
                        ${node.muted ? '🔊 Unmute' : '🔇 Mute'}
                        </button>
                        </div>

                        <div class="control-group">
                        <h4>Frequency</h4>
                        <input type="number" min="87.5" max="108.0" step="0.1"
                        value="${(node.frequency / 100).toFixed(1)}"
                        onchange="setFrequency(${node.id}, this.value)"> MHz
                        <br>
                        <button onclick="testNode(${node.id})">🧪 Test Comm</button>
                        </div>
                        </div>

                        <div style="margin-top: 15px;">
                        <button onclick="toggleRelay(${node.id}, ${!node.relay})">
                        🔌 ${node.relay ? 'Turn OFF' : 'Turn ON'} Relay
                        </button>
                        <button onclick="resetNode(${node.id})" class="danger">🔄 Reset Node</button>
                        </div>
                        `;

                        container.appendChild(nodeDiv);
                });
            }

            // Control functions
            function setVolume(nodeId, volume) {
                fetch('/set_volume', {
                        method: 'POST',
                        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                        body: `node_id=${nodeId}&volume=${volume}`
                });
            }

            function setFrequency(nodeId, frequency) {
                const freq = Math.round(parseFloat(frequency) * 100);
                fetch('/set_frequency', {
                        method: 'POST',
                        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                        body: `node_id=${nodeId}&frequency=${freq}`
                });
            }

            function toggleRelay(nodeId, state) {
                fetch('/set_relay', {
                        method: 'POST',
                        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                        body: `node_id=${nodeId}&state=${state ? 1 : 0}`
                }).then(() => updateStatus());
            }

            function toggleMute(nodeId, mute) {
                fetch('/set_mute', {
                        method: 'POST',
                        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                        body: `node_id=${nodeId}&mute=${mute ? 1 : 0}`
                }).then(() => updateStatus());
            }

            function resetNode(nodeId) {
                if (confirm(`Are you sure you want to reset node ${nodeId}?`)) {
                    fetch('/reset_node', {
                            method: 'POST',
                            headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                            body: `node_id=${nodeId}`
                    });
                }
            }

            function testNode(nodeId) {
                fetch('/test_node', {
                        method: 'POST',
                        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                        body: `node_id=${nodeId}`
                }).then(() => {
                        alert(`Communication test initiated for node ${nodeId}. Check debug log for results.`);
                });
            }

            // Global controls
            function setGlobalVolume() {
                const volume = document.getElementById('global-volume').value;
                setVolume(0, volume); // Node ID 0 = broadcast
            }

            function setGlobalFrequency() {
                const frequency = document.getElementById('global-frequency').value;
                setFrequency(0, frequency); // Node ID 0 = broadcast
            }

            function setGlobalRelay(state) {
                toggleRelay(0, state); // Node ID 0 = broadcast
            }

            function setGlobalMute(mute) {
                toggleMute(0, mute); // Node ID 0 = broadcast
            }

            function discoverNodes() {
                fetch('/discover_nodes', { method: 'POST' })
                        .then(() => {
                        alert('Node discovery initiated. Nodes should appear in 10-30 seconds.');
                        setTimeout(updateStatus, 2000);
                });
            }

            function resetStats() {
                if (confirm('Reset all communication statistics?')) {
                    fetch('/reset_stats', { method: 'POST' })
                            .then(() => updateStatus());
                }
            }

            // Auto-refresh
            setInterval(updateStatus, 2000);
            document.addEventListener('DOMContentLoaded', updateStatus);
            </script>
              </body>
                </html>
            )";

            _webServer->send(200, "text/html", html);
            Serial.println("✅ Enhanced web UI served successfully");
        }

        void ControllerApplication::handleStatus() {
            if (_verboseDebugging) {
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

                // Enhanced debug info
                nodeObj["total_commands"] = status.totalCommands;
                nodeObj["successful_commands"] = status.successfulCommands;
                nodeObj["failed_commands"] = status.failedCommands;
                nodeObj["last_command"] = status.lastCommand.c_str();
                nodeObj["last_command_success"] = status.lastCommandSuccess;

                if (status.hasSensors) {
                    nodeObj["has_sensors"] = true;
                    nodeObj["temperature"] = status.temperature;
                    nodeObj["humidity"] = status.humidity;
                } else {
                    nodeObj["has_sensors"] = false;
                }

                if (!status.errorMessage.empty()) {
                    nodeObj["error"] = status.errorMessage.c_str();
                }
            }

            String jsonResponse;
            serializeJson(doc, jsonResponse);

            _webServer->send(200, "application/json", jsonResponse);

            if (_verboseDebugging) {
                Serial.printf("✅ Status sent for %d nodes\n", nodeCount);
            }
        }

        void ControllerApplication::handleDetailedStatus() {
            String response = getDetailedSystemStatus();
            _webServer->send(200, "application/json", response);
        }

        void ControllerApplication::handleCommStats() {
            DynamicJsonDocument doc(1024);

            doc["total_sent"] = _commStats.totalMessagesSent;
            doc["total_received"] = _commStats.totalMessagesReceived;
            doc["total_retries"] = _commStats.totalRetries;
            doc["total_timeouts"] = _commStats.totalTimeouts;
            doc["total_errors"] = _commStats.totalErrors;
            doc["success_rate"] = _commStats.messageSuccessRate;
            doc["avg_response_time"] = _commStats.averageResponseTime;
            doc["pending_messages"] = _pendingMessages.size();
            doc["uptime"] = millis() - _commStats.lastResetTime;

            String response;
            serializeJson(doc, response);
            _webServer->send(200, "application/json", response);
        }

        void ControllerApplication::handleSystemDiagnostics() {
            // Will implement system diagnostics
            _webServer->send(200, "text/plain", "System diagnostics not yet implemented");
        }

        void ControllerApplication::handleNodeDiagnostics() {
            // Will implement node-specific diagnostics
            _webServer->send(200, "text/plain", "Node diagnostics not yet implemented");
        }

        void ControllerApplication::handleSetVolume() {
            if (!_webServer->hasArg("node_id") || !_webServer->hasArg("volume")) {
                Serial.println("🌐 Web request: POST /set_volume - Missing parameters");
                _webServer->send(400, "text/plain", "Missing parameters");
                return;
            }

            uint8_t nodeId = _webServer->arg("node_id").toInt();
            uint8_t volume = _webServer->arg("volume").toInt();

            Serial.printf("🌐 Web request: POST /set_volume - Node: %d, Volume: %d\n", nodeId, volume);

            bool success = setNodeVolume(nodeId, volume);
            _webServer->send(success ? 200 : 500, "text/plain", success ? "OK" : "Failed");

            Serial.printf("🌐 Volume command %s\n", success ? "queued successfully" : "failed");
        }

        void ControllerApplication::handleSetFrequency() {
            if (!_webServer->hasArg("node_id") || !_webServer->hasArg("frequency")) {
                Serial.println("🌐 Web request: POST /set_frequency - Missing parameters");
                _webServer->send(400, "text/plain", "Missing parameters");
                return;
            }

            uint8_t nodeId = _webServer->arg("node_id").toInt();
            uint16_t frequency = _webServer->arg("frequency").toInt();

            Serial.printf("🌐 Web request: POST /set_frequency - Node: %d, Frequency: %d (%.1f MHz)\n",
                          nodeId, frequency, frequency/100.0);

            bool success = setNodeFrequency(nodeId, frequency);
            _webServer->send(success ? 200 : 500, "text/plain", success ? "OK" : "Failed");

            Serial.printf("🌐 Frequency command %s\n", success ? "queued successfully" : "failed");
        }

        void ControllerApplication::handleSetRelay() {
            if (!_webServer->hasArg("node_id") || !_webServer->hasArg("state")) {
                Serial.println("🌐 Web request: POST /set_relay - Missing parameters");
                _webServer->send(400, "text/plain", "Missing parameters");
                return;
            }

            uint8_t nodeId = _webServer->arg("node_id").toInt();
            bool state = _webServer->arg("state").toInt() != 0;

            Serial.printf("🌐 Web request: POST /set_relay - Node: %d, State: %s\n",
                          nodeId, state ? "ON" : "OFF");

            bool success = setNodeRelayState(nodeId, state);
            _webServer->send(success ? 200 : 500, "text/plain", success ? "OK" : "Failed");

            Serial.printf("🌐 Relay command %s\n", success ? "queued successfully" : "failed");
        }

        void ControllerApplication::handleSetMute() {
            if (!_webServer->hasArg("node_id") || !_webServer->hasArg("mute")) {
                Serial.println("🌐 Web request: POST /set_mute - Missing parameters");
                _webServer->send(400, "text/plain", "Missing parameters");
                return;
            }

            uint8_t nodeId = _webServer->arg("node_id").toInt();
            bool mute = _webServer->arg("mute").toInt() != 0;

            Serial.printf("🌐 Web request: POST /set_mute - Node: %d, Mute: %s\n",
                          nodeId, mute ? "ON" : "OFF");

            bool success = setNodeMute(nodeId, mute);
            _webServer->send(success ? 200 : 500, "text/plain", success ? "OK" : "Failed");

            Serial.printf("🌐 Mute command %s\n", success ? "queued successfully" : "failed");
        }

        void ControllerApplication::handleResetNode() {
            if (!_webServer->hasArg("node_id")) {
                Serial.println("🌐 Web request: POST /reset_node - Missing parameters");
                _webServer->send(400, "text/plain", "Missing parameters");
                return;
            }

            uint8_t nodeId = _webServer->arg("node_id").toInt();
            Serial.printf("🌐 Web request: POST /reset_node - Node: %d\n", nodeId);

            bool success = resetNode(nodeId);
            _webServer->send(success ? 200 : 500, "text/plain", success ? "OK" : "Failed");

            Serial.printf("🌐 Reset command %s\n", success ? "queued successfully" : "failed");
        }

        void ControllerApplication::handleDiscoverNodes() {
            Serial.println("🌐 Web request: POST /discover_nodes");

            int discovered = discoverNodes();
            _webServer->send(200, "text/plain", "Discovery initiated for " + String(discovered) + " nodes");

            Serial.printf("🌐 Node discovery initiated for %d nodes\n", discovered);
        }

        void ControllerApplication::handleTestNode() {
            if (!_webServer->hasArg("node_id")) {
                _webServer->send(400, "text/plain", "Missing node_id parameter");
                return;
            }

            uint8_t nodeId = _webServer->arg("node_id").toInt();
            Serial.printf("🌐 Web request: POST /test_node - Node: %d\n", nodeId);

            bool success = testNodeCommunication(nodeId);
            _webServer->send(200, "text/plain", success ? "Test initiated" : "Test failed");
        }

        void ControllerApplication::handleResetStats() {
            Serial.println("🌐 Web request: POST /reset_stats");

            resetCommunicationStats();
            _webServer->send(200, "text/plain", "Statistics reset");

            Serial.println("🌐 Statistics reset via web request");
        }

        void ControllerApplication::handleCommTest() {
            Serial.println("🌐 Web request: POST /comm_test");

            // Perform communication test
            _commModule->performDiagnostics();
            _webServer->send(200, "text/plain", "Communication test performed - check serial log");

            Serial.println("🌐 Communication test performed via web request");
        }

    } // namespace controller
} // namespace szogfm