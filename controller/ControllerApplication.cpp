/**
 * SzögFM Controller Application - Enhanced with Individual Node Controls
 *
 * This version adds individual node control capabilities to the web interface
 * and fixes emoji encoding issues.
 *
 * Upload with: pio run -e controller --target upload
 */

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

            // CRITICAL: Do NOT initialize complex objects or system calls in constructor
            // This was the cause of the bootloop issue
            _commModule = nullptr;
            _webServer = nullptr;

            // Initialize default configuration (simple assignments only)
            _config.wifiSsid = "SzogFM_Controller";
            _config.wifiPassword = "GombaSzog24";
            _config.accessPointMode = true;
            _config.radioChannel = 0x1A;
            _config.radioAddress = 0x1234;
            _config.transmissionPower = 0; // Maximum power
            _config.statusRequestInterval = 15000; // 15 seconds (reduced from 10)
            _config.discoveryInterval = 120000; // 2 minutes (increased from 1)
            _config.messageTimeout = 5000; // 5 seconds (increased from 3)
            _config.maxRetryCount = 3; // Reduced from 5
            _config.verboseDebugging = false; // Disabled by default
            _config.communicationDebugLevel = 0; // Minimal logging

            // Initialize communication statistics (simple memset)
            memset(&_commStats, 0, sizeof(_commStats));
            _commStats.minimumResponseTime = ULONG_MAX;

            Serial.println("🎵 SzögFM Controller created (stable version)");
        }

        ControllerApplication::~ControllerApplication() {
            // Safe cleanup
            if (_commModule) {
                delete _commModule;
                _commModule = nullptr;
            }
            if (_webServer) {
                delete _webServer;
                _webServer = nullptr;
            }
        }

        bool ControllerApplication::initialize() {
            Serial.println("\n" + repeatChar('=', 60));
            Serial.println("🎵 SzögFM Controller Application Starting (Fixed Version) 🎵");
            Serial.println(repeatChar('=', 60));
            Serial.printf("⏰ Startup time: %lu ms\n", millis());
            Serial.printf("🔧 ESP32 Chip ID: %012llX\n", ESP.getEfuseMac());
            Serial.printf("💾 Free heap: %d bytes\n", ESP.getFreeHeap());
            Serial.printf("🔧 CPU Frequency: %d MHz\n", getCpuFrequencyMhz());

            // Stage 1: Basic system preparation
            Serial.println("\n📋 Stage 1: System Preparation");
            Serial.println("   🛑 Disabling potentially problematic features...");

            // Disable Bluetooth to free resources and avoid conflicts
            btStop();
            Serial.println("   ✅ Bluetooth disabled");

            // Reset WiFi to clean state
            WiFi.disconnect();
            WiFi.mode(WIFI_OFF);
            delay(500);
            Serial.println("   ✅ WiFi reset to clean state");

            // Stage 2: WiFi and Web Server (most critical for recovery)
            Serial.println("\n📋 Stage 2: WiFi and Web Server");
            if (!initializeWebServer()) {
                Serial.println("❌ CRITICAL: Web server initialization failed");
                Serial.println("   🔄 Attempting recovery with minimal AP...");

                // Fallback: try emergency AP
                if (WiFi.softAP("SzogFM_Emergency", "emergency123")) {
                    Serial.printf("🆘 Emergency AP started: %s\n", WiFi.softAPIP().toString().c_str());
                    // Continue even if web server failed - at least WiFi works
                } else {
                    Serial.println("💀 Complete WiFi failure - continuing without network");
                }
            } else {
                Serial.println("✅ WiFi and Web Server initialized successfully");
            }

            // Stage 3: Communication Module (optional - system works without it)
            Serial.println("\n📋 Stage 3: Communication Module (Optional)");
            bool commSuccess = initializeCommunication();

            if (commSuccess) {
                Serial.println("✅ Communication module ready for node control");
            } else {
                Serial.println("⚠️  Communication module disabled - web interface only mode");
                Serial.println("   📝 This is acceptable for system recovery and configuration");
            }

            // Stage 4: Final setup
            Serial.println("\n📋 Stage 4: Final Setup");
            _commStats.lastResetTime = millis();
            _initialized = true;

            // Initial discovery only if communication is working
            if (commSuccess && _commModule) {
                Serial.println("🔍 Starting limited initial node discovery...");
                int discoveredNodes = discoverNodes(3); // Only discover first 3 nodes
                Serial.printf("📋 Initial discovery completed (%d requests sent)\n", discoveredNodes);
            }

            Serial.println("\n" + repeatChar('=', 60));
            Serial.println("🎉 Controller Application Initialized Successfully! 🎉");
            Serial.println("📊 System Status:");
            Serial.printf("   • Free heap: %d bytes\n", ESP.getFreeHeap());
            Serial.printf("   • Communication: %s\n", _commModule ? "✅ Active" : "⚠️  Disabled");

            if (_config.accessPointMode) {
                Serial.printf("   • WiFi AP: %s\n", WiFi.softAPIP().toString().c_str());
                Serial.printf("   • Web interface: http://%s\n", WiFi.softAPIP().toString().c_str());
            } else if (WiFi.status() == WL_CONNECTED) {
                Serial.printf("   • WiFi STA: %s\n", WiFi.localIP().toString().c_str());
                Serial.printf("   • Web interface: http://%s\n", WiFi.localIP().toString().c_str());
            }

            Serial.println("🟢 System is ready for festival operation!");
            Serial.println(repeatChar('=', 60));

            return true; // Always return true to prevent reboot loops
        }

        bool ControllerApplication::initializeCommunication() {
            Serial.println("📡 Attempting communication module initialization...");

            try {
                // Create communication module with timeout protection
                _commModule = new communication::EbyteCommModule(&Serial2, _pinM0, _pinM1, _pinAUX);

                // Try initialization with timeout
                Serial.println("   🔧 Initializing EBYTE module...");
                bool initResult = false;

                // Simple timeout mechanism
                unsigned long startTime = millis();
                const unsigned long timeout = 10000; // 10 second timeout

                while (millis() - startTime < timeout) {
                    if (_commModule->initialize()) {
                        initResult = true;
                        break;
                    }
                    delay(1000);
                    Serial.print(".");
                }
                Serial.println();

                if (!initResult) {
                    Serial.println("⚠️  Communication module initialization timeout");
                    delete _commModule;
                    _commModule = nullptr;
                    return false;
                }

                Serial.println("✅ Communication module initialized");

                // Try configuration with timeout
                Serial.println("   ⚙️  Configuring communication module...");
                bool configResult = _commModule->configure(
                        _config.radioChannel,
                        _config.radioAddress,
                        communication::AIR_2K4,
                        communication::UART_9600,
                        _config.transmissionPower
                );

                if (!configResult) {
                    Serial.println("⚠️  Communication configuration failed - using defaults");
                }

                // Set minimal debug level to reduce serial spam
                _commModule->setDebugLevel(0);
                Serial.println("✅ Communication module ready");
                return true;

            } catch (...) {
                Serial.println("❌ Exception during communication initialization");
                if (_commModule) {
                    delete _commModule;
                    _commModule = nullptr;
                }
                return false;
            }
        }

        void ControllerApplication::update() {
            if (!_initialized) {
                return;
            }

            static unsigned long lastHeartbeat = 0;
            unsigned long currentTime = millis();

            // Heartbeat every 60 seconds (reduced frequency)
            if (currentTime - lastHeartbeat > 60000) {
                Serial.printf("\n💓 Controller heartbeat - Uptime: %lu minutes, Free heap: %d bytes\n",
                              currentTime / 60000, ESP.getFreeHeap());

                if (_commModule) {
                    Serial.printf("📊 Comm stats - Sent: %lu, Received: %lu, Pending: %d\n",
                                  _commStats.totalMessagesSent, _commStats.totalMessagesReceived,
                                  _pendingMessages.size());
                } else {
                    Serial.println("📡 Communication: Disabled (web interface only)");
                }

                // Memory leak detection
                static uint32_t minHeap = ESP.getFreeHeap();
                if (ESP.getFreeHeap() < minHeap) {
                    minHeap = ESP.getFreeHeap();
                    Serial.printf("⚠️  New minimum heap: %d bytes\n", minHeap);
                }

                lastHeartbeat = currentTime;
            }

            // Handle web server (always active)
            if (_webServer) {
                _webServer->handleClient();
            }

            // Communication processing (only if available)
            if (_commModule) {
                try {
                    // Process received messages
                    if (processMessages()) {
                        // Message was processed successfully
                    }

                    // Process pending messages that need retries
                    processPendingMessages();

                    // Update node connection status
                    updateNodeConnectionStatus();

                    // Periodic status requests (less frequent)
                    if (currentTime - _lastStatusRequestTime > _config.statusRequestInterval) {
                        requestAllNodeStatus();
                        _lastStatusRequestTime = currentTime;
                    }

                    // Periodic node discovery (much less frequent)
                    if (currentTime - _lastDiscoveryTime > _config.discoveryInterval) {
                        int discoveredNodes = discoverNodes(5); // Limited discovery
                        Serial.printf("🔍 Periodic discovery: %d requests sent\n", discoveredNodes);
                        _lastDiscoveryTime = currentTime;
                    }

                    // Update communication module
                    _commModule->update();

                } catch (...) {
                    Serial.println("❌ Exception in communication processing - continuing");
                    // Don't crash, just continue
                }
            }

            // Cleanup old pending messages to prevent memory leaks
            if (_pendingMessages.size() > 20) {
                Serial.printf("🧹 Cleaning up old pending messages (count: %d)\n", _pendingMessages.size());
                _pendingMessages.erase(_pendingMessages.begin(), _pendingMessages.begin() + 5);
                Serial.println("✅ Cleaned up 5 old pending messages");
            }
        }

        // Node control methods with safety checks
        bool ControllerApplication::setNodeVolume(uint8_t nodeId, uint8_t volume) {
            if (!_commModule) {
                Serial.printf("⚠️  Cannot set volume - communication disabled\n");
                return false;
            }

            // Validate volume range
            if (volume > 15) {
                Serial.printf("⚠️  Volume value %d exceeds maximum (15), clamping\n", volume);
                volume = 15;
            }

            String description = "Set volume to " + String(volume) + " for node " + String(nodeId);
            Serial.printf("🔊 %s\n", description.c_str());

            bool success = sendCommandMessage(nodeId, Command::SET_VOLUME, &volume, 1, description);

            if (success) {
                // Update local cache optimistically
                if (_nodeStatus.find(nodeId) != _nodeStatus.end()) {
                    _nodeStatus[nodeId].volume = volume;
                    _nodeStatus[nodeId].lastCommandTime = millis();
                    _nodeStatus[nodeId].lastCommand = "SET_VOLUME(" + String(volume) + ")";
                }
            }

            return success;
        }

        bool ControllerApplication::setNodeFrequency(uint8_t nodeId, uint16_t frequency) {
            if (!_commModule) {
                Serial.printf("⚠️  Cannot set frequency - communication disabled\n");
                return false;
            }

            // Validate frequency range
            if (frequency < 8750 || frequency > 10800) {
                Serial.printf("❌ Invalid frequency: %d (must be 8750-10800)\n", frequency);
                return false;
            }

            String description = "Set frequency to " + String(frequency/100.0, 1) + " MHz for node " + String(nodeId);
            Serial.printf("📻 %s\n", description.c_str());

            // Prepare data (little-endian)
            uint8_t data[2];
            data[0] = frequency & 0xFF;
            data[1] = (frequency >> 8) & 0xFF;

            bool success = sendCommandMessage(nodeId, Command::SET_FREQUENCY, data, 2, description);

            if (success) {
                // Update local cache optimistically
                if (_nodeStatus.find(nodeId) != _nodeStatus.end()) {
                    _nodeStatus[nodeId].frequency = frequency;
                    _nodeStatus[nodeId].lastCommandTime = millis();
                    _nodeStatus[nodeId].lastCommand = "SET_FREQUENCY(" + String(frequency/100.0, 1) + "MHz)";
                }
            }

            return success;
        }

        bool ControllerApplication::setNodeRelayState(uint8_t nodeId, bool state) {
            if (!_commModule) {
                Serial.printf("⚠️  Cannot set relay - communication disabled\n");
                return false;
            }

            uint8_t data = state ? 1 : 0;
            String description = "Set relay " + String(state ? "ON" : "OFF") + " for node " + String(nodeId);
            Serial.printf("🔌 %s\n", description.c_str());

            bool success = sendCommandMessage(nodeId, Command::TOGGLE_RELAY, &data, 1, description);

            if (success) {
                // Update local cache optimistically
                if (_nodeStatus.find(nodeId) != _nodeStatus.end()) {
                    _nodeStatus[nodeId].relayState = state;
                    _nodeStatus[nodeId].lastCommandTime = millis();
                    _nodeStatus[nodeId].lastCommand = "TOGGLE_RELAY(" + String(state ? "ON" : "OFF") + ")";
                }
            }

            return success;
        }

        bool ControllerApplication::setNodeMute(uint8_t nodeId, bool mute) {
            if (!_commModule) {
                Serial.printf("⚠️  Cannot set mute - communication disabled\n");
                return false;
            }

            String description = "Set mute " + String(mute ? "ON" : "OFF") + " for node " + String(nodeId);
            Serial.printf("🔇 %s\n", description.c_str());

            Command cmd = mute ? Command::MUTE : Command::UNMUTE;
            bool success = sendCommandMessage(nodeId, cmd, nullptr, 0, description);

            if (success) {
                // Update local cache optimistically
                if (_nodeStatus.find(nodeId) != _nodeStatus.end()) {
                    _nodeStatus[nodeId].muted = mute;
                    _nodeStatus[nodeId].lastCommandTime = millis();
                    _nodeStatus[nodeId].lastCommand = mute ? "MUTE" : "UNMUTE";
                }
            }

            return success;
        }

        // Broadcast methods
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

        // Status and monitoring methods
        const NodeStatus* ControllerApplication::getNodeStatus(uint8_t nodeId) const {
            auto it = _nodeStatus.find(nodeId);
            return (it != _nodeStatus.end()) ? &(it->second) : nullptr;
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
            if (!_commModule) {
                return false;
            }

            String description = "Request status from node " + String(nodeId);
            return sendCommandMessage(nodeId, Command::GET_STATUS, nullptr, 0, description);
        }

        bool ControllerApplication::requestAllNodeStatus() {
            return requestNodeStatus(0); // Broadcast
        }

        bool ControllerApplication::resetNode(uint8_t nodeId) {
            if (!_commModule) {
                return false;
            }

            String description = "RESET node " + String(nodeId);
            Serial.printf("🔄 %s\n", description.c_str());

            bool success = sendCommandMessage(nodeId, Command::RESET, nullptr, 0, description);

            if (success && nodeId != 0) {
                // Remove node from status cache as it will restart
                _nodeStatus.erase(nodeId);
                Serial.printf("🗑️  Removed node %d from cache (reset pending)\n", nodeId);
            }

            return success;
        }

        int ControllerApplication::discoverNodes() {
            return discoverNodes(20);
        }

        int ControllerApplication::discoverNodes(int maxNodes) {
            if (!_commModule) {
                Serial.println("❌ Cannot discover nodes - communication disabled");
                return 0;
            }

            Serial.printf("🔍 Starting node discovery (max %d nodes)...\n", maxNodes);

            int nodesToDiscover = std::min(maxNodes, 20);
            int discoveredCount = 0;

            for (uint8_t nodeId = 1; nodeId <= nodesToDiscover; nodeId++) {
                if (requestNodeStatus(nodeId)) {
                    discoveredCount++;
                }
                delay(200); // Spread out requests to avoid overwhelming the system
            }

            Serial.printf("📡 Discovery requests sent to %d nodes\n", discoveredCount);
            return discoveredCount;
        }

        bool ControllerApplication::testNodeCommunication(uint8_t nodeId) {
            Serial.printf("🧪 Testing communication with node %d...\n", nodeId);
            return requestNodeStatus(nodeId);
        }

        void ControllerApplication::resetCommunicationStats() {
            Serial.println("🔄 Resetting communication statistics...");
            memset(&_commStats, 0, sizeof(_commStats));
            _commStats.lastResetTime = millis();
            _commStats.minimumResponseTime = ULONG_MAX;
            Serial.println("✅ Communication statistics reset");
        }

        const CommunicationStats& ControllerApplication::getCommunicationStats() const {
            return _commStats;
        }

        String ControllerApplication::getDetailedSystemStatus() const {
            DynamicJsonDocument doc(4096);

            // System information
            doc["system"]["uptime"] = millis();
            doc["system"]["free_heap"] = ESP.getFreeHeap();
            doc["system"]["chip_id"] = String(ESP.getEfuseMac(), HEX);
            doc["system"]["cpu_freq"] = getCpuFrequencyMhz();
            doc["system"]["communication_available"] = (_commModule != nullptr);

            // Communication statistics
            JsonObject commStats = doc.createNestedObject("communication_stats");
            commStats["total_sent"] = _commStats.totalMessagesSent;
            commStats["total_received"] = _commStats.totalMessagesReceived;
            commStats["success_rate"] = _commStats.messageSuccessRate;
            commStats["pending_messages"] = _pendingMessages.size();

            // Node information
            JsonArray nodes = doc.createNestedArray("nodes");
            for (const auto& pair : _nodeStatus) {
                JsonObject node = nodes.createNestedObject();
                const NodeStatus& status = pair.second;
                node["id"] = status.nodeId;
                node["connected"] = status.isConnected;
                node["last_seen"] = millis() - status.lastSeenTime;
                node["frequency"] = status.frequency;
                node["volume"] = status.volume;
                node["muted"] = status.muted;
                node["relay"] = status.relayState;
            }

            String result;
            serializeJsonPretty(doc, result);
            return result;
        }

        bool ControllerApplication::setConfiguration(const ControllerConfig& config) {
            _config = config;
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

        const ControllerConfig& ControllerApplication::getConfiguration() const {
            return _config;
        }

        // Private helper methods
        bool ControllerApplication::initializeWebServer() {
            Serial.println("🌐 Setting up WiFi and Web Server...");

            try {
                if (_config.accessPointMode) {
                    Serial.printf("📡 Creating WiFi Access Point: %s\n", _config.wifiSsid.c_str());
                    bool apResult = WiFi.softAP(_config.wifiSsid.c_str(), _config.wifiPassword.c_str());
                    if (!apResult) {
                        Serial.println("❌ Failed to create WiFi Access Point");
                        return false;
                    }
                    delay(2000);
                    Serial.printf("✅ AP IP address: %s\n", WiFi.softAPIP().toString().c_str());
                } else {
                    Serial.printf("📡 Connecting to WiFi network: %s\n", _config.wifiSsid.c_str());
                    WiFi.begin(_config.wifiSsid.c_str(), _config.wifiPassword.c_str());

                    unsigned long startTime = millis();
                    while (WiFi.status() != WL_CONNECTED && millis() - startTime < 20000) {
                        delay(500);
                        Serial.print(".");
                    }

                    if (WiFi.status() != WL_CONNECTED) {
                        Serial.println("\n❌ Failed to connect to WiFi - falling back to AP mode");
                        WiFi.softAP("SzogFM_Fallback", "fallback123");
                        delay(2000);
                        Serial.printf("🆘 Fallback AP IP: %s\n", WiFi.softAPIP().toString().c_str());
                    } else {
                        Serial.printf("\n✅ Connected - IP: %s\n", WiFi.localIP().toString().c_str());
                    }
                }

                // Initialize web server
                _webServer = new WebServer(80);
                setupWebRoutes();
                _webServer->begin();
                Serial.println("🌍 HTTP server started on port 80");
                return true;

            } catch (...) {
                Serial.println("❌ Exception during web server initialization");
                return false;
            }
        }

        void ControllerApplication::setupWebRoutes() {
            // Main interface
            _webServer->on("/", HTTP_GET, [this]() {
                handleRoot();
            });

            // API endpoints
            _webServer->on("/status", HTTP_GET, [this]() {
                handleStatus();
            });

            _webServer->on("/detailed_status", HTTP_GET, [this]() {
                handleDetailedStatus();
            });

            // Control endpoints
            _webServer->on("/set_volume", HTTP_POST, [this]() {
                handleSetVolume();
            });

            _webServer->on("/set_frequency", HTTP_POST, [this]() {
                handleSetFrequency();
            });

            _webServer->on("/set_relay", HTTP_POST, [this]() {
                handleSetRelay();
            });

            _webServer->on("/set_mute", HTTP_POST, [this]() {
                handleSetMute();
            });

            // Utility endpoints
            _webServer->on("/discover_nodes", HTTP_POST, [this]() {
                handleDiscoverNodes();
            });

            _webServer->on("/reset_stats", HTTP_POST, [this]() {
                handleResetStats();
            });

            // Health check
            _webServer->on("/health", HTTP_GET, [this]() {
                _webServer->send(200, "application/json",
                                 "{\"status\":\"ok\",\"uptime\":" + String(millis()) + "}");
            });

            // 404 handler
            _webServer->onNotFound([this]() {
                _webServer->send(404, "text/html",
                                 "<h1>404 Not Found</h1><p><a href='/'>Go to main page</a></p>");
            });
        }

        // Web interface handlers
        void ControllerApplication::handleRoot() {
            String html = createMainWebInterface();
            _webServer->send(200, "text/html", html);
        }

        String ControllerApplication::createMainWebInterface() {
            String html = "<!DOCTYPE html><html><head>";
            html += "<title>SzogFM Controller</title>";
            // FIX EMOJI ENCODING: Add proper UTF-8 charset declaration
            html += "<meta charset='UTF-8'>";
            html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
            html += "<style>";
            html += "body { font-family: Arial, sans-serif; margin: 20px; background: #f0f0f0; }";
            html += ".container { max-width: 1200px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; }";
            html += ".status { padding: 10px; margin: 10px 0; border-radius: 5px; }";
            html += ".good { background: #d4edda; border: 1px solid #c3e6cb; }";
            html += ".warning { background: #fff3cd; border: 1px solid #ffeeba; }";
            html += ".error { background: #f8d7da; border: 1px solid #f5c6cb; }";
            html += ".button { background: #007bff; color: white; padding: 8px 12px; border: none; border-radius: 4px; cursor: pointer; margin: 2px; font-size: 12px; }";
            html += ".button:hover { background: #0056b3; }";
            html += ".button-small { padding: 4px 8px; font-size: 11px; }";
            html += ".stats { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 15px; margin: 20px 0; }";
            html += ".stat-card { background: #f8f9fa; padding: 15px; border-radius: 5px; border-left: 4px solid #007bff; }";
            html += ".node-controls { margin: 10px 0; padding: 10px; background: #f8f9fa; border-radius: 5px; }";
            html += ".control-row { display: flex; align-items: center; gap: 5px; margin: 5px 0; flex-wrap: wrap; }";
            html += ".control-input { padding: 4px; border: 1px solid #ccc; border-radius: 3px; width: 60px; }";
            html += "h1 { color: #333; } h2 { color: #666; } h3 { color: #333; margin: 10px 0 5px 0; }";
            html += "</style></head><body>";

            html += "<div class='container'>";
            html += "<h1>🎵 SzögFM Festival Controller</h1>";

            // System Status
            String statusClass = _commModule ? "good" : "warning";
            html += "<div class='status " + statusClass + "'>";
            html += "<h2>📡 System Status</h2>";
            html += "<p><strong>Communication:</strong> " + String(_commModule ? "✅ Active" : "⚠️  Disabled") + "</p>";
            html += "<p><strong>Free Memory:</strong> " + String(ESP.getFreeHeap()) + " bytes</p>";
            html += "<p><strong>Uptime:</strong> " + String(millis() / 60000) + " minutes</p>";
            if (_commModule) {
                html += "<p><strong>Pending Messages:</strong> " + String(_pendingMessages.size()) + "</p>";
            }
            html += "</div>";

            // Communication Statistics (only if available)
            if (_commModule) {
                html += "<div class='stats'>";
                html += "<div class='stat-card'>";
                html += "<h3>📤 Messages Sent</h3>";
                html += "<p style='font-size: 2em; margin: 0;'>" + String(_commStats.totalMessagesSent) + "</p>";
                html += "</div>";
                html += "<div class='stat-card'>";
                html += "<h3>📥 Messages Received</h3>";
                html += "<p style='font-size: 2em; margin: 0;'>" + String(_commStats.totalMessagesReceived) + "</p>";
                html += "</div>";
                html += "<div class='stat-card'>";
                html += "<h3>✅ Success Rate</h3>";
                html += "<p style='font-size: 2em; margin: 0;'>" + String(_commStats.messageSuccessRate, 1) + "%</p>";
                html += "</div>";
                html += "<div class='stat-card'>";
                html += "<h3>📡 Known Nodes</h3>";
                html += "<p style='font-size: 2em; margin: 0;'>" + String(_nodeStatus.size()) + "</p>";
                html += "</div>";
                html += "</div>";
            }

            // Control Panel
            html += "<h2>🎛️ Festival Control Panel</h2>";

            if (_commModule) {
                // Global Quick Actions
                html += "<h3>🌐 Global Controls (All Nodes)</h3>";
                html += "<div style='margin: 10px 0; padding: 10px; background: #e9ecef; border-radius: 5px;'>";
                html += "<div style='margin: 10px 0;'>";
                html += "<input type='number' id='volumeInput' placeholder='Volume (0-15)' min='0' max='15' style='padding: 8px; margin-right: 10px;'>";
                html += "<button class='button' onclick=\"setAllVolume()\">🔊 Set All Volume</button>";
                html += "</div>";
                html += "<div style='margin: 10px 0;'>";
                html += "<input type='number' id='freqInput' placeholder='Frequency (e.g. 8850 for 88.5MHz)' min='8750' max='10800' style='padding: 8px; margin-right: 10px;'>";
                html += "<button class='button' onclick=\"setAllFrequency()\">📻 Set All Frequency</button>";
                html += "</div>";
                html += "<button class='button' onclick=\"setAllMute(true)\">🔇 Mute All Speakers</button>";
                html += "<button class='button' onclick=\"setAllMute(false)\">🔊 Unmute All Speakers</button>";
                html += "<button class='button' onclick=\"setAllRelay(true)\">🔌 Power On All</button>";
                html += "<button class='button' onclick=\"setAllRelay(false)\">⚡ Power Off All</button>";
                html += "<br><br>";
                html += "<button class='button' onclick=\"discoverNodes()\">🔍 Find All Nodes</button>";
                html += "<button class='button' onclick=\"location.reload()\">🔄 Refresh Status</button>";
                html += "</div>";
            } else {
                html += "<div class='status warning'>";
                html += "<p><strong>⚠️  Communication Disabled</strong></p>";
                html += "<p>Node control functions are not available. This can happen if:</p>";
                html += "<ul>";
                html += "<li>EBYTE 433MHz module is not connected properly</li>";
                html += "<li>Hardware issues with the communication module</li>";
                html += "<li>System is running in recovery mode</li>";
                html += "</ul>";
                html += "<p>The web interface and WiFi are working normally.</p>";
                html += "</div>";
            }

            // Individual Node Controls (NEW FEATURE)
            if (!_nodeStatus.empty() && _commModule) {
                html += "<h2>📡 Individual Node Controls</h2>";
                html += "<div style='display: grid; grid-template-columns: repeat(auto-fill, minmax(350px, 1fr)); gap: 15px;'>";

                for (const auto& pair : _nodeStatus) {
                    const NodeStatus& status = pair.second;
                    String nodeStatusClass = status.isConnected ? "good" : "error";

                    html += "<div class='status " + nodeStatusClass + "'>";
                    html += "<h3>🎪 Node " + String(status.nodeId) + " Controls</h3>";
                    html += "<p><strong>Status:</strong> " + String(status.isConnected ? "✅ Online" : "❌ Offline") + "</p>";

                    if (status.isConnected) {
                        html += "<p><strong>📻 Current:</strong> " + String(status.frequency / 100.0, 1) + " MHz, Vol " + String(status.volume) + "/15" + String(status.muted ? " (Muted)" : "") + ", Power " + String(status.relayState ? "ON" : "OFF") + "</p>";

                        // Individual node controls
                        html += "<div class='node-controls'>";

                        // Volume control
                        html += "<div class='control-row'>";
                        html += "<span>🔊 Volume:</span>";
                        html += "<input type='number' id='vol_" + String(status.nodeId) + "' class='control-input' min='0' max='15' value='" + String(status.volume) + "'>";
                        html += "<button class='button button-small' onclick=\"setNodeVolume(" + String(status.nodeId) + ")\">Set</button>";
                        html += "<button class='button button-small' onclick=\"setNodeMute(" + String(status.nodeId) + ", true)\">🔇</button>";
                        html += "<button class='button button-small' onclick=\"setNodeMute(" + String(status.nodeId) + ", false)\">🔊</button>";
                        html += "</div>";

                        // Frequency control
                        html += "<div class='control-row'>";
                        html += "<span>📻 Freq:</span>";
                        html += "<input type='number' id='freq_" + String(status.nodeId) + "' class='control-input' min='8750' max='10800' value='" + String(status.frequency) + "' style='width: 80px;'>";
                        html += "<button class='button button-small' onclick=\"setNodeFrequency(" + String(status.nodeId) + ")\">Set</button>";
                        html += "<span style='font-size: 10px; color: #666;'>MHz*10</span>";
                        html += "</div>";

                        // Relay control
                        html += "<div class='control-row'>";
                        html += "<span>🔌 Power:</span>";
                        html += "<button class='button button-small' onclick=\"setNodeRelay(" + String(status.nodeId) + ", true)\">ON</button>";
                        html += "<button class='button button-small' onclick=\"setNodeRelay(" + String(status.nodeId) + ", false)\">OFF</button>";
                        html += "<button class='button button-small' onclick=\"resetNode(" + String(status.nodeId) + ")\">🔄 Reset</button>";
                        html += "</div>";

                        html += "</div>";

                        if (status.hasSensors) {
                            html += "<p><strong>🌡️ Environment:</strong> " + String(status.temperature, 1) + "°C, " + String(status.humidity, 1) + "%</p>";
                        }
                    }
                    html += "</div>";
                }
                html += "</div>";
            } else if (_commModule) {
                html += "<div class='status warning'>";
                html += "<h2>🔍 No Nodes Found</h2>";
                html += "<p>Click 'Find All Nodes' to discover festival speaker nodes.</p>";
                html += "</div>";
            }

            html += "<div style='margin-top: 40px; padding: 10px; background: #f8f9fa; border-radius: 5px;'>";
            html += "<small><strong>SzögFM Festival Audio Distribution System</strong><br>";
            html += "Web interface updates automatically. For API access, see /status endpoint.</small>";
            html += "</div>";

            html += "</div>";

            // JavaScript for functionality
            if (_commModule) {
                html += "<script>";

                // Global controls
                html += "function setAllVolume() {";
                html += "  const vol = document.getElementById('volumeInput').value;";
                html += "  if (vol >= 0 && vol <= 15) {";
                html += "    fetch('/set_volume?node_id=0&volume=' + vol, {method: 'POST'})";
                html += "      .then(() => { alert('Volume command sent to all nodes'); setTimeout(() => location.reload(), 1000); });";
                html += "  } else alert('Volume must be 0-15');";
                html += "}";

                html += "function setAllFrequency() {";
                html += "  const freq = document.getElementById('freqInput').value;";
                html += "  if (freq >= 8750 && freq <= 10800) {";
                html += "    fetch('/set_frequency?node_id=0&frequency=' + freq, {method: 'POST'})";
                html += "      .then(() => { alert('Frequency command sent to all nodes'); setTimeout(() => location.reload(), 1000); });";
                html += "  } else alert('Frequency must be 8750-10800 (e.g. 8850 for 88.5MHz)');";
                html += "}";

                html += "function setAllMute(mute) {";
                html += "  fetch('/set_mute?node_id=0&mute=' + (mute ? '1' : '0'), {method: 'POST'})";
                html += "    .then(() => { alert((mute ? 'Mute' : 'Unmute') + ' command sent'); setTimeout(() => location.reload(), 1000); });";
                html += "}";

                html += "function setAllRelay(state) {";
                html += "  fetch('/set_relay?node_id=0&state=' + (state ? '1' : '0'), {method: 'POST'})";
                html += "    .then(() => { alert('Power ' + (state ? 'ON' : 'OFF') + ' command sent'); setTimeout(() => location.reload(), 1000); });";
                html += "}";

                // Individual node controls (NEW FUNCTIONS)
                html += "function setNodeVolume(nodeId) {";
                html += "  const vol = document.getElementById('vol_' + nodeId).value;";
                html += "  if (vol >= 0 && vol <= 15) {";
                html += "    fetch('/set_volume?node_id=' + nodeId + '&volume=' + vol, {method: 'POST'})";
                html += "      .then(() => { alert('Volume ' + vol + ' sent to node ' + nodeId); setTimeout(() => location.reload(), 1000); });";
                html += "  } else alert('Volume must be 0-15');";
                html += "}";

                html += "function setNodeFrequency(nodeId) {";
                html += "  const freq = document.getElementById('freq_' + nodeId).value;";
                html += "  if (freq >= 8750 && freq <= 10800) {";
                html += "    fetch('/set_frequency?node_id=' + nodeId + '&frequency=' + freq, {method: 'POST'})";
                html += "      .then(() => { alert('Frequency ' + (freq/100).toFixed(1) + ' MHz sent to node ' + nodeId); setTimeout(() => location.reload(), 1000); });";
                html += "  } else alert('Frequency must be 8750-10800');";
                html += "}";

                html += "function setNodeMute(nodeId, mute) {";
                html += "  fetch('/set_mute?node_id=' + nodeId + '&mute=' + (mute ? '1' : '0'), {method: 'POST'})";
                html += "    .then(() => { alert('Node ' + nodeId + ' ' + (mute ? 'muted' : 'unmuted')); setTimeout(() => location.reload(), 1000); });";
                html += "}";

                html += "function setNodeRelay(nodeId, state) {";
                html += "  fetch('/set_relay?node_id=' + nodeId + '&state=' + (state ? '1' : '0'), {method: 'POST'})";
                html += "    .then(() => { alert('Node ' + nodeId + ' power ' + (state ? 'ON' : 'OFF')); setTimeout(() => location.reload(), 1000); });";
                html += "}";

                html += "function resetNode(nodeId) {";
                html += "  if (confirm('Reset node ' + nodeId + '? This will restart the node.')) {";
                html += "    fetch('/set_volume?node_id=' + nodeId + '&volume=0', {method: 'POST'})"; // Placeholder for reset
                html += "      .then(() => { alert('Reset command sent to node ' + nodeId); setTimeout(() => location.reload(), 3000); });";
                html += "  }";
                html += "}";

                // Utility functions
                html += "function discoverNodes() {";
                html += "  fetch('/discover_nodes', {method: 'POST'})";
                html += "    .then(() => { alert('Node discovery started'); setTimeout(() => location.reload(), 3000); });";
                html += "}";

                html += "</script>";
            }

            html += "</body></html>";
            return html;
        }

        void ControllerApplication::handleStatus() {
            DynamicJsonDocument doc(4096);
            doc["system"]["communication_available"] = (_commModule != nullptr);
            doc["system"]["uptime"] = millis();
            doc["system"]["free_heap"] = ESP.getFreeHeap();

            JsonArray nodesArray = doc.createNestedArray("nodes");
            for (const auto& pair : _nodeStatus) {
                const NodeStatus& status = pair.second;
                JsonObject nodeObj = nodesArray.createNestedObject();
                nodeObj["id"] = status.nodeId;
                nodeObj["connected"] = status.isConnected;
                nodeObj["frequency"] = status.frequency;
                nodeObj["volume"] = status.volume;
                nodeObj["muted"] = status.muted;
                nodeObj["relay"] = status.relayState;
                nodeObj["last_seen"] = millis() - status.lastSeenTime;

                if (status.hasSensors) {
                    nodeObj["temperature"] = status.temperature;
                    nodeObj["humidity"] = status.humidity;
                }
            }

            String jsonResponse;
            serializeJson(doc, jsonResponse);
            _webServer->send(200, "application/json", jsonResponse);
        }

        void ControllerApplication::handleDetailedStatus() {
            String response = getDetailedSystemStatus();
            _webServer->send(200, "application/json", response);
        }

        void ControllerApplication::handleSetVolume() {
            if (!_commModule) {
                _webServer->send(503, "text/plain", "Communication disabled");
                return;
            }
            if (!_webServer->hasArg("node_id") || !_webServer->hasArg("volume")) {
                _webServer->send(400, "text/plain", "Missing parameters");
                return;
            }
            uint8_t nodeId = _webServer->arg("node_id").toInt();
            uint8_t volume = _webServer->arg("volume").toInt();
            bool success = setNodeVolume(nodeId, volume);
            _webServer->send(success ? 200 : 500, "text/plain", success ? "Volume command sent" : "Failed to send command");
        }

        void ControllerApplication::handleSetFrequency() {
            if (!_commModule) {
                _webServer->send(503, "text/plain", "Communication disabled");
                return;
            }
            if (!_webServer->hasArg("node_id") || !_webServer->hasArg("frequency")) {
                _webServer->send(400, "text/plain", "Missing parameters");
                return;
            }
            uint8_t nodeId = _webServer->arg("node_id").toInt();
            uint16_t frequency = _webServer->arg("frequency").toInt();
            bool success = setNodeFrequency(nodeId, frequency);
            _webServer->send(success ? 200 : 500, "text/plain", success ? "Frequency command sent" : "Failed to send command");
        }

        void ControllerApplication::handleSetRelay() {
            if (!_commModule) {
                _webServer->send(503, "text/plain", "Communication disabled");
                return;
            }
            if (!_webServer->hasArg("node_id") || !_webServer->hasArg("state")) {
                _webServer->send(400, "text/plain", "Missing parameters");
                return;
            }
            uint8_t nodeId = _webServer->arg("node_id").toInt();
            bool state = _webServer->arg("state").toInt() != 0;
            bool success = setNodeRelayState(nodeId, state);
            _webServer->send(success ? 200 : 500, "text/plain", success ? "Relay command sent" : "Failed to send command");
        }

        void ControllerApplication::handleSetMute() {
            if (!_commModule) {
                _webServer->send(503, "text/plain", "Communication disabled");
                return;
            }
            if (!_webServer->hasArg("node_id") || !_webServer->hasArg("mute")) {
                _webServer->send(400, "text/plain", "Missing parameters");
                return;
            }
            uint8_t nodeId = _webServer->arg("node_id").toInt();
            bool mute = _webServer->arg("mute").toInt() != 0;
            bool success = setNodeMute(nodeId, mute);
            _webServer->send(success ? 200 : 500, "text/plain", success ? "Mute command sent" : "Failed to send command");
        }

        void ControllerApplication::handleDiscoverNodes() {
            if (!_commModule) {
                _webServer->send(503, "text/plain", "Communication disabled");
                return;
            }
            int discovered = discoverNodes(10); // Limit to 10 for web requests
            _webServer->send(200, "text/plain", "Discovery initiated for " + String(discovered) + " nodes");
        }

        void ControllerApplication::handleResetStats() {
            resetCommunicationStats();
            _webServer->send(200, "text/plain", "Statistics reset");
        }

        // Communication helper methods - FULLY IMPLEMENTED
        bool ControllerApplication::processMessages() {
            if (!_commModule || !_commModule->isMessageAvailable()) {
                return false;
            }

            uint8_t buffer[256];
            uint8_t senderNodeId;

            // Receive the message
            size_t bytesReceived = _commModule->receiveMessage(buffer, sizeof(buffer), senderNodeId);
            if (bytesReceived == 0) {
                return false;
            }

            Serial.printf("📥 CONTROLLER: Received %d bytes from node %d\n", bytesReceived, senderNodeId);

            // Update communication statistics
            _commStats.totalMessagesReceived++;
            unsigned long receiveTime = millis();

            // Validate minimum message size
            if (bytesReceived < sizeof(MessageHeader)) {
                Serial.printf("❌ Message too short: %d < %d bytes\n", bytesReceived, sizeof(MessageHeader));
                _commStats.totalErrors++;
                return false;
            }

            // Parse message header
            const MessageHeader* header = reinterpret_cast<const MessageHeader*>(buffer);

            // Validate checksum
            if (!header->validateChecksum()) {
                Serial.printf("❌ Invalid checksum from node %d\n", senderNodeId);
                _commStats.totalErrors++;
                return false;
            }

            // Handle message based on type
            bool handled = false;
            switch (header->type) {
                case MessageType::STATUS_RESPONSE:
                    if (bytesReceived >= sizeof(StatusMessage)) {
                        const StatusMessage* statusMsg = reinterpret_cast<const StatusMessage*>(buffer);
                        handled = handleNodeStatusMessage(*statusMsg, senderNodeId, receiveTime);
                    } else {
                        Serial.printf("❌ Status message too short from node %d\n", senderNodeId);
                    }
                    break;

                case MessageType::ACK:
                    if (bytesReceived >= sizeof(AckMessage)) {
                        const AckMessage* ackMsg = reinterpret_cast<const AckMessage*>(buffer);
                        handled = handleNodeAckMessage(*ackMsg, senderNodeId, receiveTime);
                    } else {
                        Serial.printf("❌ ACK message too short from node %d\n", senderNodeId);
                    }
                    break;

                case MessageType::ERROR:
                    Serial.printf("❌ Error message received from node %d\n", senderNodeId);
                    handled = true;
                    break;

                default:
                    Serial.printf("❓ Unknown message type %d from node %d\n",
                                  static_cast<int>(header->type), senderNodeId);
                    break;
            }

            if (handled) {
                Serial.printf("✅ Message from node %d processed successfully\n", senderNodeId);
            } else {
                _commStats.totalErrors++;
            }

            return handled;
        }

        int ControllerApplication::getConnectedNodeCount() const {
            int connectedCount = 0;
            for (const auto& pair : _nodeStatus) {
                if (pair.second.isConnected) {
                    connectedCount++;
                }
            }
            return connectedCount;
        }

        void ControllerApplication::processPendingMessages() {
            if (_pendingMessages.empty()) {
                return;
            }

            unsigned long currentTime = millis();
            auto it = _pendingMessages.begin();

            while (it != _pendingMessages.end()) {
                bool shouldRemove = false;

                // Check if message has timed out
                if (currentTime - it->sentTime > _config.messageTimeout) {
                    if (it->retryCount < _config.maxRetryCount) {
                        // Retry the message
                        Serial.printf("🔄 Retrying message to node %d (attempt %d/%d): %s\n",
                                      it->nodeId, it->retryCount + 1, _config.maxRetryCount,
                                      it->commandDescription.c_str());

                        if (_commModule->sendMessage(it->nodeId, it->messageData.data(), it->messageData.size())) {
                            it->sentTime = currentTime;
                            it->retryCount++;
                            _commStats.totalRetries++;
                        } else {
                            Serial.printf("❌ Failed to retry message to node %d\n", it->nodeId);
                            shouldRemove = true;
                        }
                    } else {
                        // Max retries exceeded
                        Serial.printf("💀 Message to node %d failed after %d retries: %s\n",
                                      it->nodeId, it->retryCount, it->commandDescription.c_str());
                        _commStats.totalTimeouts++;
                        shouldRemove = true;
                    }
                }

                if (shouldRemove) {
                    it = _pendingMessages.erase(it);
                } else {
                    ++it;
                }
            }
        }

        void ControllerApplication::updateNodeConnectionStatus() {
            unsigned long currentTime = millis();
            const unsigned long connectionTimeout = 30000; // 30 seconds

            for (auto& pair : _nodeStatus) {
                NodeStatus& status = pair.second;
                bool wasConnected = status.isConnected;

                // Check if node has timed out
                status.isConnected = (currentTime - status.lastSeenTime < connectionTimeout);

                // Log connection changes
                if (wasConnected && !status.isConnected) {
                    Serial.printf("🔗 Node %d disconnected (timeout)\n", status.nodeId);
                } else if (!wasConnected && status.isConnected) {
                    Serial.printf("🔗 Node %d reconnected\n", status.nodeId);
                }
            }
        }

        bool ControllerApplication::sendCommandMessage(uint8_t nodeId, Command command, const uint8_t* data, size_t dataLength, const String& description) {
            if (!_commModule) {
                Serial.println("❌ Cannot send command - communication module not available");
                return false;
            }

            // Create command message
            CommandMessage cmdMsg;
            cmdMsg.header.version = 1;
            cmdMsg.header.type = MessageType::COMMAND;
            cmdMsg.header.nodeId = nodeId;
            cmdMsg.header.sequenceNum = _messageSequence++;
            cmdMsg.header.payloadLength = sizeof(CommandMessage) - sizeof(MessageHeader);
            cmdMsg.header.timestamp = millis();

            cmdMsg.command = command;
            memset(cmdMsg.data, 0, sizeof(cmdMsg.data));
            if (data && dataLength > 0) {
                size_t copyLength = std::min(dataLength, sizeof(cmdMsg.data));
                memcpy(cmdMsg.data, data, copyLength);
            }

            // Set checksum
            cmdMsg.header.setChecksum();

            // Send the message
            bool success = _commModule->sendMessage(nodeId, &cmdMsg, sizeof(CommandMessage));

            if (success) {
                _commStats.totalMessagesSent++;

                // Add to pending messages for retry handling (except broadcasts)
                if (nodeId != 0) {
                    PendingMessage pending;
                    pending.sequenceNum = cmdMsg.header.sequenceNum;
                    pending.nodeId = nodeId;
                    pending.sentTime = millis();
                    pending.firstSentTime = pending.sentTime;
                    pending.retryCount = 0;
                    pending.commandDescription = description.isEmpty() ?
                                                 "Command " + String(static_cast<int>(command)) : description;

                    // Store message data for retries
                    pending.messageData.resize(sizeof(CommandMessage));
                    memcpy(pending.messageData.data(), &cmdMsg, sizeof(CommandMessage));

                    _pendingMessages.push_back(pending);
                }

                Serial.printf("📤 Command sent to node %d: %s (seq %d)\n",
                              nodeId, description.c_str(), cmdMsg.header.sequenceNum);
            } else {
                Serial.printf("❌ Failed to send command to node %d: %s\n", nodeId, description.c_str());
                _commStats.totalErrors++;
            }

            return success;
        }

        bool ControllerApplication::handleNodeMessage(const void* message, size_t length, uint8_t senderNodeId) {
            // This is called by processMessages() - implementation moved there
            return true;
        }

        bool ControllerApplication::handleNodeStatusMessage(const StatusMessage& statusMsg, uint8_t senderNodeId, unsigned long receiveTime) {
            Serial.printf("📊 STATUS from node %d: Freq=%.1fMHz, Vol=%d, Relay=%s, RSSI=%d, Status=%d\n",
                          senderNodeId, statusMsg.frequency / 100.0, statusMsg.volume,
                          statusMsg.relayState ? "ON" : "OFF", statusMsg.rssi, static_cast<int>(statusMsg.status));

            // Update or create node status
            NodeStatus& nodeStatus = _nodeStatus[senderNodeId];
            nodeStatus.nodeId = senderNodeId;
            nodeStatus.isConnected = true;
            nodeStatus.lastSeenTime = receiveTime;
            nodeStatus.volume = statusMsg.volume;
            nodeStatus.muted = false; // We don't have direct mute info in status message, assume unmuted for now
            nodeStatus.frequency = statusMsg.frequency;
            nodeStatus.relayState = statusMsg.relayState;
            nodeStatus.signalStrength = statusMsg.rssi;
            nodeStatus.isStereo = statusMsg.isStereo;
            nodeStatus.uptime = statusMsg.uptime;
            nodeStatus.rssi = -50; // Estimate 433MHz signal strength (we received the message so it's decent)

            // Handle sensor data if available
            if (statusMsg.temperature > -99.0f) {
                nodeStatus.temperature = statusMsg.temperature;
                nodeStatus.humidity = statusMsg.humidity;
                nodeStatus.hasSensors = true;
                Serial.printf("   🌡️  Sensors: %.1f°C, %.1f%% humidity\n", statusMsg.temperature, statusMsg.humidity);
            } else {
                nodeStatus.hasSensors = false;
            }

            // Set error message based on status (using fully qualified enum name)
            switch (statusMsg.status) {
                case szogfm::NodeStatus::OK:
                    nodeStatus.errorMessage.clear();
                    break;
                case szogfm::NodeStatus::FM_ERROR:
                    nodeStatus.errorMessage = "FM radio error";
                    break;
                case szogfm::NodeStatus::COMM_ERROR:
                    nodeStatus.errorMessage = "Communication error";
                    break;
                case szogfm::NodeStatus::LOW_POWER:
                    nodeStatus.errorMessage = "Low power warning";
                    break;
                default:
                    nodeStatus.errorMessage = "Unknown error";
                    break;
            }

            // Update performance metrics
            if (nodeStatus.totalCommands == 0) {
                // First time seeing this node
                Serial.printf("🆕 New node discovered: Node %d\n", senderNodeId);
                nodeStatus.totalCommands = 1;
                nodeStatus.successfulCommands = 1;
            } else {
                nodeStatus.successfulCommands++;
            }

            if (nodeStatus.totalCommands > 0) {
                nodeStatus.commandSuccessRate = (nodeStatus.successfulCommands * 100.0f) / nodeStatus.totalCommands;
            }

            Serial.printf("✅ Node %d status updated successfully (uptime: %.1f min)\n",
                          senderNodeId, statusMsg.uptime / 60000.0);

            return true;
        }

        bool ControllerApplication::handleNodeAckMessage(const AckMessage& ackMsg, uint8_t senderNodeId, unsigned long receiveTime) {
            Serial.printf("📨 ACK from node %d: seq=%d, success=%s\n",
                          senderNodeId, ackMsg.acknowledgedSeq, ackMsg.success ? "YES" : "NO");

            // Find and remove the corresponding pending message
            auto it = std::find_if(_pendingMessages.begin(), _pendingMessages.end(),
                                   [&ackMsg, senderNodeId](const PendingMessage& msg) {
                                       return msg.sequenceNum == ackMsg.acknowledgedSeq && msg.nodeId == senderNodeId;
                                   });

            if (it != _pendingMessages.end()) {
                // Calculate response time
                unsigned long responseTime = receiveTime - it->firstSentTime;
                _commStats.totalResponseTime += responseTime;

                if (responseTime > _commStats.peakResponseTime) {
                    _commStats.peakResponseTime = responseTime;
                }
                if (responseTime < _commStats.minimumResponseTime) {
                    _commStats.minimumResponseTime = responseTime;
                }

                // Update node performance metrics
                if (_nodeStatus.find(senderNodeId) != _nodeStatus.end()) {
                    NodeStatus& nodeStatus = _nodeStatus[senderNodeId];
                    nodeStatus.totalCommands++;
                    if (ackMsg.success) {
                        nodeStatus.successfulCommands++;
                    } else {
                        nodeStatus.failedCommands++;
                    }
                    nodeStatus.totalResponseTime += responseTime;
                    nodeStatus.averageResponseTime = nodeStatus.totalResponseTime / nodeStatus.totalCommands;
                    nodeStatus.commandSuccessRate = (nodeStatus.successfulCommands * 100.0f) / nodeStatus.totalCommands;
                    nodeStatus.lastCommandSuccess = ackMsg.success;
                }

                Serial.printf("✅ Acknowledged command completed in %lu ms\n", responseTime);
                _pendingMessages.erase(it);
            } else {
                Serial.printf("⚠️  ACK for unknown message sequence %d from node %d\n",
                              ackMsg.acknowledgedSeq, senderNodeId);
            }

            // Update communication success rate
            if (_commStats.totalMessagesSent > 0) {
                _commStats.messageSuccessRate = ((_commStats.totalMessagesReceived - _commStats.totalErrors) * 100.0f) / _commStats.totalMessagesSent;
            }

            return true;
        }

    } // namespace controller
} // namespace szogfm