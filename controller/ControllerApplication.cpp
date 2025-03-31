#include "ControllerApplication.h"
#include <ArduinoJson.h>

namespace szogfm {
    namespace controller {

        ControllerApplication::ControllerApplication()
                : _initialized(false), _messageSequence(0),
                  _lastStatusRequestTime(0), _lastWebUpdateTime(0),

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
        }

        bool ControllerApplication::initialize() {
            Serial.println("Initializing ControllerApplication...");

            // Initialize communication module with more debug output
            Serial.println("Creating communication module...");
            _commModule = new communication::EbyteCommModule(&Serial2, _pinM0, _pinM1, _pinAUX);

            Serial.println("Initializing communication module...");
            if (!_commModule->initialize()) {
                Serial.println("Failed to initialize communication module");
                Serial.println("Last error: " + _commModule->getLastError());
                return false;
            }
            Serial.println("Communication module initialized successfully");

            // Configure communication module
            Serial.println("Configuring communication module...");
            // Use the proper constants for air data rate and UART baud rate
            // Air data rate: 1 = 2.4k baud (AIR_2K4)
            // UART baud rate: 3 = 9600 baud (UART_9600)
            // Power level: 0 = 20dBm (max power)
            if (!_commModule->configure(0x1A, 0x1234,
                                        communication::AIR_2K4, communication::UART_9600, 0)) {
                Serial.println("Failed to configure communication module");
                Serial.println("Last error: " + _commModule->getLastError());
                // Don't return false here - we'll continue anyway as the module may still work
                Serial.println("Continuing despite configuration issues...");
            } else {
                Serial.println("Communication module configured successfully");
            }

            // Initialize WiFi and web server
            Serial.println("Initializing web server...");
            if (!initializeWebServer()) {
                Serial.println("Failed to initialize web server");
                return false;
            }
            Serial.println("Web server initialized successfully");

            _initialized = true;

            Serial.println("ControllerApplication initialized successfully");
            return true;
        }

        void ControllerApplication::update() {
            if (!_initialized) {
                return;
            }

            // Process any received messages
            processMessages();

            // Process any pending messages that need to be retried
            processPendingMessages();

            // Update node connection status
            updateNodeConnectionStatus();

            // Check if we should request status updates from all nodes
            unsigned long currentTime = millis();
            if (currentTime - _lastStatusRequestTime > STATUS_REQUEST_INTERVAL) {
                requestAllNodeStatus();
                _lastStatusRequestTime = currentTime;
            }

            // Handle web server requests
            _webServer->handleClient();

            // Update communication module
            _commModule->update();
        }

        bool ControllerApplication::setNodeVolume(uint8_t nodeId, uint8_t volume) {
            // Validate volume range
            if (volume > 15) {
                volume = 15;
            }

            // Send command to set volume
            return sendCommandMessage(nodeId, Command::SET_VOLUME, &volume, 1);
        }

        bool ControllerApplication::setNodeFrequency(uint8_t nodeId, uint16_t frequency) {
            // Validate frequency range
            if (frequency < 8750 || frequency > 10800) {
                return false;
            }

            // Prepare data (little-endian: low byte first, high byte second)
            uint8_t data[2];
            data[0] = frequency & 0xFF;
            data[1] = (frequency >> 8) & 0xFF;

            // Send command to set frequency
            return sendCommandMessage(nodeId, Command::SET_FREQUENCY, data, 2);
        }

        bool ControllerApplication::setNodeRelayState(uint8_t nodeId, bool state) {
            // Prepare data
            uint8_t data = state ? 1 : 0;

            // Send command to set relay state
            return sendCommandMessage(nodeId, Command::TOGGLE_RELAY, &data, 1);
        }

        bool ControllerApplication::setNodeMute(uint8_t nodeId, bool mute) {
            // Send command to set mute state
            Command cmd = mute ? Command::MUTE : Command::UNMUTE;
            return sendCommandMessage(nodeId, cmd, nullptr, 0);
        }

        bool ControllerApplication::setAllNodesVolume(uint8_t volume) {
            // Send command to all nodes (node ID 0 = broadcast)
            return setNodeVolume(0, volume);
        }

        bool ControllerApplication::setAllNodesFrequency(uint16_t frequency) {
            // Send command to all nodes
            return setNodeFrequency(0, frequency);
        }

        bool ControllerApplication::setAllNodesRelayState(bool state) {
            // Send command to all nodes
            return setNodeRelayState(0, state);
        }

        bool ControllerApplication::setAllNodesMute(bool mute) {
            // Send command to all nodes
            return setNodeMute(0, mute);
        }

        const NodeStatus* ControllerApplication::getNodeStatus(uint8_t nodeId) const {
            // Find node status
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
            // Send command to request status
            return sendCommandMessage(nodeId, Command::GET_STATUS, nullptr, 0);
        }

        bool ControllerApplication::requestAllNodeStatus() {
            // Send status request to all nodes
            return requestNodeStatus(0);
        }

        bool ControllerApplication::resetNode(uint8_t nodeId) {
            // Send command to reset node
            return sendCommandMessage(nodeId, Command::RESET, nullptr, 0);
        }

        bool ControllerApplication::handleNodeMessage(const void* message, size_t length, uint8_t senderNodeId) {
            if (!message || length < sizeof(MessageHeader)) {
                return false;
            }

            // Parse message header
            const MessageHeader* header = static_cast<const MessageHeader*>(message);

            // Validate checksum
            if (!header->validateChecksum()) {
                Serial.println("Received message has invalid checksum");
                return false;
            }

            // Process message based on type
            switch (header->type) {
                case MessageType::STATUS_RESPONSE:
                    if (length >= sizeof(StatusMessage)) {
                        const StatusMessage* status = static_cast<const StatusMessage*>(message);
                        return handleStatusMessage(*status);
                    }
                    break;

                case MessageType::ACK:
                    if (length >= sizeof(AckMessage)) {
                        const AckMessage* ack = static_cast<const AckMessage*>(message);
                        return handleAckMessage(*ack);
                    }
                    break;

                case MessageType::ERROR:
                    return handleErrorMessage(*header);

                default:
                    Serial.println("Received unknown message type");
                    break;
            }

            return false;
        }

        bool ControllerApplication::initializeWebServer() {
            // Initialize WiFi
            if (_wifiApMode) {
                // Set up WiFi access point
                WiFi.softAP(_wifiSsid, _wifiPassword);
                Serial.print("AP IP address: ");
                Serial.println(WiFi.softAPIP());
            } else {
                // Connect to WiFi network
                WiFi.begin(_wifiSsid, _wifiPassword);

                // Wait for connection (with timeout)
                unsigned long startTime = millis();
                while (WiFi.status() != WL_CONNECTED && millis() - startTime < 30000) {
                    delay(500);
                    Serial.print(".");
                }

                if (WiFi.status() != WL_CONNECTED) {
                    Serial.println("Failed to connect to WiFi");
                    return false;
                }

                Serial.println("");
                Serial.print("Connected to ");
                Serial.println(_wifiSsid);
                Serial.print("IP address: ");
                Serial.println(WiFi.localIP());
            }

            // Set up mDNS responder
            if (MDNS.begin("szogfm")) {
                Serial.println("mDNS responder started");
            }

            // Initialize web server
            _webServer = new WebServer(80);

            // Set up routes
            _webServer->on("/", HTTP_GET, [this]() { handleRoot(); });
            _webServer->on("/status", HTTP_GET, [this]() { handleStatus(); });
            _webServer->on("/set_volume", HTTP_POST, [this]() { handleSetVolume(); });
            _webServer->on("/set_frequency", HTTP_POST, [this]() { handleSetFrequency(); });
            _webServer->on("/set_relay", HTTP_POST, [this]() { handleSetRelay(); });
            _webServer->on("/set_mute", HTTP_POST, [this]() { handleSetMute(); });
            _webServer->on("/reset_node", HTTP_POST, [this]() { handleResetNode(); });

            // Start server
            _webServer->begin();
            Serial.println("HTTP server started");

            return true;
        }

        bool ControllerApplication::processMessages() {
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

            // Handle the message
            return handleNodeMessage(buffer, bytesReceived, senderNodeId);
        }

        void ControllerApplication::processPendingMessages() {
            unsigned long currentTime = millis();

            // Check for messages that need to be retried
            for (auto it = _pendingMessages.begin(); it != _pendingMessages.end(); ) {
                if (currentTime - it->sentTime > MESSAGE_TIMEOUT) {
                    // Message timed out, retry if retry count is not exceeded
                    if (it->retryCount < MAX_RETRY_COUNT) {
                        // Retry sending the message
                        if (_commModule->sendMessage(it->nodeId, it->messageData.data(), it->messageData.size())) {
                            it->sentTime = currentTime;
                            it->retryCount++;
                            ++it;
                        } else {
                            // Failed to retry, remove from list
                            it = _pendingMessages.erase(it);
                        }
                    } else {
                        // Max retry count exceeded, remove from list
                        it = _pendingMessages.erase(it);
                    }
                } else {
                    ++it;
                }
            }
        }

        bool ControllerApplication::sendCommandMessage(uint8_t nodeId, Command command, const uint8_t* data, size_t dataLength) {
            if (!_initialized || !_commModule) {
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
            }

            // Compute header checksum
            cmdMsg.header.setChecksum();

            // Send message
            if (_commModule->sendMessage(nodeId, &cmdMsg, sizeof(MessageHeader) + sizeof(Command) + dataLength)) {
                // Add to pending messages list for retry handling
                PendingMessage pending;
                pending.sequenceNum = cmdMsg.header.sequenceNum;
                pending.nodeId = nodeId;
                pending.sentTime = millis();
                pending.retryCount = 0;

                // Copy message data
                pending.messageData.resize(sizeof(MessageHeader) + sizeof(Command) + dataLength);
                memcpy(pending.messageData.data(), &cmdMsg, pending.messageData.size());

                _pendingMessages.push_back(pending);

                return true;
            }

            return false;
        }

        bool ControllerApplication::handleStatusMessage(const szogfm::StatusMessage& status) {
            uint8_t nodeId = status.header.nodeId;

            // Create or update node status
            NodeStatus& nodeStatus = _nodeStatus[nodeId];

            // Update status information
            nodeStatus.nodeId = nodeId;
            nodeStatus.isConnected = true;
            nodeStatus.lastSeenTime = millis();
            nodeStatus.rssi = status.rssi;
            nodeStatus.volume = status.volume;
            nodeStatus.muted = false; // Not directly provided in status message
            nodeStatus.frequency = status.frequency;
            nodeStatus.relayState = status.relayState;
            nodeStatus.signalStrength = status.rssi;
            nodeStatus.isStereo = status.isStereo;
            nodeStatus.uptime = status.uptime;
            nodeStatus.errorMessage = ""; // No error

            // Update sensor data if available
            if (status.temperature > -50.0f && status.humidity >= 0.0f) { // Valid sensor values
                nodeStatus.temperature = status.temperature;
                nodeStatus.humidity = status.humidity;
                nodeStatus.hasSensors = true;
            } else {
                nodeStatus.hasSensors = false;
            }

            Serial.print("Updated status for node ");
            Serial.println(nodeId);

            return true;
        }

        bool ControllerApplication::handleAckMessage(const szogfm::AckMessage& ack) {
            uint16_t ackSeq = ack.acknowledgedSeq;

            // Find and remove the pending message that was acknowledged
            for (auto it = _pendingMessages.begin(); it != _pendingMessages.end(); ++it) {
                if (it->sequenceNum == ackSeq) {
                    _pendingMessages.erase(it);

                    Serial.print("Received ACK for message ");
                    Serial.println(ackSeq);

                    return true;
                }
            }

            Serial.print("Received ACK for unknown message ");
            Serial.println(ackSeq);

            return false;
        }

        bool ControllerApplication::handleErrorMessage(const MessageHeader& error) {
            Serial.print("Received error message from node ");
            Serial.println(error.nodeId);

            // Update node status to indicate error
            if (_nodeStatus.find(error.nodeId) != _nodeStatus.end()) {
                _nodeStatus[error.nodeId].errorMessage = "Error reported by node";
            }

            return true;
        }

        void ControllerApplication::updateNodeConnectionStatus() {
            unsigned long currentTime = millis();
            const unsigned long NODE_TIMEOUT = 60000; // 60 seconds

            // Check for nodes that haven't been seen recently
            for (auto& pair : _nodeStatus) {
                NodeStatus& status = pair.second;

                if (status.isConnected && currentTime - status.lastSeenTime > NODE_TIMEOUT) {
                    status.isConnected = false;

                    Serial.print("Node ");
                    Serial.print(status.nodeId);
                    Serial.println(" disconnected (timeout)");
                }
            }
        }

        void ControllerApplication::handleRoot() {
            // Serve the main HTML page
            String html = "<!DOCTYPE html><html><head>";
            html += "<title>SzogFM Controller</title>";
            html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
            html += "<style>";
            html += "body { font-family: Arial, sans-serif; margin: 0; padding: 20px; }";
            html += ".node { border: 1px solid #ccc; padding: 10px; margin-bottom: 10px; border-radius: 5px; }";
            html += ".connected { background-color: #d4edda; }";
            html += ".disconnected { background-color: #f8d7da; }";
            html += ".slider { width: 100%; }";
            html += ".controls { margin-top: 10px; }";
            html += "button { padding: 5px 10px; margin-right: 5px; }";
            html += "</style>";
            html += "<script>";

            // JavaScript to update status and control nodes
            html += "function updateStatus() {";
            html += "  fetch('/status').then(response => response.json()).then(data => {";
            html += "    const container = document.getElementById('nodes');";
            html += "    container.innerHTML = '';";
            html += "    data.nodes.forEach(node => {";
            html += "      const nodeDiv = document.createElement('div');";
            html += "      nodeDiv.className = 'node ' + (node.connected ? 'connected' : 'disconnected');";
            html += "      nodeDiv.innerHTML = `";
            html += "        <h3>Node ${node.id} - ${node.connected ? 'Connected' : 'Disconnected'}</h3>";
            html += "        <p>Frequency: ${node.frequency / 100} MHz</p>";
            html += "        <p>Volume: ${node.volume} / 15</p>";
            html += "        <p>Relay: ${node.relay ? 'ON' : 'OFF'}</p>";
            html += "        ${node.has_sensors ? `<p>Temperature: ${node.temperature.toFixed(1)}°C, Humidity: ${node.humidity.toFixed(1)}%</p>` : ''}";
            html += "        <div class='controls'>`;";

            // Add volume control
            html += "      nodeDiv.innerHTML += `<label>Volume: <input type='range' class='slider' min='0' max='15' value='${node.volume}' id='vol_${node.id}' oninput='setVolume(${node.id}, this.value)'></label><br>`;";

            // Add frequency control
            html += "      nodeDiv.innerHTML += `<label>Frequency: <input type='number' min='875' max='1080' step='0.1' value='${node.frequency / 100}' id='freq_${node.id}' onchange='setFrequency(${node.id}, this.value)'> MHz</label><br>`;";

            // Add relay control
            html += "      nodeDiv.innerHTML += `<button onclick='toggleRelay(${node.id}, ${!node.relay})'>${node.relay ? 'Turn OFF' : 'Turn ON'} Relay</button>`;";

            // Add mute control
            html += "      nodeDiv.innerHTML += `<button onclick='toggleMute(${node.id}, ${!node.muted})'>${node.muted ? 'Unmute' : 'Mute'}</button>`;";

            // Add reset button
            html += "      nodeDiv.innerHTML += `<button onclick='resetNode(${node.id})'>Reset Node</button>`;";

            html += "      nodeDiv.innerHTML += `</div>`;";
            html += "      container.appendChild(nodeDiv);";
            html += "    });";
            html += "  });";
            html += "}";

            // Function to set volume
            html += "function setVolume(nodeId, volume) {";
            html += "  fetch('/set_volume', {";
            html += "    method: 'POST',";
            html += "    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },";
            html += "    body: `node_id=${nodeId}&volume=${volume}`";
            html += "  });";
            html += "}";

            // Function to set frequency
            html += "function setFrequency(nodeId, frequency) {";
            html += "  const freq = Math.round(parseFloat(frequency) * 100);";
            html += "  fetch('/set_frequency', {";
            html += "    method: 'POST',";
            html += "    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },";
            html += "    body: `node_id=${nodeId}&frequency=${freq}`";
            html += "  });";
            html += "}";

            // Function to toggle relay
            html += "function toggleRelay(nodeId, state) {";
            html += "  fetch('/set_relay', {";
            html += "    method: 'POST',";
            html += "    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },";
            html += "    body: `node_id=${nodeId}&state=${state ? 1 : 0}`";
            html += "  }).then(() => updateStatus());";
            html += "}";

            // Function to toggle mute
            html += "function toggleMute(nodeId, mute) {";
            html += "  fetch('/set_mute', {";
            html += "    method: 'POST',";
            html += "    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },";
            html += "    body: `node_id=${nodeId}&mute=${mute ? 1 : 0}`";
            html += "  }).then(() => updateStatus());";
            html += "}";

            // Function to reset node
            html += "function resetNode(nodeId) {";
            html += "  if (confirm('Are you sure you want to reset node ' + nodeId + '?')) {";
            html += "    fetch('/reset_node', {";
            html += "      method: 'POST',";
            html += "      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },";
            html += "      body: `node_id=${nodeId}`";
            html += "    });";
            html += "  }";
            html += "}";

            // Function to set all nodes
            html += "function setAllVolume() {";
            html += "  const volume = document.getElementById('all_volume').value;";
            html += "  fetch('/set_volume', {";
            html += "    method: 'POST',";
            html += "    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },";
            html += "    body: `node_id=0&volume=${volume}`";
            html += "  });";
            html += "}";

            html += "function setAllFrequency() {";
            html += "  const frequency = Math.round(parseFloat(document.getElementById('all_frequency').value) * 100);";
            html += "  fetch('/set_frequency', {";
            html += "    method: 'POST',";
            html += "    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },";
            html += "    body: `node_id=0&frequency=${frequency}`";
            html += "  });";
            html += "}";

            html += "function setAllRelay(state) {";
            html += "  fetch('/set_relay', {";
            html += "    method: 'POST',";
            html += "    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },";
            html += "    body: `node_id=0&state=${state ? 1 : 0}`";
            html += "  }).then(() => updateStatus());";
            html += "}";

            html += "function setAllMute(mute) {";
            html += "  fetch('/set_mute', {";
            html += "    method: 'POST',";
            html += "    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },";
            html += "    body: `node_id=0&mute=${mute ? 1 : 0}`";
            html += "  }).then(() => updateStatus());";
            html += "}";

            // Auto-refresh status
            html += "setInterval(updateStatus, 2000);";
            html += "document.addEventListener('DOMContentLoaded', updateStatus);";

            html += "</script>";
            html += "</head><body>";
            html += "<h1>SzogFM Controller</h1>";

            // Global controls
            html += "<div class='node'>";
            html += "<h2>All Nodes</h2>";
            html += "<div class='controls'>";
            html += "<label>Volume: <input type='range' class='slider' min='0' max='15' value='8' id='all_volume'></label>";
            html += "<button onclick='setAllVolume()'>Set All</button><br>";
            html += "<label>Frequency: <input type='number' min='875' max='1080' step='0.1' value='88.5' id='all_frequency'> MHz</label>";
            html += "<button onclick='setAllFrequency()'>Set All</button><br>";
            html += "<button onclick='setAllRelay(true)'>All Relays ON</button>";
            html += "<button onclick='setAllRelay(false)'>All Relays OFF</button>";
            html += "<button onclick='setAllMute(true)'>Mute All</button>";
            html += "<button onclick='setAllMute(false)'>Unmute All</button>";
            html += "</div>";
            html += "</div>";

            // Node container
            html += "<h2>Nodes</h2>";
            html += "<div id='nodes'></div>";

            html += "</body></html>";

            _webServer->send(200, "text/html", html);
        }

        void ControllerApplication::handleStatus() {
            // Create JSON response with node status
            DynamicJsonDocument doc(4096);
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
                nodeObj["rssi"] = status.rssi;
                nodeObj["stereo"] = status.isStereo;
                nodeObj["uptime"] = status.uptime;

                if (status.hasSensors) {
                    nodeObj["has_sensors"] = true;
                    nodeObj["temperature"] = status.temperature;
                    nodeObj["humidity"] = status.humidity;
                } else {
                    nodeObj["has_sensors"] = false;
                }

                if (!status.errorMessage.empty()) {
                    nodeObj["error"] = status.errorMessage;
                }
            }

            String jsonResponse;
            serializeJson(doc, jsonResponse);

            _webServer->send(200, "application/json", jsonResponse);
        }

        void ControllerApplication::handleSetVolume() {
            // Check parameters
            if (!_webServer->hasArg("node_id") || !_webServer->hasArg("volume")) {
                _webServer->send(400, "text/plain", "Missing parameters");
                return;
            }

            uint8_t nodeId = _webServer->arg("node_id").toInt();
            uint8_t volume = _webServer->arg("volume").toInt();

            // Set volume
            bool success = setNodeVolume(nodeId, volume);

            _webServer->send(success ? 200 : 500, "text/plain", success ? "OK" : "Failed");
        }

        void ControllerApplication::handleSetFrequency() {
            // Check parameters
            if (!_webServer->hasArg("node_id") || !_webServer->hasArg("frequency")) {
                _webServer->send(400, "text/plain", "Missing parameters");
                return;
            }

            uint8_t nodeId = _webServer->arg("node_id").toInt();
            uint16_t frequency = _webServer->arg("frequency").toInt();

            // Set frequency
            bool success = setNodeFrequency(nodeId, frequency);

            _webServer->send(success ? 200 : 500, "text/plain", success ? "OK" : "Failed");
        }

        void ControllerApplication::handleSetRelay() {
            // Check parameters
            if (!_webServer->hasArg("node_id") || !_webServer->hasArg("state")) {
                _webServer->send(400, "text/plain", "Missing parameters");
                return;
            }

            uint8_t nodeId = _webServer->arg("node_id").toInt();
            bool state = _webServer->arg("state").toInt() != 0;

            // Set relay state
            bool success = setNodeRelayState(nodeId, state);

            _webServer->send(success ? 200 : 500, "text/plain", success ? "OK" : "Failed");
        }

        void ControllerApplication::handleSetMute() {
            // Check parameters
            if (!_webServer->hasArg("node_id") || !_webServer->hasArg("mute")) {
                _webServer->send(400, "text/plain", "Missing parameters");
                return;
            }

            uint8_t nodeId = _webServer->arg("node_id").toInt();
            bool mute = _webServer->arg("mute").toInt() != 0;

            // Set mute state
            bool success = setNodeMute(nodeId, mute);

            _webServer->send(success ? 200 : 500, "text/plain", success ? "OK" : "Failed");
        }

        void ControllerApplication::handleResetNode() {
            // Check parameters
            if (!_webServer->hasArg("node_id")) {
                _webServer->send(400, "text/plain", "Missing parameters");
                return;
            }

            uint8_t nodeId = _webServer->arg("node_id").toInt();

            // Reset node
            bool success = resetNode(nodeId);

            _webServer->send(success ? 200 : 500, "text/plain", success ? "OK" : "Failed");
        }

    } // namespace controller
} // namespace szogfm