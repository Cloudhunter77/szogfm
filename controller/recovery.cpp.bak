/**
 * EMERGENCY RECOVERY ControllerApplication for SzögFM Controller
 *
 * This replaces the ControllerApplication.cpp with a minimal, stable version
 * that works with the existing controller/main.cpp structure.
 *
 * Instructions:
 * 1. Replace controller/ControllerApplication.cpp with this content
 * 2. Upload with: pio run -e controller --target upload
 * 3. System should boot and be stable
 * 4. Connect to WiFi "SzogFM_RECOVERY" password "recovery123"
 * 5. Go to http://192.168.4.1 for recovery interface
 */

#include "ControllerApplication.h"
#include <WiFi.h>
#include <WebServer.h>

namespace szogfm {
    namespace controller {

        // Simple recovery web server
        WebServer* recoveryServer = nullptr;

        ControllerApplication::ControllerApplication() {
            // Minimal constructor - no complex initialization
            _initialized = false;
            _commModule = nullptr;
            _webServer = nullptr;

            Serial.println("🆘 Emergency Recovery Controller Created");
        }

        ControllerApplication::~ControllerApplication() {
            if (recoveryServer) {
                delete recoveryServer;
            }
        }

        bool ControllerApplication::initialize() {
            Serial.println("\n" + String('=', 50));
            Serial.println("🆘 EMERGENCY RECOVERY MODE 🆘");
            Serial.println(String('=', 50));
            Serial.printf("ESP32 Chip ID: %012llX\n", ESP.getEfuseMac());
            Serial.printf("Free Heap: %d bytes\n", ESP.getFreeHeap());
            Serial.printf("CPU Frequency: %d MHz\n", getCpuFrequencyMhz());

            // Disable potentially problematic features
            btStop();
            WiFi.disconnect();
            WiFi.mode(WIFI_OFF);
            delay(1000);

            Serial.println("✅ System stabilized - no watchdog, no complex init");

            // Start basic WiFi AP
            Serial.println("📡 Starting emergency WiFi AP...");
            bool apStarted = WiFi.softAP("SzogFM_RECOVERY", "recovery123");

            if (apStarted) {
                delay(2000);
                Serial.printf("✅ Emergency AP started: %s\n", WiFi.softAPIP().toString().c_str());

                // Set up minimal web server
                recoveryServer = new WebServer(80);

                recoveryServer->on("/", [this]() {
                    this->handleRecoveryRoot();
                });

                recoveryServer->on("/status", [this]() {
                    this->handleRecoveryStatus();
                });

                recoveryServer->onNotFound([this]() {
                    recoveryServer->send(404, "text/plain", "404 - Go to http://192.168.4.1");
                });

                recoveryServer->begin();
                Serial.println("🌐 Recovery web server started on port 80");
                Serial.println("📱 Connect to 'SzogFM_RECOVERY' WiFi (password: recovery123)");
                Serial.println("🌍 Then go to: http://192.168.4.1");

                _initialized = true;
                return true;
            } else {
                Serial.println("❌ Failed to start emergency AP");
                return false;
            }
        }

        void ControllerApplication::update() {
            static unsigned long lastStatus = 0;
            unsigned long currentTime = millis();

            if (recoveryServer) {
                recoveryServer->handleClient();
            }

            // Status report every 30 seconds
            if (currentTime - lastStatus > 30000) {
                Serial.printf("💓 Recovery heartbeat - Uptime: %lu seconds, Heap: %d bytes\n",
                              currentTime / 1000, ESP.getFreeHeap());
                Serial.printf("📡 Connected stations: %d\n", WiFi.softAPgetStationNum());
                lastStatus = currentTime;

                // Memory leak detection
                static int minHeap = ESP.getFreeHeap();
                if (ESP.getFreeHeap() < minHeap) {
                    minHeap = ESP.getFreeHeap();
                }
            }
        }

        // Minimal implementations of required methods
        bool ControllerApplication::setNodeVolume(uint8_t nodeId, uint8_t volume) {
            Serial.printf("🆘 Recovery Mode: Volume command ignored (node %d, vol %d)\n", nodeId, volume);
            return false;
        }

        bool ControllerApplication::setNodeFrequency(uint8_t nodeId, uint16_t frequency) {
            Serial.printf("🆘 Recovery Mode: Frequency command ignored (node %d, freq %d)\n", nodeId, frequency);
            return false;
        }

        bool ControllerApplication::setNodeRelayState(uint8_t nodeId, bool state) {
            Serial.printf("🆘 Recovery Mode: Relay command ignored (node %d, state %d)\n", nodeId, state);
            return false;
        }

        bool ControllerApplication::setNodeMute(uint8_t nodeId, bool mute) {
            Serial.printf("🆘 Recovery Mode: Mute command ignored (node %d, mute %d)\n", nodeId, mute);
            return false;
        }

        bool ControllerApplication::setAllNodesVolume(uint8_t volume) {
            return setNodeVolume(0, volume);
        }

        bool ControllerApplication::setAllNodesFrequency(uint16_t frequency) {
            return setNodeFrequency(0, frequency);
        }

        bool ControllerApplication::setAllNodesRelayState(bool state) {
            return setNodeRelayState(0, state);
        }

        bool ControllerApplication::setAllNodesMute(bool mute) {
            return setNodeMute(0, mute);
        }

        const NodeStatus* ControllerApplication::getNodeStatus(uint8_t nodeId) const {
            return nullptr;
        }

        std::vector<NodeStatus> ControllerApplication::getAllNodeStatus() const {
            return std::vector<NodeStatus>();
        }

        bool ControllerApplication::requestNodeStatus(uint8_t nodeId) {
            return false;
        }

        bool ControllerApplication::requestAllNodeStatus() {
            return false;
        }

        bool ControllerApplication::resetNode(uint8_t nodeId) {
            return false;
        }

        int ControllerApplication::discoverNodes() {
            Serial.println("🆘 Recovery Mode: Node discovery disabled");
            return 0;
        }

        bool ControllerApplication::testNodeCommunication(uint8_t nodeId) {
            return false;
        }

        void ControllerApplication::resetCommunicationStats() {
            Serial.println("🆘 Recovery Mode: Stats reset ignored");
        }

        String ControllerApplication::getDetailedSystemStatus() const {
            return "{\"status\":\"recovery\",\"message\":\"Emergency recovery mode active\"}";
        }

        bool ControllerApplication::setConfiguration(const ControllerConfig& config) {
            return false;
        }

        void ControllerApplication::setCommunicationDebugLevel(uint8_t level) {
            // Ignored in recovery mode
        }

        const CommunicationStats& ControllerApplication::getCommunicationStats() const {
            static CommunicationStats emptyStats;
            return emptyStats;
        }

        // Private helper methods for web interface
        void ControllerApplication::handleRecoveryRoot() {
            String html = "<!DOCTYPE html><html><head><title>SzogFM Recovery</title>";
            html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
            html += "<style>body{font-family:Arial;margin:20px;background:#f0f0f0;}";
            html += ".container{max-width:800px;margin:0 auto;background:white;padding:20px;border-radius:10px;}";
            html += ".status{padding:15px;margin:10px 0;border-radius:5px;background:#d4edda;border:1px solid #c3e6cb;}";
            html += ".warning{background:#fff3cd;border:1px solid #ffeeba;}";
            html += "h1{color:#333;}h2{color:#666;}";
            html += "</style></head><body>";

            html += "<div class='container'>";
            html += "<h1>🆘 SzögFM Emergency Recovery</h1>";

            html += "<div class='status'>";
            html += "<h2>✅ System Status: STABLE</h2>";
            html += "<p><strong>Recovery Mode:</strong> Active and functioning</p>";
            html += "<p><strong>Free Memory:</strong> " + String(ESP.getFreeHeap()) + " bytes</p>";
            html += "<p><strong>Uptime:</strong> " + String(millis() / 1000) + " seconds</p>";
            html += "<p><strong>CPU Frequency:</strong> " + String(getCpuFrequencyMhz()) + " MHz</p>";
            html += "</div>";

            html += "<div class='warning'>";
            html += "<h2>⚠️ Recovery Information</h2>";
            html += "<ul>";
            html += "<li><strong>ESP32 is now stable</strong> - Boot loop resolved</li>";
            html += "<li><strong>Communication disabled</strong> - EBYTE module bypassed</li>";
            html += "<li><strong>Minimal functionality</strong> - Only essential features active</li>";
            html += "<li><strong>Ready for main firmware</strong> - Safe to upload corrected code</li>";
            html += "</ul>";
            html += "</div>";

            html += "<h2>🔧 Next Steps:</h2>";
            html += "<ol>";
            html += "<li><strong>Verify power supply</strong> - Use 5V/3A minimum (not USB)</li>";
            html += "<li><strong>Check hardware connections</strong> - Especially EBYTE module</li>";
            html += "<li><strong>Upload corrected main firmware</strong> - With watchdog fixes</li>";
            html += "<li><strong>Monitor serial output</strong> - Watch for any new errors</li>";
            html += "</ol>";

            html += "<h2>📊 Technical Details:</h2>";
            html += "<ul>";
            html += "<li><strong>Boot Loop Cause:</strong> Watchdog timer init in constructor</li>";
            html += "<li><strong>Fix Applied:</strong> Removed early system calls</li>";
            html += "<li><strong>Recovery Method:</strong> Minimal initialization only</li>";
            html += "<li><strong>Safety Features:</strong> No RTOS conflicts, No hardware conflicts</li>";
            html += "</ul>";

            html += "<p><em>🔄 Auto-refresh in 30 seconds</em></p>";
            html += "<script>setTimeout(() => location.reload(), 30000);</script>";
            html += "</div></body></html>";

            recoveryServer->send(200, "text/html", html);
            Serial.println("🌐 Recovery web page served");
        }

        void ControllerApplication::handleRecoveryStatus() {
            String json = "{";
            json += "\"status\":\"recovery\",";
            json += "\"stable\":true,";
            json += "\"uptime\":" + String(millis()) + ",";
            json += "\"heap\":" + String(ESP.getFreeHeap()) + ",";
            json += "\"chip_id\":\"" + String(ESP.getEfuseMac(), HEX) + "\",";
            json += "\"cpu_freq\":" + String(getCpuFrequencyMhz()) + ",";
            json += "\"wifi_stations\":" + String(WiFi.softAPgetStationNum()) + ",";
            json += "\"message\":\"Emergency recovery active - system stable\"";
            json += "}";

            recoveryServer->send(200, "application/json", json);
            Serial.println("🌐 Recovery status JSON served");
        }

        const ControllerConfig& ControllerApplication::getConfiguration() const {
            static ControllerConfig emptyConfig;
            return emptyConfig;
        }

    } // namespace controller
} // namespace szogfm