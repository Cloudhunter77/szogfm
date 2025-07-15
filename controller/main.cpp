#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include "ControllerApplication.h"

// Create controller application instance
szogfm::controller::ControllerApplication controllerApp;

// Watchdog timer variables
hw_timer_t *watchdogTimer = NULL;
volatile bool watchdogTriggered = false;

// Boot counter for debugging boot loops
RTC_DATA_ATTR int bootCount = 0;

/**
 * Watchdog timer interrupt handler
 * This will trigger if the main loop becomes unresponsive
 */
void IRAM_ATTR watchdogISR() {
    watchdogTriggered = true;
}

/**
 * Initialize watchdog timer
 * 60 second timeout for controller (longer than nodes due to web traffic)
 */
void initWatchdog() {
    Serial.println("🐕 Initializing controller watchdog timer (60 second timeout)");
    watchdogTimer = timerBegin(0, 80, true);  // Timer 0, prescaler 80 (1MHz), count up
    timerAttachInterrupt(watchdogTimer, &watchdogISR, true);  // Edge triggered
    timerAlarmWrite(watchdogTimer, 60000000, true);  // 60 seconds in microseconds
    timerAlarmEnable(watchdogTimer);
    Serial.println("✅ Watchdog timer enabled");
}

/**
 * Reset watchdog timer - call this regularly in main loop
 */
void feedWatchdog() {
    if (watchdogTimer) {
        timerWrite(watchdogTimer, 0);  // Reset timer to 0
    }
}

/**
 * Safe system restart with proper cleanup
 */
void safeRestart(const String& reason) {
    Serial.println("\n🔄 CONTROLLER SAFE RESTART INITIATED");
    Serial.println("Reason: " + reason);
    Serial.printf("Boot count will be: %d\n", bootCount + 1);

    // Disable watchdog to prevent it triggering during restart
    if (watchdogTimer) {
        timerEnd(watchdogTimer);
    }

    // Stop WiFi services gracefully
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);

    // Give time for cleanup
    delay(3000);

    // Restart
    ESP.restart();
}

/**
 * Print comprehensive system status
 */
void printSystemStatus() {
    Serial.println("\n📊 CONTROLLER SYSTEM STATUS:");
    Serial.printf("   🆔 Chip ID: %012llX\n", ESP.getEfuseMac());
    Serial.printf("   ⏰ Uptime: %lu minutes\n", millis() / 60000);
    Serial.printf("   💾 Free heap: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("   🔢 Boot count: %d\n", bootCount);
    Serial.printf("   🔧 CPU freq: %d MHz\n", getCpuFrequencyMhz());
    Serial.printf("   🌡️  Temperature: %.1f°C\n", temperatureRead());

    // WiFi status
    if (WiFi.getMode() == WIFI_AP) {
        Serial.printf("   📡 WiFi mode: Access Point\n");
        Serial.printf("   📶 AP IP: %s\n", WiFi.softAPIP().toString().c_str());
        Serial.printf("   👥 Connected stations: %d\n", WiFi.softAPgetStationNum());
    } else if (WiFi.getMode() == WIFI_STA && WiFi.status() == WL_CONNECTED) {
        Serial.printf("   📡 WiFi mode: Station (connected)\n");
        Serial.printf("   📶 Station IP: %s\n", WiFi.localIP().toString().c_str());
        Serial.printf("   📊 RSSI: %d dBm\n", WiFi.RSSI());
    } else {
        Serial.printf("   📡 WiFi mode: Disabled or disconnected\n");
    }

    // Controller-specific status
    String detailedStatus = controllerApp.getDetailedSystemStatus();
    Serial.println("\n🎛️  Controller Application Status:");
    Serial.println(detailedStatus);
}

void setup() {
    // Increment boot counter
    bootCount++;

    // Initialize serial communication for debugging
    Serial.begin(115200);

    // Wait for serial port to connect with timeout
    unsigned long serialStart = millis();
    while (!Serial && millis() - serialStart < 3000) {
        delay(10);
    }

    // Check for boot loop (more than 3 reboots is suspicious for controller)
    if (bootCount > 3) {
        Serial.println("\n🚨 CONTROLLER BOOT LOOP DETECTED!");
        Serial.printf("Boot count: %d (too many reboots)\n", bootCount);
        Serial.println("Entering emergency recovery mode...");

        // Reset boot counter
        bootCount = 0;

        // Emergency recovery mode - minimal WiFi AP only
        Serial.println("🆘 Starting emergency recovery WiFi AP...");
        WiFi.softAP("SzogFM_EMERGENCY", "emergency123");
        delay(2000);
        Serial.printf("🆘 Emergency AP started: %s\n", WiFi.softAPIP().toString().c_str());
        Serial.println("🌐 Connect to 'SzogFM_EMERGENCY' and go to http://192.168.4.1");

        // Simple emergency web server
        WiFiServer emergencyServer(80);
        emergencyServer.begin();

        pinMode(2, OUTPUT);
        while (true) {
            // Blink LED
            digitalWrite(2, HIGH);
            delay(500);
            digitalWrite(2, LOW);
            delay(500);

            // Handle emergency web requests
            WiFiClient client = emergencyServer.available();
            if (client) {
                String request = client.readStringUntil('\r');
                client.flush();

                String response = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n";
                response += "<!DOCTYPE html><html><head><title>SzogFM Emergency Recovery</title></head><body>";
                response += "<h1>🆘 SzogFM Controller Emergency Recovery</h1>";
                response += "<p><strong>Status:</strong> Boot loop detected, system in recovery mode</p>";
                response += "<p><strong>Boot count was:</strong> " + String(bootCount) + "</p>";
                response += "<p><strong>Free memory:</strong> " + String(ESP.getFreeHeap()) + " bytes</p>";
                response += "<p><strong>Uptime:</strong> " + String(millis() / 1000) + " seconds</p>";
                response += "<h2>Recovery Actions:</h2>";
                response += "<p>1. Check power supply (use 5V/3A minimum, not USB)</p>";
                response += "<p>2. Verify EBYTE 433MHz module connections</p>";
                response += "<p>3. Upload corrected firmware</p>";
                response += "<p>4. Check serial monitor for error details</p>";
                response += "<p><em>System is stable in recovery mode</em></p>";
                response += "</body></html>";

                client.print(response);
                client.stop();

                Serial.println("📱 Emergency web page served");
            }

            // Handle serial commands
            if (Serial.available()) {
                String input = Serial.readStringUntil('\n');
                input.trim();
                if (input == "reset") {
                    Serial.println("Manual reset from emergency mode");
                    ESP.restart();
                } else if (input == "status") {
                    printSystemStatus();
                } else if (input == "help") {
                    Serial.println("Emergency mode commands: reset, status, help");
                }
            }
        }
    }

    // Normal startup
    Serial.println("\n\n===========================================");
    Serial.println("🎵 SzögFM Controller Application Starting (FIXED VERSION) 🎵");
    Serial.println("===========================================\n");
    Serial.printf("⏰ Boot time: %lu ms\n", millis());
    Serial.printf("🔢 Boot count: %d\n", bootCount);
    Serial.printf("🔧 ESP32 Chip ID: %012llX\n", ESP.getEfuseMac());
    Serial.printf("💾 Free heap: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("🔧 CPU Frequency: %d MHz\n", getCpuFrequencyMhz());

    // Print critical fix information
    Serial.println("\n🔧 CRITICAL FIXES APPLIED:");
    Serial.println("   ✅ Fixed node command filtering (relay commands work correctly)");
    Serial.println("   ✅ Enhanced web interface with individual node controls");
    Serial.println("   ✅ Improved error handling and recovery");
    Serial.println("   ✅ Watchdog timer for system stability");
    Serial.println("   ✅ Boot loop detection and emergency recovery");
    Serial.println("   ✅ Fixed reset function in web interface");

    // Initialize watchdog timer
    initWatchdog();

    // Initialize the controller application with comprehensive error handling
    Serial.println("\n🚀 Initializing controller application...");
    bool initSuccess = false;

    try {
        initSuccess = controllerApp.initialize();
    } catch (const std::exception& e) {
        Serial.println("❌ Exception during initialization: " + String(e.what()));
        initSuccess = false;
    } catch (...) {
        Serial.println("❌ Unknown exception during initialization");
        initSuccess = false;
    }

    if (!initSuccess) {
        Serial.println("❌ Failed to initialize controller application!");
        Serial.println("🔄 System will continue in limited mode...");
        Serial.println("\n🛠️  TROUBLESHOOTING STEPS:");
        Serial.println("1. Check WiFi credentials and network availability");
        Serial.println("2. Verify EBYTE 433MHz module connections (pins M0=4, M1=32, AUX=33)");
        Serial.println("3. Ensure 12V power supply is adequate (5V/3A minimum)");
        Serial.println("4. Check serial monitor for specific error details");
        Serial.println("5. Try connecting to fallback AP if main WiFi fails");

        // Continue in limited mode rather than rebooting
        Serial.println("📝 Continuing in limited mode - web interface may still work");
    } else {
        Serial.println("✅ Controller application initialized successfully");

        // Reset boot counter on successful initialization
        bootCount = 0;

        // Print connection information
        if (WiFi.getMode() == WIFI_AP) {
            Serial.println("📡 WiFi Access Point Mode:");
            Serial.printf("   🌐 Network: SzogFM_Controller\n");
            Serial.printf("   🔐 Password: GombaSzog24\n");
            Serial.printf("   📍 IP Address: %s\n", WiFi.softAPIP().toString().c_str());
            Serial.printf("   🌍 Web Interface: http://%s\n", WiFi.softAPIP().toString().c_str());
        } else if (WiFi.status() == WL_CONNECTED) {
            Serial.println("📡 WiFi Station Mode:");
            Serial.printf("   📍 IP Address: %s\n", WiFi.localIP().toString().c_str());
            Serial.printf("   🌍 Web Interface: http://%s\n", WiFi.localIP().toString().c_str());
        }

        // Set up mDNS for easy access
        if (MDNS.begin("szogfm-controller")) {
            Serial.println("🔗 mDNS responder started: http://szogfm-controller.local");
        }
    }

    Serial.println("\n🎛️  MANUAL CONTROLS:");
    Serial.println("   • Send 'help' via serial for debugging commands");
    Serial.println("   • Send 'status' for detailed status information");
    Serial.println("   • Send 'nodes' to list all known nodes");
    Serial.println("   • Send 'discover' to find new nodes");
    Serial.println("   • Send 'reset' to manually restart controller");

    Serial.println("\n" + String('=', 50));
    Serial.println("✅ Setup complete - Controller ready for festival operation");
    Serial.println(String('=', 50));
}

void loop() {
    static unsigned long lastWatchdogFeed = 0;
    static unsigned long lastStatusPrint = 0;
    static unsigned long lastMemoryCheck = 0;
    static uint32_t minHeap = ESP.getFreeHeap();

    unsigned long currentTime = millis();

    // Check for watchdog trigger
    if (watchdogTriggered) {
        Serial.println("\n🚨 CONTROLLER WATCHDOG TRIGGERED - System was unresponsive!");
        safeRestart("Watchdog timeout");
    }

    // Feed watchdog every 10 seconds (controller has more tolerance)
    if (currentTime - lastWatchdogFeed > 10000) {
        feedWatchdog();
        lastWatchdogFeed = currentTime;
    }

    // Memory leak detection every 60 seconds
    if (currentTime - lastMemoryCheck > 60000) {
        uint32_t currentHeap = ESP.getFreeHeap();
        if (currentHeap < minHeap) {
            minHeap = currentHeap;
            Serial.printf("⚠️  New minimum heap: %d bytes (potential memory leak?)\n", minHeap);

            // If heap drops below 100KB, restart to prevent crashes
            if (minHeap < 100000) {
                Serial.println("🚨 Critical memory shortage - restarting for stability");
                safeRestart("Low memory (" + String(minHeap) + " bytes)");
            }
        }
        lastMemoryCheck = currentTime;
    }

    // Handle serial commands for debugging
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        command.toLowerCase();

        if (command == "help") {
            Serial.println("\n📋 AVAILABLE CONTROLLER COMMANDS:");
            Serial.println("   help       - Show this help menu");
            Serial.println("   status     - Show detailed controller status");
            Serial.println("   nodes      - List all known nodes");
            Serial.println("   discover   - Discover new nodes");
            Serial.println("   reset      - Restart the controller");
            Serial.println("   memory     - Show memory information");
            Serial.println("   wifi       - Show WiFi information");
            Serial.println("   stats      - Show communication statistics");
            Serial.println("   volume <n> <v> - Set volume for node n to v (0-15)");
            Serial.println("   mute <n>   - Mute node n (or 0 for all)");
            Serial.println("   unmute <n> - Unmute node n (or 0 for all)");

        } else if (command == "status") {
            printSystemStatus();

        } else if (command == "nodes") {
            Serial.println("\n📡 KNOWN NODES:");
            std::vector<szogfm::controller::NodeStatus> nodes = controllerApp.getAllNodeStatus();
            if (nodes.empty()) {
                Serial.println("   No nodes discovered yet. Try 'discover' command.");
            } else {
                for (const auto& node : nodes) {
                    Serial.printf("   Node %d: %s, Freq=%.1fMHz, Vol=%d, Relay=%s\n",
                                  node.nodeId,
                                  node.isConnected ? "Online" : "Offline",
                                  node.frequency / 100.0,
                                  node.volume,
                                  node.relayState ? "ON" : "OFF");
                }
            }

        } else if (command == "discover") {
            Serial.println("🔍 Starting node discovery...");
            int discovered = controllerApp.discoverNodes(10);
            Serial.printf("📡 Discovery requests sent to %d nodes\n", discovered);

        } else if (command == "reset") {
            Serial.println("🔄 Manual reset requested via serial");
            safeRestart("Manual reset command");

        } else if (command == "memory") {
            Serial.println("\n💾 MEMORY INFORMATION:");
            Serial.printf("   Free heap: %d bytes\n", ESP.getFreeHeap());
            Serial.printf("   Minimum heap: %d bytes\n", minHeap);
            Serial.printf("   Largest free block: %d bytes\n", ESP.getMaxAllocHeap());
            Serial.printf("   Total heap: %d bytes\n", ESP.getHeapSize());

        } else if (command == "wifi") {
            Serial.println("\n📡 WIFI INFORMATION:");
            if (WiFi.getMode() == WIFI_AP) {
                Serial.printf("   Mode: Access Point\n");
                Serial.printf("   SSID: %s\n", WiFi.softAPSSID().c_str());
                Serial.printf("   IP: %s\n", WiFi.softAPIP().toString().c_str());
                Serial.printf("   Connected stations: %d\n", WiFi.softAPgetStationNum());
            } else if (WiFi.getMode() == WIFI_STA) {
                Serial.printf("   Mode: Station\n");
                Serial.printf("   Status: %s\n", WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
                if (WiFi.status() == WL_CONNECTED) {
                    Serial.printf("   SSID: %s\n", WiFi.SSID().c_str());
                    Serial.printf("   IP: %s\n", WiFi.localIP().toString().c_str());
                    Serial.printf("   RSSI: %d dBm\n", WiFi.RSSI());
                }
            } else {
                Serial.printf("   Mode: Disabled\n");
            }

        } else if (command == "stats") {
            Serial.println("\n📊 COMMUNICATION STATISTICS:");
            const auto& stats = controllerApp.getCommunicationStats();
            Serial.printf("   Messages sent: %lu\n", stats.totalMessagesSent);
            Serial.printf("   Messages received: %lu\n", stats.totalMessagesReceived);
            Serial.printf("   Success rate: %.1f%%\n", stats.messageSuccessRate);
            Serial.printf("   Total errors: %lu\n", stats.totalErrors);
            Serial.printf("   Total retries: %lu\n", stats.totalRetries);

        } else if (command.startsWith("volume ")) {
            // Parse volume command: "volume <node> <level>"
            int firstSpace = command.indexOf(' ');
            int secondSpace = command.indexOf(' ', firstSpace + 1);
            if (firstSpace > 0 && secondSpace > 0) {
                uint8_t nodeId = command.substring(firstSpace + 1, secondSpace).toInt();
                uint8_t volume = command.substring(secondSpace + 1).toInt();
                Serial.printf("🔊 Setting volume for node %d to %d\n", nodeId, volume);
                bool success = controllerApp.setNodeVolume(nodeId, volume);
                Serial.printf("Result: %s\n", success ? "✅ Command sent" : "❌ Failed");
            } else {
                Serial.println("❌ Usage: volume <node_id> <volume_level>");
            }

        } else if (command.startsWith("mute ")) {
            uint8_t nodeId = command.substring(5).toInt();
            Serial.printf("🔇 Muting node %d\n", nodeId);
            bool success = controllerApp.setNodeMute(nodeId, true);
            Serial.printf("Result: %s\n", success ? "✅ Command sent" : "❌ Failed");

        } else if (command.startsWith("unmute ")) {
            uint8_t nodeId = command.substring(7).toInt();
            Serial.printf("🔊 Unmuting node %d\n", nodeId);
            bool success = controllerApp.setNodeMute(nodeId, false);
            Serial.printf("Result: %s\n", success ? "✅ Command sent" : "❌ Failed");

        } else if (command.length() > 0) {
            Serial.println("❓ Unknown command: " + command);
            Serial.println("💡 Type 'help' for available commands");
        }
    }

    // Update the controller application with error handling
    try {
        controllerApp.update();
    } catch (const std::exception& e) {
        Serial.println("❌ Exception in controller main loop: " + String(e.what()));
        // Don't restart immediately, just log and continue
    } catch (...) {
        Serial.println("❌ Unknown exception in controller main loop");
        // Don't restart immediately, just log and continue
    }

    // Brief status update every 5 minutes
    if (currentTime - lastStatusPrint > 300000) {
        int connectedNodes = controllerApp.getConnectedNodeCount();
        Serial.printf("💓 Controller heartbeat - Uptime: %lu min, Heap: %d bytes, Nodes: %d connected\n",
                      currentTime / 60000, ESP.getFreeHeap(), connectedNodes);

        // Show WiFi client count if in AP mode
        if (WiFi.getMode() == WIFI_AP) {
            Serial.printf("📱 WiFi clients connected: %d\n", WiFi.softAPgetStationNum());
        }

        lastStatusPrint = currentTime;
    }

    // Small delay to prevent CPU hogging and allow other tasks
    delay(10);
}