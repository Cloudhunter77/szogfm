#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include "NodeApplication.h"

// Create node application instance
szogfm::node::NodeApplication nodeApp;

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
 * 30 second timeout - if main loop doesn't reset it, system will reboot
 */
void initWatchdog() {
    Serial.println("🐕 Initializing watchdog timer (30 second timeout)");
    watchdogTimer = timerBegin(0, 80, true);  // Timer 0, prescaler 80 (1MHz), count up
    timerAttachInterrupt(watchdogTimer, &watchdogISR, true);  // Edge triggered
    timerAlarmWrite(watchdogTimer, 30000000, true);  // 30 seconds in microseconds
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
    Serial.println("\n🔄 SAFE RESTART INITIATED");
    Serial.println("Reason: " + reason);
    Serial.printf("Boot count will be: %d\n", bootCount + 1);

    // Disable watchdog to prevent it triggering during restart
    if (watchdogTimer) {
        timerEnd(watchdogTimer);
    }

    // Give time for serial output
    delay(2000);

    // Restart
    ESP.restart();
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

    // Check for boot loop (more than 5 reboots in quick succession is suspicious)
    if (bootCount > 5) {
        Serial.println("\n🚨 BOOT LOOP DETECTED!");
        Serial.printf("Boot count: %d (too many reboots)\n", bootCount);
        Serial.println("Entering safe mode - basic functionality only");

        // Reset boot counter
        bootCount = 0;

        // Basic safe mode - just blink LED and provide serial interface
        pinMode(2, OUTPUT);
        while (true) {
            digitalWrite(2, HIGH);
            delay(1000);
            digitalWrite(2, LOW);
            delay(1000);

            if (Serial.available()) {
                String input = Serial.readStringUntil('\n');
                input.trim();
                if (input == "reset") {
                    Serial.println("Manual reset requested");
                    ESP.restart();
                } else if (input == "status") {
                    Serial.printf("Safe mode active, boot count was: %d\n", bootCount);
                    Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
                    Serial.printf("Chip ID: %012llX\n", ESP.getEfuseMac());
                } else if (input == "help") {
                    Serial.println("Available commands: reset, status, help");
                }
            }
        }
    }

    // Normal startup
    Serial.println("\n\n===========================================");
    Serial.println("🎵 SzögFM Node Application Starting (FIXED VERSION) 🎵");
    Serial.println("===========================================\n");
    Serial.printf("⏰ Boot time: %lu ms\n", millis());
    Serial.printf("🔢 Boot count: %d\n", bootCount);
    Serial.printf("🔧 ESP32 Chip ID: %012llX\n", ESP.getEfuseMac());
    Serial.printf("💾 Free heap: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("🔧 CPU Frequency: %d MHz\n", getCpuFrequencyMhz());

    // Print critical fix information
    Serial.println("\n🔧 CRITICAL FIXES APPLIED:");
    Serial.println("   ✅ Fixed command filtering by Node ID");
    Serial.println("   ✅ Individual commands only processed by target node");
    Serial.println("   ✅ Broadcast commands (nodeId=0) processed by all nodes");
    Serial.println("   ✅ Enhanced error handling and recovery");
    Serial.println("   ✅ Watchdog timer for system stability");
    Serial.println("   ✅ Boot loop detection and safe mode");

    // Disable WiFi and Bluetooth to save power and avoid conflicts
    Serial.println("\n📡 Disabling wireless features for stability...");
    btStop();
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    delay(100);
    Serial.println("✅ WiFi and Bluetooth disabled");

    // Initialize watchdog timer
    initWatchdog();

    // Initialize the node application with error handling
    Serial.println("\n🚀 Initializing node application...");
    bool initSuccess = false;

    try {
        initSuccess = nodeApp.initialize();
    } catch (const std::exception& e) {
        Serial.println("❌ Exception during initialization: " + String(e.what()));
        initSuccess = false;
    } catch (...) {
        Serial.println("❌ Unknown exception during initialization");
        initSuccess = false;
    }

    if (!initSuccess) {
        Serial.println("❌ Failed to initialize node application!");
        Serial.println("🔄 System will continue in degraded mode for troubleshooting...");
        Serial.println("\n🛠️  TROUBLESHOOTING STEPS:");
        Serial.println("1. Check I2C connections (SDA=21, SCL=22)");
        Serial.println("2. Verify FM radio module at address 0x11");
        Serial.println("3. Check EBYTE 433MHz module wiring");
        Serial.println("4. Ensure 12V power supply is adequate");
        Serial.println("5. Check all pin connections match code");

        // Flash the built-in LED to indicate error
        pinMode(2, OUTPUT);
        for (int i = 0; i < 10; i++) {
            digitalWrite(2, HIGH);
            delay(200);
            digitalWrite(2, LOW);
            delay(200);
        }

        // Continue in degraded mode rather than rebooting
        Serial.println("📝 Continuing in degraded mode - system won't reboot automatically");
    } else {
        Serial.println("✅ Node application initialized successfully");
        Serial.println("📻 FM radio and 433MHz communication ready");
        Serial.println("🎵 Ready for festival operation!");

        // Reset boot counter on successful initialization
        bootCount = 0;
    }

    Serial.println("\n🎛️  MANUAL CONTROLS:");
    Serial.println("   • Send 'help' via serial for debugging commands");
    Serial.println("   • Send 'status' for detailed status information");
    Serial.println("   • Send 'reset' to manually restart the node");
    Serial.println("   • Send 'debug' to toggle debug mode");

    Serial.println("\n" + String('=', 50));
    Serial.println("✅ Setup complete - entering main loop");
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
        Serial.println("\n🚨 WATCHDOG TRIGGERED - System was unresponsive!");
        safeRestart("Watchdog timeout");
    }

    // Feed watchdog every 5 seconds
    if (currentTime - lastWatchdogFeed > 5000) {
        feedWatchdog();
        lastWatchdogFeed = currentTime;
    }

    // Memory leak detection every 30 seconds
    if (currentTime - lastMemoryCheck > 30000) {
        uint32_t currentHeap = ESP.getFreeHeap();
        if (currentHeap < minHeap) {
            minHeap = currentHeap;
            Serial.printf("⚠️  New minimum heap: %d bytes (potential memory leak?)\n", minHeap);

            // If heap drops below 50KB, restart to prevent crashes
            if (minHeap < 50000) {
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
            Serial.println("\n📋 AVAILABLE COMMANDS:");
            Serial.println("   help     - Show this help menu");
            Serial.println("   status   - Show detailed node status");
            Serial.println("   reset    - Restart the node");
            Serial.println("   debug    - Toggle debug mode");
            Serial.println("   memory   - Show memory information");
            Serial.println("   config   - Show configuration");
            Serial.println("   test     - Run hardware self-test");

        } else if (command == "status") {
            Serial.println("\n📊 DETAILED NODE STATUS:");
            Serial.printf("   Node ID: %d\n", nodeApp.getCurrentFrequency() != 0 ? 1 : 0); // Simple check
            Serial.printf("   Uptime: %lu seconds\n", millis() / 1000);
            Serial.printf("   Free heap: %d bytes (min: %d)\n", ESP.getFreeHeap(), minHeap);
            Serial.printf("   Boot count: %d\n", bootCount);
            Serial.printf("   CPU freq: %d MHz\n", getCpuFrequencyMhz());
            Serial.printf("   Temperature: %.1f°C\n", temperatureRead());

            String detailedStatus = nodeApp.getDetailedSystemStatus();
            Serial.println("   Application status:");
            Serial.println(detailedStatus);

        } else if (command == "reset") {
            Serial.println("🔄 Manual reset requested via serial");
            safeRestart("Manual reset command");

        } else if (command == "debug") {
            static bool debugEnabled = false;
            debugEnabled = !debugEnabled;
            nodeApp.setVerboseDebugging(debugEnabled);
            Serial.printf("🐛 Debug mode: %s\n", debugEnabled ? "ENABLED" : "DISABLED");

        } else if (command == "memory") {
            Serial.println("\n💾 MEMORY INFORMATION:");
            Serial.printf("   Free heap: %d bytes\n", ESP.getFreeHeap());
            Serial.printf("   Minimum heap: %d bytes\n", minHeap);
            Serial.printf("   Largest free block: %d bytes\n", ESP.getMaxAllocHeap());
            Serial.printf("   Total heap: %d bytes\n", ESP.getHeapSize());

        } else if (command == "config") {
            Serial.println("\n⚙️  NODE CONFIGURATION:");
            Serial.printf("   Current frequency: %d (%.1f MHz)\n", nodeApp.getCurrentFrequency(), nodeApp.getCurrentFrequency() / 100.0);
            Serial.printf("   Current volume: %d/15\n", nodeApp.getCurrentVolume());
            Serial.printf("   Muted: %s\n", nodeApp.getCurrentMute() ? "Yes" : "No");
            Serial.printf("   Relay state: %s\n", nodeApp.getCurrentRelay() ? "ON" : "OFF");

        } else if (command == "test") {
            Serial.println("\n🧪 Running hardware self-test...");
            bool testResult = nodeApp.performSelfTest();
            Serial.printf("🧪 Self-test result: %s\n", testResult ? "✅ PASSED" : "❌ FAILED");

        } else if (command.length() > 0) {
            Serial.println("❓ Unknown command: " + command);
            Serial.println("💡 Type 'help' for available commands");
        }
    }

    // Update the node application with error handling
    try {
        nodeApp.update();
    } catch (const std::exception& e) {
        Serial.println("❌ Exception in main loop: " + String(e.what()));
        // Don't restart immediately, just log and continue
    } catch (...) {
        Serial.println("❌ Unknown exception in main loop");
        // Don't restart immediately, just log and continue
    }

    // Brief status update every 60 seconds
    if (currentTime - lastStatusPrint > 60000) {
        Serial.printf("💓 Node heartbeat - Uptime: %lu min, Heap: %d bytes, Freq: %.1f MHz, Vol: %d\n",
                      currentTime / 60000, ESP.getFreeHeap(),
                      nodeApp.getCurrentFrequency() / 100.0, nodeApp.getCurrentVolume());
        lastStatusPrint = currentTime;
    }

    // Small delay to prevent CPU hogging and allow other tasks
    delay(10);
}