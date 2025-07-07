#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include "NodeApplication.h"

// Create node application instance
szogfm::node::NodeApplication nodeApp;

void setup() {
    // Initialize serial communication for debugging
    Serial.begin(115200);

    // Wait for serial port to connect
    delay(1000);

    // Disable WiFi and Bluetooth to save power
    btStop();
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);

    Serial.println("\n\n===========================================");
    Serial.println("🎵 SzögFM Node Application Starting 🎵");
    Serial.println("===========================================\n");
    Serial.printf("⏰ Boot time: %lu ms\n", millis());
    Serial.printf("🔧 ESP32 Chip ID: %012llX\n", ESP.getEfuseMac());
    Serial.printf("💾 Free heap: %d bytes\n", ESP.getFreeHeap());

    // Initialize the node application
    if (!nodeApp.initialize()) {
        Serial.println("❌ Failed to initialize node application!");
        Serial.println("🔄 System will continue in safe mode...");

        // Don't halt - continue for troubleshooting
        // Flash the built-in LED to indicate error
        pinMode(2, OUTPUT);
        for (int i = 0; i < 10; i++) {
            digitalWrite(2, HIGH);
            delay(200);
            digitalWrite(2, LOW);
            delay(200);
        }
    } else {
        Serial.println("✅ Node application initialized successfully");
        Serial.println("📻 FM radio and 433MHz communication ready");
        Serial.println("🎵 Ready for festival operation!");
    }
}

void loop() {
    // Update the node application
    nodeApp.update();

    // Small delay to prevent CPU hogging
    delay(10);
}