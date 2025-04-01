#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include "NodeApplication.h"
#include "../common/display/SpiSetup.h"

// Enable DHT sensor if available
// #define ENABLE_DHT_SENSOR

// Create node application instance
szogfm::node::NodeApplication nodeApp;

void setup() {
    // Initialize serial communication for debugging
    Serial.begin(115200);

    // Wait for serial port to connect
    delay(1000);

    Serial.println("\n\n===========================================");
    Serial.println("SzogFM Node Application Starting");
    Serial.println("===========================================\n");

    // Disable WiFi and Bluetooth to save power
    btStop();
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);

    // Initialize SPI with VSPI pins for ST7789 display
    Serial.println("Configuring SPI bus for ST7789 display...");

    // Configure SPI with CLK=18, MISO=19, MOSI=23, CS=12
    szogfm::display::SpiSetup::configureSpi(18, 19, 23, 12);

    // Initialize GPIO pins for display DC and RST
    pinMode(25, OUTPUT); // DC pin
    pinMode(26, OUTPUT); // RST pin
    digitalWrite(26, HIGH); // RST active high

    // Initialize the node application
    if (!nodeApp.initialize()) {
        Serial.println("Failed to initialize node application!");
        // Flash the built-in LED to indicate error
        pinMode(2, OUTPUT);
        while (true) {
            digitalWrite(2, HIGH);
            delay(100);
            digitalWrite(2, LOW);
            delay(100);
        }
    }

    Serial.println("Node application initialized successfully");
}

void loop() {
    // Update the node application
    nodeApp.update();

    // Small delay to prevent CPU hogging
    delay(10);
}