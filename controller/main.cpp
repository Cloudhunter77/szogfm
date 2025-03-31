#include <Arduino.h>
#include "ControllerApplication.h"

// Create controller application instance
szogfm::controller::ControllerApplication controllerApp;

void setup() {
    // Initialize serial communication for debugging
    Serial.begin(115200);

    // Wait for serial port to connect
    delay(1000);

    Serial.println("\n\n===========================================");
    Serial.println("SzogFM Controller Application Starting");
    Serial.println("===========================================\n");

    // Initialize the controller application
    if (!controllerApp.initialize()) {
        Serial.println("Failed to initialize controller application!");
        // Flash the built-in LED to indicate error
        pinMode(2, OUTPUT);
        while (true) {
            digitalWrite(2, HIGH);
            delay(100);
            digitalWrite(2, LOW);
            delay(100);
        }
    }

    Serial.println("Controller application initialized successfully");
}

void loop() {
    // Update the controller application
    controllerApp.update();

    // Small delay to prevent CPU hogging
    delay(10);
}