#include <Arduino.h>
#include <EBYTE.h>

#define PIN_M0 4
#define PIN_M1 32
#define PIN_AUX 33

EBYTE ebyte(&Serial2, PIN_M0, PIN_M1, PIN_AUX);

void setup() {
    Serial.begin(115200);
    Serial2.begin(9600);

    delay(500);
    Serial.println("Testing EBYTE module");

    if (ebyte.init()) {
        Serial.println("EBYTE module initialized successfully");
        ebyte.PrintParameters();
    } else {
        Serial.println("EBYTE module initialization failed");
    }
}

void loop() {
    // Nothing here
}