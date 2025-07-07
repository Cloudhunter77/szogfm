/**
 * Simple Node Status Display
 * Clean, easy-to-read status output for festival operation
 */

#ifndef SZOGFM_SIMPLE_NODE_STATUS_H
#define SZOGFM_SIMPLE_NODE_STATUS_H

#include <Arduino.h>

namespace szogfm {
    namespace node {

        class SimpleNodeStatus {
        public:
            struct NodeInfo {
                uint8_t nodeId;
                bool connected;
                uint16_t frequency;
                uint8_t volume;
                bool muted;
                bool relayOn;
                int fmSignal;
                int commSignal;
                unsigned long lastCommand;
                unsigned long uptime;
                String lastError;
            };

            static void printHeader() {
                Serial.println("\n\n" + String('=', 70));
                Serial.println("🎵                    SzögFM Festival Node                    🎵");
                Serial.println(String('=', 70));
            }

            static void printStatus(const NodeInfo& info) {
                // Clear screen for clean display
                Serial.print("\033[2J\033[H"); // ANSI clear screen and go to top

                printHeader();

                // Main status line
                Serial.printf("📊 NODE %d STATUS: ", info.nodeId);
                if (info.connected) {
                    Serial.println("🟢 CONNECTED TO CONTROLLER");
                } else {
                    Serial.println("🔴 DISCONNECTED");
                }

                Serial.println(String('-', 70));

                // Audio status
                Serial.printf("🎵 FM RADIO:     %.1f MHz", info.frequency / 100.0);
                if (info.fmSignal > 0) {
                    if (info.fmSignal > 40) Serial.print("  📶 EXCELLENT");
                    else if (info.fmSignal > 25) Serial.print("  📶 GOOD");
                    else if (info.fmSignal > 15) Serial.print("  📶 FAIR");
                    else Serial.print("  📶 WEAK");
                    Serial.printf(" (%d)", info.fmSignal);
                }
                Serial.println();

                Serial.printf("🔊 VOLUME:       %d/15", info.volume);
                if (info.muted) {
                    Serial.print("  🔇 MUTED");
                }
                Serial.println();

                Serial.printf("🔌 SPEAKER:      %s", info.relayOn ? "🟢 POWERED ON" : "🔴 POWERED OFF");
                Serial.println();

                // Communication status
                Serial.println();
                Serial.printf("📡 CONTROLLER:   ");
                if (info.connected) {
                    Serial.print("🟢 ONLINE");
                    if (info.commSignal != 0) {
                        Serial.printf("  📶 Signal: %d", info.commSignal);
                    }
                    if (info.lastCommand > 0) {
                        unsigned long ago = (millis() - info.lastCommand) / 1000;
                        if (ago < 60) {
                            Serial.printf("  ⏰ Last command: %lus ago", ago);
                        } else {
                            Serial.printf("  ⏰ Last command: %lum ago", ago / 60);
                        }
                    }
                } else {
                    Serial.print("🔴 OFFLINE");
                    unsigned long ago = (millis() - info.lastCommand) / 1000;
                    if (ago < 3600) {
                        Serial.printf("  ⏰ Lost %lum ago", ago / 60);
                    } else {
                        Serial.printf("  ⏰ Lost %luh ago", ago / 3600);
                    }
                }
                Serial.println();

                // System status
                Serial.println();
                unsigned long uptimeMin = info.uptime / 60000;
                if (uptimeMin < 60) {
                    Serial.printf("⏰ UPTIME:       %lu minutes", uptimeMin);
                } else {
                    Serial.printf("⏰ UPTIME:       %luh %lum", uptimeMin / 60, uptimeMin % 60);
                }
                Serial.println();

                Serial.printf("💾 MEMORY:       %d bytes free", ESP.getFreeHeap());
                Serial.println();

                // Error status
                if (info.lastError.length() > 0) {
                    Serial.println();
                    Serial.println("🚨 LAST ERROR:   " + info.lastError);
                }

                Serial.println(String('=', 70));

                // Quick status indicator
                Serial.print("🚦 OVERALL: ");
                if (info.connected && info.fmSignal > 15 && info.lastError.length() == 0) {
                    Serial.println("🟢 ALL SYSTEMS OK");
                } else if (info.connected) {
                    Serial.println("🟡 CONNECTED BUT CHECK SIGNALS");
                } else {
                    Serial.println("🔴 NEEDS ATTENTION");
                }

                Serial.println();
                Serial.println("Press RESET to restart node | Ctrl+C to exit monitor");
                Serial.println();
            }

            static void printConnectionTest() {
                Serial.println("\n🔍 TESTING CONNECTION TO CONTROLLER...");
                Serial.println("Looking for:");
                Serial.println("  📤 Outgoing status messages");
                Serial.println("  📥 Incoming commands");
                Serial.println("  🤝 Command acknowledgments");
                Serial.println("\nWait 30 seconds for results...\n");
            }

            static void printConnectionSuccess() {
                Serial.println("\n🎉 CONNECTION TEST SUCCESSFUL!");
                Serial.println("✅ Controller is responding");
                Serial.println("✅ Commands are being received");
                Serial.println("✅ Node is part of the network\n");
            }

            static void printConnectionFailed() {
                Serial.println("\n❌ CONNECTION TEST FAILED!");
                Serial.println("🔧 Troubleshooting steps:");
                Serial.println("  1. Check controller is powered and running");
                Serial.println("  2. Verify 433MHz antennas connected");
                Serial.println("  3. Check distance (try moving closer)");
                Serial.println("  4. Verify same radio channel/address");
                Serial.println("  5. Check for interference\n");
            }

            static void printBootSequence(const String& step, bool success, const String& details = "") {
                static int stepCount = 0;
                stepCount++;

                Serial.printf("[%d] %s: ", stepCount, step.c_str());
                if (success) {
                    Serial.print("✅ ");
                    if (details.length() > 0) {
                        Serial.print(details);
                    } else {
                        Serial.print("OK");
                    }
                } else {
                    Serial.print("❌ ");
                    if (details.length() > 0) {
                        Serial.print("FAILED - " + details);
                    } else {
                        Serial.print("FAILED");
                    }
                }
                Serial.println();
            }
        };

    } // namespace node
} // namespace szogfm

#endif // SZOGFM_SIMPLE_NODE_STATUS_H