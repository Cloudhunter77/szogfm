#ifndef SZOGFM_CONTROLLERAPPLICATION_H
#define SZOGFM_CONTROLLERAPPLICATION_H

#include "../common/messages.h"
#include "../common/communication/EbyteCommModule.h"
#include <vector>
#include <map>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>

namespace szogfm {
    namespace controller {

/**
 * Structure to hold node status information
 */
        struct NodeStatus {
            uint8_t nodeId;
            bool isConnected;
            unsigned long lastSeenTime;
            int rssi;
            uint8_t volume;
            bool muted;
            uint16_t frequency;
            bool relayState;
            int signalStrength;
            bool isStereo;
            float temperature;
            float humidity;
            bool hasSensors;
            unsigned long uptime;
            std::string errorMessage;

            // Enhanced debugging fields
            unsigned long lastCommandTime;
            std::string lastCommand;
            bool lastCommandSuccess;
            uint16_t totalCommands;
            uint16_t successfulCommands;
            uint16_t failedCommands;
            unsigned long totalResponseTime;
            std::string firmwareVersion;
        };

/**
 * Structure to hold communication statistics
 */
        struct CommunicationStats {
            unsigned long totalMessagesSent;
            unsigned long totalMessagesReceived;
            unsigned long totalRetries;
            unsigned long totalTimeouts;
            unsigned long totalErrors;
            float averageResponseTime;
            float messageSuccessRate;
            unsigned long lastResetTime;
        };

/**
 * Controller application class for SzogFM project
 * This class manages the central controller functionality
 */
        class ControllerApplication {
        public:
            /**
             * Constructor
             */
            ControllerApplication();

            /**
             * Initialize the application
             * @return true if initialization was successful, false otherwise
             */
            bool initialize();

            /**
             * Run the application main loop
             * Should be called repeatedly in the Arduino loop() function
             */
            void update();

            /**
             * Set the volume for a specific node
             * @param nodeId Node ID
             * @param volume Volume level (0-15)
             * @return true if the command was sent successfully, false otherwise
             */
            bool setNodeVolume(uint8_t nodeId, uint8_t volume);

            /**
             * Set the frequency for a specific node
             * @param nodeId Node ID
             * @param frequency FM frequency in 10kHz units (e.g., 8850 for 88.5 MHz)
             * @return true if the command was sent successfully, false otherwise
             */
            bool setNodeFrequency(uint8_t nodeId, uint16_t frequency);

            /**
             * Set the relay state for a specific node
             * @param nodeId Node ID
             * @param state Relay state (true = on, false = off)
             * @return true if the command was sent successfully, false otherwise
             */
            bool setNodeRelayState(uint8_t nodeId, bool state);

            /**
             * Set the mute state for a specific node
             * @param nodeId Node ID
             * @param mute Mute state (true = muted, false = unmuted)
             * @return true if the command was sent successfully, false otherwise
             */
            bool setNodeMute(uint8_t nodeId, bool mute);

            /**
             * Set the volume for all nodes
             * @param volume Volume level (0-15)
             * @return true if the command was sent successfully to all nodes, false otherwise
             */
            bool setAllNodesVolume(uint8_t volume);

            /**
             * Set the frequency for all nodes
             * @param frequency FM frequency in 10kHz units (e.g., 8850 for 88.5 MHz)
             * @return true if the command was sent successfully to all nodes, false otherwise
             */
            bool setAllNodesFrequency(uint16_t frequency);

            /**
             * Set the relay state for all nodes
             * @param state Relay state (true = on, false = off)
             * @return true if the command was sent successfully to all nodes, false otherwise
             */
            bool setAllNodesRelayState(bool state);

            /**
             * Set the mute state for all nodes
             * @param mute Mute state (true = muted, false = unmuted)
             * @return true if the command was sent successfully to all nodes, false otherwise
             */
            bool setAllNodesMute(bool mute);

            /**
             * Get status information for a specific node
             * @param nodeId Node ID
             * @return Node status information
             */
            const NodeStatus* getNodeStatus(uint8_t nodeId) const;

            /**
             * Get status information for all nodes
             * @return Vector of node status information
             */
            std::vector<NodeStatus> getAllNodeStatus() const;

            /**
             * Request status update from a specific node
             * @param nodeId Node ID
             * @return true if the request was sent successfully, false otherwise
             */
            bool requestNodeStatus(uint8_t nodeId);

            /**
             * Request status update from all nodes
             * @return true if the request was sent successfully to all known nodes, false otherwise
             */
            bool requestAllNodeStatus();

            /**
             * Reset a specific node
             * @param nodeId Node ID
             * @return true if the reset command was sent successfully, false otherwise
             */
            bool resetNode(uint8_t nodeId);

            /**
             * Handle a received message from a node
             * @param message Pointer to the message buffer
             * @param length Length of the message
             * @param senderNodeId Node ID of the sender
             * @return true if the message was handled successfully, false otherwise
             */
            bool handleNodeMessage(const void* message, size_t length, uint8_t senderNodeId);

            /**
             * Get communication statistics
             * @return Communication statistics structure
             */
            const CommunicationStats& getCommunicationStats() const { return _commStats; }

            /**
             * Reset communication statistics
             */
            void resetCommunicationStats();

            /**
             * Get detailed system status for debugging
             * @return JSON string with detailed status
             */
            String getDetailedSystemStatus() const;

            /**
             * Enable or disable verbose debugging
             * @param enable true to enable verbose debugging
             */
            void setVerboseDebugging(bool enable) { _verboseDebugging = enable; }

            /**
             * Force discovery of new nodes
             * @return number of nodes discovered
             */
            int discoverNodes();

            /**
             * Test communication with a specific node
             * @param nodeId Node ID to test
             * @return true if communication test successful
             */
            bool testNodeCommunication(uint8_t nodeId);

        private:
            // Components
            communication::EbyteCommModule* _commModule;
            WebServer* _webServer;

            // State variables
            bool _initialized;
            std::map<uint8_t, NodeStatus> _nodeStatus;
            uint16_t _messageSequence;
            unsigned long _lastStatusRequestTime;
            unsigned long _lastWebUpdateTime;
            unsigned long _lastDiscoveryTime;
            bool _verboseDebugging;

            // Communication statistics
            CommunicationStats _commStats;

            // Pending messages
            struct PendingMessage {
                uint16_t sequenceNum;
                uint8_t nodeId;
                unsigned long sentTime;
                uint8_t retryCount;
                std::vector<uint8_t> messageData;
                std::string commandDescription;
                unsigned long firstSentTime;
            };
            std::vector<PendingMessage> _pendingMessages;

            // Pin definitions (can be adjusted in constructor)
            int _pinM0;
            int _pinM1;
            int _pinAUX;

            // Constants
            static constexpr unsigned long STATUS_REQUEST_INTERVAL = 10000;    // 10 seconds
            static constexpr unsigned long WEB_UPDATE_INTERVAL = 1000;         // 1 second
            static constexpr unsigned long MESSAGE_TIMEOUT = 3000;             // 3 seconds (increased)
            static constexpr unsigned long DISCOVERY_INTERVAL = 60000;         // 60 seconds
            static constexpr uint8_t MAX_RETRY_COUNT = 5;                      // Increased retry count
            static constexpr unsigned long NODE_TIMEOUT = 120000;              // 2 minutes (increased)

            // WiFi configuration
            const char* _wifiSsid;
            const char* _wifiPassword;
            bool _wifiApMode;

            /**
             * Initialize the web server
             * @return true if initialization was successful, false otherwise
             */
            bool initializeWebServer();

            /**
             * Process received messages
             * @return true if a message was processed, false otherwise
             */
            bool processMessages();

            /**
             * Process any pending messages that need to be retried
             */
            void processPendingMessages();

            /**
             * Send a command message to a node
             * @param nodeId Node ID
             * @param command Command to send
             * @param data Data to include with the command
             * @param dataLength Length of the data
             * @param description Human-readable description of the command
             * @return true if the message was sent successfully, false otherwise
             */
            bool sendCommandMessage(uint8_t nodeId, Command command, const uint8_t* data,
                                    size_t dataLength, const String& description = "");

            /**
             * Handle a received status message from a node
             * @param status Status message
             * @return true if the message was handled successfully, false otherwise
             */
            bool handleStatusMessage(const szogfm::StatusMessage& status);

            /**
             * Handle a received acknowledgement message from a node
             * @param ack Acknowledgement message
             * @return true if the message was handled successfully, false otherwise
             */
            bool handleAckMessage(const szogfm::AckMessage& ack);

            /**
             * Handle a received error message from a node
             * @param error Error message
             * @return true if the message was handled successfully, false otherwise
             */
            bool handleErrorMessage(const MessageHeader& error);

            /**
             * Update node connection status
             * Check for nodes that haven't been seen recently and mark as disconnected
             */
            void updateNodeConnectionStatus();

            /**
             * Log detailed message information
             */
            void logMessageDetails(const String& direction, uint8_t nodeId,
                                   const String& messageType, const void* message, size_t length);

            /**
             * Update communication statistics
             */
            void updateCommStats(bool success, unsigned long responseTime = 0);

            // Enhanced web server handler functions
            void handleRoot();
            void handleStatus();
            void handleDetailedStatus();
            void handleCommStats();
            void handleSystemDiagnostics();
            void handleNodeDiagnostics();
            void handleSetVolume();
            void handleSetFrequency();
            void handleSetRelay();
            void handleSetMute();
            void handleResetNode();
            void handleDiscoverNodes();
            void handleTestNode();
            void handleResetStats();
            void handleCommTest();
        };

    } // namespace controller
} // namespace szogfm

#endif // SZOGFM_CONTROLLERAPPLICATION_H