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
 * Structure to hold node status information with enhanced debugging capabilities
 */
        struct NodeStatus {
            uint8_t nodeId;                     // Node identifier (1-20)
            String nodeName;                    // Custom name/location for this node (e.g., "Main Stage", "Food Court")
            bool isConnected;                   // Current connection status
            unsigned long lastSeenTime;        // Last time we received data from this node
            int rssi;                          // Signal strength of 433MHz link
            uint8_t volume;                    // Current volume level (0-15)
            bool muted;                        // Current mute state
            uint16_t frequency;                // Current FM frequency in 10kHz units
            bool relayState;                   // Current relay state (on/off)
            int signalStrength;                // FM signal strength (RSSI)
            bool isStereo;                     // FM reception stereo status
            float temperature;                 // Temperature from sensor (if available)
            float humidity;                    // Humidity from sensor (if available)
            bool hasSensors;                   // Whether node has environmental sensors
            unsigned long uptime;              // Node uptime in milliseconds
            String errorMessage;               // Last error message from node

            // Enhanced debugging and performance tracking fields
            unsigned long lastCommandTime;     // Time of last command sent to this node
            String lastCommand;                // Description of last command sent
            bool lastCommandSuccess;           // Success status of last command
            uint16_t totalCommands;            // Total commands sent to this node
            uint16_t successfulCommands;       // Number of successful commands
            uint16_t failedCommands;           // Number of failed commands
            unsigned long totalResponseTime;   // Cumulative response time for statistics
            String firmwareVersion;            // Node firmware version (if available)

            // Node quality metrics
            float commandSuccessRate;          // Calculated success rate percentage
            float averageResponseTime;         // Average response time in milliseconds
            unsigned long lastErrorTime;       // Time of last error
            String lastErrorMessage;           // Last error details

            // NEW: Recent activity statistics (last 10 minutes)
            struct RecentStats {
                unsigned long windowStart;      // Start time of current 10-minute window
                uint16_t messagesExchanged;     // Messages exchanged in current window
                uint16_t successfulMessages;   // Successful messages in current window
                uint16_t failedMessages;       // Failed messages in current window
                unsigned long lastResponseTime; // Time of last successful response

                RecentStats() : windowStart(0), messagesExchanged(0),
                                successfulMessages(0), failedMessages(0), lastResponseTime(0) {}
            } recentStats;
        };

/**
 * Structure to hold comprehensive communication statistics
 */
        struct CommunicationStats {
            unsigned long totalMessagesSent;     // Total messages transmitted
            unsigned long totalMessagesReceived; // Total messages received successfully
            unsigned long totalRetries;          // Total retry attempts
            unsigned long totalTimeouts;         // Total timeout occurrences
            unsigned long totalErrors;           // Total communication errors
            float averageResponseTime;           // Average response time in ms
            float messageSuccessRate;            // Overall success rate percentage
            unsigned long lastResetTime;         // Time when stats were last reset
            unsigned long totalResponseTime;     // Cumulative response time
            unsigned long peakResponseTime;      // Longest response time recorded
            unsigned long minimumResponseTime;   // Shortest response time recorded
        };

/**
 * Configuration structure for controller settings
 */
        struct ControllerConfig {
            // WiFi settings
            String wifiSsid;                    // WiFi network name or AP name
            String wifiPassword;                // WiFi password
            bool accessPointMode;               // True for AP mode, false for station mode

            // Communication settings
            uint8_t radioChannel;               // 433MHz channel (default 0x1A)
            uint16_t radioAddress;              // Network address (default 0x1234)
            uint8_t transmissionPower;          // TX power level (0-3)

            // Timing settings
            unsigned long statusRequestInterval; // How often to request status from nodes
            unsigned long discoveryInterval;     // How often to discover new nodes
            unsigned long messageTimeout;        // Timeout for message acknowledgments
            uint8_t maxRetryCount;               // Maximum retry attempts for messages

            // Debug settings
            bool verboseDebugging;              // Enable detailed debug output
            uint8_t communicationDebugLevel;   // EBYTE module debug level (0-2)
        };

/**
 * Controller application class for SzogFM project
 * This class manages the central controller functionality for the distributed FM system
 *
 * Key responsibilities:
 * - Manage communication with multiple FM receiver nodes via 433MHz LORA
 * - Provide web interface for system control and monitoring
 * - Track node status and performance metrics
 * - Handle message routing, retries, and error recovery
 * - Coordinate system-wide operations (volume control, frequency changes, etc.)
 */
        class ControllerApplication {
        public:
            /**
             * Constructor - Initialize controller with default configuration
             */
            ControllerApplication();

            /**
             * Destructor - Clean up resources
             */
            ~ControllerApplication();

            /**
             * Initialize the controller application
             * Sets up communication module, WiFi, web server, and performs initial discovery
             * @return true if initialization was successful, false otherwise
             */
            bool initialize();

            /**
             * Main application update loop
             * Processes messages, handles retries, updates status, serves web requests
             * Should be called repeatedly in the Arduino loop() function
             */
            void update();

            // ===========================================
            // NODE CONTROL METHODS
            // ===========================================

            /**
             * Set the volume for a specific node
             * @param nodeId Node ID (1-20), or 0 for broadcast to all nodes
             * @param volume Volume level (0-15, will be clamped if out of range)
             * @return true if the command was queued successfully, false otherwise
             */
            bool setNodeVolume(uint8_t nodeId, uint8_t volume);

            /**
             * Set the FM frequency for a specific node
             * @param nodeId Node ID (1-20), or 0 for broadcast to all nodes
             * @param frequency FM frequency in 10kHz units (8750-10800 for 87.5-108.0 MHz)
             * @return true if the command was queued successfully, false otherwise
             */
            bool setNodeFrequency(uint8_t nodeId, uint16_t frequency);

            /**
             * Set the relay state for a specific node (speaker power control)
             * @param nodeId Node ID (1-20), or 0 for broadcast to all nodes
             * @param state Relay state (true = on/powered, false = off/unpowered)
             * @return true if the command was queued successfully, false otherwise
             */
            bool setNodeRelayState(uint8_t nodeId, bool state);

            /**
             * Set the mute state for a specific node
             * @param nodeId Node ID (1-20), or 0 for broadcast to all nodes
             * @param mute Mute state (true = muted, false = unmuted)
             * @return true if the command was queued successfully, false otherwise
             */
            bool setNodeMute(uint8_t nodeId, bool mute);

            // ===========================================
            // BROADCAST CONTROL METHODS (ALL NODES)
            // ===========================================

            /**
             * Set the volume for all nodes simultaneously
             * @param volume Volume level (0-15)
             * @return true if the broadcast command was queued successfully
             */
            bool setAllNodesVolume(uint8_t volume);

            /**
             * Set the FM frequency for all nodes simultaneously
             * @param frequency FM frequency in 10kHz units (8750-10800)
             * @return true if the broadcast command was queued successfully
             */
            bool setAllNodesFrequency(uint16_t frequency);

            /**
             * Set the relay state for all nodes simultaneously
             * @param state Relay state (true = on, false = off)
             * @return true if the broadcast command was queued successfully
             */
            bool setAllNodesRelayState(bool state);

            /**
             * Set the mute state for all nodes simultaneously
             * @param mute Mute state (true = muted, false = unmuted)
             * @return true if the broadcast command was queued successfully
             */
            bool setAllNodesMute(bool mute);

            // ===========================================
            // STATUS AND MONITORING METHODS
            // ===========================================

            /**
             * Get status information for a specific node
             * @param nodeId Node ID to query
             * @return Pointer to node status structure, or nullptr if node not found
             */
            const NodeStatus* getNodeStatus(uint8_t nodeId) const;

            /**
             * Get status information for all known nodes
             * @return Vector containing status information for all nodes
             */
            std::vector<NodeStatus> getAllNodeStatus() const;

            /**
             * Request status update from a specific node
             * @param nodeId Node ID to request status from
             * @return true if the request was queued successfully
             */
            bool requestNodeStatus(uint8_t nodeId);

            /**
             * Request status update from all known nodes
             * @return true if the requests were queued successfully
             */
            bool requestAllNodeStatus();

            /**
             * Get communication statistics
             * @return Reference to communication statistics structure
             */
            const CommunicationStats& getCommunicationStats() const;

            /**
             * Reset communication statistics to zero
             */
            void resetCommunicationStats();

            /**
             * Get detailed system status as JSON string for debugging
             * @return JSON string with comprehensive system status
             */
            String getDetailedSystemStatus() const;

            // ===========================================
            // SYSTEM CONTROL METHODS
            // ===========================================

            /**
             * Reset a specific node (causes node to restart)
             * @param nodeId Node ID to reset
             * @return true if the reset command was queued successfully
             */
            bool resetNode(uint8_t nodeId);

            /**
             * Force discovery of new nodes
             * Broadcasts status requests to all possible node IDs (1-20)
             * @return Number of discovery requests sent
             */
            int discoverNodes();

            /**
             * Force discovery of new nodes with limit
             * Broadcasts status requests to a limited number of node IDs
             * @param maxNodes Maximum number of nodes to discover (1-20)
             * @return Number of discovery requests sent
             */
            int discoverNodes(int maxNodes);

            /**
             * Test communication with a specific node
             * @param nodeId Node ID to test communication with
             * @return true if communication test was initiated successfully
             */
            bool testNodeCommunication(uint8_t nodeId);

            // ===========================================
            // NODE NAMING AND MANAGEMENT METHODS
            // ===========================================

            /**
             * Set a custom name/location for a node
             * @param nodeId Node ID to rename
             * @param name New name/location for the node (e.g., "Main Stage", "Food Court")
             * @return true if the name was set successfully
             */
            bool setNodeName(uint8_t nodeId, const String& name);

            /**
             * Get the custom name/location for a node
             * @param nodeId Node ID to get name for
             * @return Custom name if set, or default name if not set
             */
            String getNodeName(uint8_t nodeId) const;

            /**
             * Reset node name to default
             * @param nodeId Node ID to reset name for
             * @return true if the name was reset successfully
             */
            bool resetNodeName(uint8_t nodeId);

            /**
             * Delete/remove a node from the system
             * @param nodeId Node ID to remove
             * @return true if the node was removed successfully
             */
            bool deleteNode(uint8_t nodeId);

            /**
             * Update recent statistics for a node (10-minute window)
             * @param nodeId Node ID to update stats for
             * @param success Whether the last operation was successful
             */
            void updateNodeRecentStats(uint8_t nodeId, bool success);

            /**
             * Get time since last response from a node in human-readable format
             * @param nodeId Node ID to check
             * @return String like "2m 30s ago" or "Never"
             */
            String getTimeSinceLastResponse(uint8_t nodeId) const;

            /**
             * Clean up old statistics (call periodically)
             */
            void cleanupOldStatistics();

            /**
             * Save node names to persistent storage
             * @return true if names were saved successfully
             */
            bool saveNodeNames();

            /**
             * Load node names from persistent storage
             * @return true if names were loaded successfully
             */
            bool loadNodeNames();

            /**
             * Get count of currently connected nodes
             * @return Number of nodes that are currently connected
             */
            int getConnectedNodeCount() const;

            // ===========================================
            // CONFIGURATION METHODS
            // ===========================================

            /**
             * Get current controller configuration
             * @return Reference to configuration structure
             */
            const ControllerConfig& getConfiguration() const;

            /**
             * Update controller configuration
             * @param config New configuration to apply
             * @return true if configuration was applied successfully
             */
            bool setConfiguration(const ControllerConfig& config);

            /**
             * Enable or disable verbose debugging output
             * @param enable true to enable verbose debugging, false to disable
             */
            void setVerboseDebugging(bool enable) { _config.verboseDebugging = enable; }

            /**
             * Set the debug level for the communication module
             * @param level Debug level (0=off, 1=basic, 2=verbose with hex dumps)
             */
            void setCommunicationDebugLevel(uint8_t level);

            /**
             * Handle a received message from a node
             * @param message Pointer to the received message buffer
             * @param length Length of the message in bytes
             * @param senderNodeId Node ID of the message sender
             * @return true if the message was handled successfully
             */
            bool handleNodeMessage(const void* message, size_t length, uint8_t senderNodeId);

            /**
             * Handle a status response message from a node
             * @param statusMsg The status message received
             * @param senderNodeId ID of the node that sent the message
             * @param receiveTime Time when the message was received
             * @return true if the message was handled successfully
             */
            bool handleNodeStatusMessage(const StatusMessage& statusMsg, uint8_t senderNodeId, unsigned long receiveTime);

            /**
             * Handle an acknowledgment message from a node
             * @param ackMsg The acknowledgment message received
             * @param senderNodeId ID of the node that sent the message
             * @param receiveTime Time when the message was received
             * @return true if the message was handled successfully
             */
            bool handleNodeAckMessage(const AckMessage& ackMsg, uint8_t senderNodeId, unsigned long receiveTime);

        private:
            // ===========================================
            // MEMBER VARIABLES
            // ===========================================

            // Core components
            communication::EbyteCommModule* _commModule;  // 433MHz communication module
            WebServer* _webServer;                        // HTTP server for web interface

            // Configuration and state
            ControllerConfig _config;                     // Controller configuration
            bool _initialized;                            // Initialization status flag
            std::map<uint8_t, NodeStatus> _nodeStatus;    // Status of all known nodes
            uint16_t _messageSequence;                    // Message sequence counter

            // Timing variables
            unsigned long _lastStatusRequestTime;         // Last time we requested status from nodes
            unsigned long _lastWebUpdateTime;             // Last time web interface was updated
            unsigned long _lastDiscoveryTime;             // Last time we ran node discovery

            // Communication statistics
            CommunicationStats _commStats;                // Communication performance metrics

            // Node name storage for custom location names
            std::map<uint8_t, String> _nodeNames;         // Custom names for nodes (persistent)

            // Message retry system
            struct PendingMessage {
                uint16_t sequenceNum;                     // Message sequence number
                uint8_t nodeId;                           // Target node ID
                unsigned long sentTime;                   // Time message was sent
                unsigned long firstSentTime;              // Time of first send attempt
                uint8_t retryCount;                       // Number of retry attempts
                std::vector<uint8_t> messageData;         // Copy of message data for retries
                String commandDescription;                // Human-readable command description
            };
            std::vector<PendingMessage> _pendingMessages; // Messages awaiting acknowledgment

            // Pin definitions for EBYTE module
            int _pinM0;                                   // M0 control pin
            int _pinM1;                                   // M1 control pin
            int _pinAUX;                                  // AUX status pin

            // WiFi configuration
            const char* _wifiSsid;                        // WiFi network name
            const char* _wifiPassword;                    // WiFi password
            bool _wifiApMode;                             // Access point vs station mode

            // ===========================================
            // PRIVATE HELPER METHODS
            // ===========================================

            /**
             * Initialize the communication module (EBYTE 433MHz)
             * @return true if initialization was successful
             */
            bool initializeCommunication();

            /**
             * Initialize the WiFi connection and web server
             * @return true if initialization was successful
             */
            bool initializeWebServer();

            /**
             * Set up all web server routes and handlers
             */
            void setupWebRoutes();

            /**
             * Create the main web interface HTML
             * @return HTML string for the main interface
             */
            String createMainWebInterface();

            /**
             * Process any received messages from nodes
             * @return true if a message was processed
             */
            bool processMessages();

            /**
             * Process pending messages that need to be retried
             * Handles timeouts, retries, and failure cleanup
             */
            void processPendingMessages();

            /**
             * Send a command message to a node with retry handling
             * @param nodeId Target node ID (0 for broadcast)
             * @param command Command type to send
             * @param data Command data payload (can be nullptr)
             * @param dataLength Length of data payload
             * @param description Human-readable command description
             * @return true if message was queued successfully
             */
            bool sendCommandMessage(uint8_t nodeId, Command command, const uint8_t* data,
                                    size_t dataLength, const String& description = "");

            /**
             * Update node connection status based on timeouts
             * Marks nodes as disconnected if they haven't responded recently
             */
            void updateNodeConnectionStatus();

            // ===========================================
            // WEB SERVER HANDLER METHODS
            // ===========================================

            void handleRoot();                            // Main web interface
            void handleStatus();                          // Node status API
            void handleDetailedStatus();                  // Detailed system status API
            void handleSetVolume();                       // Volume control API
            void handleSetFrequency();                    // Frequency control API
            void handleSetRelay();                        // Relay control API
            void handleSetMute();                         // Mute control API
            void handleSetNodeName();                     // Node rename API
            void handleDeleteNode();                      // Node delete API
            void handleDiscoverNodes();                   // Node discovery API
            void handleResetStats();                      // Statistics reset API
        };

    } // namespace controller
} // namespace szogfm

#endif // SZOGFM_CONTROLLERAPPLICATION_H