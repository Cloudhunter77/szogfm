#ifndef SZOGFM_NODEAPPLICATION_H
#define SZOGFM_NODEAPPLICATION_H

#include "../common/messages.h"
#include "../common/NodeConfig.h"
#include "../common/communication/EbyteCommModule.h"
#include "../common/radio/RDA5807Radio.h"
#include "../common/display/SSD1306Display.h"
#include "../common/input/ButtonHandler.h"

namespace szogfm {
    namespace node {

/**
 * Structure to hold node statistics for debugging
 */
        struct NodeStatistics {
            unsigned long bootTime;
            unsigned long totalCommandsReceived;
            unsigned long successfulCommands;
            unsigned long failedCommands;
            unsigned long totalStatusRequests;
            unsigned long totalButtonPresses;
            unsigned long totalDisplayUpdates;
            unsigned long lastRadioUpdate;
            unsigned long lastCommTest;
            float commandSuccessRate;

            // Radio statistics
            struct {
                unsigned long frequencyChanges;
                unsigned long volumeChanges;
                unsigned long muteToggles;
                int lastRssi;
                bool lastStereoState;
                unsigned long lastFrequencyChange;
                unsigned long lastVolumeChange;
            } radio;

            // Communication statistics
            struct {
                unsigned long messagesReceived;
                unsigned long messagesSent;
                unsigned long acksSent;
                unsigned long communicationErrors;
                unsigned long lastMessageReceived;
                unsigned long lastMessageSent;
                int lastSignalStrength;
            } communication;
        };

/**
 * Node application class for SzogFM project
 * This class integrates all components and manages the node functionality
 */
        class NodeApplication {
        public:
            /**
             * Constructor
             */
            NodeApplication();

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
             * Handle a received command message
             * @param message Command message to handle
             * @return true if the command was handled successfully, false otherwise
             */
            bool handleCommand(const CommandMessage& message);

            /**
             * Handle a status request message
             * @param message Status request message to handle
             * @return true if the request was handled successfully, false otherwise
             */
            bool handleStatusRequest(const MessageHeader& message);

            /**
             * Send a status message to the controller
             * @return true if the message was sent successfully, false otherwise
             */
            bool sendStatusMessage();

            /**
             * Handle an error
             * @param message Error message
             */
            void handleError(const String& message);

            /**
             * Get node statistics for debugging
             * @return Node statistics structure
             */
            const NodeStatistics& getStatistics() const { return _stats; }

            /**
             * Enable or disable verbose debugging
             * @param enable true to enable verbose debugging
             */
            void setVerboseDebugging(bool enable) { _verboseDebugging = enable; }

            /**
             * Perform a comprehensive self-test
             * @return true if all tests pass, false otherwise
             */
            bool performSelfTest();

            /**
             * Get detailed system status for debugging
             * @return JSON string with detailed status
             */
            String getDetailedSystemStatus() const;

            /**
             * Test all hardware components
             * @return true if all components are working
             */
            bool testHardwareComponents();

        private:
            // Components
            NodeConfig _config;
            communication::EbyteCommModule* _commModule;
            radio::RDA5807Radio* _radio;
            display::SSD1306Display* _display;
            input::ButtonHandler* _buttonHandler;

            // State variables
            bool _initialized;
            bool _connected;
            unsigned long _lastStatusTime;
            unsigned long _lastDisplayUpdateTime;
            unsigned long _lastHeartbeatTime;
            unsigned long _lastReceivedMessageTime;
            unsigned long _lastSelfTestTime;
            int _connectionSignalStrength;
            uint16_t _messageSequence;
            bool _hasSensors;
            float _temperature;
            float _humidity;
            bool _verboseDebugging;

            // Statistics
            NodeStatistics _stats;

            // Pin definitions (can be adjusted in constructor)
            int _pinRelayControl;
            int _pinUserButton;
            int _pinSDASensor;
            int _pinSCLSensor;
            int _pinM0;
            int _pinM1;
            int _pinAUX;

            // Constants
            static constexpr unsigned long STATUS_INTERVAL = 5000;        // 5 seconds
            static constexpr unsigned long DISPLAY_UPDATE_INTERVAL = 250; // 250 ms
            static constexpr unsigned long HEARTBEAT_INTERVAL = 30000;    // 30 seconds
            static constexpr unsigned long CONNECTION_TIMEOUT = 60000;    // 60 seconds
            static constexpr unsigned long SELF_TEST_INTERVAL = 300000;   // 5 minutes

            /**
             * Set up button callbacks
             */
            void setupButtonCallbacks();

            /**
             * Update the display with current state
             */
            void updateDisplay();

            /**
             * Initialize sensors if available
             * @return true if initialization was successful, false otherwise
             */
            bool initializeSensors();

            /**
             * Read sensor data if available
             */
            void readSensors();

            /**
             * Apply volume setting to the radio
             */
            void applyVolume();

            /**
             * Apply relay state
             */
            void applyRelayState();

            /**
             * Check connection status
             */
            void checkConnection();

            /**
             * Process received messages
             * @return true if a message was processed, false otherwise
             */
            bool processMessages();

            /**
             * Send a heartbeat message to the controller
             * @return true if the message was sent successfully, false otherwise
             */
            bool sendHeartbeat();

            /**
             * Process a SET_VOLUME command with detailed logging
             * @param volume New volume value
             * @return true if the command was processed successfully, false otherwise
             */
            bool processSetVolumeCommand(uint8_t volume);

            /**
             * Process a SET_FREQUENCY command with detailed logging
             * @param frequency New frequency value
             * @return true if the command was processed successfully, false otherwise
             */
            bool processSetFrequencyCommand(uint16_t frequency);

            /**
             * Process a TOGGLE_RELAY command with detailed logging
             * @param state New relay state
             * @return true if the command was processed successfully, false otherwise
             */
            bool processToggleRelayCommand(bool state);

            /**
             * Process a MUTE command with detailed logging
             * @param mute Whether to mute or unmute
             * @return true if the command was processed successfully, false otherwise
             */
            bool processMuteCommand(bool mute);

            /**
             * Send an acknowledgement for a received message
             * @param sequenceNum Sequence number of the message to acknowledge
             * @param success Whether the command was successful
             * @param errorCode Error code if the command was not successful
             * @return true if the acknowledgement was sent successfully, false otherwise
             */
            bool sendAcknowledgement(uint16_t sequenceNum, bool success, uint8_t errorCode = 0);

            /**
             * Update node statistics
             */
            void updateStatistics();

            /**
             * Log detailed message information
             */
            void logMessageDetails(const String& direction, const String& messageType,
                                   const void* message, size_t length);

            /**
             * Log radio status information
             */
            void logRadioStatus();

            /**
             * Log communication status information
             */
            void logCommunicationStatus();

            /**
             * Perform periodic self-tests
             */
            void performPeriodicSelfTest();

            /**
             * Test I2C bus and devices
             * @return true if I2C is working correctly
             */
            bool testI2CBus();

            /**
             * Test radio functionality
             * @return true if radio is working correctly
             */
            bool testRadio();

            /**
             * Test display functionality
             * @return true if display is working correctly
             */
            bool testDisplay();

            /**
             * Test communication module
             * @return true if communication is working correctly
             */
            bool testCommunication();

            /**
             * Show startup information on display
             */
            void showStartupInfo();

            /**
             * Show error information on display
             */
            void showErrorInfo(const String& error);
        };

    } // namespace node
} // namespace szogfm

#endif // SZOGFM_NODEAPPLICATION_H