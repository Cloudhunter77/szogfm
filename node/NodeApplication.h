#ifndef SZOGFM_NODEAPPLICATION_H
#define SZOGFM_NODEAPPLICATION_H

#include "../common/messages.h"
#include "../common/NodeConfig.h"
#include "../common/communication/EbyteCommModule.h"
#include "../common/radio/RDA5807Radio.h"
#include "../common/display/ST7789Display.h"
#include "../common/input/ButtonHandler.h"

namespace szogfm {
    namespace node {

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

        private:
            // Components
            NodeConfig _config;
            communication::EbyteCommModule* _commModule;
            radio::RDA5807Radio* _radio;
            display::ST7789Display* _display;
            input::ButtonHandler* _buttonHandler;

            // State variables
            bool _initialized;
            bool _connected;
            unsigned long _lastStatusTime;
            unsigned long _lastDisplayUpdateTime;
            unsigned long _lastHeartbeatTime;
            unsigned long _lastReceivedMessageTime;
            int _connectionSignalStrength;
            uint16_t _messageSequence;
            bool _hasSensors;
            float _temperature;
            float _humidity;

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
             * Process a SET_VOLUME command
             * @param volume New volume value
             * @return true if the command was processed successfully, false otherwise
             */
            bool processSetVolumeCommand(uint8_t volume);

            /**
             * Process a SET_FREQUENCY command
             * @param frequency New frequency value
             * @return true if the command was processed successfully, false otherwise
             */
            bool processSetFrequencyCommand(uint16_t frequency);

            /**
             * Process a TOGGLE_RELAY command
             * @param state New relay state
             * @return true if the command was processed successfully, false otherwise
             */
            bool processToggleRelayCommand(bool state);

            /**
             * Process a MUTE command
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
        };

    } // namespace node
} // namespace szogfm

#endif // SZOGFM_NODEAPPLICATION_H