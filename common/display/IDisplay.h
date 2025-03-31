#ifndef SZOGFM_IDISPLAY_H
#define SZOGFM_IDISPLAY_H

#include <Arduino.h>

namespace szogfm {
    namespace display {

/**
 * Interface for display modules
 * This abstract class defines the interface that all display modules must implement,
 * providing a standard way to display information.
 */
        class IDisplay {
        public:
            virtual ~IDisplay() = default;

            /**
             * Initialize the display
             * @return true if initialization was successful, false otherwise
             */
            virtual bool initialize() = 0;

            /**
             * Clear the display
             * @return true if operation was successful, false otherwise
             */
            virtual bool clear() = 0;

            /**
             * Display the FM frequency
             * @param frequency FM frequency in 10kHz units (e.g., 8850 for 88.5 MHz)
             * @return true if operation was successful, false otherwise
             */
            virtual bool displayFrequency(uint16_t frequency) = 0;

            /**
             * Display the volume level
             * @param volume Volume level (0-15)
             * @param muted Whether the audio is muted
             * @return true if operation was successful, false otherwise
             */
            virtual bool displayVolume(uint8_t volume, bool muted) = 0;

            /**
             * Display the connection status
             * @param connected Whether the node is connected to the controller
             * @param signalStrength Signal strength (0-100)
             * @return true if operation was successful, false otherwise
             */
            virtual bool displayConnectionStatus(bool connected, int signalStrength) = 0;

            /**
             * Display sensor data (temperature and humidity)
             * @param temperature Temperature in Celsius
             * @param humidity Humidity percentage
             * @return true if operation was successful, false otherwise
             */
            virtual bool displaySensorData(float temperature, float humidity) = 0;

            /**
             * Display an error message
             * @param error Error message to display
             * @return true if operation was successful, false otherwise
             */
            virtual bool displayError(const String& error) = 0;

            /**
             * Display node information
             * @param nodeId Node ID
             * @param relayState Relay state (true = on, false = off)
             * @return true if operation was successful, false otherwise
             */
            virtual bool displayNodeInfo(uint8_t nodeId, bool relayState) = 0;

            /**
             * Update the display with the current state
             * This method should be called regularly to refresh the display.
             * @return true if operation was successful, false otherwise
             */
            virtual bool update() = 0;

            /**
             * Get the last error that occurred
             * @return String describing the last error
             */
            virtual String getLastError() const = 0;
        };

    } // namespace display
} // namespace szogfm

#endif // SZOGFM_IDISPLAY_H