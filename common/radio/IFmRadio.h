#ifndef SZOGFM_IFMRADIO_H
#define SZOGFM_IFMRADIO_H

#include <Arduino.h>

namespace szogfm {
    namespace radio {

/**
 * Interface for FM radio modules
 * This abstract class defines the interface that all FM radio modules must implement,
 * providing a standard way to control FM radios regardless of the specific chip.
 */
        class IFmRadio {
        public:
            virtual ~IFmRadio() = default;

            /**
             * Initialize the FM radio module
             * @return true if initialization was successful, false otherwise
             */
            virtual bool initialize() = 0;

            /**
             * Set the FM frequency
             * @param frequency FM frequency in 10kHz units (e.g., 8850 for 88.5 MHz)
             * @return true if frequency was set successfully, false otherwise
             */
            virtual bool setFrequency(uint16_t frequency) = 0;

            /**
             * Get the current FM frequency
             * @return Current frequency in 10kHz units
             */
            virtual uint16_t getFrequency() const = 0;

            /**
             * Set the volume level
             * @param volume Volume level (0-15)
             * @return true if volume was set successfully, false otherwise
             */
            virtual bool setVolume(uint8_t volume) = 0;

            /**
             * Get the current volume level
             * @return Current volume level
             */
            virtual uint8_t getVolume() const = 0;

            /**
             * Mute or unmute the audio output
             * @param mute true to mute, false to unmute
             * @return true if operation was successful, false otherwise
             */
            virtual bool setMute(bool mute) = 0;

            /**
             * Check if the audio is muted
             * @return true if muted, false otherwise
             */
            virtual bool isMuted() const = 0;

            /**
             * Set mono or stereo reception mode
             * @param mono true for mono, false for stereo
             * @return true if operation was successful, false otherwise
             */
            virtual bool setMono(bool mono) = 0;

            /**
             * Check if the current reception is stereo
             * @return true if stereo, false if mono
             */
            virtual bool isStereo() const = 0;

            /**
             * Get the signal strength (RSSI) of the current station
             * @return Signal strength value (implementation-specific)
             */
            virtual int getRssi() const = 0;

            /**
             * Seek to the next available station
             * @param seekUp true to seek up, false to seek down
             * @return true if a station was found, false otherwise
             */
            virtual bool seek(bool seekUp) = 0;

            /**
             * Check if a station is currently tuned in
             * @return true if a station is tuned in, false otherwise
             */
            virtual bool isTunedToStation() const = 0;

            /**
             * Get the last error that occurred
             * @return String describing the last error
             */
            virtual String getLastError() const = 0;

            /**
             * Perform routine maintenance tasks
             * This method should be called regularly to allow the module to perform
             * any necessary maintenance tasks.
             */
            virtual void update() = 0;
        };

    } // namespace radio
} // namespace szogfm

#endif // SZOGFM_IFMRADIO_H