#ifndef SZOGFM_NODECONFIG_H
#define SZOGFM_NODECONFIG_H

#include <Arduino.h>
#include <EEPROM.h>

namespace szogfm {

/**
 * Class for storing and retrieving node configuration
 * This class handles reading/writing to ESP32 preferences for persistent storage
 */
    class NodeConfig {
    public:
        /**
         * Constructor
         * @param nodeId Default node ID (used if no stored value exists)
         */
        NodeConfig(uint8_t defaultNodeId = 1);

        /**
         * Load configuration from EEPROM
         * @return true if loaded successfully, false otherwise
         */
        bool load();

        /**
         * Save configuration to EEPROM
         * @return true if saved successfully, false otherwise
         */
        bool save();

        /**
         * Reset configuration to factory defaults
         * @param save Whether to save the reset configuration to EEPROM
         * @return true if reset successfully, false otherwise
         */
        bool reset(bool save = true);

        /**
         * Get the node ID
         * @return Node ID
         */
        uint8_t getNodeId() const { return _nodeId; }

        /**
         * Set the node ID
         * @param nodeId New node ID
         */
        void setNodeId(uint8_t nodeId) { _nodeId = nodeId; _modified = true; }

        /**
         * Get the FM frequency
         * @return FM frequency in 10kHz units (e.g., 8850 for 88.5 MHz)
         */
        uint16_t getFmFrequency() const { return _fmFrequency; }

        /**
         * Set the FM frequency
         * @param frequency FM frequency in 10kHz units
         */
        void setFmFrequency(uint16_t frequency) { _fmFrequency = frequency; _modified = true; }

        /**
         * Get the volume level
         * @return Volume level (0-15)
         */
        uint8_t getVolume() const { return _volume; }

        /**
         * Set the volume level
         * @param volume Volume level (0-15)
         */
        void setVolume(uint8_t volume) { _volume = volume; _modified = true; }

        /**
         * Get the mute state
         * @return true if muted, false otherwise
         */
        bool isMuted() const { return _muted; }

        /**
         * Set the mute state
         * @param muted true to mute, false to unmute
         */
        void setMuted(bool muted) { _muted = muted; _modified = true; }

        /**
         * Get the relay state
         * @return true if relay is on, false otherwise
         */
        bool getRelayState() const { return _relayState; }

        /**
         * Set the relay state
         * @param state true to turn on, false to turn off
         */
        void setRelayState(bool state) { _relayState = state; _modified = true; }

        /**
         * Get the radio communication address
         * @return Radio address
         */
        uint16_t getRadioAddress() const { return _radioAddress; }

        /**
         * Set the radio communication address
         * @param address Radio address
         */
        void setRadioAddress(uint16_t address) { _radioAddress = address; _modified = true; }

        /**
         * Get the radio communication channel
         * @return Radio channel
         */
        uint8_t getRadioChannel() const { return _radioChannel; }

        /**
         * Set the radio communication channel
         * @param channel Radio channel
         */
        void setRadioChannel(uint8_t channel) { _radioChannel = channel; _modified = true; }

        /**
         * Get the OLED display brightness
         * @return Display brightness (0-255)
         */
        uint8_t getDisplayBrightness() const { return _displayBrightness; }

        /**
         * Set the OLED display brightness
         * @param brightness Display brightness (0-255)
         */
        void setDisplayBrightness(uint8_t brightness) { _displayBrightness = brightness; _modified = true; }

        /**
         * Check if the configuration has been modified since the last save
         * @return true if modified, false otherwise
         */
        bool isModified() const { return _modified; }

        /**
         * Get the configuration version
         * @return Configuration version
         */
        uint8_t getVersion() const { return _version; }

    private:
        // Configuration data
        uint8_t _version;            // Configuration version
        uint8_t _nodeId;             // Node ID
        uint16_t _fmFrequency;       // FM frequency in 10kHz units
        uint8_t _volume;             // Volume level (0-15)
        bool _muted;                 // Mute state
        bool _relayState;            // Relay state
        uint16_t _radioAddress;      // Radio communication address
        uint8_t _radioChannel;       // Radio communication channel
        uint8_t _displayBrightness;  // OLED display brightness

        // Internal state
        bool _modified;              // Whether the configuration has been modified

        // Configuration storage
        static constexpr size_t EEPROM_SIZE = 64;
        static constexpr uint32_t CONFIG_MAGIC = 0x53464D00; // "SFM" + version 0
        static constexpr uint8_t CURRENT_VERSION = 1;

        // EEPROM layout
        struct ConfigLayout {
            uint32_t magic;         // Magic number to validate configuration
            uint8_t version;        // Configuration version
            uint8_t nodeId;         // Node ID
            uint16_t fmFrequency;   // FM frequency
            uint8_t volume;         // Volume level
            uint8_t flags;          // Bit flags for boolean settings
            uint16_t radioAddress;  // Radio address
            uint8_t radioChannel;   // Radio channel
            uint8_t displayBrightness; // Display brightness
            uint8_t reserved[8];    // Reserved for future use
            uint8_t checksum;       // Simple checksum
        };

        // Flag bit positions
        static constexpr uint8_t FLAG_MUTED = 0;
        static constexpr uint8_t FLAG_RELAY_STATE = 1;

        /**
         * Calculate checksum for configuration data
         * @param config Configuration data
         * @return Checksum
         */
        uint8_t calculateChecksum(const ConfigLayout& config) const;
    };

} // namespace szogfm

#endif // SZOGFM_NODECONFIG_H