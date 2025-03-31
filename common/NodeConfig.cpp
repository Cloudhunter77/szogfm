#include "NodeConfig.h"

namespace szogfm {

    NodeConfig::NodeConfig(uint8_t defaultNodeId)
            : _version(CURRENT_VERSION), _nodeId(defaultNodeId), _fmFrequency(8850),
              _volume(8), _muted(false), _relayState(false),
              _radioAddress(0x1234), _radioChannel(0x1A), _displayBrightness(128),
              _modified(false) {
    }

    bool NodeConfig::load() {
        // Initialize EEPROM
        if (!EEPROM.begin(EEPROM_SIZE)) {
            return false;
        }

        // Read configuration data from EEPROM
        ConfigLayout config;
        EEPROM.readBytes(0, &config, sizeof(ConfigLayout));

        // Validate magic number
        if (config.magic != CONFIG_MAGIC) {
            // No valid configuration found, initialize with defaults
            reset(false); // Don't save yet
            return false;
        }

        // Validate checksum
        uint8_t checksum = calculateChecksum(config);
        if (checksum != config.checksum) {
            // Invalid checksum, configuration may be corrupted
            reset(false); // Don't save yet
            return false;
        }

        // Load configuration values
        _version = config.version;
        _nodeId = config.nodeId;
        _fmFrequency = config.fmFrequency;
        _volume = config.volume;
        _muted = (config.flags & (1 << FLAG_MUTED)) != 0;
        _relayState = (config.flags & (1 << FLAG_RELAY_STATE)) != 0;
        _radioAddress = config.radioAddress;
        _radioChannel = config.radioChannel;
        _displayBrightness = config.displayBrightness;

        _modified = false;

        return true;
    }

    bool NodeConfig::save() {
        // Prepare configuration data
        ConfigLayout config;
        config.magic = CONFIG_MAGIC;
        config.version = _version;
        config.nodeId = _nodeId;
        config.fmFrequency = _fmFrequency;
        config.volume = _volume;

        // Pack boolean flags
        config.flags = 0;
        if (_muted) config.flags |= (1 << FLAG_MUTED);
        if (_relayState) config.flags |= (1 << FLAG_RELAY_STATE);

        config.radioAddress = _radioAddress;
        config.radioChannel = _radioChannel;
        config.displayBrightness = _displayBrightness;

        // Clear reserved bytes
        memset(config.reserved, 0, sizeof(config.reserved));

        // Calculate and set checksum
        config.checksum = calculateChecksum(config);

        // Write to EEPROM
        EEPROM.writeBytes(0, &config, sizeof(ConfigLayout));
        bool success = EEPROM.commit();

        if (success) {
            _modified = false;
        }

        return success;
    }

    bool NodeConfig::reset(bool save) {
        // Reset to default values
        _version = CURRENT_VERSION;
        // Keep the existing node ID
        _fmFrequency = 8850; // 88.5 MHz
        _volume = 8;
        _muted = false;
        _relayState = false;
        _radioAddress = 0x1234;
        _radioChannel = 0x1A;
        _displayBrightness = 128;

        _modified = true;

        // Save to EEPROM if requested
        if (save) {
            return this->save();
        }

        return true;
    }

    uint8_t NodeConfig::calculateChecksum(const ConfigLayout& config) const {
        // Calculate simple XOR checksum over all bytes except the checksum itself
        const uint8_t* data = reinterpret_cast<const uint8_t*>(&config);
        uint8_t checksum = 0;

        // Skip the last byte (the checksum itself)
        for (size_t i = 0; i < sizeof(ConfigLayout) - 1; i++) {
            checksum ^= data[i];
        }

        return checksum;
    }

} // namespace szogfm