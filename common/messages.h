#ifndef SZOGFM_MESSAGES_H
#define SZOGFM_MESSAGES_H

#include <Arduino.h>
#include <cstdint>

namespace szogfm {

// Constants for system configuration
    constexpr uint8_t MAX_NODES = 20;
    constexpr uint16_t DEFAULT_FM_FREQUENCY = 8850; // 88.50 MHz (stored as 8850)
    constexpr uint8_t MAX_VOLUME = 15; // Maximum volume level
    constexpr uint16_t MESSAGE_TIMEOUT_MS = 1000; // Timeout for message acknowledgement
    constexpr uint8_t MESSAGE_RETRY_COUNT = 3; // Number of times to retry sending a message

// Message types for 433MHz communication
    enum class MessageType : uint8_t {
        COMMAND = 0x01,        // Command message from controller to node
        STATUS_REQUEST = 0x02, // Request for node status
        STATUS_RESPONSE = 0x03, // Node status response
        ACK = 0x04,            // Acknowledge message receipt
        ERROR = 0xFF           // Error message
    };

// Commands that can be sent to nodes
    enum class Command : uint8_t {
        SET_VOLUME = 0x01,     // Set the volume level
        SET_FREQUENCY = 0x02,  // Set the FM frequency
        TOGGLE_RELAY = 0x03,   // Toggle the relay (on/off)
        RESET = 0x04,          // Reset the node
        GET_STATUS = 0x05,     // Request node status
        MUTE = 0x06,           // Mute audio
        UNMUTE = 0x07          // Unmute audio
    };

// Status of a node
    enum class NodeStatus : uint8_t {
        OK = 0x00,             // Node is functioning normally
        FM_ERROR = 0x01,       // Error with FM receiver
        COMM_ERROR = 0x02,     // Communication error
        LOW_POWER = 0x03,      // Low power warning
        OTHER_ERROR = 0xFF     // Other error
    };

// Base message header for all communications
#pragma pack(push, 1) // Ensure struct is packed without padding
    struct MessageHeader {
        uint8_t version = 1;           // Protocol version
        MessageType type;              // Type of message
        uint8_t nodeId;                // Target/source node ID (0 = broadcast)
        uint16_t sequenceNum;          // Sequence number for tracking messages
        uint16_t payloadLength;        // Length of payload in bytes
        uint32_t timestamp;            // Message timestamp (milliseconds since boot)
        uint8_t checksum;              // Simple checksum for message validation

        // Compute checksum for this header (excluding the checksum field itself)
        uint8_t computeChecksum() const {
            const uint8_t* data = reinterpret_cast<const uint8_t*>(this);
            uint8_t sum = 0;

            // Sum all bytes in the header except checksum
            for (size_t i = 0; i < sizeof(MessageHeader) - 1; i++) {
                sum += data[i];
            }
            return sum;
        }

        // Validate the received header's checksum
        bool validateChecksum() const {
            return checksum == computeChecksum();
        }

        // Set the checksum field based on current header values
        void setChecksum() {
            checksum = computeChecksum();
        }
    };

// Command message structure
    struct CommandMessage {
        MessageHeader header;
        Command command;
        uint8_t data[4];  // Command-specific data (up to 4 bytes)

        // Helper methods for setting specific command data
        void setVolumeCommand(uint8_t volume) {
            command = Command::SET_VOLUME;
            data[0] = volume;
        }

        void setFrequencyCommand(uint16_t frequency) {
            command = Command::SET_FREQUENCY;
            data[0] = frequency & 0xFF;
            data[1] = (frequency >> 8) & 0xFF;
        }

        void setRelayCommand(bool state) {
            command = Command::TOGGLE_RELAY;
            data[0] = state ? 1 : 0;
        }
    };

// Status response message structure
    struct StatusMessage {
        MessageHeader header;
        NodeStatus status;          // Current node status
        uint8_t volume;             // Current volume level
        uint16_t frequency;         // Current frequency (multiply by 0.01 for MHz)
        bool relayState;            // Current relay state
        int16_t rssi;               // Signal strength (dBm) of FM signal
        bool isStereo;              // Whether FM signal is stereo
        uint32_t uptime;            // Milliseconds since node boot
        float temperature;          // Temperature in Celsius (if available, -100 if not)
        float humidity;             // Humidity percentage (if available, -1 if not)
    };

// Acknowledgment message structure
    struct AckMessage {
        MessageHeader header;
        uint16_t acknowledgedSeq;   // Sequence number being acknowledged
        bool success;               // Whether the command was successful
        uint8_t errorCode;          // Error code if not successful
    };

#pragma pack(pop)

// Function to calculate CRC checksum for a data buffer
    inline uint16_t calculateCRC(const uint8_t* data, size_t length) {
        uint16_t crc = 0xFFFF;

        for (size_t i = 0; i < length; i++) {
            crc ^= data[i];
            for (uint8_t j = 0; j < 8; j++) {
                if (crc & 0x0001) {
                    crc = (crc >> 1) ^ 0xA001;
                } else {
                    crc = crc >> 1;
                }
            }
        }

        return crc;
    }

} // namespace szogfm

#endif // SZOGFM_MESSAGES_H