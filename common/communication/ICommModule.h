#ifndef SZOGFM_ICOMMMODULE_H
#define SZOGFM_ICOMMMODULE_H

#include <Arduino.h>
#include "../messages.h"

namespace szogfm {
    namespace communication {

/**
 * Interface for communication modules
 * This abstract class defines the interface that all communication modules must implement,
 * providing a standard way to send and receive messages regardless of the physical medium.
 */
        class ICommModule {
        public:
            virtual ~ICommModule() = default;

            /**
             * Initialize the communication module
             * @return true if initialization was successful, false otherwise
             */
            virtual bool initialize() = 0;

            /**
             * Send a message to a specific node
             * @param nodeId ID of the node to send the message to (0 for broadcast)
             * @param message Pointer to the message to send
             * @param length Length of the message in bytes
             * @return true if the message was sent successfully, false otherwise
             */
            virtual bool sendMessage(uint8_t nodeId, const void* message, size_t length) = 0;

            /**
             * Check if there is a message available to be received
             * @return true if a message is available, false otherwise
             */
            virtual bool isMessageAvailable() = 0;

            /**
             * Receive a message
             * @param buffer Buffer to store the received message
             * @param maxLength Maximum length of the buffer
             * @param[out] senderNodeId ID of the node that sent the message
             * @return Number of bytes received, 0 if no message was available or error occurred
             */
            virtual size_t receiveMessage(void* buffer, size_t maxLength, uint8_t& senderNodeId) = 0;

            /**
             * Get the last error that occurred
             * @return String describing the last error
             */
            virtual String getLastError() const = 0;

            /**
             * Check if the module is currently transmitting
             * @return true if the module is transmitting, false otherwise
             */
            virtual bool isTransmitting() const = 0;

            /**
             * Get the estimated signal strength of the last received message
             * @return Signal strength in dBm (usually negative, more negative = weaker signal)
             */
            virtual int getSignalStrength() const = 0;

            /**
             * Set the transmission power
             * @param level Power level (implementation-specific)
             * @return true if power was set successfully, false otherwise
             */
            virtual bool setTransmissionPower(uint8_t level) = 0;

            /**
             * Perform routine maintenance tasks
             * This method should be called regularly to allow the module to perform
             * any necessary maintenance tasks like buffer cleanup, etc.
             */
            virtual void update() = 0;
        };

    } // namespace communication
} // namespace szogfm

#endif // SZOGFM_ICOMMMODULE_H