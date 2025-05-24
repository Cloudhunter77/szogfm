#ifndef SZOGFM_EBYTECOMMMODULE_H
#define SZOGFM_EBYTECOMMMODULE_H

#include "ICommModule.h"
#include "EbyteConstants.h"
#include <EBYTE.h>

namespace szogfm {
    namespace communication {

/**
 * Implementation of the communication module interface for EBYTE 433MHz modules
 */
        class EbyteCommModule : public ICommModule {
        public:
            /**
             * Constructor
             * @param serial Serial interface to use for communication with the EBYTE module
             * @param pinM0 M0 pin connected to the module
             * @param pinM1 M1 pin connected to the module
             * @param pinAUX AUX pin connected to the module
             */
            EbyteCommModule(HardwareSerial* serial, int pinM0, int pinM1, int pinAUX);

            /**
             * Destructor
             */
            ~EbyteCommModule() override;

            // Interface implementations
            bool initialize() override;
            bool sendMessage(uint8_t nodeId, const void* message, size_t length) override;
            bool isMessageAvailable() override;
            size_t receiveMessage(void* buffer, size_t maxLength, uint8_t& senderNodeId) override;
            String getLastError() const override;
            bool isTransmitting() const override;
            int getSignalStrength() const override;
            bool setTransmissionPower(uint8_t level) override;
            void update() override;

            /**
             * Configure module parameters
             * @param channel RF channel to use (0-31)
             * @param address Module address (0-65535)
             * @param airDataRate Air data rate (see EBYTE documentation)
             * @param uartBaud UART baud rate (see EBYTE documentation)
             * @param powerLevel Transmission power level (0-3)
             * @return true if configuration was successful, false otherwise
             */
            bool configure(uint8_t channel, uint16_t address, uint8_t airDataRate,
                           uint8_t uartBaud, uint8_t powerLevel);

            /**
             * Print module parameters to Serial (for debugging)
             */
            void printParameters();

            /**
             * Set debug level for verbose logging
             * @param level Debug level (0=off, 1=basic, 2=verbose with hex dumps)
             */
            void setDebugLevel(uint8_t level);

            /**
             * Perform diagnostics on the module
             * This tests various aspects of the module's functionality
             */
            void performDiagnostics();

        private:
            EBYTE* _ebyte;
            HardwareSerial* _serial;
            int _pinM0;
            int _pinM1;
            int _pinAUX;
            String _lastError;
            bool _initialized;
            int _lastRssi;
            uint8_t _channel;
            uint16_t _address;
            uint8_t _debugLevel;

            // Buffer for received messages
            static constexpr size_t RX_BUFFER_SIZE = 256;
            uint8_t _rxBuffer[RX_BUFFER_SIZE];
            size_t _rxBufferLen;
            bool _hasMessage;

            /**
             * Test if the module is responding to basic communication
             * @return true if module responds correctly, false otherwise
             */
            bool testCommunication();

            /**
             * Set the operating mode of the EBYTE module
             * @param mode Operating mode (0-3)
             * @return true if mode was set successfully, false otherwise
             */
            bool setMode(uint8_t mode);

            /**
             * Wait for the AUX pin to go high (module ready)
             * @param timeout Timeout in milliseconds
             * @return true if AUX pin went high within timeout, false otherwise
             */
            bool waitForAUX(unsigned long timeout = 1000);

            /**
             * Dump message contents as hex for debugging
             * @param message Message data pointer
             * @param length Message length in bytes
             */
            void dumpMessageHex(const void* message, size_t length);

            /**
             * Test mode transitions functionality
             * @return true if mode switching works correctly
             */
            bool testModeTransitions();

            /**
             * Test configuration mode access
             * @return true if configuration mode can be accessed
             */
            bool testConfigurationMode();

            /**
             * Test parameter reading capability
             * @return true if parameters can be read from the module
             */
            bool testParameterReading();

            /**
             * Set mode with automatic retries
             * @param mode Target operating mode
             * @param maxRetries Maximum number of retry attempts
             * @return true if mode was set successfully
             */
            bool setModeWithRetries(uint8_t mode, int maxRetries);

            /**
             * Wait for AUX pin with timeout and detailed logging
             * @param timeout Timeout in milliseconds
             * @return true if AUX went high within timeout
             */
            bool waitForAUXWithTimeout(unsigned long timeout);

            /**
             * Get current mode as human-readable string
             * @return String describing current mode
             */
            String getCurrentModeString() const;

            /**
             * Get mode name as string
             * @param mode Mode number
             * @return String describing the mode
             */
            String getModeString(uint8_t mode) const;
        };

    } // namespace communication
} // namespace szogfm

#endif // SZOGFM_EBYTECOMMMODULE_H