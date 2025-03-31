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

            // Buffer for received messages
            static constexpr size_t RX_BUFFER_SIZE = 256;
            uint8_t _rxBuffer[RX_BUFFER_SIZE];
            size_t _rxBufferLen;
            bool _hasMessage;

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
        };

    } // namespace communication
} // namespace szogfm

#endif // SZOGFM_EBYTECOMMMODULE_H