#ifndef SZOGFM_SPISETUP_H
#define SZOGFM_SPISETUP_H

#include <Arduino.h>
#include <SPI.h>

namespace szogfm {
    namespace display {

/**
 * Helper class for setting up SPI with specific pins
 */
        class SpiSetup {
        public:
            /**
             * Configure the SPI bus with the specified pins
             * @param clkPin SPI clock pin
             * @param misoPin SPI MISO pin
             * @param mosiPin SPI MOSI pin
             * @param csPin SPI chip select pin
             * @return true if successful, false otherwise
             */
            static bool configureSpi(int8_t clkPin, int8_t misoPin, int8_t mosiPin, int8_t csPin) {
                // Initialize SPI with specific pins
                SPI.begin(clkPin, misoPin, mosiPin, csPin);

                // Set SPI parameters
                SPI.setFrequency(8000000); // 8MHz
                SPI.setDataMode(SPI_MODE0);
                SPI.setBitOrder(MSBFIRST);

                return true;
            }

            /**
             * Configure VSPI with our custom pins on ESP32
             * Custom VSPI pins: CLK=18, MISO=19, MOSI=23, CS=35
             * @return true if successful, false otherwise
             */
            static bool configureVspi() {
                return configureSpi(18, 19, 23, 35);
            }

            /**
             * Configure HSPI with default pins on ESP32
             * HSPI default pins: CLK=14, MISO=12, MOSI=13, CS=15
             * @return true if successful, false otherwise
             */
            static bool configureHspi() {
                return configureSpi(14, 12, 13, 15);
            }
        };

    } // namespace display
} // namespace szogfm

#endif // SZOGFM_SPISETUP_H