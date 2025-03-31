#ifndef SZOGFM_IDISPLAYSPI_H
#define SZOGFM_IDISPLAYSPI_H

#include "IDisplay.h"

namespace szogfm {
    namespace display {

/**
 * Interface for SPI display modules
 * This extends the base IDisplay interface with SPI-specific functionality
 */
        class IDisplaySPI : public IDisplay {
        public:
            virtual ~IDisplaySPI() = default;

            /**
             * Set the SPI Clock Divider for the display
             * @param divider SPI clock divider value
             * @return true if operation was successful, false otherwise
             */
            virtual bool setSpiClockDivider(uint8_t divider) = 0;

            /**
             * Enable or disable hardware SPI
             * @param enable true to enable hardware SPI, false for software SPI
             * @return true if operation was successful, false otherwise
             */
            virtual bool setHardwareSpi(bool enable) = 0;
        };

    } // namespace display
} // namespace szogfm

#endif // SZOGFM_IDISPLAYSPI_H