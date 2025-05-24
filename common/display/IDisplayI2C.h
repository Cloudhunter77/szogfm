#ifndef SZOGFM_IDISPLAYI2C_H
#define SZOGFM_IDISPLAYI2C_H

#include "IDisplay.h"

namespace szogfm {
    namespace display {

/**
 * Interface for I2C display modules
 * This extends the base IDisplay interface with I2C-specific functionality
 */
        class IDisplayI2C : public IDisplay {
        public:
            virtual ~IDisplayI2C() = default;

            /**
             * Set the I2C address for the display
             * @param address I2C address
             * @return true if operation was successful, false otherwise
             */
            virtual bool setI2CAddress(uint8_t address) = 0;

            /**
             * Set the I2C clock frequency
             * @param frequency Clock frequency in Hz
             * @return true if operation was successful, false otherwise
             */
            virtual bool setI2CFrequency(uint32_t frequency) = 0;
        };

    } // namespace display
} // namespace szogfm

#endif // SZOGFM_IDISPLAYI2C_H