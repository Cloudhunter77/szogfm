#ifndef SZOGFM_OLEDDISPLAYSPI_H
#define SZOGFM_OLEDDISPLAYSPI_H

#include "IDisplaySPI.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

namespace szogfm {
    namespace display {

/**
 * Implementation of the display interface for SSD1306 OLED displays over SPI
 */
        class OledDisplaySPI : public IDisplaySPI {
        public:
            /**
             * Constructor
             * @param width Display width in pixels
             * @param height Display height in pixels
             * @param clkPin SPI clock pin
             * @param misoPin SPI MISO pin (not used for OLED, but needed for SPI bus configuration)
             * @param mosiPin SPI MOSI pin
             * @param csPin SPI chip select pin
             * @param dcPin Data/Command pin
             * @param resetPin Reset pin (-1 if not used)
             */
            OledDisplaySPI(uint8_t width = 128, uint8_t height = 64,
                           int8_t clkPin = 18, int8_t misoPin = 19, int8_t mosiPin = 23,
                           int8_t csPin = 5, int8_t dcPin = 17, int8_t resetPin = -1);

            /**
             * Destructor
             */
            ~OledDisplaySPI() override;

            // Interface implementations from IDisplay
            bool initialize() override;
            bool clear() override;
            bool displayFrequency(uint16_t frequency) override;
            bool displayVolume(uint8_t volume, bool muted) override;
            bool displayConnectionStatus(bool connected, int signalStrength) override;
            bool displaySensorData(float temperature, float humidity) override;
            bool displayError(const String& error) override;
            bool displayNodeInfo(uint8_t nodeId, bool relayState) override;
            bool update() override;
            String getLastError() const override;

            // Interface implementations from IDisplaySPI
            bool setSpiClockDivider(uint8_t divider) override;
            bool setHardwareSpi(bool enable) override;

            /**
             * Enable or disable automatic updates
             * @param enable true to enable, false to disable
             */
            void setAutoUpdate(bool enable);

            /**
             * Set the font size for text
             * @param size Font size multiplier (1-3)
             */
            void setFontSize(uint8_t size);

            /**
             * Set the display contrast
             * @param contrast Contrast value (0-255)
             * @return true if operation was successful, false otherwise
             */
            bool setContrast(uint8_t contrast);

            /**
             * Enable or disable the display
             * @param enable true to enable, false to disable
             * @return true if operation was successful, false otherwise
             */
            bool enableDisplay(bool enable);

            /**
             * Draw a progress bar
             * @param x X coordinate
             * @param y Y coordinate
             * @param width Width of the progress bar
             * @param height Height of the progress bar
             * @param progress Progress value (0-100)
             * @return true if operation was successful, false otherwise
             */
            bool drawProgressBar(int x, int y, int width, int height, int progress);

            /**
             * Display a custom message
             * @param message Message to display
             * @param x X coordinate (optional, default is centered)
             * @param y Y coordinate (optional, default is centered)
             * @param size Font size (optional, default is current font size)
             * @return true if operation was successful, false otherwise
             */
            bool displayMessage(const String& message, int x = -1, int y = -1, uint8_t size = 0);

        private:
            Adafruit_SSD1306* _display;
            int8_t _clkPin;
            int8_t _misoPin;
            int8_t _mosiPin;
            int8_t _csPin;
            int8_t _dcPin;
            int8_t _resetPin;
            uint8_t _width;
            uint8_t _height;
            uint8_t _fontSize;
            bool _autoUpdate;
            mutable String _lastError;
            uint8_t _spiClockDivider;
            bool _useHardwareSpi;

            // Cached display state to avoid redrawing unchanged elements
            uint16_t _cachedFrequency;
            uint8_t _cachedVolume;
            bool _cachedMuted;
            bool _cachedConnected;
            int _cachedSignalStrength;
            float _cachedTemperature;
            float _cachedHumidity;
            uint8_t _cachedNodeId;
            bool _cachedRelayState;
            String _cachedError;
            bool _needsFullRedraw;
            unsigned long _lastUpdateTime;

            // Layout constants
            static constexpr int FREQUENCY_Y = 0;
            static constexpr int VOLUME_Y = 16;
            static constexpr int STATUS_Y = 32;
            static constexpr int INFO_Y = 48;

            /**
             * Draw a battery icon
             * @param x X coordinate
             * @param y Y coordinate
             * @param percentage Battery percentage (0-100)
             * @param width Width of the battery icon
             * @param height Height of the battery icon
             */
            void drawBattery(int x, int y, int percentage, int width = 20, int height = 10);

            /**
             * Draw a signal strength icon
             * @param x X coordinate
             * @param y Y coordinate
             * @param strength Signal strength (0-4)
             */
            void drawSignalStrength(int x, int y, int strength);

            /**
             * Draw a refresh indicator to show the display is actively updating
             */
            void drawRefreshIndicator();

            /**
             * Calculate the centered X position for a text string
             * @param text Text to center
             * @param size Font size
             * @return X coordinate for centered text
             */
            int getCenteredX(const String& text, uint8_t size) const;
        };

    } // namespace display
} // namespace szogfm

#endif // SZOGFM_OLEDDISPLAYSPI_H