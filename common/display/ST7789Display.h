#ifndef SZOGFM_ST7789DISPLAY_H
#define SZOGFM_ST7789DISPLAY_H

#include "IDisplaySPI.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

namespace szogfm {
namespace display {

/**
 * Implementation of the display interface for ST7789 TFT displays
 */
class ST7789Display : public IDisplaySPI {
public:
    /**
     * Constructor
     * @param width Display width in pixels
     * @param height Display height in pixels
     * @param clkPin SPI clock pin
     * @param misoPin SPI MISO pin (not used directly by display)
     * @param mosiPin SPI MOSI pin
     * @param csPin SPI chip select pin
     * @param dcPin Data/Command pin
     * @param resetPin Reset pin
     */
    ST7789Display(uint16_t width = 240, uint16_t height = 240, 
                 int8_t clkPin = 18, int8_t misoPin = 19, int8_t mosiPin = 23, 
                 int8_t csPin = 35, int8_t dcPin = 25, int8_t resetPin = 26);
    
    /**
     * Destructor
     */
    ~ST7789Display() override;
    
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
     * Set the display rotation
     * @param rotation Rotation value (0-3)
     * @return true if successful, false otherwise
     */
    bool setRotation(uint8_t rotation);
    
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
    Adafruit_ST7789* _display;
    int8_t _clkPin;
    int8_t _misoPin;
    int8_t _mosiPin;
    int8_t _csPin;
    int8_t _dcPin;
    int8_t _resetPin;
    uint16_t _width;
    uint16_t _height;
    uint8_t _fontSize;
    bool _autoUpdate;
    mutable String _lastError;
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
    
    // Color definitions
    static const uint16_t COLOR_BLACK = 0x0000;
    static const uint16_t COLOR_WHITE = 0xFFFF;
    static const uint16_t COLOR_RED = 0xF800;
    static const uint16_t COLOR_GREEN = 0x07E0;
    static const uint16_t COLOR_BLUE = 0x001F;
    static const uint16_t COLOR_YELLOW = 0xFFE0;
    static const uint16_t COLOR_CYAN = 0x07FF;
    static const uint16_t COLOR_MAGENTA = 0xF81F;
    
    // Layout constants
    static constexpr int FREQUENCY_Y = 20;
    static constexpr int VOLUME_Y = 70;
    static constexpr int STATUS_Y = 120;
    static constexpr int INFO_Y = 170;
    
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

#endif // SZOGFM_ST7789DISPLAY_H