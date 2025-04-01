#include "ST7789Display.h"

namespace szogfm {
    namespace display {

        ST7789Display::ST7789Display(uint16_t width, uint16_t height,
                                     int8_t clkPin, int8_t misoPin, int8_t mosiPin,
                                     int8_t csPin, int8_t dcPin, int8_t resetPin)
                : _clkPin(clkPin), _misoPin(misoPin), _mosiPin(mosiPin),
                  _csPin(csPin), _dcPin(dcPin), _resetPin(resetPin),
                  _width(width), _height(height), _fontSize(1), _autoUpdate(true),
                  _useHardwareSpi(true),
                  _cachedFrequency(0), _cachedVolume(0), _cachedMuted(false),
                  _cachedConnected(false), _cachedSignalStrength(0),
                  _cachedTemperature(0.0f), _cachedHumidity(0.0f),
                  _cachedNodeId(0), _cachedRelayState(false),
                  _needsFullRedraw(true)
        {
            _lastUpdateTime = 0;

            // Create ST7789 display instance with hardware SPI
            _display = new Adafruit_ST7789(&SPI, _csPin, _dcPin, _resetPin);
        }

        ST7789Display::~ST7789Display() {
            delete _display;
        }

        bool ST7789Display::initialize() {
            // Initialize the SPI bus if needed
            if (_useHardwareSpi) {
                SPI.begin(_clkPin, _misoPin, _mosiPin, _csPin);
            }

            // Initialize the display
            Serial.println("Initializing ST7789 display...");
            _display->init(_width, _height);

            // Set default rotation and fill with black
            _display->setRotation(0);  // Portrait orientation
            _display->fillScreen(COLOR_BLACK);

            // Set default text properties
            _display->setTextSize(_fontSize);
            _display->setTextColor(COLOR_WHITE);
            _display->setCursor(0, 0);

            // Display startup screen
            _display->fillScreen(COLOR_BLACK);

            _display->setTextSize(2);
            _display->setTextColor(COLOR_GREEN);
            _display->setCursor(getCenteredX("SzogFM", 2), 40);
            _display->println("SzogFM");

            _display->setTextSize(1);
            _display->setTextColor(COLOR_WHITE);
            _display->setCursor(getCenteredX("Remote FM Controller", 1), 80);
            _display->println("Remote FM Controller");

            _display->setTextSize(1);
            _display->setCursor(getCenteredX("Initializing...", 1), 120);
            _display->println("Initializing...");

            delay(2000);

            _display->fillScreen(COLOR_BLACK);
            _display->setTextSize(_fontSize);
            _display->setTextColor(COLOR_WHITE);

            _needsFullRedraw = true;

            Serial.println("ST7789 display initialized successfully");
            return true;
        }

        bool ST7789Display::clear() {
            _display->fillScreen(COLOR_BLACK);
            _needsFullRedraw = true;
            return true;
        }

        bool ST7789Display::displayFrequency(uint16_t frequency) {
            if (frequency == _cachedFrequency && !_needsFullRedraw) {
                return true; // No change, no need to update
            }

            // Format frequency as "XXX.X MHz"
            char freqStr[15];
            sprintf(freqStr, "%d.%d MHz", frequency / 100, (frequency % 100) / 10);

            // Clear the frequency area
            _display->fillRect(0, FREQUENCY_Y, _width, 40, COLOR_BLACK);

            // Display the frequency label
            _display->setTextSize(1);
            _display->setTextColor(COLOR_CYAN);
            _display->setCursor(10, FREQUENCY_Y);
            _display->print("FM FREQUENCY");

            // Display the frequency
            _display->setTextSize(3);
            _display->setTextColor(COLOR_WHITE);
            _display->setCursor(getCenteredX(freqStr, 3), FREQUENCY_Y + 15);
            _display->print(freqStr);

            // Reset text size
            _display->setTextSize(_fontSize);

            _cachedFrequency = frequency;

            if (_autoUpdate) {
                update();
            }

            return true;
        }

        bool ST7789Display::displayVolume(uint8_t volume, bool muted) {
            if (volume == _cachedVolume && muted == _cachedMuted && !_needsFullRedraw) {
                return true; // No change, no need to update
            }

            // Clear the volume area
            _display->fillRect(0, VOLUME_Y, _width, 40, COLOR_BLACK);

            // Display the volume label
            _display->setTextSize(1);
            _display->setTextColor(COLOR_YELLOW);
            _display->setCursor(10, VOLUME_Y);
            _display->print("VOLUME");

            if (muted) {
                // Display muted indicator
                _display->setTextSize(2);
                _display->setTextColor(COLOR_RED);
                _display->setCursor(getCenteredX("MUTED", 2), VOLUME_Y + 15);
                _display->print("MUTED");
            } else {
                // Draw volume bar
                _display->drawRect(20, VOLUME_Y + 15, _width - 40, 15, COLOR_WHITE);
                int fillWidth = ((volume * (_width - 44)) / 15);
                if (fillWidth > 0) {
                    _display->fillRect(22, VOLUME_Y + 17, fillWidth, 11, COLOR_GREEN);
                }

                // Display volume value
                char volStr[5];
                sprintf(volStr, "%d", volume);
                _display->setTextSize(1);
                _display->setTextColor(COLOR_WHITE);
                _display->setCursor(_width - 30, VOLUME_Y + 18);
                _display->print(volStr);
            }

            _cachedVolume = volume;
            _cachedMuted = muted;

            if (_autoUpdate) {
                update();
            }

            return true;
        }

        bool ST7789Display::displayConnectionStatus(bool connected, int signalStrength) {
            if (connected == _cachedConnected && signalStrength == _cachedSignalStrength && !_needsFullRedraw) {
                return true; // No change, no need to update
            }

            // Clear the status area
            _display->fillRect(0, STATUS_Y, _width, 40, COLOR_BLACK);

            // Display connection status label
            _display->setTextSize(1);
            _display->setTextColor(COLOR_MAGENTA);
            _display->setCursor(10, STATUS_Y);
            _display->print("STATUS");

            // Display connection status
            _display->setTextSize(2);
            if (connected) {
                _display->setTextColor(COLOR_GREEN);
                _display->setCursor(20, STATUS_Y + 15);
                _display->print("Connected");

                // Map signal strength to 0-4 scale
                int strength = 0;
                if (signalStrength >= -60) strength = 4;
                else if (signalStrength >= -70) strength = 3;
                else if (signalStrength >= -80) strength = 2;
                else if (signalStrength >= -90) strength = 1;

                // Draw signal strength icon
                drawSignalStrength(_width - 50, STATUS_Y + 15, strength);
            } else {
                _display->setTextColor(COLOR_RED);
                _display->setCursor(20, STATUS_Y + 15);
                _display->print("Disconnected");
            }

            _cachedConnected = connected;
            _cachedSignalStrength = signalStrength;

            if (_autoUpdate) {
                update();
            }

            return true;
        }

        bool ST7789Display::displaySensorData(float temperature, float humidity) {
            // Only update if values have changed or full redraw is needed
            if (abs(temperature - _cachedTemperature) < 0.1f &&
                abs(humidity - _cachedHumidity) < 0.1f &&
                !_needsFullRedraw) {
                return true;
            }

            // Clear the sensor data area (right half of info area)
            _display->fillRect(_width/2, INFO_Y, _width/2, 40, COLOR_BLACK);

            // Format temperature and humidity
            char tempStr[10], humStr[10];
            sprintf(tempStr, "%.1fC", temperature);
            sprintf(humStr, "%.1f%%", humidity);

            // Display temperature and humidity
            _display->setTextSize(1);
            _display->setTextColor(COLOR_CYAN);
            _display->setCursor(_width/2 + 10, INFO_Y + 5);
            _display->print("Temp: ");
            _display->print(tempStr);

            _display->setCursor(_width/2 + 10, INFO_Y + 20);
            _display->print("Hum: ");
            _display->print(humStr);

            _cachedTemperature = temperature;
            _cachedHumidity = humidity;

            if (_autoUpdate) {
                update();
            }

            return true;
        }

        bool ST7789Display::displayError(const String& error) {
            if (error == _cachedError && !_needsFullRedraw) {
                return true; // No change, no need to update
            }

            // Clear the display
            _display->fillScreen(COLOR_BLACK);

            // Display error header
            _display->setTextSize(2);
            _display->setTextColor(COLOR_RED);
            _display->setCursor(10, 10);
            _display->print("ERROR:");

            // Display error message (wrap if necessary)
            _display->setTextSize(1);
            _display->setTextColor(COLOR_WHITE);

            // Simple word wrapping
            String remainingText = error;
            int lineHeight = 10; // 10 pixels per line with font size 1
            int y = 40;
            int maxCharsPerLine = _width / 6; // Approximate characters per line

            while (remainingText.length() > 0 && y < _height) {
                String line;
                if (remainingText.length() <= maxCharsPerLine) {
                    line = remainingText;
                    remainingText = "";
                } else {
                    // Find a space to break the line
                    int breakPos = maxCharsPerLine;
                    while (breakPos > 0 && remainingText.charAt(breakPos) != ' ') {
                        breakPos--;
                    }

                    // If no space found, just break at max chars
                    if (breakPos == 0) {
                        breakPos = maxCharsPerLine;
                    }

                    line = remainingText.substring(0, breakPos);
                    remainingText = remainingText.substring(breakPos + 1); // Skip the space
                }

                _display->setCursor(10, y);
                _display->print(line);
                y += lineHeight;
            }

            _cachedError = error;
            update(); // Always update immediately for errors

            return true;
        }

        bool ST7789Display::displayNodeInfo(uint8_t nodeId, bool relayState) {
            if (nodeId == _cachedNodeId && relayState == _cachedRelayState && !_needsFullRedraw) {
                return true; // No change, no need to update
            }

            // Clear the node info area (left half of info area)
            _display->fillRect(0, INFO_Y, _width/2, 40, COLOR_BLACK);

            // Display node info label
            _display->setTextSize(1);
            _display->setTextColor(COLOR_GREEN);
            _display->setCursor(10, INFO_Y);
            _display->print("NODE INFO");

            // Display node ID
            _display->setTextSize(1);
            _display->setTextColor(COLOR_WHITE);
            _display->setCursor(10, INFO_Y + 15);
            _display->print("Node: ");
            _display->print(nodeId);

            // Display relay state
            _display->setCursor(10, INFO_Y + 30);
            _display->print("Relay: ");
            if (relayState) {
                _display->setTextColor(COLOR_GREEN);
                _display->print("ON");
            } else {
                _display->setTextColor(COLOR_RED);
                _display->print("OFF");
            }

            _cachedNodeId = nodeId;
            _cachedRelayState = relayState;

            if (_autoUpdate) {
                update();
            }

            return true;
        }

        bool ST7789Display::update() {
            // Draw refresh indicator to show the display is active
            drawRefreshIndicator();

            // No explicit display update needed for ST7789 (updates immediately)
            _needsFullRedraw = false;
            _lastUpdateTime = millis();

            return true;
        }

        String ST7789Display::getLastError() const {
            return _lastError;
        }

        bool ST7789Display::setSpiClockDivider(uint8_t divider) {
            // Not directly applicable for Adafruit_ST7789,
            // which manages SPI configuration internally
            return true;
        }

        bool ST7789Display::setHardwareSpi(bool enable) {
            _useHardwareSpi = enable;
            return true;
        }

        void ST7789Display::setAutoUpdate(bool enable) {
            _autoUpdate = enable;
        }

        void ST7789Display::setFontSize(uint8_t size) {
            if (size < 1) size = 1;
            if (size > 3) size = 3;

            _fontSize = size;
            _display->setTextSize(_fontSize);
        }

        bool ST7789Display::setRotation(uint8_t rotation) {
            if (rotation > 3) rotation = 0;

            _display->setRotation(rotation);
            return true;
        }

        bool ST7789Display::drawProgressBar(int x, int y, int width, int height, int progress) {
            // Validate progress value
            if (progress < 0) progress = 0;
            if (progress > 100) progress = 100;

            // Draw border
            _display->drawRect(x, y, width, height, COLOR_WHITE);

            // Calculate fill width
            int fillWidth = ((width - 2) * progress) / 100;

            // Draw fill
            if (fillWidth > 0) {
                _display->fillRect(x + 1, y + 1, fillWidth, height - 2, COLOR_GREEN);
            }

            return true;
        }

        bool ST7789Display::displayMessage(const String& message, int x, int y, uint8_t size) {
            // Save current font size
            uint8_t oldSize = _fontSize;

            // Set new font size if specified
            if (size > 0) {
                setFontSize(size);
            }

            // Calculate position if centered
            if (x < 0) {
                x = getCenteredX(message, _fontSize);
            }
            if (y < 0) {
                y = (_height - (_fontSize * 8)) / 2;
            }

            // Display message
            _display->setCursor(x, y);
            _display->setTextColor(COLOR_WHITE);
            _display->print(message);

            // Restore font size
            if (size > 0) {
                setFontSize(oldSize);
            }

            if (_autoUpdate) {
                update();
            }

            return true;
        }

        void ST7789Display::drawSignalStrength(int x, int y, int strength) {
            // Draw 4 bars of increasing height
            for (int i = 0; i < 4; i++) {
                int barHeight = 4 + (i * 3);
                int barWidth = 6;
                int barSpacing = 2;
                int barX = x + (i * (barWidth + barSpacing));
                int barY = y + (12 - barHeight);

                if (i < strength) {
                    _display->fillRect(barX, barY, barWidth, barHeight, COLOR_GREEN);
                } else {
                    _display->drawRect(barX, barY, barWidth, barHeight, COLOR_WHITE);
                }
            }
        }

        void ST7789Display::drawRefreshIndicator() {
            // Draw a small indicator that rotates each update
            static uint8_t refreshState = 0;
            static const uint8_t refreshX = _width - 10;
            static const uint8_t refreshY = 5;
            static const uint8_t refreshSize = 5;

            _display->fillRect(refreshX - refreshSize, refreshY - refreshSize,
                               refreshSize*2, refreshSize*2, COLOR_BLACK);

            switch (refreshState) {
                case 0:
                    _display->drawFastHLine(refreshX - refreshSize/2, refreshY, refreshSize, COLOR_WHITE);
                    break;
                case 1:
                    _display->drawLine(refreshX + refreshSize/2, refreshY - refreshSize/2,
                                       refreshX - refreshSize/2, refreshY + refreshSize/2, COLOR_WHITE);
                    break;
                case 2:
                    _display->drawFastVLine(refreshX, refreshY - refreshSize/2, refreshSize, COLOR_WHITE);
                    break;
                case 3:
                    _display->drawLine(refreshX - refreshSize/2, refreshY - refreshSize/2,
                                       refreshX + refreshSize/2, refreshY + refreshSize/2, COLOR_WHITE);
                    break;
            }

            refreshState = (refreshState + 1) % 4;
        }

        int ST7789Display::getCenteredX(const String& text, uint8_t size) const {
            return (_width - (text.length() * 6 * size)) / 2;
        }

    } // namespace display
} // namespace szogfm