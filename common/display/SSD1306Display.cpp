#include "SSD1306Display.h"

namespace szogfm {
    namespace display {

        SSD1306Display::SSD1306Display(uint8_t width, uint8_t height,
                                       int8_t sdaPin, int8_t sclPin,
                                       int8_t resetPin, uint8_t address)
                : _sdaPin(sdaPin), _sclPin(sclPin), _resetPin(resetPin),
                  _width(width), _height(height), _address(address),
                  _i2cFrequency(400000), // Default to 400kHz
                  _fontSize(1), _autoUpdate(true),
                  _cachedFrequency(0), _cachedVolume(0), _cachedMuted(false),
                  _cachedConnected(false), _cachedSignalStrength(0),
                  _cachedTemperature(0.0f), _cachedHumidity(0.0f),
                  _cachedNodeId(0), _cachedRelayState(false),
                  _needsFullRedraw(true), _lastUpdateTime(0),
                  _fmSignalStrength(0) {

            _display = new Adafruit_SSD1306(_width, _height, &Wire, _resetPin);
        }

        SSD1306Display::~SSD1306Display() {
            delete _display;
        }

        bool SSD1306Display::initialize() {
            // Initialize I2C
            Wire.begin(_sdaPin, _sclPin);
            Wire.setClock(_i2cFrequency);

            // Initialize the display
            if (!_display->begin(SSD1306_SWITCHCAPVCC, _address)) {
                _lastError = "SSD1306 allocation failed";
                Serial.println("ERROR: " + _lastError);
                return false;
            }

            // Initial setup
            _display->clearDisplay();
            _display->setTextSize(_fontSize);
            _display->setTextColor(SSD1306_WHITE);
            _display->setCursor(0, 0);
            _display->cp437(true); // Use full 256 char 'Code Page 437' font

            // Display startup screen
            _display->clearDisplay();
            _display->setTextSize(2);
            _display->setCursor(getCenteredX("SzogFM", 2), 16);
            _display->println("SzogFM");
            _display->setTextSize(1);
            _display->setCursor(getCenteredX("Remote FM Controller", 1), 40);
            _display->println("Remote FM Controller");
            _display->display();
            delay(2000);

            _display->clearDisplay();
            _display->setTextSize(_fontSize);
            _display->display();

            _needsFullRedraw = true;

            Serial.println("SSD1306 display initialized successfully");
            return true;
        }

        bool SSD1306Display::clear() {
            _display->clearDisplay();
            _display->display();
            _needsFullRedraw = true;
            return true;
        }

        bool SSD1306Display::displayFrequency(uint16_t frequency) {
            if (frequency == _cachedFrequency && !_needsFullRedraw) {
                return true; // No change, no need to update
            }

            // Format frequency as "XXX.X MHz"
            char freqStr[15];
            sprintf(freqStr, "%d.%d MHz", frequency / 100, (frequency % 100) / 10);

            // Clear the frequency area
            _display->fillRect(0, FREQUENCY_Y, _width, 16, SSD1306_BLACK);

            // Display the frequency label
            _display->setTextSize(1);
            _display->setCursor(0, FREQUENCY_Y);
            _display->print("FM: ");
            _display->print(freqStr);

            // Display FM signal strength if available
            // We'll show signal strength as part of the frequency display
            if (_fmSignalStrength > 0) {
                _display->setCursor(_width - 40, FREQUENCY_Y);
                _display->print("RSSI:");
                _display->print(_fmSignalStrength);
            }

            // Reset text size
            _display->setTextSize(_fontSize);

            _cachedFrequency = frequency;

            if (_autoUpdate) {
                update();
            }

            return true;
        }

        bool SSD1306Display::displayVolume(uint8_t volume, bool muted) {
            if (volume == _cachedVolume && muted == _cachedMuted && !_needsFullRedraw) {
                return true; // No change, no need to update
            }

            // Clear the volume area
            _display->fillRect(0, VOLUME_Y, _width, 16, SSD1306_BLACK);

            // Display the volume label
            _display->setTextSize(1);
            _display->setCursor(0, VOLUME_Y);
            _display->print("Volume: ");

            if (muted) {
                // Display muted indicator
                _display->setCursor(50, VOLUME_Y);
                _display->print("MUTED");
            } else {
                // Display volume value
                char volStr[5];
                sprintf(volStr, "%d", volume);
                _display->setCursor(50, VOLUME_Y);
                _display->print(volStr);

                // Draw volume bar
                drawProgressBar(0, VOLUME_Y + 8, _width, 6, (volume * 100) / 15);
            }

            _cachedVolume = volume;
            _cachedMuted = muted;

            if (_autoUpdate) {
                update();
            }

            return true;
        }

        bool SSD1306Display::displayConnectionStatus(bool connected, int signalStrength) {
            if (connected == _cachedConnected && signalStrength == _cachedSignalStrength && !_needsFullRedraw) {
                return true; // No change, no need to update
            }

            // Clear the status area
            _display->fillRect(0, STATUS_Y, _width, 16, SSD1306_BLACK);

            // Display connection status
            _display->setTextSize(1);
            _display->setCursor(0, STATUS_Y);
            _display->print("Status: ");

            if (connected) {
                _display->print("Connected");

                // Map signal strength to 0-4 scale
                int strength = 0;
                if (signalStrength >= -60) strength = 4;
                else if (signalStrength >= -70) strength = 3;
                else if (signalStrength >= -80) strength = 2;
                else if (signalStrength >= -90) strength = 1;

                // Draw signal strength icon
                drawSignalStrength(_width - 25, STATUS_Y + 4, strength);
            } else {
                _display->print("Disconnected");
            }

            _cachedConnected = connected;
            _cachedSignalStrength = signalStrength;

            if (_autoUpdate) {
                update();
            }

            return true;
        }

        bool SSD1306Display::displaySensorData(float temperature, float humidity) {
            // Only update if values have changed or full redraw is needed
            if (abs(temperature - _cachedTemperature) < 0.1f &&
                abs(humidity - _cachedHumidity) < 0.1f &&
                !_needsFullRedraw) {
                return true;
            }

            // Clear the sensor data area (bottom half of info area)
            _display->fillRect(0, INFO_Y + 8, _width, 8, SSD1306_BLACK);

            // Format temperature and humidity
            char tempStr[10], humStr[10];
            sprintf(tempStr, "%.1fC", temperature);
            sprintf(humStr, "%.1f%%", humidity);

            // Display temperature and humidity
            _display->setTextSize(1);
            _display->setCursor(0, INFO_Y + 8);
            _display->print("T:");
            _display->print(tempStr);

            _display->setCursor(_width / 2, INFO_Y + 8);
            _display->print("H:");
            _display->print(humStr);

            _cachedTemperature = temperature;
            _cachedHumidity = humidity;

            if (_autoUpdate) {
                update();
            }

            return true;
        }

        bool SSD1306Display::displayError(const String& error) {
            if (error == _cachedError && !_needsFullRedraw) {
                return true; // No change, no need to update
            }

            // Clear the display
            _display->clearDisplay();

            // Display error header
            _display->setTextSize(1);
            _display->setCursor(0, 0);
            _display->print("ERROR:");

            // Display error message (wrap if necessary)
            _display->setCursor(0, 16);

            // Simple word wrapping
            String remainingText = error;
            int lineHeight = 8; // 8 pixels per line with font size 1
            int y = 16;
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

                _display->setCursor(0, y);
                _display->print(line);
                y += lineHeight;
            }

            _cachedError = error;
            _display->display(); // Always display errors immediately

            return true;
        }

        bool SSD1306Display::displayNodeInfo(uint8_t nodeId, bool relayState) {
            if (nodeId == _cachedNodeId && relayState == _cachedRelayState && !_needsFullRedraw) {
                return true; // No change, no need to update
            }

            // Clear the node info area
            _display->fillRect(0, INFO_Y, _width, 8, SSD1306_BLACK);

            // Display node ID
            _display->setTextSize(1);
            _display->setCursor(0, INFO_Y);
            _display->print("Node: ");
            _display->print(nodeId);

            // Display relay state
            _display->setCursor(_width / 2, INFO_Y);
            _display->print("Relay: ");
            _display->print(relayState ? "ON" : "OFF");

            _cachedNodeId = nodeId;
            _cachedRelayState = relayState;

            if (_autoUpdate) {
                update();
            }

            return true;
        }

        bool SSD1306Display::update() {
            // Draw refresh indicator to show the display is active
            drawRefreshIndicator();

            // Update display
            _display->display();
            _needsFullRedraw = false;
            _lastUpdateTime = millis();

            return true;
        }

        String SSD1306Display::getLastError() const {
            return _lastError;
        }

        bool SSD1306Display::setI2CAddress(uint8_t address) {
            _address = address;
            return true;
        }

        bool SSD1306Display::setI2CFrequency(uint32_t frequency) {
            _i2cFrequency = frequency;

            // Update the current I2C bus clock frequency
            Wire.setClock(frequency);

            return true;
        }

        void SSD1306Display::setAutoUpdate(bool enable) {
            _autoUpdate = enable;
        }

        void SSD1306Display::setFontSize(uint8_t size) {
            if (size < 1) size = 1;
            if (size > 3) size = 3;

            _fontSize = size;
            _display->setTextSize(_fontSize);
        }

        bool SSD1306Display::setContrast(uint8_t contrast) {
            _display->ssd1306_command(SSD1306_SETCONTRAST);
            _display->ssd1306_command(contrast);
            return true;
        }

        bool SSD1306Display::enableDisplay(bool enable) {
            if (enable) {
                _display->ssd1306_command(SSD1306_DISPLAYON);
            } else {
                _display->ssd1306_command(SSD1306_DISPLAYOFF);
            }
            return true;
        }

        bool SSD1306Display::drawProgressBar(int x, int y, int width, int height, int progress) {
            // Validate progress value
            if (progress < 0) progress = 0;
            if (progress > 100) progress = 100;

            // Draw border
            _display->drawRect(x, y, width, height, SSD1306_WHITE);

            // Calculate fill width
            int fillWidth = ((width - 2) * progress) / 100;

            // Draw fill
            if (fillWidth > 0) {
                _display->fillRect(x + 1, y + 1, fillWidth, height - 2, SSD1306_WHITE);
            }

            return true;
        }

        bool SSD1306Display::displayMessage(const String& message, int x, int y, uint8_t size) {
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

        void SSD1306Display::drawBattery(int x, int y, int percentage, int width, int height) {
            // Draw battery outline
            _display->drawRect(x, y, width - 2, height, SSD1306_WHITE);
            _display->drawRect(x + width - 2, y + 2, 2, height - 4, SSD1306_WHITE);

            // Calculate fill width
            int fillWidth = ((width - 4) * percentage) / 100;

            // Draw fill
            if (fillWidth > 0) {
                _display->fillRect(x + 1, y + 1, fillWidth, height - 2, SSD1306_WHITE);
            }
        }

        void SSD1306Display::drawSignalStrength(int x, int y, int strength) {
            // Draw 4 bars of increasing height
            for (int i = 0; i < 4; i++) {
                int barHeight = 3 + (i * 2);
                if (i < strength) {
                    _display->fillRect(x + (i * 5), y + (9 - barHeight), 3, barHeight, SSD1306_WHITE);
                } else {
                    _display->drawRect(x + (i * 5), y + (9 - barHeight), 3, barHeight, SSD1306_WHITE);
                }
            }
        }

        void SSD1306Display::drawRefreshIndicator() {
            // Draw a small indicator that rotates each update
            static uint8_t refreshState = 0;
            static const uint8_t refreshX = _width - 5;
            static const uint8_t refreshY = 0;

            _display->fillRect(refreshX, refreshY, 5, 5, SSD1306_BLACK);

            switch (refreshState) {
                case 0:
                    _display->drawFastHLine(refreshX + 1, refreshY + 2, 3, SSD1306_WHITE);
                    break;
                case 1:
                    _display->drawLine(refreshX + 3, refreshY + 1, refreshX + 1, refreshY + 3, SSD1306_WHITE);
                    break;
                case 2:
                    _display->drawFastVLine(refreshX + 2, refreshY + 1, 3, SSD1306_WHITE);
                    break;
                case 3:
                    _display->drawLine(refreshX + 1, refreshY + 1, refreshX + 3, refreshY + 3, SSD1306_WHITE);
                    break;
            }

            refreshState = (refreshState + 1) % 4;
        }

        int SSD1306Display::getCenteredX(const String& text, uint8_t size) const {
            return (_width - (text.length() * 6 * size)) / 2;
        }

    } // namespace display
} // namespace szogfm