#include "ButtonHandler.h"

namespace szogfm {
    namespace input {

        ButtonHandler::ButtonHandler(uint8_t analogPin, unsigned long debounceDelay)
                : _analogPin(analogPin), _debounceDelay(debounceDelay),
                  _lastDebounceTime(0), _lastAnalogValue(0),
                  _lastButtonType(ButtonType::CUSTOM), _buttonPressed(false),
                  _buttonCount(0) {
        }

        void ButtonHandler::initialize() {
            // Configure the analog pin as input with internal pull-up for stability
            pinMode(_analogPin, INPUT_PULLUP);
        }

        bool ButtonHandler::update() {
            // Read analog value with improved filtering
            int analogValue = readAnalogValue();

            // IMPROVED: Ignore values in the noise zone (below 800)
            if (analogValue < 800) {
                _buttonPressed = false;
                return false;
            }

            // Check if a button is pressed (valid analog range)
            if (analogValue > 800 && analogValue < 4090) { // Wider valid range but with filtering
                // Check for debounce
                unsigned long currentTime = millis();
                if (currentTime - _lastDebounceTime > _debounceDelay) {
                    int buttonIndex = findPressedButton(analogValue);

                    if (buttonIndex >= 0 && !_buttonPressed) {
                        // ADDITIONAL STABILITY CHECK: Re-read analog value to confirm
                        delay(10); // Small delay
                        int confirmValue = readAnalogValue();

                        // Check if the reading is stable (within reasonable tolerance)
                        if (abs(analogValue - confirmValue) > 100) {
                            // Readings are too different, ignore this press (likely noise)
                            return false;
                        }

                        // Button press confirmed as stable
                        _lastButtonType = _buttons[buttonIndex].type;
                        _lastAnalogValue = analogValue;
                        _buttonPressed = true;

                        // Call the callback if set
                        if (_buttons[buttonIndex].callback) {
                            _buttons[buttonIndex].callback();
                        }

                        _lastDebounceTime = currentTime;
                        return true;
                    }
                }
            } else {
                // No button is pressed or value is in invalid range
                _buttonPressed = false;
            }

            return false;
        }

        int ButtonHandler::addButton(ButtonType type, int minValue, int maxValue, std::function<void()> callback) {
            if (_buttonCount >= MAX_BUTTONS) {
                return -1; // Maximum number of buttons reached
            }

            // Add the button with validation
            _buttons[_buttonCount].type = type;
            _buttons[_buttonCount].minValue = minValue;
            _buttons[_buttonCount].maxValue = maxValue;
            _buttons[_buttonCount].callback = callback;

            // Validate button ranges don't overlap
            for (int i = 0; i < _buttonCount; i++) {
                // Check for overlap with existing buttons
                if ((minValue >= _buttons[i].minValue && minValue <= _buttons[i].maxValue) ||
                    (maxValue >= _buttons[i].minValue && maxValue <= _buttons[i].maxValue) ||
                    (minValue <= _buttons[i].minValue && maxValue >= _buttons[i].maxValue)) {

                    Serial.printf("⚠️  Button range overlap detected! New: %d-%d, Existing: %d-%d\n",
                                  minValue, maxValue, _buttons[i].minValue, _buttons[i].maxValue);
                }
            }

            return _buttonCount++;
        }

        ButtonType ButtonHandler::getLastButtonType() const {
            return _lastButtonType;
        }

        int ButtonHandler::getLastAnalogValue() const {
            return _lastAnalogValue;
        }

        bool ButtonHandler::setButtonCallback(ButtonType type, std::function<void()> callback) {
            for (int i = 0; i < _buttonCount; i++) {
                if (_buttons[i].type == type) {
                    _buttons[i].callback = callback;
                    return true;
                }
            }

            return false; // Button type not found
        }

        int ButtonHandler::readAnalogValue() {
            // IMPROVED: Take multiple samples and return average for noise reduction
            const int numSamples = 3;
            long total = 0;

            for (int i = 0; i < numSamples; i++) {
                total += analogRead(_analogPin);
                if (i < numSamples - 1) {
                    delayMicroseconds(50); // Small delay between samples
                }
            }

            return total / numSamples;
        }

        int ButtonHandler::findPressedButton(int analogValue) {
            // Find button with exact range match
            for (int i = 0; i < _buttonCount; i++) {
                if (analogValue >= _buttons[i].minValue && analogValue <= _buttons[i].maxValue) {
                    return i;
                }
            }

            return -1; // No button found
        }

    } // namespace input
} // namespace szogfm