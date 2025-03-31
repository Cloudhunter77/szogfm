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
            // Configure the analog pin as input
            pinMode(_analogPin, INPUT);
        }

        bool ButtonHandler::update() {
            // Read analog value
            int analogValue = readAnalogValue();

            // Check if a button is pressed
            if (analogValue > 5) { // Threshold for a valid button press
                // Check for debounce
                unsigned long currentTime = millis();
                if (currentTime - _lastDebounceTime > _debounceDelay) {
                    int buttonIndex = findPressedButton(analogValue);

                    if (buttonIndex >= 0 && !_buttonPressed) {
                        // Button press detected
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
                // No button is pressed
                _buttonPressed = false;
            }

            return false;
        }

        int ButtonHandler::addButton(ButtonType type, int minValue, int maxValue, std::function<void()> callback) {
            if (_buttonCount >= MAX_BUTTONS) {
                return -1; // Maximum number of buttons reached
            }

            // Add the button
            _buttons[_buttonCount].type = type;
            _buttons[_buttonCount].minValue = minValue;
            _buttons[_buttonCount].maxValue = maxValue;
            _buttons[_buttonCount].callback = callback;

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
            // Read the analog value
            return analogRead(_analogPin);
        }

        int ButtonHandler::findPressedButton(int analogValue) {
            for (int i = 0; i < _buttonCount; i++) {
                if (analogValue >= _buttons[i].minValue && analogValue <= _buttons[i].maxValue) {
                    return i;
                }
            }

            return -1; // No button found
        }

    } // namespace input
} // namespace szogfm