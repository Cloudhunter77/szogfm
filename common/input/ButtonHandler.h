#ifndef SZOGFM_BUTTONHANDLER_H
#define SZOGFM_BUTTONHANDLER_H

#include <Arduino.h>
#include <functional>

namespace szogfm {
    namespace input {

/**
 * Enum for button types
 */
        enum class ButtonType : uint8_t {
            VOLUME_UP,
            VOLUME_DOWN,
            FREQUENCY_UP,
            FREQUENCY_DOWN,
            MUTE,
            RELAY_TOGGLE,
            CUSTOM
        };

/**
 * Button handler class for handling button inputs through a resistor network
 * This class reads button presses from an analog pin and processes them
 */
        class ButtonHandler {
        public:
            /**
             * Constructor
             * @param analogPin Analog pin to read button presses from
             * @param debounceDelay Debounce delay in milliseconds
             */
            ButtonHandler(uint8_t analogPin, unsigned long debounceDelay = 50);

            /**
             * Initialize the button handler
             */
            void initialize();

            /**
             * Update the button state
             * Should be called in the main loop
             * @return true if a button was pressed, false otherwise
             */
            bool update();

            /**
             * Add a button with a specific analog threshold range
             * @param type Button type
             * @param minValue Minimum analog value for this button
             * @param maxValue Maximum analog value for this button
             * @param callback Function to call when the button is pressed
             * @return Button index
             */
            int addButton(ButtonType type, int minValue, int maxValue, std::function<void()> callback);

            /**
             * Get the last pressed button type
             * @return Button type
             */
            ButtonType getLastButtonType() const;

            /**
             * Get the last analog value read
             * @return Analog value
             */
            int getLastAnalogValue() const;

            /**
             * Set the callback for a specific button type
             * @param type Button type
             * @param callback Function to call when the button is pressed
             * @return true if the callback was set, false otherwise
             */
            bool setButtonCallback(ButtonType type, std::function<void()> callback);

        private:
            struct Button {
                ButtonType type;
                int minValue;
                int maxValue;
                std::function<void()> callback;
            };

            uint8_t _analogPin;
            unsigned long _debounceDelay;
            unsigned long _lastDebounceTime;
            int _lastAnalogValue;
            ButtonType _lastButtonType;
            bool _buttonPressed;

            static constexpr int MAX_BUTTONS = 10;
            Button _buttons[MAX_BUTTONS];
            int _buttonCount;

            /**
             * Read the analog value from the pin
             * @return Analog value
             */
            int readAnalogValue();

            /**
             * Find which button was pressed based on the analog value
             * @param analogValue Analog value
             * @return Button index or -1 if no button was found
             */
            int findPressedButton(int analogValue);
        };

    } // namespace input
} // namespace szogfm

#endif // SZOGFM_BUTTONHANDLER_H