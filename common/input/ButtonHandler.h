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
 * This class reads button presses from an analog pin and processes them with
 * advanced noise filtering and stability checking to prevent false triggers.
 */
        class ButtonHandler {
        public:
            /**
             * Constructor
             * @param analogPin Analog pin to read button presses from
             * @param debounceDelay Debounce delay in milliseconds (default 150ms for better noise immunity)
             */
            ButtonHandler(uint8_t analogPin, unsigned long debounceDelay = 150);

            /**
             * Initialize the button handler
             * Sets up the analog pin with appropriate configuration
             */
            void initialize();

            /**
             * Update the button state
             * Should be called in the main loop
             * Includes advanced noise filtering and stability checking
             * @return true if a button was pressed, false otherwise
             */
            bool update();

            /**
             * Add a button with a specific analog threshold range
             * Includes validation to check for overlapping ranges
             * @param type Button type
             * @param minValue Minimum analog value for this button
             * @param maxValue Maximum analog value for this button
             * @param callback Function to call when the button is pressed
             * @return Button index, or -1 if failed to add
             */
            int addButton(ButtonType type, int minValue, int maxValue, std::function<void()> callback);

            /**
             * Get the last pressed button type
             * @return Button type of the last confirmed button press
             */
            ButtonType getLastButtonType() const;

            /**
             * Get the last analog value read during a confirmed button press
             * @return Analog value (0-4095) of the last confirmed button press
             */
            int getLastAnalogValue() const;

            /**
             * Set the callback for a specific button type
             * @param type Button type to set callback for
             * @param callback Function to call when the button is pressed
             * @return true if the callback was set, false if button type not found
             */
            bool setButtonCallback(ButtonType type, std::function<void()> callback);

        private:
            /**
             * Structure to hold button configuration
             */
            struct Button {
                ButtonType type;               // Type of button
                int minValue;                  // Minimum analog value for detection
                int maxValue;                  // Maximum analog value for detection
                std::function<void()> callback; // Function to call when pressed
            };

            uint8_t _analogPin;               // Analog pin number
            unsigned long _debounceDelay;     // Debounce delay in milliseconds
            unsigned long _lastDebounceTime;  // Time of last successful button detection
            int _lastAnalogValue;             // Last confirmed analog value
            ButtonType _lastButtonType;       // Last confirmed button type
            bool _buttonPressed;              // Current button press state

            static constexpr int MAX_BUTTONS = 10; // Maximum number of buttons supported
            Button _buttons[MAX_BUTTONS];     // Array of configured buttons
            int _buttonCount;                 // Number of buttons currently configured

            /**
             * Read the analog value from the pin with noise filtering
             * Takes multiple samples and averages them for stability
             * @return Filtered analog value (0-4095)
             */
            int readAnalogValue();

            /**
             * Find which button was pressed based on the analog value
             * @param analogValue Analog value to check against button ranges
             * @return Button index or -1 if no button matches
             */
            int findPressedButton(int analogValue);
        };

    } // namespace input
} // namespace szogfm

#endif // SZOGFM_BUTTONHANDLER_H