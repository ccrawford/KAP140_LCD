#ifndef G5CONTROL_H
#define G5CONTROL_H

#include <Arduino.h>
#include <Wire.h>
#include <RotaryEncoder.h>
#include <AceButton.h>

using namespace ace_button;

/**
 * @brief Represents one G5 control unit with rotary encoder, buttons, LED, and I2C interface
 *
 * This class encapsulates:
 * - Rotary encoder with integrated button
 * - Separate power/function button
 * - LED for visual feedback
 * - I2C slave communication
 * - Interrupt signaling to master
 * - Button event detection (click, press, long press)
 */
class G5Control {
public:
    /**
     * @brief Button event types that can be sent to the master device
     */
    enum ButtonEventType : uint8_t {
        BUTTON_IDLE = 0,
        BUTTON_CLICKED = 1,
        BUTTON_PRESSED = 2,
        BUTTON_LONG_PRESSED = 3,
        BUTTON_RELEASED = 4
    };

    /**
     * @brief Construct a new G5Control object
     *
     * @param encoderPinA Rotary encoder pin A
     * @param encoderPinB Rotary encoder pin B
     * @param encoderButton Encoder integrated button pin
     * @param powerButton Separate power/function button pin
     * @param ledPin LED control pin
     * @param interruptPin Interrupt signal pin to ESP32 master
     * @param wire Reference to I2C Wire interface (Wire or Wire1)
     * @param i2cAddr I2C slave address
     * @param name Descriptive name for debug output (e.g., "PFD", "HSI")
     */
    G5Control(uint8_t encoderPinA, uint8_t encoderPinB, uint8_t encoderButton,
              uint8_t powerButton, uint8_t ledPin, uint8_t interruptPin,
              TwoWire& wire, uint8_t i2cAddr, const char* name = "");

    /**
     * @brief Initialize the control unit (call from setup())
     */
    void begin();

    /**
     * @brief Update encoder and button states (call from loop())
     */
    void update();

    /**
     * @brief Handle I2C request from master - sends current state
     * Call this from the I2C onRequest callback
     */
    void handleRequest();

    /**
     * @brief Handle I2C receive from master - processes commands
     * Call this from the I2C onReceive callback
     *
     * @param byteCount Number of bytes received
     */
    void handleReceive(int byteCount);

    /**
     * @brief Get pointer to the encoder button for event handler registration
     */
    AceButton* getEncoderButton() { return &_encoderButton; }

    /**
     * @brief Get pointer to the power button for event handler registration
     */
    AceButton* getPowerButton() { return &_powerButton; }

    /**
     * @brief Handle button events (to be called by AceButton event handler)
     *
     * @param button Pointer to the button that triggered the event
     * @param eventType Type of event (clicked, pressed, long pressed, etc.)
     * @param buttonState Current button state
     */
    void handleButtonEvent(AceButton* button, uint8_t eventType, uint8_t buttonState);

private:
    // Hardware pin assignments
    uint8_t _encoderPinA;
    uint8_t _encoderPinB;
    uint8_t _encoderButtonPin;
    uint8_t _powerButtonPin;
    uint8_t _ledPin;
    uint8_t _interruptPin;

    // I2C interface
    TwoWire* _wire;
    uint8_t _i2cAddr;

    // Hardware objects
    RotaryEncoder _encoder;
    AceButton _encoderButton;
    AceButton _powerButton;

    // State tracking
    int _encoderDelta;              // Accumulated encoder position change
    bool _dataChanged;              // Flag indicating data needs to be sent
    bool _ledState;                 // Current LED state
    ButtonEventType _encoderButtonEvent;  // Last encoder button event
    ButtonEventType _powerButtonEvent;    // Last power button event

    // Timing
    unsigned long _lastTransmission;  // Timestamp of last I2C transmission
    unsigned long _lastUpdate;        // Timestamp of last encoder update
    int _lastPosition;                // Last encoder position

    // Debug
    const char* _name;

    /**
     * @brief Signal to ESP32 master that data has changed via interrupt pin
     */
    void signalDataChange();

    /**
     * @brief Convert AceButton event type to our ButtonEventType enum
     */
    ButtonEventType convertButtonEvent(uint8_t aceEventType);
};

#endif // G5CONTROL_H
