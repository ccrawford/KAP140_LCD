#include "G5Control.h"

G5Control::G5Control(uint8_t encoderPinA, uint8_t encoderPinB, uint8_t encoderButton,
                     uint8_t powerButton, uint8_t ledPin, uint8_t interruptPin,
                     TwoWire &wire, uint8_t i2cAddr, const char *name)
    : _encoderPinA(encoderPinA),
      _encoderPinB(encoderPinB),
      _encoderButtonPin(encoderButton),
      _powerButtonPin(powerButton),
      _ledPin(ledPin),
      _interruptPin(interruptPin),
      _wire(&wire),
      _i2cAddr(i2cAddr),
      _encoder(encoderPinA, encoderPinB, RotaryEncoder::LatchMode::FOUR3),
      _encoderButton(encoderButton),
      _powerButton(powerButton),
      _encoderDelta(0),
      _dataChanged(false),
      _ledState(false),
      _encoderButtonEvent(BUTTON_IDLE),
      _powerButtonEvent(BUTTON_IDLE),
      _lastTransmission(0),
      _lastUpdate(0),
      _lastPosition(0),
      _name(name)
{
}

void G5Control::begin()
{
    // Configure LED pin
    pinMode(_ledPin, OUTPUT);
    digitalWrite(_ledPin, LOW);

    // Configure button pins with pull-ups
    pinMode(_powerButtonPin, INPUT_PULLUP);
    pinMode(_encoderButtonPin, INPUT_PULLUP);

    // Configure interrupt pin (active LOW signal to master)
    pinMode(_interruptPin, OUTPUT);
    digitalWrite(_interruptPin, HIGH);

    // Initialize timing
    _lastUpdate = millis();

    Serial.print("G5Control '");
    Serial.print(_name);
    Serial.println("' initialized");
}

void G5Control::update()
{
    // Update encoder
    _encoder.tick();

    // Update buttons
    _encoderButton.check();
    _powerButton.check();

    // Check for encoder position changes
    int newPos = _encoder.getPosition();
    if (_dataChanged || ((newPos != _lastPosition) && (millis() - _lastUpdate) > 50))
    {
        // Accumulate delta
        _encoderDelta += (newPos - _lastPosition);

        // Debug output
        Serial.printf("[%ld] %s - Pos: %d  Dir: %d  Delta: %d  EncBtn: %d (%d)  PwrBtn: %d (%d)\n",
                      millis(), _name, newPos, _encoder.getDirection(),
                      _encoderDelta,
                      digitalRead(_encoderButtonPin), _encoderButtonEvent,
                      digitalRead(_powerButtonPin), _powerButtonEvent);

        // Signal master
        signalDataChange();

        // Update state
        _lastPosition = newPos;
        _lastUpdate = millis();
        _dataChanged = false;
    }
}

void G5Control::handleRequest()
{
    // I2C data format: [encoder_delta, encoder_button_event, power_button_event]
    _wire->write((int8_t)_encoderDelta);
    _wire->write((uint8_t)_encoderButtonEvent);
    _wire->write((uint8_t)_powerButtonEvent);

    Serial.printf("%s I2C Request: delta=%d, encBtn=%d, pwrBtn=%d\n",
                  _name, _encoderDelta, _encoderButtonEvent, _powerButtonEvent);

    // Reset state after transmission
    _encoderDelta = 0;
    _encoderButtonEvent = BUTTON_IDLE;
    _powerButtonEvent = BUTTON_IDLE;
    _dataChanged = false;
    _lastTransmission = millis();
}

void G5Control::handleReceive(int byteCount)
{
    Serial.printf("%s I2C Receive (%d bytes)\n", _name, byteCount);

    if (byteCount > 0)
    {
        byte command = _wire->read();

        // Command 0x01: LED control
        if (command == 0x01 && byteCount > 1)
        {
            byte ledValue = _wire->read();
            _ledState = (ledValue != 0);
            digitalWrite(_ledPin, _ledState ? HIGH : LOW);
            Serial.printf("%s LED -> %s\n", _name, _ledState ? "ON" : "OFF");
        }

        // Read any remaining bytes to prevent buffer issues
        while (_wire->available())
        {
            _wire->read();
        }
    }
}

void G5Control::handleButtonEvent(AceButton *button, uint8_t eventType, uint8_t buttonState)
{
    ButtonEventType event = convertButtonEvent(eventType);

    // Determine which button and update its event state
    if (button == &_encoderButton)
    {
        _encoderButtonEvent = event;
        Serial.printf("%s Encoder Button Event: %d\n", _name, event);
    }
    else if (button == &_powerButton)
    {
        _powerButtonEvent = event;
        Serial.printf("%s Power Button Event: %d\n", _name, event);
    }

    // Signal that data has changed
    _dataChanged = true;
}

void G5Control::signalDataChange()
{
    // Pull interrupt line LOW briefly to signal ESP32
    digitalWrite(_interruptPin, LOW);
    delay(1);
    digitalWrite(_interruptPin, HIGH);
}

G5Control::ButtonEventType G5Control::convertButtonEvent(uint8_t aceEventType)
{
    switch (aceEventType)
    {
    case AceButton::kEventClicked:
        return BUTTON_CLICKED;
    case AceButton::kEventPressed:
        //return BUTTON_CLICKED; // OVERRIDE!
        return BUTTON_PRESSED;
    case AceButton::kEventLongPressed:
        return BUTTON_LONG_PRESSED;
    case AceButton::kEventReleased:
        return BUTTON_RELEASED;
    default:
        return BUTTON_IDLE;
    }
}
