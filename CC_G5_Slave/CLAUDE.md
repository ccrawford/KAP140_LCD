# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a CC_G5_Slave project - an Arduino-based I2C slave device designed to interface with flight simulation hardware. The device acts as a rotary encoder controller that communicates with an ESP32 master via I2C protocol.

## Architecture

The project implements a simple I2C slave device with the following key components:

- **I2C Slave Interface**: Responds to requests from ESP32 master at address 0x08
- **Rotary Encoder Handler**: Uses RotaryEncoder library to track encoder position and direction 
- **Interrupt Signaling**: Signals data changes to ESP32 via dedicated interrupt pin (GPIO 10)
- **Button Input**: Handles encoder built-in button and additional external button
- **LED Control**: Receives LED state commands from ESP32 master

### Hardware Configuration
- Target Platform: Raspberry Pi Pico (generic board)
- Framework: Arduino
- I2C pins: SDA=8, SCL=9
- Encoder pins: A=2, B=3, Button=4
- Additional button: Pin 5
- LED control: Pin 6
- Interrupt to ESP32: Pin 10

### Communication Protocol
- **I2C Request Response**: Sends 3 bytes [encoder_delta, encoder_button, extra_button]
- **I2C Command Reception**: Receives LED control commands (0x01 + LED state)
- **Interrupt Signaling**: Pulls interrupt pin LOW briefly to notify ESP32 of data changes

## Development Commands

This project uses PlatformIO for building and uploading:

```bash
# Build the project
platformio run

# Upload to device
platformio run --target upload

# Clean build artifacts
platformio run --target clean

# Monitor serial output
platformio device monitor
```

## Key Files

- `src/main.cpp`: Main application logic with I2C slave implementation
- `platformio.ini`: PlatformIO project configuration with Raspberry Pi platform settings
- Dependencies: RotaryEncoder library (v1.5.3) by mathertel

## Code Structure

The main.cpp implements a state machine with:
- `setup()`: Initializes I2C slave, serial communication, and pin configurations
- `loop()`: Continuously polls rotary encoder for position changes
- `signal_data_change()`: Generates interrupt pulse to notify ESP32
- `request_event()`: I2C callback to send encoder/button data to master
- `receive_event()`: I2C callback to receive LED control commands from master

The device maintains encoder delta (change since last read), button states, and LED state, resetting the delta after each I2C data transmission.