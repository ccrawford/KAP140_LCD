#include <Arduino.h>
#include "G5Control.h"

// Pin definitions for PFD (Primary Flight Display) control
#define PFD_ENCODER_PIN_A 18
#define PFD_ENCODER_PIN_B 19
#define PFD_ENCODER_PIN_BTN 17
#define PFD_POWER_BUTTON 16
#define PFD_LED_PIN 10
#define I2C_SDA_PIN 20
#define I2C_SCL_PIN 21
#define PFD_INT_PIN 22
#define I2C_SLAVE_ADDR 0x08

// Pin definitions for HSI (Horizontal Situation Indicator) control
#define HSI_ENCODER_PIN_A 2
#define HSI_ENCODER_PIN_B 3
#define HSI_ENCODER_PIN_BTN 4
#define HSI_POWER_BUTTON 14
#define HSI_LED_PIN 15
#define I2C2_SDA_PIN 26
#define I2C2_SCL_PIN 27
#define HSI_INT_PIN 28
#define I2C2_SLAVE_ADDR 0x08

// Create G5Control instances
G5Control pfdControl(
    PFD_ENCODER_PIN_A, PFD_ENCODER_PIN_B, PFD_ENCODER_PIN_BTN,
    PFD_POWER_BUTTON, PFD_LED_PIN, PFD_INT_PIN,
    Wire, I2C_SLAVE_ADDR, "PFD"
);

G5Control hsiControl(
    HSI_ENCODER_PIN_A, HSI_ENCODER_PIN_B, HSI_ENCODER_PIN_BTN,
    HSI_POWER_BUTTON, HSI_LED_PIN, HSI_INT_PIN,
    Wire1, I2C2_SLAVE_ADDR, "HSI"
);

// ========== I2C Callback Wrappers ==========
// These wrapper functions are needed because Wire.onRequest/onReceive
// require plain function pointers, not member functions

void pfd_request_event() {
    pfdControl.handleRequest();
}

void pfd_receive_event(int byteCount) {
    pfdControl.handleReceive(byteCount);
}

void hsi_request_event() {
    hsiControl.handleRequest();
}

void hsi_receive_event(int byteCount) {
    hsiControl.handleReceive(byteCount);
}

// ========== Button Event Handler ==========
// This handler is called by AceButton when any button event occurs
// We need to determine which control it belongs to and route accordingly

void handleButtonEvent(AceButton* button, uint8_t eventType, uint8_t buttonState) {
    // Check if the button belongs to PFD control
    if (button == pfdControl.getEncoderButton() || button == pfdControl.getPowerButton()) {
        pfdControl.handleButtonEvent(button, eventType, buttonState);
    }
    // Check if the button belongs to HSI control
    else if (button == hsiControl.getEncoderButton() || button == hsiControl.getPowerButton()) {
        hsiControl.handleButtonEvent(button, eventType, buttonState);
    }
}

void setup() {
    delay(5);

    Serial.begin(115200);
    Serial.println("===========================================");
    Serial.println("G5 Dual Control I2C Slave Device");
    Serial.println("October 2025 - CAC");
    Serial.println("Supports: Click, Press, Long Press events");
    Serial.println("===========================================");

    // Configure AceButton global settings
    ButtonConfig* buttonConfig = ButtonConfig::getSystemButtonConfig();
    buttonConfig->setEventHandler(handleButtonEvent);
    buttonConfig->setFeature(ButtonConfig::kFeatureClick);
    // buttonConfig->setFeature(ButtonConfig::k);
    buttonConfig->setFeature(ButtonConfig::kFeatureLongPress);
    buttonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterClick);
    buttonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterLongPress);

    // Initialize PFD control
    Serial.println("\n--- Initializing PFD Control ---");
    pfdControl.begin();

    // Setup I2C for PFD
    Wire.setSDA(I2C_SDA_PIN);
    Wire.setSCL(I2C_SCL_PIN);
    Wire.begin(I2C_SLAVE_ADDR);
    Wire.onRequest(pfd_request_event);
    Wire.onReceive(pfd_receive_event);
    Serial.printf("PFD I2C initialized at address 0x%02X\n", I2C_SLAVE_ADDR);

    // Initialize HSI control
    Serial.println("\n--- Initializing HSI Control ---");
    hsiControl.begin();

    // Setup I2C for HSI
    Wire1.setSDA(I2C2_SDA_PIN);
    Wire1.setSCL(I2C2_SCL_PIN);
    Wire1.begin(I2C2_SLAVE_ADDR);
    Wire1.onRequest(hsi_request_event);
    Wire1.onReceive(hsi_receive_event);
    Serial.printf("HSI I2C initialized at address 0x%02X\n", I2C2_SLAVE_ADDR);

    Serial.println("\n===========================================");
    Serial.println("Initialization complete. Ready.");
    Serial.println("===========================================\n");
}

void loop() {
    // Update both controls
    pfdControl.update();
    hsiControl.update();
}
