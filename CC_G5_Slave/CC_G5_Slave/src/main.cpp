#include <Arduino.h>
#include <Wire.h>
#include <RotaryEncoder.h>

// Pin definitions
#define ENCODER_PIN_A 2     // Rotary encoder pin A
#define ENCODER_PIN_B 3     // Rotary encoder pin B
#define ENCODER_PIN_BTN 4   // Encoder built-in button
#define EXTRA_BUTTON 5      // Second separate button
#define LED_PIN 6           // LED pin
#define I2C_SDA_PIN 8       // I2C SDA pin
#define I2C_SCL_PIN 9       // I2C SCL pin
#define INT_PIN 10          // Interrupt pin to ESP32 (connects to GPIO16 on ESP32)
#define I2C_SLAVE_ADDR 0x08 // I2C slave address

RotaryEncoder encoder(ENCODER_PIN_A, ENCODER_PIN_B, RotaryEncoder::LatchMode::FOUR3);

void request_event();
void receive_event(int byte_count);
void signal_data_change();  // Tell the esp that it needs to fetch data. 
void button_interrupt();

int encoder_delta = 0;
bool data_changed = false;
bool led_state = false;
bool encoder_button = false;
bool extra_button = false;

unsigned long last_transmission = 0;



void setup()
{
  Serial.begin(115200);
  // while (!Serial)
  //   ;


  pinMode(INT_PIN, OUTPUT);
  digitalWrite(INT_PIN, HIGH);

  pinMode(EXTRA_BUTTON, INPUT_PULLUP);
  pinMode(ENCODER_PIN_BTN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EXTRA_BUTTON), button_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_BTN), button_interrupt, CHANGE);


  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin(I2C_SLAVE_ADDR);
  Wire.onRequest(request_event);
  Wire.onReceive(receive_event);

  Serial.println("G5 i2c slave");
}

void loop()
{
  static int pos = 0;
  encoder.tick();
  int newPos = encoder.getPosition();
  if (data_changed || (pos != newPos && (millis() - last_transmission) > 100))   // Only send every few milliseconds
  {
    Serial.printf("[%ld] Pos: %d  Dir: %d EncBtn: %d  ExtraBtn: %d\n",millis(), newPos, encoder.getDirection(), digitalRead(ENCODER_PIN_BTN), digitalRead(EXTRA_BUTTON));
    encoder_delta = newPos - pos;
    signal_data_change();
    pos = newPos;
    last_transmission = millis();
    data_changed = false;
  }
}

void button_interrupt()
{
  data_changed = true;
}

 // Signal to ESP32 that data has changed via interrupt pin
 void signal_data_change() {
     // Pull interrupt line LOW to signal ESP32
     digitalWrite(INT_PIN, LOW);
     
     // Short pulse (1ms) is enough for ESP32 to detect
     delay(1);
     digitalWrite(INT_PIN, HIGH);
     // Leave data_changed flag set until data is read by ESP32
 }


 // Handler for I2C data requests from master
 void request_event() {
   // Format: [encoder_delta (1 byte), encoder_button (1 byte), extra_button (1 byte)]
   Wire.write(encoder_delta);
   Wire.write(!digitalRead(ENCODER_PIN_BTN));
   Wire.write(!digitalRead(EXTRA_BUTTON));
   
   // Reset delta and data_changed flag after sending
   encoder_delta = 0;
   data_changed = false;
   last_transmission = millis();
   
 //  Serial.println("Data sent to ESP32");
 }
 
 // Handler for I2C commands from master
 void receive_event(int byte_count) {
   if (byte_count > 0) {
     byte command = Wire.read();
     
     // Command for LED control
     if (command == 0x01) {
       if (byte_count > 1) {
         byte led_value = Wire.read();
         led_state = (led_value != 0);
         digitalWrite(LED_PIN, led_state ? HIGH : LOW);
         Serial.print("LED state changed to: ");
         Serial.println(led_state ? "ON" : "OFF");
       }
     }
     
     // Read any remaining bytes (prevent buffer issues)
     while (Wire.available()) {
       Wire.read();
     }
   }
 }