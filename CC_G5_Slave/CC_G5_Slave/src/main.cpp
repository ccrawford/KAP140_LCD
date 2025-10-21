#include <Arduino.h>
#include <Wire.h>
#include <RotaryEncoder.h>
#include <AceButton.h>

// Pin definitions
#define ENCODER_PIN_A 18     // Rotary encoder pin A
#define ENCODER_PIN_B 19     // Rotary encoder pin B
#define ENCODER_PIN_BTN 17   // Encoder built-in button
#define EXTRA_BUTTON 16      // Second separate button
#define LED_PIN 10           // LED pin
#define I2C_SDA_PIN 20       // I2C SDA pin
#define I2C_SCL_PIN 21       // I2C SCL pin
#define INT_PIN 22          // Interrupt pin to ESP32 (connects to GPIO16 on ESP32)

// Channel 2
#define ENCODER2_PIN_A 2     // Rotary encoder pin A
#define ENCODER2_PIN_B 3     // Rotary encoder pin B
#define ENCODER2_PIN_BTN 4   // Encoder built-in button
#define EXTRA2_BUTTON 14      // Second separate button
#define LED2_PIN 15           // LED pin
#define I2C2_SDA_PIN 26       // I2C SDA pin
#define I2C2_SCL_PIN 27       // I2C SCL pin
#define INT2_PIN 28          // Interrupt pin to ESP32 (connects to GPIO16 on ESP32)


#define I2C_SLAVE_ADDR 0x08 // I2C slave address
#define I2C2_SLAVE_ADDR 0x08 // I2C slave address

RotaryEncoder encoder(ENCODER_PIN_A, ENCODER_PIN_B, RotaryEncoder::LatchMode::FOUR3);
RotaryEncoder encoder2(ENCODER2_PIN_A, ENCODER2_PIN_B, RotaryEncoder::LatchMode::FOUR3);

using namespace ace_button;
AceButton pfdButton(EXTRA_BUTTON);

void handleButtonEvent(AceButton*, uint8_t, uint8_t);


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


void request2_event();
void receive2_event(int byte_count);
void signal2_data_change();  // Tell the esp that it needs to fetch data. 
void button2_interrupt();

int encoder2_delta = 0;
bool data2_changed = false;
bool led2_state = false;
bool encoder2_button = false;
bool extra2_button = false;

unsigned long last2_transmission = 0;




void setup()
{
  delay(5);
  
  Serial.begin(115200);
  Serial.println("G5 Pico I2C input device. OCT 2025. CAC");


  pinMode(INT_PIN, OUTPUT);
  digitalWrite(INT_PIN, HIGH);

//  pinMode(EXTRA_BUTTON, INPUT_PULLUP);

  ButtonConfig* buttonConfig = ButtonConfig::getSystemButtonConfig();
  buttonConfig->setEventHandler(handleButtonEvent);
  buttonConfig->setFeature(ButtonConfig::kFeatureClick);
  buttonConfig->setFeature(ButtonConfig::kFeatureLongPress);

  pinMode(ENCODER_PIN_BTN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EXTRA_BUTTON), button_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_BTN), button_interrupt, CHANGE);


  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin(I2C_SLAVE_ADDR);
  Wire.onRequest(request_event);
  Wire.onReceive(receive_event);

  Serial.println("G5 i2c slave");

  // Setup channel 2
  
  pinMode(INT2_PIN, OUTPUT);
  digitalWrite(INT2_PIN, HIGH);

  pinMode(EXTRA2_BUTTON, INPUT_PULLUP);
  pinMode(ENCODER2_PIN_BTN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EXTRA2_BUTTON), button2_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_PIN_BTN), button2_interrupt, CHANGE);


  Wire1.setSDA(I2C2_SDA_PIN);
  Wire1.setSCL(I2C2_SCL_PIN);
  Wire1.begin(I2C2_SLAVE_ADDR);
  Wire1.onRequest(request2_event);
  Wire1.onReceive(receive2_event);

}

void loop()
{

  static auto lastUpdate = millis();
  static int pos = 0;
  encoder.tick();

  int newPos = encoder.getPosition();
//   if ((data_changed || (pos != newPos)) && ((millis() - last_transmission) > 100))   // Only send every few milliseconds
  if (data_changed || ( (pos != newPos) && (millis() - lastUpdate) > 50 ))   // Only send every few milliseconds
  {
    encoder_delta += (newPos - pos) ;
    Serial.printf("[%ld] Pos: %d  Dir: %d EncDelta: %d EncBtn: %d  ExtraBtn: %d\n",millis(), newPos, encoder.getDirection(), encoder_delta, digitalRead(ENCODER_PIN_BTN), digitalRead(EXTRA_BUTTON));
    signal_data_change();
    pos = newPos;
    lastUpdate = millis();
    data_changed = false;
  }

  static auto last2Update = millis();
  static int pos2 = 0;
  encoder2.tick();

  int new2Pos = encoder2.getPosition();
//   if ((data_changed || (pos != newPos)) && ((millis() - last_transmission) > 100))   // Only send every few milliseconds
  if (data2_changed || ( (pos2 != new2Pos) && (millis() - last2Update) > 50 ))   // Only send every few milliseconds
  {
    encoder2_delta += (new2Pos - pos2) ;
    Serial.printf("[%ld] Pos2: %d  Dir: %d Enc2Delta: %d Enc2Btn: %d  Extra2Btn: %d\n",millis(), new2Pos, encoder2.getDirection(), encoder2_delta, digitalRead(ENCODER2_PIN_BTN), digitalRead(EXTRA2_BUTTON));
    signal2_data_change();
    pos2 = new2Pos;
    last2Update = millis();
    data2_changed = false;
  }





}

void button_interrupt()
{
  data_changed = true;
}

void button2_interrupt()
{
  data2_changed = true;
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

  // Signal to ESP32 that data has changed via interrupt pin
 void signal2_data_change() {
     // Pull interrupt line LOW to signal ESP32
     digitalWrite(INT2_PIN, LOW);
     
     // Short pulse (1ms) is enough for ESP32 to detect
     delay(1);
     digitalWrite(INT2_PIN, HIGH);
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
   
 }

  // Handler for I2C data requests from master
 void request2_event() {
   // Format: [encoder_delta (1 byte), encoder_button (1 byte), extra_button (1 byte)]
   Wire1.write(encoder2_delta);
   Wire1.write(!digitalRead(ENCODER2_PIN_BTN));
   Wire1.write(!digitalRead(EXTRA2_BUTTON));
   
   // Reset delta and data_changed flag after sending
   encoder2_delta = 0;
   data2_changed = false;
   last2_transmission = millis();
   
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

  // Handler for I2C commands from master
 void receive2_event(int byte_count) {
   if (byte_count > 0) {
     byte command = Wire1.read();
     
     // Command for LED control
     if (command == 0x01) {
       if (byte_count > 1) {
         byte led2_value = Wire1.read();
         led2_state = (led2_value != 0);
         digitalWrite(LED2_PIN, led2_state ? HIGH : LOW);
         Serial.print("LED2 state changed to: ");
         Serial.println(led2_state ? "ON" : "OFF");
       }
     }
     
     // Read any remaining bytes (prevent buffer issues)
     while (Wire1.available()) {
       Wire1.read();
     }
   }
 }

 void handleButtonEvent(AceButton* button, uint8_t eventType, uint8_t buttonState) {
  
 }