#include <Wire.h>
#include <Adafruit_MCP4728.h>
#include <SPI.h>

// --- Pin Definitions ---
const int ENCODER_A = 2;
const int ENCODER_B = 3;
const int ENCODER_SW = 4;
const int FAN_PWM_PIN = 5;

// Status LEDs
const int LED_CH1 = 6;
const int LED_CH2 = 7;
const int LED_CH3 = 8;
const int LED_SATURATION = 9; 

// SPI TMP126 (Nano Every default SPI hardware: MOSI=11, MISO=12, SCK=13)
const int TMP126_CS = 10; 

// --- Global Variables ---
Adafruit_MCP4728 mcp;

uint16_t dacValues[3] = {2048, 2048, 2048}; 
int currentChannel = 0;                     

// Encoder State
int lastStateA;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50; 
bool lastButtonState = HIGH;

// Serial Parsing Buffer
String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(115200);
  inputString.reserve(30); // Reserve memory for serial string parsing
  
  // Initialize Pins
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(ENCODER_SW, INPUT_PULLUP);
  
  pinMode(FAN_PWM_PIN, OUTPUT);
  
  pinMode(LED_CH1, OUTPUT);
  pinMode(LED_CH2, OUTPUT);
  pinMode(LED_CH3, OUTPUT);
  pinMode(LED_SATURATION, OUTPUT);
  
  pinMode(TMP126_CS, OUTPUT);
  digitalWrite(TMP126_CS, HIGH); // Deselect TMP126 initially

  // Initialize SPI for TMP126
  SPI.begin();

  // Initialize I2C and MCP4728
  if (!mcp.begin(0x60)) { 
    Serial.println("ERR: MCP4728 not found!");
    while (1) { delay(10); }
  }

  lastStateA = digitalRead(ENCODER_A);

  updateLEDs();
  updateDAC();
}

void loop() {
  handleEncoderButton();
  handleEncoderRotation();
  
  // Check if a full serial command has arrived
  if (stringComplete) {
    processSerialCommand(inputString);
    inputString = "";      // Clear string for next command
    stringComplete = false;
  }
}

// --- Serial & Sensor Functions ---

// Collects serial data on the fly without blocking the main loop execution
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n' || inChar == '\r') {
      if (inputString.length() > 0) {
        stringComplete = true;
      }
    } else {
      inputString += inChar;
    }
  }
}

void processSerialCommand(String command) {
  command.trim(); // Strip extra spaces
  
  if (command.startsWith("SET_FAN:")) {
    int valueStringIndex = command.indexOf(':');
    int fanValue = command.substring(valueStringIndex + 1).toInt();
    
    // Constrain to typical 8-bit PWM spectrum (0-255)
    fanValue = constrain(fanValue, 0, 255);
    analogWrite(FAN_PWM_PIN, fanValue);
    
    Serial.print("SUCCESS: Fan speed set to ");
    Serial.println(fanValue);
  } 
  else if (command == "GET_TEMP") {
    float temperature = readTMP126();
    Serial.print("TEMP: ");
    Serial.print(temperature, 2); // Print out temperature with 2 decimal places
    Serial.println(" C");
  } 
  else {
    Serial.println("ERR: Unknown Command. Use 'SET_FAN:value' or 'GET_TEMP'");
  }
}

// Reads and decodes temperature data out of the TMP126 via SPI
float readTMP126() {
  // Set SPI settings: Up to 10 MHz clock speed, MSB First, SPI Mode 0
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(TMP126_CS, LOW);

  // Read 2 bytes from the TMP126 Temperature Register
  uint8_t msb = SPI.transfer(0x00); 
  uint8_t lsb = SPI.transfer(0x00);

  digitalWrite(TMP126_CS, HIGH);
  SPI.endTransaction();

  // Combine bytes into a signed 16-bit integer
  int16_t rawData = (msb << 8) | lsb;

  // The TMP126 features a 14-bit resolution scaled down inside a 16-bit window, 
  // or a straight 14-bit/16-bit map depending on configuration mode. 
  // Standard conversion for standard 14-bit data right-justified (0.03125°C per LSB):
  float tempC = (rawData >> 2) * 0.03125; 

  return tempC;
}

// --- UI & Peripheral Control Logic ---

void handleEncoderButton() {
  bool reading = digitalRead(ENCODER_SW);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading == LOW && lastButtonState == HIGH) {
      currentChannel++;
      if (currentChannel > 2) currentChannel = 0;
      
      Serial.print("SYS: Active Channel -> ");
      Serial.println(currentChannel + 1);
      updateLEDs();
    }
  }
  lastButtonState = reading;
}

void handleEncoderRotation() {
  int currentStateA = digitalRead(ENCODER_A);
  if (currentStateA != lastStateA && currentStateA == LOW) {
    if (digitalRead(ENCODER_B) != currentStateA) {
      if (dacValues[currentChannel] < 4045) dacValues[currentChannel] += 50;
      else dacValues[currentChannel] = 4095;
    } else {
      if (dacValues[currentChannel] > 50) dacValues[currentChannel] -= 50;
      else dacValues[currentChannel] = 0;
    }

    Serial.print("SYS: Ch ");
    Serial.print(currentChannel + 1);
    Serial.print(" -> Value: ");
    Serial.println(dacValues[currentChannel]);

    updateDAC();
    checkSaturation();
  }
  lastStateA = currentStateA;
}

void updateLEDs() {
  digitalWrite(LED_CH1, currentChannel == 0 ? HIGH : LOW);
  digitalWrite(LED_CH2, currentChannel == 1 ? HIGH : LOW);
  digitalWrite(LED_CH3, currentChannel == 2 ? HIGH : LOW);
  checkSaturation();
}

void updateDAC() {
  mcp.setChannelValue(MCP4728_CHANNEL_A, dacValues[0]);
  mcp.setChannelValue(MCP4728_CHANNEL_B, dacValues[1]);
  mcp.setChannelValue(MCP4728_CHANNEL_C, dacValues[2]);
}

void checkSaturation() {
  if (dacValues[currentChannel] == 0 || dacValues[currentChannel] == 4095) {
    digitalWrite(LED_SATURATION, HIGH);
  } else {
    digitalWrite(LED_SATURATION, LOW);
  }
}