#include <Wire.h>
#include <Adafruit_MCP4728.h>
#include <SPI.h>

// --- Pin Definitions ---
const int ENCODER_A = 2;
const int ENCODER_B = 3;
const int ENCODER_SW = 4;
const int FAN_PWM_PIN = 5;  // Tied to TCA0 WO2 (Split Mode Low Byte)

// Status LEDs
const int LED_CH1 = 6;
const int LED_CH2 = 7;
const int LED_CH3 = 8;
const int LED_SATURATION = 9; 

// SPI TMP126
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

void init25kHzPWM() {
  // 1. Force TCA0 Timer into Split Mode (gives us six 8-bit PWM channels)
  TCA0.SPLIT.CTRLA = 0; // Turn off timer momentarily to configure safely
  
  // Set Prescaler to 4 (TCA_SPLIT_CLKSEL_DIV4_gc) and Enable Timer
  TCA0.SPLIT.CTRLA = TCA_SPLIT_CLKSEL_DIV4_gc | TCA_SPLIT_ENABLE_bm;
  
  // Set Split Mode configuration bit
  TCA0.SPLIT.CTRLD = TCA_SPLIT_SPLIT_bm; 

  // 2. Define the Low Byte Period to achieve exactly 25kHz
  // Formula: 16MHz / (Prescaler * (LPER + 1)) -> 16,000,000 / (4 * 100) = 25,000 Hz
  TCA0.SPLIT.LPER = 99; 

  // 3. Connect Pin D5 (which is physical Port B, pin 2 / WO2) to the Timer Output Compare
  TCA0.SPLIT.CTRLB |= TCA_SPLIT_HCMP2EN_bm; 

  // 4. Initialize fan duty cycle to 0% (off) initially
  TCA0.SPLIT.LCMP2 = 0; 
  
  pinMode(FAN_PWM_PIN, OUTPUT);
}

// Custom function to handle our custom 0-100 range for the 25kHz timer
void setFanDutyCycle(uint8_t duty) {
  // Constrain incoming value between 0% and 100%
  if (duty > 100) duty = 100;
  
  // Write the value to the Compare Match register for Pin D5
  TCA0.SPLIT.LCMP2 = duty;
}

void setup() {
  Serial.begin(115200);
  inputString.reserve(30);
  
  // Initialize Pins
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(ENCODER_SW, INPUT_PULLUP);
  
  pinMode(LED_CH1, OUTPUT);
  pinMode(LED_CH2, OUTPUT);
  pinMode(LED_CH3, OUTPUT);
  pinMode(LED_SATURATION, OUTPUT);
  
  pinMode(TMP126_CS, OUTPUT);
  digitalWrite(TMP126_CS, HIGH);

  // Initialize Hardware Peripherals
  SPI.begin();
  init25kHzPWM(); // Set up native 25kHz PWM on pin D5

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
  
  if (stringComplete) {
    processSerialCommand(inputString);
    inputString = "";      
    stringComplete = false;
  }
}

// --- Serial & Sensor Functions ---

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
  command.trim();
  
  if (command.startsWith("SET_FAN:")) {
    int valueStringIndex = command.indexOf(':');
    int fanValue = command.substring(valueStringIndex + 1).toInt();
    
    // The range is now 0 to 100 (0% to 100% Duty Cycle)
    fanValue = constrain(fanValue, 0, 100);
    setFanDutyCycle(fanValue);
    
    Serial.print("SUCCESS: Fan speed set to ");
    Serial.print(fanValue);
    Serial.println("%");
  } 
  else if (command == "GET_TEMP") {
    float temperature = readTMP126();
    Serial.print("TEMP: ");
    Serial.print(temperature, 2); 
    Serial.println(" C");
  } 
  else {
    Serial.println("ERR: Unknown Command. Use 'SET_FAN:0-100' or 'GET_TEMP'");
  }
}

float readTMP126() {
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(TMP126_CS, LOW);

  uint8_t msb = SPI.transfer(0x00); 
  uint8_t lsb = SPI.transfer(0x00);

  digitalWrite(TMP126_CS, HIGH);
  SPI.endTransaction();

  int16_t rawData = (msb << 8) | lsb;
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