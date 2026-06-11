#include <Wire.h>
#include <Adafruit_MCP4728.h>
#include <SPI.h>

// ==========================================
// CONFIGURATION FLAGS
// ==========================================
#define DEBUG_MODE 0  // 1 = Debugging (Spammy Serial), 0 = Production (Silent/Clean Protocol)

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
  TCA0.SPLIT.CTRLA = 0; 
  TCA0.SPLIT.CTRLA = TCA_SPLIT_CLKSEL_DIV4_gc | TCA_SPLIT_ENABLE_bm;
  TCA0.SPLIT.CTRLD = TCA_SPLIT_SPLIT_bm; 
  TCA0.SPLIT.LPER = 99; 
  TCA0.SPLIT.CTRLB |= TCA_SPLIT_HCMP2EN_bm; 
  TCA0.SPLIT.LCMP2 = 0; 
  pinMode(FAN_PWM_PIN, OUTPUT);
}

void setFanDutyCycle(uint8_t duty) {
  if (duty > 100) duty = 100;
  TCA0.SPLIT.LCMP2 = duty;
}

void setup() {
  Serial.begin(115200);
  inputString.reserve(30);
  
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(ENCODER_SW, INPUT_PULLUP);
  
  pinMode(LED_CH1, OUTPUT);
  pinMode(LED_CH2, OUTPUT);
  pinMode(LED_CH3, OUTPUT);
  pinMode(LED_SATURATION, OUTPUT);
  
  pinMode(TMP126_CS, OUTPUT);
  digitalWrite(TMP126_CS, HIGH);

  SPI.begin();
  init25kHzPWM(); 

  if (!mcp.begin(0x60)) { 
    Serial.println("ERR:HW_MCP"); 
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

// --- Telemetry Output ---
void transmitSystemState() {
  bool isSaturated = (dacValues[currentChannel] == 0 || dacValues[currentChannel] == 4095);
  Serial.print("CH:");
  Serial.print(currentChannel + 1);
  Serial.print(",VAL:");
  Serial.print(dacValues[currentChannel]);
  Serial.print(",SAT:");
  Serial.println(isSaturated ? "1" : "0");
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
    fanValue = constrain(fanValue, 0, 100);
    setFanDutyCycle(fanValue);
    
    Serial.print("RSP:FAN,");
    Serial.println(fanValue);
  } 
  else if (command == "GET_TEMP") {
    float temperature = readTMP126();
    Serial.print("RSP:TMP,");
    Serial.println(temperature, 2); 
  } 
  else {
    Serial.println("ERR:CMD");
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
  return (rawData >> 2) * 0.03125; 
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
      
      updateLEDs();
      
      // Conditional compile: Only alerts PC on press if debugging is active
      #if DEBUG_MODE
        transmitSystemState(); 
      #endif
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

    updateDAC();
    checkSaturation();
    
    // Conditional compile: Only alerts PC on rotate if debugging is active
    #if DEBUG_MODE
      transmitSystemState(); 
    #endif
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