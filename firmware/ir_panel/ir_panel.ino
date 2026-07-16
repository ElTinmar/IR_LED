#include <Wire.h>
#include <Adafruit_MCP4728.h>
#include <SPI.h>

// ==========================================
// CONFIGURATION FLAGS
// ==========================================
#define DEBUG_MODE 1  // 1 = Debugging (Spammy Serial), 0 = Production (Silent/Clean Protocol)

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

// ==========================================
// SMART MCP PROXY CLASS (Runtime Fallback)
// ==========================================
class SmartMCP {
private:
  Adafruit_MCP4728 realMcp;
  bool fallbackActive = false;

public:
  bool begin(uint8_t address) {
    // 1. Safe physical I2C Pin Check (Nano Every A4/A5)
    pinMode(A4, INPUT_PULLUP);
    pinMode(A5, INPUT_PULLUP);
    delay(5); // Let pins settle
    
    bool busIsOk = (digitalRead(A4) == HIGH && digitalRead(A5) == HIGH);

    if (!busIsOk) {
      Serial.println("WARN: HW_I2C_BUS_DEAD! Falling back to MOCK MCP.");
      fallbackActive = true;
      return true; // Return true to allow system setup to proceed
    }

    // 2. Start I2C bus and attempt physical chip initialization
    Wire.begin();
    if (!realMcp.begin(address)) { 
      Serial.println("WARN: HW_MCP_MISSING! Falling back to MOCK MCP.");
      fallbackActive = true;
      return true; 
    }

    // Physical hardware is healthy
    Serial.println("SYS: Physical MCP4728 initialized successfully.");
    fallbackActive = false;
    return true;
  }

  bool setChannelValue(MCP4728_channel_t channel, uint16_t value, 
                       MCP4728_vref_t vref = MCP4728_VREF_VDD, 
                       MCP4728_gain_t gain = MCP4728_GAIN_1X, 
                       MCP4728_pd_mode_t pd = MCP4728_PD_MODE_NORMAL) {
    if (fallbackActive) {
      #if DEBUG_MODE
        Serial.print("MOCK: Ch ");
        Serial.print(channel);
        Serial.print(" -> ");
        Serial.println(value);
      #endif
      return true;
    } else {
      return realMcp.setChannelValue(channel, value, vref, gain, pd);
    }
  }

  bool isMockActive() const { return fallbackActive; }
};

SmartMCP mcp; // Instantiate our smart proxy instead of the raw driver

// --- Global Variables ---
uint16_t dacValues[3] = {2048, 2048, 2048}; 
int currentChannel = 0;                     

// Encoder State
int lastStateA;
bool lastButtonReading = HIGH;
bool debouncedButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// Serial Parsing Buffer
String inputString = "";

void init25kHzPWM() {
  TCA0.SPLIT.CTRLA = 0; 
  TCA0.SPLIT.CTRLA = TCA_SPLIT_CLKSEL_DIV4_gc | TCA_SPLIT_ENABLE_bm;
  TCA0.SPLIT.CTRLD = TCA_SPLIT_SPLITM_bm; 
  TCA0.SPLIT.LPER = 99; 
  TCA0.SPLIT.CTRLB &= ~(TCA_SPLIT_LCMP0EN_bm | TCA_SPLIT_LCMP1EN_bm);
  TCA0.SPLIT.CTRLB |= TCA_SPLIT_LCMP2EN_bm; // Fixed: LCMP2EN for Pin D5 / WO2
  TCA0.SPLIT.LCMP2 = 0; 
  pinMode(FAN_PWM_PIN, OUTPUT);
}

void setFanDutyCycle(uint8_t duty) {
  if (duty > 100) duty = 100;
  TCA0.SPLIT.LCMP2 = duty;
}

void setup() {
  Serial.begin(115200);
  
  unsigned long startWait = millis();
  while (!Serial && (millis() - startWait < 2000)) { 
    delay(10); 
  }
  
  Serial.println("SYS:BOOTING...");
  inputString.reserve(30);
  
  // Configure Peripheral Control Pins
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(ENCODER_SW, INPUT_PULLUP);
  
  pinMode(LED_CH1, OUTPUT);
  pinMode(LED_CH2, OUTPUT);
  pinMode(LED_CH3, OUTPUT);
  pinMode(LED_SATURATION, OUTPUT);
  
  pinMode(TMP126_CS, OUTPUT);
  digitalWrite(TMP126_CS, HIGH);

  // Initialize SPI & Timer PWM
  SPI.begin();
  init25kHzPWM(); 

  // Initialize the Smart MCP (handles bus testing & auto-fallback internally)
  mcp.begin(0x60);

  lastStateA = digitalRead(ENCODER_A);
  updateLEDs();
  updateDAC();
  
  Serial.println("SYS:READY");
}

void loop() {
  handleEncoderButton();
  handleEncoderRotation();
  
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n' || inChar == '\r') {
      if (inputString.length() > 0) {
        processSerialCommand(inputString);
        inputString = "";      
      }
    } else {
      inputString += inChar;
    }
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
  Serial.print(isSaturated ? "1" : "0");
  Serial.print(",MOCK:");
  Serial.println(mcp.isMockActive() ? "1" : "0");
}

// --- Serial Command Parser ---
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
  else if (command.startsWith("SET_DAC:")) {
    int colonIndex = command.indexOf(':');
    int commaIndex = command.indexOf(',');
    
    if (colonIndex != -1 && commaIndex != -1) {
      int targetChannel = command.substring(colonIndex + 1, commaIndex).toInt() - 1; 
      int dacValue = command.substring(commaIndex + 1).toInt();
      
      if (targetChannel >= 0 && targetChannel <= 2) {
        dacValues[targetChannel] = constrain(dacValue, 0, 4095);
        
        updateDAC();
        checkSaturation();
        
        Serial.print("RSP:DAC_SET,");
        Serial.print(targetChannel + 1);
        Serial.print(",");
        Serial.println(dacValues[targetChannel]);
      } else {
        Serial.println("ERR:INVALID_CH");
      }
    } else {
      Serial.println("ERR:INVALID_FORMAT");
    }
  }
  else if (command == "GET_DAC") {
    Serial.print("RSP:DAC_VALS,");
    Serial.print(dacValues[0]);
    Serial.print(",");
    Serial.print(dacValues[1]);
    Serial.print(",");
    Serial.println(dacValues[2]);
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

  if (reading != lastButtonReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != debouncedButtonState) {
      debouncedButtonState = reading;

      if (debouncedButtonState == LOW) {
        currentChannel++;
        if (currentChannel > 2) currentChannel = 0;
        
        updateLEDs();
        
        #if DEBUG_MODE
          transmitSystemState(); 
        #endif
      }
    }
  }

  lastButtonReading = reading;
}

void handleEncoderRotation() {
  int currentStateA = digitalRead(ENCODER_A);
  if (currentStateA != lastStateA && currentStateA == LOW) {
    if (digitalRead(ENCODER_B) == currentStateA) {
      if (dacValues[currentChannel] < 4045) dacValues[currentChannel] += 50;
      else dacValues[currentChannel] = 4095;
    } else {
      if (dacValues[currentChannel] > 50) dacValues[currentChannel] -= 50;
      else dacValues[currentChannel] = 0;
    }

    updateDAC();
    checkSaturation();
    
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