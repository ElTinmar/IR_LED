#include <Wire.h>
#include <Adafruit_MCP4728.h>
#include <SPI.h>

// ==========================================
// CONFIGURATION FLAGS
// ==========================================
#define DEBUG_MODE 1  // 1 = Debugging (Spammy Serial), 0 = Production (Silent/Clean Protocol)
#define USE_MOCK_MCP 0

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

#if USE_MOCK_MCP
class MockMCP {
public:
  bool begin(uint8_t address) {
    Serial.print("MOCK: Simulated MCP4728 online at 0x");
    Serial.println(address, HEX);
    return true; 
  }
  bool setChannelValue(MCP4728_channel_t channel, uint16_t value, 
                       MCP4728_vref_t vref = MCP4728_VREF_VDD, 
                       MCP4728_gain_t gain = MCP4728_GAIN_1X, 
                       MCP4728_pd_mode_t pd = MCP4728_PD_MODE_NORMAL) {
    // Silently accept the values to run state machine smoothly
    #if DEBUG_MODE
      Serial.print("MOCK: Ch ");
      Serial.print(channel);
      Serial.print(" -> ");
      Serial.println(value);
    #endif
    return true;
  }
};
MockMCP mcp; // Instantiate the software simulator
#else
Adafruit_MCP4728 mcp; // Instantiate the actual physical driver
#endif

void init25kHzPWM() {
  // 1. Force TCA0 Timer into Split Mode (gives us six 8-bit PWM channels)
  TCA0.SPLIT.CTRLA = 0; // Turn off timer momentarily to configure safely
  
  // Set Prescaler to 4 (TCA_SPLIT_CLKSEL_DIV4_gc) and Enable Timer
  TCA0.SPLIT.CTRLA = TCA_SPLIT_CLKSEL_DIV4_gc | TCA_SPLIT_ENABLE_bm;
  
  // Set Split Mode configuration bit (Fixed register typo here)
  TCA0.SPLIT.CTRLD = TCA_SPLIT_SPLITM_bm; 

  // 2. Define the Low Byte Period to achieve exactly 25kHz
  // Formula: 16MHz / (Prescaler * (LPER + 1)) -> 16,000,000 / (4 * 100) = 25,000 Hz
  TCA0.SPLIT.LPER = 99; 

  // 3. Connect Pin D5 (which is physical Port B, pin 2 / WO2) to the Timer Output Compare
  TCA0.SPLIT.CTRLB |= TCA_SPLIT_LCMP2EN_bm; 

  // 4. Initialize fan duty cycle to 0% (off) initially
  TCA0.SPLIT.LCMP2 = 0; 
  
  pinMode(FAN_PWM_PIN, OUTPUT);
}

void setFanDutyCycle(uint8_t duty) {
  if (duty > 100) duty = 100;
  TCA0.SPLIT.LCMP2 = duty;
}

void setup() {
  Serial.begin(115200);
  
  // Wait up to 2 seconds for Serial Monitor to connect so we don't miss startup messages
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

  // --- SAFE I2C CHECK FOR MEGAAVR (NANO EVERY) ---
  // Temporarily set I2C pins as inputs with pullups to see if they are physically clamped LOW
  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);
  delay(5); // Let the pins settle
  
  bool busIsOk = (digitalRead(A4) == HIGH && digitalRead(A5) == HIGH);

  if (!busIsOk) {
    Serial.println("ERR:HW_I2C_BUS_DEAD (Check if MCP has power!)");
    while (1) { 
      digitalWrite(LED_SATURATION, HIGH);
      delay(250);
      digitalWrite(LED_SATURATION, LOW);
      delay(250);
    }
  }

  Wire.begin();

  // Attempt to initialize the MCP4728 DAC
  if (!mcp.begin(0x60)) { 
    Serial.println("ERR:HW_MCP_MISSING"); 
    while (1) { 
      digitalWrite(LED_SATURATION, HIGH);
      delay(100);
      digitalWrite(LED_SATURATION, LOW);
      delay(100);
    } 
  }

  // If we made it here, the I2C bus is happy and the MCP is alive!
  lastStateA = digitalRead(ENCODER_A);
  updateLEDs();
  updateDAC();
  
  Serial.println("SYS:READY");
}

void loop() {
  handleEncoderButton();
  handleEncoderRotation();
  
  // Active Serial Polling (Replaces broken serialEvent)
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
  Serial.println(isSaturated ? "1" : "0");
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
    // Format: SET_DAC:ch,val (e.g., SET_DAC:1,2048)
    int colonIndex = command.indexOf(':');
    int commaIndex = command.indexOf(',');
    
    if (colonIndex != -1 && commaIndex != -1) {
      int targetChannel = command.substring(colonIndex + 1, commaIndex).toInt() - 1; // Convert 1-3 to index 0-2
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

  // If the physical switch state changed (due to noise or pressing), reset the timer
  if (reading != lastButtonReading) {
    lastDebounceTime = millis();
  }

  // Once the reading has been stable for longer than the debounce delay...
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // ...check if this is a genuine change in button state
    if (reading != debouncedButtonState) {
      debouncedButtonState = reading;

      // If the newly confirmed state is LOW (button pressed)
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

  // Save the raw reading for the next loop
  lastButtonReading = reading;
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